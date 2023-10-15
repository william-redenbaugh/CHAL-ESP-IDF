#include "global_includes.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

typedef enum wifi_init_status
{
    WIFI_STATUS_UNINIT,
    WIFI_STATUS_STA
} wifi_init_status_t;

// Current wifi status
static wifi_init_status_t current_status = WIFI_STATUS_UNINIT;

#define WIFI_LOGGING
#ifdef WIFI_LOGGING
#define WIFI_LOG(e, ...) os_println(e)
#else
#define WIFI_LOG(e, ...) (void)e
#endif

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define EXAMPLE_ESP_MAXIMUM_RETRY 30

static const char *TAG = "wifi station";
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

esp_netif_t *current_netif_handler = NULL;
typedef struct os_udp_server_esp32_t
{
} os_udp_server_esp32_t;

static void os_udp_server_thread(void *parameters)
{
    os_udp_server_instance_t *udp_server = (os_udp_server_instance_t *)parameters;
    int addr_family = 10;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;
    for (;;)
    {
        if (addr_family == AF_INET)
        {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = udp_server->params.port;
            ip_protocol = IPPROTO_IP;
        }
        else if (addr_family == AF_INET6)
        {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port = udp_server->params.port;
            ip_protocol = IPPROTO_IPV6;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        int enable = 1;
        lwip_setsockopt(sock, IPPROTO_IP, IP_PKTINFO, &enable, sizeof(enable));

        if (addr_family == AF_INET6)
        {
            // Note that by default IPV6 binds to both protocols, it is must be disabled
            // if both protocols used at the same time (used in CI)
            int opt = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
        }

        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0)
        {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(source_addr);

        struct iovec iov;
        struct msghdr msg;
        struct cmsghdr *cmsgtmp;
        u8_t cmsg_buf[CMSG_SPACE(sizeof(struct in_pktinfo))];

        iov.iov_base = udp_server->recv_buffer;
        iov.iov_len = udp_server->params.max_buffer_size;
        msg.msg_control = cmsg_buf;
        msg.msg_controllen = sizeof(cmsg_buf);
        msg.msg_flags = 0;
        msg.msg_iov = &iov;
        msg.msg_iovlen = 1;
        msg.msg_name = (struct sockaddr *)&source_addr;
        msg.msg_namelen = socklen;
    }
}

int os_udp_init_server(os_udp_server_instance_t *udp_server, os_udp_server_params_t udp_server_params)
{
    if (udp_server == NULL)
    {
        return OS_RET_NULL_PTR;
    }

    // Copy all the params over in one fell swoop
    udp_server->params = udp_server_params;

    // Default construct the udp server
    udp_server->internal_udp_server_struct = malloc(sizeof(os_udp_server_esp32_t));
    memset(&udp_server->internal_udp_server_struct, 0, sizeof(os_udp_server_esp32_t));
    udp_server->recv_buffer = (uint8_t *)malloc(sizeof(uint8_t) * udp_server_params.max_buffer_size);

    udp_server->deconstructed = 0;

    // Thread handling socket info
    os_add_thread(os_udp_server_thread, (void *)udp_server, 2048, NULL);

    return OS_RET_OK;
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            WIFI_LOG("retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        WIFI_LOG("connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

int os_wifi_start_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();
    int ret = esp_to_os(esp_netif_init());

    if (ret != OS_RET_OK)
    {
        return ret;
    }

    ret = esp_to_os(esp_event_loop_create_default());
    if (ret != OS_RET_OK)
    {
        return ret;
    }

    current_netif_handler = esp_netif_create_default_wifi_sta();
    if (current_netif_handler == NULL)
    {
        return OS_RET_INT_ERR;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_to_os(esp_wifi_init(&cfg));
    if (ret != OS_RET_OK)
    {
        return ret;
    }

    ret = esp_to_os(esp_wifi_set_mode(WIFI_MODE_STA));
    ret = esp_to_os(esp_wifi_start());

    return ret;
}

int os_wifi_connect_sta(char *ssid, char *password)
{

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    int ret = esp_to_os(esp_event_handler_instance_register(WIFI_EVENT,
                                                            ESP_EVENT_ANY_ID,
                                                            &event_handler,
                                                            NULL,
                                                            &instance_any_id));

    if (ret != OS_RET_OK)
    {
        return ret;
    }

    ret = esp_to_os(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    if (ret != OS_RET_OK)
    {
        return ret;
    }

    wifi_config_t wifi_config = {
        .sta = {
            .threshold =
                {
                    .authmode = WIFI_AUTH_WPA2_PSK,
                },
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };

    // Copy information
    strcpy((char *)wifi_config.sta.ssid, ssid);
    strcpy((char *)wifi_config.sta.password, password);

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    WIFI_LOG("wifi_init_sta finished.\n");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        WIFI_LOG("connected to ap SSID:%s password:%s\n",
                 ssid, password);
        return OS_RET_OK;
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        WIFI_LOG("Failed to connect to SSID:%s, password:%s\n",
                 ssid, password);
        return OS_RET_CONNECTION_FAILED;
    }
    else
    {
        WIFI_LOG("UNEXPECTED EVENT");
        return OS_RET_INT_ERR;
    }

    return OS_RET_OK;
}