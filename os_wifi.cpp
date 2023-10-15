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
#include "lwip/sys.h"

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