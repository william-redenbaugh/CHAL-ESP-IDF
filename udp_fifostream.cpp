#include "global_includes.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

static const uint32_t BUFFER_SIZE = 4096;

struct udp_fifo_t{
    uint16_t port;
    bool socket_open;   
    TaskHandle_t handle;
//    ESP32DMASPI::Master master;
    uint8_t* tx_buff;
    uint8_t* rx_buff;
};

static void udp_server_task(void *pvParameters)
{
    udp_fifo_t *fifo = (udp_fifo_t*)pvParameters;
    uint8_t rx_buffer[4096];
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    while (1) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(fifo->port);
        ip_protocol = IPPROTO_IP;
        

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            os_printf("Unable to create socket: errno %d", errno);
            break;
        }
        os_printf("Socket created");

        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        //setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            os_printf("Socket unable to bind: errno %d", errno);
        }
        os_printf("Socket bound, port %d", fifo->port);

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(source_addr);

        while (1) {
            os_printf("Waiting for data");

            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
            // Error occurred during receiving
            if (len < 0) {
                os_printf("recvfrom failed: errno %d\n", errno);
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (source_addr.ss_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
                } else if (source_addr.ss_family == PF_INET6) {
                    inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
                os_printf("Received %d bytes from %s:", len, addr_str);
                os_printf("%s", rx_buffer);

                //fifo->master.transfer(rx_buffer, len);
            }
        }

        if (sock != -1) {
            os_printf("Shutting down socket and restarting...\n");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}


udp_fifo_t *generate_udp_fifo(udp_fifo_init_t init_params){
    udp_fifo_t *fifo = new udp_fifo_t;

/*    
    fifo->tx_buff = fifo->master.allocDMABuffer(BUFFER_SIZE);
    fifo->rx_buff = fifo->master.allocDMABuffer(BUFFER_SIZE);

    fifo->master.setDataMode(SPI_MODE0);
    fifo->master.setFrequency(4000000);
    fifo->master.setMaxTransferSize(BUFFER_SIZE);
    
    fifo->master.begin(HSPI, init_params.clk, init_params.miso, init_params.mosi, init_params.cs);

    fifo->port = init_params.port;
    xTaskCreate(udp_server_task, "udp_server", 8192, (void*)fifo, 5, &fifo->handle);
*/
    return fifo;    
}