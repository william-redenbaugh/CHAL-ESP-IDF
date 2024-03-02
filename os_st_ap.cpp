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

#ifdef OS_WIFI

static ap_station_cb ap_cb = NULL;

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    ap_station_event_t event;

    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        event = AP_STATION_EVENT_CONNECTED;
                 
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        event = AP_STATION_EVENT_DISCONNECTED;
    }
    if(ap_cb){
        wifi_event_ap_stadisconnected_t* event_data;
        ap_cb(event, event_data->mac);
    }
}

int os_start_local_ap(char *ssid, char *password, int max_clients, uint16_t channel, ap_station_cb cb){
    int password_len = strlen(password);
    int ssid_len = strlen(ssid);
    wifi_config_t wifi_config = {
        .ap = {
            .ssid_len = (uint8_t)ssid_len,
            .channel = (uint8_t)channel,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .max_connection = (uint8_t)max_clients,      
        }
    };

    if(password_len > sizeof(wifi_config.ap.password) | ssid_len > sizeof(wifi_config.ap.ssid)){
        return OS_RET_INVALID_PARAM;
    }

    esp_err_t err = esp_netif_init();

    int ret = esp_to_os(err);
    if(ret != OS_RET_OK){
        return ret;
    }

    err = esp_event_loop_create_default();
    ret = esp_to_os(err);
    if(ret != OS_RET_OK){
        return ret;
    }

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    ret = esp_to_os(err);
    if(ret != OS_RET_OK){
        return ret;
    }

    err = esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL);
    
    ret = esp_to_os(err);
    if(ret != OS_RET_OK){
        return ret;
    }                               

    // Copy over password and ssid
    memcpy(wifi_config.ap.ssid, ssid, wifi_config.ap.ssid_len);
    memcpy(wifi_config.ap.password, password, password_len);

    // No password heh
    if (password_len == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    err = esp_wifi_set_mode(WIFI_MODE_AP);
    ret = esp_to_os(err);
    if(ret != OS_RET_OK){
        return ret;
    }

    err = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    ret = esp_to_os(err);
    if(ret != OS_RET_OK){
        return ret;
    }

    err = esp_wifi_start();
    ret = esp_to_os(err);
    if(ret != OS_RET_OK){
        return ret;
    }

    return OS_RET_OK;
}
#endif