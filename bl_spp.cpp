// hal_bluetooth_serial.c
#include "global_includes.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#define SPP_SERVER_NAME "SPP_SERVER"
#define BT_DEVICE_NAME DEVICE_NAME

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const bool esp_spp_enable_l2cap_ertm = true;

static struct timeval time_new, time_old;
static long data_num = 0;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static int handle = -1;

// Queue holding all UART data.
static safe_circular_queue_t serial_queue;

static char *bda2str(uint8_t *bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18)
    {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    char bda_str[18] = {0};

    switch (event)
    {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
    {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
        {
            print("authentication success: %s bda:[%s]", param->auth_cmpl.device_name,
                  bda2str(param->auth_cmpl.bda, bda_str, sizeof(bda_str)));
        }
        else
        {
            print("authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:
    {
        print("ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit)
        {
            print("Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        }
        else
        {
            print("Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

    case ESP_BT_GAP_CFM_REQ_EVT:
        print("ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %" PRIu32, param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        print("ESP_BT_GAP_KEY_NOTIF_EVT passkey:%" PRIu32, param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        print("ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;

    case ESP_BT_GAP_MODE_CHG_EVT:
        print("ESP_BT_GAP_MODE_CHG_EVT mode:%d bda:[%s]", param->mode_chg.mode,
              bda2str(param->mode_chg.bda, bda_str, sizeof(bda_str)));
        break;

    default:
    {
        print("event: %d", event);
        break;
    }
    }
    return;
}

static void bt_spp_recv_cb(uint8_t *data, size_t len)
{
}

int hal_bluetooth_serial_send(const uint8_t *data, size_t len)
{
    if (handle == -1)
        return OS_RET_INT_ERR;

    return esp_to_os(esp_spp_write(handle, len, (uint8_t *)data));
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    char bda_str[18] = {0};

    switch (event)
    {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS)
        {
            print("ESP_SPP_INIT_EVT");
            esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
        }
        else
        {
            print("ESP_SPP_INIT_EVT status:%d", param->init.status);
        }
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        print("ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        print("ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        print("ESP_SPP_CLOSE_EVT status:%d handle:%" PRIu32 " close_by_remote:%d", param->close.status,
              param->close.handle, param->close.async);
        break;
    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS)
        {
            print("ESP_SPP_START_EVT handle:%" PRIu32 " sec_id:%d scn:%d", param->start.handle, param->start.sec_id,
                  param->start.scn);
            esp_bt_dev_set_device_name(BT_DEVICE_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        }
        else
        {
            print("ESP_SPP_START_EVT status:%d", param->start.status);
        }
        break;
    case ESP_SPP_CL_INIT_EVT:
        print("ESP_SPP_CL_INIT_EVT");
        handle = param->cl_init.handle;
        break;

    case ESP_SPP_DATA_IND_EVT:
        /*
         * We only show the data in which the data length is less than 128 here. If you want to print the data and
         * the data rate is high, it is strongly recommended to process them in other lower priority application task
         * rather than in this callback directly. Since the printing takes too much time, it may stuck the Bluetooth
         * stack and also have a effect on the throughput!
         */
        print("ESP_SPP_DATA_IND_EVT len:%d handle:%d",
              param->data_ind.len, param->data_ind.handle);
        if (param->data_ind.len < 128)
        {
            esp_log_buffer_hex("", param->data_ind.data, param->data_ind.len);
        }

        bt_spp_recv_cb(param->data_ind.data, param->data_ind.len);

        break;
    case ESP_SPP_CONG_EVT:
        print("ESP_SPP_CONG_EVT");
        break;
    case ESP_SPP_WRITE_EVT:
        print("ESP_SPP_WRITE_EVT");
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        print("ESP_SPP_SRV_OPEN_EVT status:%d handle:%" PRIu32 ", rem_bda:[%s]", param->srv_open.status,
              param->srv_open.handle, bda2str(param->srv_open.rem_bda, bda_str, sizeof(bda_str)));
        gettimeofday(&time_old, NULL);
        break;
    case ESP_SPP_SRV_STOP_EVT:
        print("ESP_SPP_SRV_STOP_EVT");
        break;
    case ESP_SPP_UNINIT_EVT:
        print("ESP_SPP_UNINIT_EVT");
        break;
    default:
        break;
    }
}

hal_bt_serial_err_t hal_bluetooth_serial_init(void)
{
    // Release all existing ble profiles
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_err_t ret;

    // Initialize the bluetooth controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK)
    {
        print("%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return HAL_BT_SERIAL_ERROR;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK)
    {
        print("%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return HAL_BT_SERIAL_ERROR;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK)
    {
        print("%s initialize bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return HAL_BT_SERIAL_ERROR;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK)
    {
        print("%s enable bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return HAL_BT_SERIAL_ERROR;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK)
    {
        print("%s gap register failed: %s", __func__, esp_err_to_name(ret));
        return HAL_BT_SERIAL_ERROR;
    }

    return HAL_BT_SERIAL_OK;
}
/*
#define RX_BUFFER_SIZE 128

static uint8_t rx_buffer[RX_BUFFER_SIZE];
static uint32_t rx_buffer_head = 0;
static uint32_t rx_buffer_tail = 0;

static esp_spp_cb_t spp_callback = NULL;

static void spp_data_received(unsigned char *data, unsigned int len) {
    // Place received data in the circular buffer
    for (size_t i = 0; i < len; i++) {
        uint32_t next_head = (rx_buffer_head + 1) % RX_BUFFER_SIZE;
        if (next_head != rx_buffer_tail) {
            rx_buffer[rx_buffer_head] = data[i];
            rx_buffer_head = next_head;
        } else {
            // Circular buffer overflow occurred
            // You may choose to handle this error differently based on your requirements
            break;
        }
    }
}

hal_bt_serial_err_t hal_bluetooth_serial_init(void) {
    // Initialize the Bluetooth stack
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_bt_controller_init(&bt_cfg);
    if (err != ESP_OK) {
        return HAL_BT_SERIAL_ERROR;
    }

    err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (err != ESP_OK) {
        return HAL_BT_SERIAL_ERROR;
    }

    // Initialize the Bluetooth SPP (Serial Port Profile) module
    err = esp_spp_init(ESP_SPP_MODE_CB);
    if (err != ESP_OK) {
        return HAL_BT_SERIAL_ERROR;
    }

    esp_spp_register_callback(spp_data_received);

    // Set the SPP security and visibility
    err = esp_bt_dev_set_device_name("MyBluetoothSerialDevice");
    if (err != ESP_OK) {
        return HAL_BT_SERIAL_ERROR;
    }

    err = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    if (err != ESP_OK) {
        return HAL_BT_SERIAL_ERROR;
    }

    return HAL_BT_SERIAL_OK;
}

hal_bt_serial_err_t hal_bluetooth_serial_send(const uint8_t *data, size_t len) {
    // Perform argument validation
    if (data == NULL || len == 0) {
        return HAL_BT_SERIAL_INVALID_ARG;
    }

    // Implement the code to send data over Bluetooth SPP here
    // For simplicity, we'll assume that data is sent in a single packet
    esp_err_t err = esp_spp_write(data, len);
    if (err != ESP_OK) {
        return HAL_BT_SERIAL_ERROR;
    }

    return HAL_BT_SERIAL_OK;
}

hal_bt_serial_err_t hal_bluetooth_serial_receive(uint8_t *data, size_t len) {
    // Perform argument validation
    if (data == NULL || len == 0) {
        return HAL_BT_SERIAL_INVALID_ARG;
    }

    // Read data from the circular buffer into the provided buffer
    uint32_t available_data = (rx_buffer_head - rx_buffer_tail) % RX_BUFFER_SIZE;

    if (available_data >= len) {
        for (size_t i = 0; i < len; i++) {
            data[i] = rx_buffer[rx_buffer_tail];
            rx_buffer_tail = (rx_buffer_tail + 1) % RX_BUFFER_SIZE;
        }
        return HAL_BT_SERIAL_OK;
    } else {
        // Insufficient data in the buffer to fulfill the request
        return HAL_BT_SERIAL_ERROR;
    }
}

void hal_bluetooth_serial_close(void) {
    // Close the Bluetooth SPP connection and cleanup
    esp_spp_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
}
*/