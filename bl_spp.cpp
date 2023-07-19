// hal_bluetooth_serial.c
#include "global_includes.h"
#include "esp_bt.h"
#include "esp_spp_api.h"
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