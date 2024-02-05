#include "global_includes.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

int os_uart_begin(os_uart_t *uart, os_uart_config_t cfg, int fd, int baud)
{

    uart->fd = fd;
    uart->bus = fd;
    uart->baud = baud;
    uart->cfg = cfg;

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_XTAL,
    };

    int intr_alloc_flags = 0;

    ESP_ERROR_CHECK(uart_driver_install(fd, 2048, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(fd, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(fd, cfg.tx_gpio, cfg.rx_gpio, -1, -1));

    return OS_RET_OK;
}

/**
 * @brief Sends data to the UART interface
 * @param os_uart_t *pointer to the UART interface
 */
int os_uart_send(os_uart_t *uart, uint8_t *buf, size_t size)
{
    int sent = uart_write_bytes(uart->fd, buf, size);
    if (sent != size)
    {
        return OS_RET_INT_ERR;
    }
    return OS_RET_OK;
}

/**
 * @brief Recieves Data from the UART interface
 * @param os_uart_t *pointer to the UART interface
 */
int os_uart_recieve(os_uart_t *uart, uint8_t *buf, size_t size)
{
    size_t bytes_back = uart_read_bytes(uart->fd, buf, size, portMAX_DELAY);
    if(bytes_back != size){
        return OS_RET_INT_ERR;
    }

    return OS_RET_OK;
}

int os_uart_recieve_timeout(os_uart_t *uart, uint8_t *buf, size_t size, uint32_t timeout_ms)
{
    size_t bytes_back = uart_read_bytes(uart->fd, buf, size, timeout_ms / portTICK_PERIOD_MS);
    if(bytes_back != size){
        return OS_RET_INT_ERR;
    }

    return OS_RET_OK;
}