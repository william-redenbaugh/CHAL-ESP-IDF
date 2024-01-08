#include "global_includes.h"
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

// ESP32 I2C bus number
#define I2C_NUM I2C_NUM_0

#ifdef CONFIG_IDF_TARGET_ESP32S3

#define I2C_SDA GPIO_NUM_21
#define I2C_SCL GPIO_NUM_26
#elif CONFIG_IDF_TARGET_ESP32C3

#define I2C_SDA GPIO_NUM_8
#define I2C_SCL GPIO_NUM_9
#else

#endif
int os_i2c_begin(os_i2c_t *i2c, int fd, int speed, os_i2c_gpio_t *gpio_conf)
{
    i2c_config_t conf;

    if (gpio_conf)
    {
        conf.sda_io_num = gpio_conf->sda_gpio;
        conf.scl_io_num = gpio_conf->scl_gpio;
    }
    else
    {
        conf.sda_io_num = I2C_SDA; // Replace with your SDA GPIO pin number
        conf.scl_io_num = I2C_SCL; // Replace with your SCL GPIO pin number
    }

    conf.mode = I2C_MODE_MASTER;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = speed;

    esp_err_t ret = i2c_param_config(I2C_NUM, &conf);

    if (ret != ESP_OK)
    {
        return OS_RET_INT_ERR; // Configuration error
    }

    ret = i2c_driver_install(I2C_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK)
    {
        return OS_RET_INT_ERR; // Driver installation error
    }

    i2c->fd = I2C_NUM;
    i2c->speed = speed;

    return OS_RET_OK;
}

int os_i2c_end(os_i2c_t *i2c)
{
    i2c_driver_delete((i2c_port_t)i2c->fd);
    return OS_RET_OK;
}

int os_i2c_setbus(os_i2c_t *i2c, uint32_t freq_hz)
{
    i2c_config_t conf = i2c_config_t();
    esp_err_t ret = i2c_param_config((i2c_port_t)i2c->fd, &conf);
    if (ret != ESP_OK)
    {
        return OS_RET_INT_ERR; // Configuration error
    }
    conf.master.clk_speed = freq_hz;
    ret = i2c_param_config((i2c_port_t)i2c->fd, &conf);
    if (ret != ESP_OK)
    {
        return OS_RET_INT_ERR; // Configuration error
    }
    return OS_RET_OK;
}

int os_i2c_send(os_i2c_t *i2c, uint8_t addr, uint8_t *buf, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buf, size, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin((i2c_port_t)i2c->fd, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK) ? OS_RET_OK : OS_RET_IO_ERROR;
}

int os_i2c_receive(os_i2c_t *i2c, uint8_t addr, uint8_t *buf, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (size > 1)
    {
        i2c_master_read(cmd, buf, size - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buf + size - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin((i2c_port_t)i2c->fd, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK) ? OS_RET_OK : OS_RET_IO_ERROR;
}
