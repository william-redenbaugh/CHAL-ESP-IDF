#ifndef _IDF_CHAL_STRUCT_DEFINITIONS_H
#define _IDF_CHAL_STRUCT_DEFINITIONS_H

#include <driver/i2s.h>

typedef struct os_i2s
{
    i2s_pin_config_t pin_config;
    int chip_select_gpio;
    i2s_port_t port;

    i2s_config_t cfg;
} os_i2s_host_t;

typedef enum
{
    STRIP_NEOPIXEL_RGB
} led_strip_type_t;

#define ZERO_BUFFER_SIZE (48)
#define BYTES_PER_COLOR (4)
#define RGB_BYTES (3)
#define SAMPLE_RATE (93750)

typedef struct os_led_strip
{
    int bus;
    led_strip_type_t type;
    int gpio;

    uint32_t numpixel;
    size_t out_buffer_size;
    uint8_t *out_buffer;
    uint8_t off_buffer[ZERO_BUFFER_SIZE];
    void *mutex;

} os_led_strip_t;

/**
 * @brief Remaps the esp errores to OS errors
 */
int esp_to_os(esp_err_t err);

#define HAL_ASSERT(e) assert(e)
#endif