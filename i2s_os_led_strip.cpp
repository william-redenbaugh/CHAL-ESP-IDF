#include "global_includes.h"
#ifdef ARDUINO_I2S_ESP32_STRIP

#define RESET_SIZE 200
#define SAMPLE_RATE (100000)
#define BYTES_FOR_LED_BYTE 4
#define NB_COLORS 3
#define BYTES_FOR_LED BYTES_FOR_LED_BYTE *NB_COLORS
#define DATA_SIZE BYTES_FOR_LED *NB_LEDS
#define RESET_SIZE 200

static uint8_t get_protocol_eq(uint8_t data, int pos)
{
    uint8_t eq = 0;
    if (data & (1 << (2 * (3 - pos))))
        eq = 0b1110;
    else
        eq = 0b1000;
    if (data & (2 << (2 * (3 - pos))))
        eq += 0b11100000;
    else
        eq += 0b10000000;
    return eq;
}

typedef struct _os_led_strip_t
{
    int bus;
    uint8_t *pixels;
    uint32_t size;
    uint8_t brightness;
} _os_led_strip_t;

_os_led_strip_t *_i2s_dma_os_led_strip_init(int bus, int gpio, uint32_t numpixels)
{
    _os_led_strip_t *strip = NULL;

    strip = new _os_led_strip_t;

    if (strip == NULL)
    {
        return NULL;
    }

    // Generate I2S config
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = 0,
        .dma_buf_count = 4,
        .use_apll = false,
    };

    i2s_pin_config_t pin_config = {.bck_io_num = -1,
                                   .ws_io_num = -1,
                                   .data_out_num = gpio,
                                   .data_in_num = -1};

    strip->size = (sizeof(uint8_t) * 12 * numpixels);
    i2s_config.dma_buf_len = 1024;
    // Install the driver
    i2s_driver_install((i2s_port_t)bus, &i2s_config, 0, NULL);

    // Set the GPIOs
    i2s_set_pin((i2s_port_t)bus, &pin_config);

    // Last data to fill out
    strip->bus = bus;
    strip->brightness = 30;
    // Buffer to keep the stuffs in heh
    strip->pixels = (uint8_t *)malloc(strip->size);

    if (strip->pixels == NULL)
    {
        printf("Couldn't allocate DMA buffer for LED Strip");
        free((void *)strip);
        return NULL;
    }

    _i2s_dma_os_led_strip_show(strip);
    return strip;
}

int free_i2s_dma_strip(_os_led_strip_t *strip)
{
    if (strip == NULL)
    {
        return OS_RET_NULL_PTR;
    }

    // Free the neopixel array
    free(strip->pixels);

    // Free the strip
    free(strip);

    return OS_RET_OK;
}

int _i2s_dma_os_led_strip_set(_os_led_strip_t *strip, uint32_t pixel, uint8_t r, uint8_t g, uint8_t b)
{
    if (strip == NULL)
    {
        return OS_RET_NULL_PTR;
    }

    r = (r * strip->brightness) >> 8;
    g = (g * strip->brightness) >> 8;
    b = (b * strip->brightness) >> 8;

    r = (r * strip->brightness) >> 8;
    g = (g * strip->brightness) >> 8;
    b = (b * strip->brightness) >> 8;

    for (int j = 0; j < 4; j++)
        strip->pixels[BYTES_FOR_LED * pixel + j] = get_protocol_eq(g, j);
    for (int j = 0; j < 4; j++)
        strip->pixels[BYTES_FOR_LED * pixel + BYTES_FOR_LED_BYTE + j] = get_protocol_eq(r, j);
    for (int j = 0; j < 4; j++)
        strip->pixels[BYTES_FOR_LED * pixel + BYTES_FOR_LED_BYTE * 2 + j] = get_protocol_eq(b, j);

    return OS_RET_OK;
}

uint8_t off_buff[200] = {0};
int _i2s_dma_os_led_strip_show(_os_led_strip_t *strip)
{

    if (strip == NULL)
    {
        return OS_RET_NULL_PTR;
    }
    size_t bytes_written;
    i2s_write((i2s_port_t)strip->bus, strip->pixels, strip->size, &bytes_written, portMAX_DELAY);
    // Off byte off white :)
    i2s_write((i2s_port_t)strip->bus, (void *)off_buff, sizeof(off_buff), &bytes_written, portMAX_DELAY);
    return OS_RET_OK;
}

int _i2s_dma_os_led_strip_set_brightness(_os_led_strip_t *strip, uint8_t brightness)
{
    strip->brightness = brightness;

    return OS_RET_OK;
}

#endif