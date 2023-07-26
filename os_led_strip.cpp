#include "global_includes.h"
#include <driver/i2s.h>

i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = (i2s_bits_per_sample_t)16,
    .channel_format = (i2s_channel_fmt_t)I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = 0,
    .dma_buf_count = 4,
    .use_apll = false,
};

static const uint16_t bitpatterns[4] = {0x88, 0x8e, 0xe8, 0xee};

int os_led_strip_init(os_led_strip_t *strip, led_strip_type_t type, int bus, int gpio, uint32_t numpixels)
{

    if (strip == NULL)
    {
        return OS_RET_NULL_PTR;
    }

    i2s_pin_config_t pin_config = {.bck_io_num = -1,
                                   .ws_io_num = -1,
                                   .data_out_num = gpio,
                                   .data_in_num = -1};

    strip->out_buffer_size = BYTES_PER_COLOR * RGB_BYTES * numpixels;
    strip->out_buffer = (uint8_t *)malloc(sizeof(uint8_t) * strip->out_buffer_size);
    i2s_config.dma_buf_len = strip->out_buffer_size;
    strip->numpixel = numpixels;

    // Allocate memory and initialize mutex
    strip->mutex = malloc(sizeof(os_mut_t));
    os_mut_init((os_mut_t *)strip->mutex);

    esp_err_t err = i2s_driver_install((i2s_port_t)bus, &i2s_config, 0, NULL);
    if (err != ESP_OK)
    {
        return esp_to_os(err);
    }

    err = i2s_set_pin((i2s_port_t)bus, &pin_config);
    if (err != ESP_OK)
    {
        return esp_to_os(err);
    }

    return OS_RET_OK;
}

int os_led_strip_set(os_led_strip_t *strip, uint32_t pixel, uint8_t r, uint8_t g, uint8_t b)
{

    if (strip == NULL)
    {
        return OS_RET_NULL_PTR;
    }

    if (pixel >= strip->numpixel)
    {
        return OS_RET_INVALID_PARAM;
    }

    uint32_t index = pixel * BYTES_PER_COLOR * RGB_BYTES;

    int ret = os_mut_entry_wait_indefinite((os_mut_t *)strip->mutex);
    if (ret != OS_RET_OK)
    {
        return ret;
    }
    // Green
    strip->out_buffer[index] = bitpatterns[g >> 6 & 0x03];
    strip->out_buffer[index + 1] = bitpatterns[g >> 4 & 0x03];
    strip->out_buffer[index + 2] = bitpatterns[g >> 2 & 0x03];
    strip->out_buffer[index + 3] = bitpatterns[g & 0x03];

    // Red bitwise manipulation
    strip->out_buffer[index + 4] = bitpatterns[r >> 6 & 0x03];
    strip->out_buffer[index + 5] = bitpatterns[r >> 4 & 0x03];
    strip->out_buffer[index + 6] = bitpatterns[r >> 2 & 0x03];
    strip->out_buffer[index + 7] = bitpatterns[r & 0x03];

    // Blue bitwise manipulation
    strip->out_buffer[index + 8] = bitpatterns[b >> 6 & 0x03];
    strip->out_buffer[index + 9] = bitpatterns[b >> 4 & 0x03];
    strip->out_buffer[index + 10] = bitpatterns[b >> 2 & 0x03];
    strip->out_buffer[index + 11] = bitpatterns[b & 0x03];
    ret = os_mut_exit((os_mut_t *)strip->mutex);
    return ret;
}

int os_led_strip_show(os_led_strip_t *strip)
{

    if (strip == NULL)
    {
        return OS_RET_NULL_PTR;
    }

    size_t bytes_written;

    int ret = os_mut_entry_wait_indefinite((os_mut_t *)strip->mutex);
    if (ret != OS_RET_OK)
    {
        return ret;
    }
    esp_err_t err = i2s_write((i2s_port_t)strip->bus, strip->out_buffer, strip->out_buffer_size, &bytes_written, portMAX_DELAY);
    if (err != ESP_OK)
    {
        return esp_to_os(err);
    }

    err = i2s_write((i2s_port_t)strip->bus, strip->off_buffer, sizeof(strip->off_buffer), &bytes_written, portMAX_DELAY);
    if (err != ESP_OK)
    {
        return esp_to_os(err);
    }
    os_thread_sleep_ms(10);

    err = i2s_zero_dma_buffer((i2s_port_t)strip->bus);
    if (err != ESP_OK)
    {
        return esp_to_os(err);
    }
    return OS_RET_OK;
}