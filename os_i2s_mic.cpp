#include "platform_cshal.h"
#include "global_includes.h"

#define TOTAL_SAMPLES 256
#define BYTES_PER_SAMPLE 4
#define MAX_DMA_BUFFER_SIZE TOTAL_SAMPLES *BYTES_PER_SAMPLE

static inline i2s_bits_per_sample_t map_bits_sample(os_i2s_sample_bits_t bits)
{
    i2s_bits_per_sample_t bits_sample;

    switch (bits)
    {
    case I2S_SAMPLE_32BITS:
        bits_sample = I2S_BITS_PER_SAMPLE_32BIT;
        break;
    case I2S_SAMPLE_8BITS:
        bits_sample = I2S_BITS_PER_SAMPLE_8BIT;
        break;
    case I2S_SAMPLE_16BITS:
        bits_sample = I2S_BITS_PER_SAMPLE_16BIT;
        break;
    default:
        bits_sample = I2S_BITS_PER_SAMPLE_32BIT;
        break;
    }

    return bits_sample;
}

static inline i2s_channel_fmt_t map_channels(os_i2s_channels_t chan)
{
    // Configure channels
    i2s_channel_fmt_t chn_fmt;

    switch (chan)
    {
    case I2S_CHANNEL_LEFT:
        chn_fmt = I2S_CHANNEL_FMT_ONLY_LEFT;
        break;

    case I2S_CHANNEL_RIGHT:
        chn_fmt = I2S_CHANNEL_FMT_ONLY_RIGHT;
        break;
#if SOC_I2S_SUPPORTS_TDM
    case I2S_CHANNEL_BOTH:
        chn_fmt = I2S_CHANNEL_FMT_MULTIPLE;
        break;
#endif
    default:
        chn_fmt = I2S_CHANNEL_FMT_ONLY_LEFT;
        break;
    }

    return chn_fmt;
}

int i2s_host_init(os_i2s_host_t *host, int bus, os_i2s_pinmap_t pinmap, os_i2s_channels_t chan, os_i2s_sample_bits_t bits, uint32_t sample_rate)
{

    // Configure I2S peripheral
    i2s_bits_per_sample_t bits_sample = map_bits_sample(bits);

    // Configure channels
    i2s_channel_fmt_t chn_fmt = map_channels(chan);

    // This generates a compiler warning... don't change it
    i2s_config_t i2s_config = i2s_config_t();
    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    i2s_config.sample_rate = sample_rate;
    i2s_config.bits_per_sample = bits_sample;
    i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    i2s_config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    i2s_config.dma_desc_num = BYTES_PER_SAMPLE;
    i2s_config.dma_frame_num = MAX_DMA_BUFFER_SIZE;
    i2s_config.use_apll = false;
    i2s_config.tx_desc_auto_clear = false;
    i2s_config.fixed_mclk = 0;
    i2s_config.channel_format = chn_fmt;

    // I REPEAT, DON'T REMOVE .communication_format = I2S_COMM_FORMAT_I2S
    host->cfg = i2s_config;
    host->port = (i2s_port_t)bus;
    host->chip_select_gpio = pinmap.serial_data_cs;
    host->pin_config.bck_io_num = pinmap.serial_clk;
    host->pin_config.ws_io_num = pinmap.left_right_clk;
    host->pin_config.data_out_num = I2S_PIN_NO_CHANGE;
    host->pin_config.data_in_num = pinmap.serial_data;

    esp_err_t err = i2s_driver_install(host->port, &host->cfg, 0, NULL);
    if (err != ESP_OK)
    {
        return esp_to_os(err);
    }
    err = i2s_set_pin(host->port, &host->pin_config);
    if (err != ESP_OK)
    {
        return esp_to_os(err);
    }
    pinMode(host->chip_select_gpio, OUTPUT);
    digitalWrite(host->chip_select_gpio, LOW);

    return OS_RET_OK;
}

int i2s_host_read(os_i2s_host_t *host, void *ptr, size_t *len)
{
    size_t bytes_read = 0;
    esp_err_t err = i2s_read(host->port, ptr, *len, &bytes_read, portMAX_DELAY);
    *len = bytes_read;
    return esp_to_os(err);
}