#include "platform_cshal.h"
#include "global_includes.h"

int i2s_host_init(os_i2s_host_t *host, int bus, os_i2s_channels_t chan, os_i2s_sample_bits_t bits, uint32_t sample_rate) {
    
    // Configure I2S peripheral
    i2s_bits_per_sample_t bits_sample;
    switch(bits){
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

    i2s_channel_fmt_t chn_fmt;

    switch (chan){
        case I2S_CHANNEL_LEFT:
            chn_fmt = I2S_CHANNEL_FMT_ONLY_LEFT;
        break;

        case I2S_CHANNEL_RIGHT:
            chn_fmt = I2S_CHANNEL_FMT_ONLY_RIGHT;
        break;

        case I2S_CHANNEL_BOTH:
            chn_fmt = I2S_CHANNEL_FMT_MULTIPLE;
        break;

        default: 
            chn_fmt = I2S_CHANNEL_FMT_ONLY_LEFT;
        break;
    }

    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = sample_rate,
        .bits_per_sample = bits_sample,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 1024,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_driver_install(host->port, &i2s_config, 0, NULL);
    i2s_set_pin(host->port, &host->pin_config);

    return OS_RET_OK;
}

int i2s_host_read(os_i2s_host_t *host, void *ptr, size_t *len) {
    size_t bytes_read = 0;
    i2s_read(host->port, ptr, *len, &bytes_read, portMAX_DELAY);
    *len = bytes_read;
    return OS_RET_OK;
}