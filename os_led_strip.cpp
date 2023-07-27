#include "global_includes.h"
#include "driver/rmt.h"
#include "esp_err.h"
#include "esp_check.h"

#define BITS_PER_LED_CMD	24

#define LED_BUFFER_ITEMS	(BITS_PER_LED_CMD)

// These values are determined by measuring pulse timing with logic analyzer and adjusting to match datasheet. 
#define T0H	16  // 0 bit high time
#define T1H	34  // 1 bit high time
#define T0L	32  // low time for either bit
#define T1L	18

static const char *TAG = "NeoPixel WS2812 Driver";

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

    strip->out_buffer_size = BITS_PER_LED_CMD * sizeof(rmt_item32_t) * numpixels;
    strip->out_buffer =  (rmt_item32_t*)malloc(BITS_PER_LED_CMD *sizeof(rmt_item32_t) * numpixels);
    strip->numpixel = numpixels;
    strip->bus = (rmt_channel_t)bus;
    strip->gpio = gpio;
    strip->type = type;

    // Allocate memory and initialize mutex
    strip->mutex = malloc(sizeof(os_mut_t));
    os_mut_init((os_mut_t *)strip->mutex);
    os_mut_entry((os_mut_t*)strip->mutex, -1);

    rmt_config_t config = {
        .rmt_mode = RMT_MODE_TX,
        .channel = (rmt_channel_t)bus,
        .gpio_num = (gpio_num_t)(gpio),
        .clk_div = 2,
        .mem_block_num = (uint8_t)3,
        .tx_config = {
            .idle_level = (rmt_idle_level_t)0,
            .carrier_en = false, 
            .loop_en = false, 
            .idle_output_en = true, 
        },
    };

    ESP_RETURN_ON_ERROR(rmt_config(&config), TAG, "Failed to configure RMT");
    ESP_RETURN_ON_ERROR(rmt_driver_install(config.channel, 0, 0), TAG, "Failed to install RMT driver");
    
    os_mut_exit((os_mut_t*)strip->mutex);
    return OS_RET_OK;
}

int os_led_strip_set(os_led_strip_t *strip, uint32_t pixel, uint8_t r, uint8_t g, uint8_t b) {
    const  uint8_t mask = 1 << (8 -1);

    if (strip == NULL)
    {
        return OS_RET_NULL_PTR;
    }

    if (pixel >= strip->numpixel)
    {
        return OS_RET_INVALID_PARAM;
    }

    int pos = pixel * BITS_PER_LED_CMD;
    int ret = os_mut_entry_wait_indefinite((os_mut_t *)strip->mutex);
    if (ret != OS_RET_OK)
    {
        return ret;
    }

    for(int n = 0; n < 8; n++){
        uint8_t bit_is_set = r & mask;
        strip->out_buffer[pos + n] = bit_is_set ? 
                        (rmt_item32_t){{{T1H, 1, T1L, 0}}} :
                        (rmt_item32_t){{{T0H, 1, T0L, 0}}};
    }

    for(int n = 0; n < 8; n++){
        uint8_t bit_is_set = g & mask;
        strip->out_buffer[pos + n + 8] = bit_is_set ? 
                        (rmt_item32_t){{{T1H, 1, T1L, 0}}} :
                        (rmt_item32_t){{{T0H, 1, T0L, 0}}};
    }

    for(int n = 0; n < 8; n++){
        uint8_t bit_is_set = b & mask;
        strip->out_buffer[pos + n + 16] = bit_is_set ? 
                        (rmt_item32_t){{{T1H, 1, T1L, 0}}} :
                        (rmt_item32_t){{{T0H, 1, T0L, 0}}};
    }
    
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
    ret = esp_to_os(rmt_write_items(strip->bus, strip->out_buffer, strip->out_buffer_size, false));
    if (ret != OS_RET_OK)
    {
        return ret;
    }
    ret = esp_to_os(rmt_wait_tx_done(strip->bus, portMAX_DELAY));
    if (ret != OS_RET_OK)
    {
        return ret;
    }
    ret = os_mut_exit((os_mut_t*)strip->mutex);
    if (ret != OS_RET_OK)
    {
        return ret;
    }
    return OS_RET_OK;
}