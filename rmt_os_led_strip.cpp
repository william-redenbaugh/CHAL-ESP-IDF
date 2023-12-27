#include "driver/rmt_tx.h"
#include "esp_err.h"
#include "esp_check.h"
#include "global_includes.h"
#include "enabled_modules.h"
#ifdef LED_STRIP_RMT

// C external libraries
extern "C"{
#include "rmt_led_strip_encoder.h"
}

#define ZERO_BUFFER_SIZE (48)
#define BYTES_PER_COLOR (4)
#define RGB_BYTES (3)
#define SAMPLE_RATE (93750)

typedef struct _os_led_strip_t
{
    int gpio;
    uint32_t numpixel;
    uint8_t brightness;
    uint8_t *pixels;
    rmt_channel_handle_t led_chan;
    rmt_tx_channel_config_t tx_chan_config;
    rmt_encoder_handle_t led_encoder;
    led_strip_encoder_config_t encoder_config;
    rmt_transmit_config_t tx_config;
} _os_led_strip_t;

#define LED_STRIP_DEBUGGING

#ifdef LED_STRIP_DEBUGGING
#define let_strip_println(e...) \
    print(e);                   \
    print("\n")
#else
#define let_strip_println(e...) void(e)
#endif

#define BITS_PER_LED_CMD 24

#define LED_BUFFER_ITEMS (BITS_PER_LED_CMD)

// These values are determined by measuring pulse timing with logic analyzer and adjusting to match datasheet.
#define T0H 16 // 0 bit high time
#define T1H 34 // 1 bit high time
#define T0L 32 // low time for either bit
#define T1L 18

// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 

_os_led_strip_t *_rmt_os_led_strip_init(int bus, int gpio, uint32_t numpixels)
{
    
    _os_led_strip_t *strip = (_os_led_strip_t *)malloc(sizeof(_os_led_strip_t));

    if (strip == NULL)
    {
        return NULL;
    }

    Serial.printf("Initializing the RMT peripheral!\n");
    // Allocate data for the pixels
    strip->pixels = (uint8_t*)malloc(sizeof(uint8_t) * numpixels * 3);
    // Clear the strips!
    memset(strip->pixels, 0, sizeof(uint8_t) * 3 * numpixels);

    Serial.printf("GPIO Value %d\n", gpio);
    strip->tx_chan_config = {
        .gpio_num = (gpio_num_t)gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };

    ESP_ERROR_CHECK(rmt_new_tx_channel(&strip->tx_chan_config, &strip->led_chan));

    Serial.printf("Install led strip encoder\n");
    strip->led_encoder = NULL;

    strip->encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&strip->encoder_config, &strip->led_encoder));

    Serial.printf("Enable RMT TX channel\n");
    ESP_ERROR_CHECK(rmt_enable(strip->led_chan));
    strip->tx_config = {
        .loop_count = 0, // no transfer loop
    };
    
    return strip;
}

int _rmt_os_led_strip_set(_os_led_strip_t *strip, uint32_t pixel, uint8_t r, uint8_t g, uint8_t b)
{
    if (strip == NULL)
    {
        return OS_RET_NULL_PTR;
    }

    if(strip->numpixel <= pixel){
        return OS_RET_INVALID_PARAM;
    }
    
    r = (r * strip->brightness) >> 8;
    g = (g * strip->brightness) >> 8;
    b = (b * strip->brightness) >> 8;

    int index = pixel * 3;
    strip->pixels[index] = g;
    strip->pixels[index + 1] = b;
    strip->pixels[index + 2] = r;

    return OS_RET_OK;
}

int _rmt_os_led_strip_show(_os_led_strip_t *strip)
{

    // Send out the data through the RMT driver!
    rmt_transmit(strip->led_chan, 
        strip->led_encoder, 
        strip->pixels, 
        strip->numpixel * 3,
        &strip->tx_config);

    //rmt_tx_wait_all_done(strip->led_chan, -1);

    if (strip == NULL)
    {
        return OS_RET_NULL_PTR;
    }

    return OS_RET_OK;
}

int _rmt_dma_os_led_strip_set_brightness(_os_led_strip_t *strip, uint8_t brightness){
    if (strip == NULL)
    {
        return OS_RET_NULL_PTR;
    }
    strip->brightness = brightness;
    return OS_RET_OK;
}

#endif