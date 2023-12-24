#include "global_includes.h"
#include "driver/rmt.h"
#include "esp_err.h"
#include "esp_check.h"

#ifdef LED_STRIP_RMT

#include "driver/rmt.h"
#define ZERO_BUFFER_SIZE (48)
#define BYTES_PER_COLOR (4)
#define RGB_BYTES (3)
#define SAMPLE_RATE (93750)

typedef struct _os_led_strip_t
{
    rmt_channel_t bus;
    rmt_config_t config;
    led_strip_type_t type;
    int gpio;

    uint32_t numpixel;
    size_t out_buffer_size;
    rmt_item32_t *out_buffer;
    uint8_t off_buffer[ZERO_BUFFER_SIZE];
    void *mutex;

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

static const char *TAG = "NeoPixel WS2812 Driver";

static const uint16_t bitpatterns[4] = {0x88, 0x8e, 0xe8, 0xee};

_os_led_strip_t _rmt_os_led_strip_init(int bus, int gpio, uint32_t numpixels)
{
    _os_led_strip_t *strip = (_os_led_strip_t *)malloc(sizeof(_os_led_strip_t));

    if (strip == NULL)
    {
        return NULL;
    }

    i2s_pin_config_t pin_config = {.bck_io_num = -1,
                                   .ws_io_num = -1,
                                   .data_out_num = gpio,
                                   .data_in_num = -1};

    strip->out_buffer_size = BITS_PER_LED_CMD * sizeof(rmt_item32_t) * numpixels;
    strip->out_buffer = (rmt_item32_t *)malloc(BITS_PER_LED_CMD * sizeof(rmt_item32_t) * numpixels);

    if (strip->out_buffer == NULL)
    {
        let_strip_println("Low memory error for allocating dma buffer");
        free(strip);
        return NULL;
    }

    strip->numpixel = numpixels;
    strip->bus = (rmt_channel_t)bus;
    strip->gpio = gpio;
    strip->type = type;

    // Allocate memory and initialize mutex
    strip->mutex = malloc(sizeof(os_mut_t));
    int ret = os_mut_init((os_mut_t *)strip->mutex);
    if (ret != OS_RET_OK)
    {
        // Free up resources
        free(strip->mutex);
        free(strip->out_buffer);
        free(strip);
        return NULL;
    }

    ret = os_mut_entry((os_mut_t *)strip->mutex, -1);
    if (ret != OS_RET_OK)
    {
        let_strip_println("Unable to acquire mutex for rmt led strip");
        // Free up resources
        free(strip->mutex);
        free(strip->out_buffer);
        free(strip);
        return NULL;
    }

    rmt_config_t config = {
        .rmt_mode = RMT_MODE_TX,
        .channel = (rmt_channel_t)bus,
        .gpio_num = (gpio_num_t)gpio,
        .clk_div = 2,
        .mem_block_num = 1,
        .tx_config = {
            .carrier_freq_hz = 38000,
            .carrier_level = RMT_CARRIER_LEVEL_HIGH,
            .idle_level = RMT_IDLE_LEVEL_LOW,
            .carrier_duty_percent = 33,
            .carrier_en = false,
            .loop_en = false,
            .idle_output_en = true,
        }};

    ret = esp_to_os(rmt_config(&config));
    if (ret != OS_RET_OK)
    {
        let_strip_println("Unabled to initialize the RMT module");
        // Free up resources
        free(strip->mutex);
        free(strip->out_buffer);
        free(strip);
        return NULL;
    }

    // esp_to_os(rmt_driver_install(config.channel, 0, 0));
    if (ret != OS_RET_OK)
    {
        let_strip_println("Unable to install the driver for the RMT module");
        // Free up resources
        free(strip->mutex);
        free(strip->out_buffer);
        free(strip);
        return NULL;
    }

    let_strip_println("Initialized LED strip correctly");
    ret = os_mut_exit(strip->mutex);
    if (ret != OS_RET_OK)
    {
        let_strip_println("Unable to free mutex for rmt led strip");
        // Free up resources
        free(strip->mutex);
        free(strip->out_buffer);
        free(strip);
        return NULL;
    }
    return strip;
}

int _rmt_os_led_strip_set(os_led_strip_t *strip, uint32_t pixel, uint8_t r, uint8_t g, uint8_t b)
{
    const uint8_t mask = 1 << (8 - 1);

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

    for (int n = 0; n < 8; n++)
    {
        uint8_t bit_is_set = r & mask;
        strip->out_buffer[pos + n] = bit_is_set ? (rmt_item32_t){{{T1H, 1, T1L, 0}}} : (rmt_item32_t){{{T0H, 1, T0L, 0}}};
    }

    for (int n = 0; n < 8; n++)
    {
        uint8_t bit_is_set = g & mask;
        strip->out_buffer[pos + n + 8] = bit_is_set ? (rmt_item32_t){{{T1H, 1, T1L, 0}}} : (rmt_item32_t){{{T0H, 1, T0L, 0}}};
    }

    for (int n = 0; n < 8; n++)
    {
        uint8_t bit_is_set = b & mask;
        strip->out_buffer[pos + n + 16] = bit_is_set ? (rmt_item32_t){{{T1H, 1, T1L, 0}}} : (rmt_item32_t){{{T0H, 1, T0L, 0}}};
    }

    ret = os_mut_exit((os_mut_t *)strip->mutex);
    return ret;
}

int _rmt_os_led_strip_show(os_led_strip_t *strip)
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
    ret = os_mut_exit((os_mut_t *)strip->mutex);
    if (ret != OS_RET_OK)
    {
        return ret;
    }
    return OS_RET_OK;
}

int _rmt_dma_os_led_strip_set_brightness(os_led_strip_t *strip, uint8_t brightness){

    return OS_RET_Ok;
}
#endif