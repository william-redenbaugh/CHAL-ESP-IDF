#include "global_includes.h"
#include "driver/ledc.h"

typedef struct led_timer_inited{
    ledc_timer_t timer;
    bool inited;
}led_timer_inited_t;

typedef struct led_chan{
    ledc_channel_t channel;
    bool inited;
    int gpio;
    led_timer_inited_t *timer;
}led_chan_t;

led_timer_inited_t led_timers[4] = {
    {
        .timer = LEDC_TIMER_0, 
        .inited = false
    },
    {
        .timer = LEDC_TIMER_1, 
        .inited = false
    },
    {
        .timer = LEDC_TIMER_2, 
        .inited = false
    },
    {
        .timer = LEDC_TIMER_3, 
        .inited = false
    }
};

led_chan_t channels[6] = {
    {
        .channel = LEDC_CHANNEL_0, 
        .inited = false,
        .gpio = -1,
    },
    {
        .channel = LEDC_CHANNEL_1, 
        .inited = false,
        .gpio = -1,
    },
    {
        .channel = LEDC_CHANNEL_2, 
        .inited = false,
        .gpio = -1,
    },
    {
        .channel = LEDC_CHANNEL_3, 
        .inited = false,
        .gpio = -1,
    },
    {
        .channel = LEDC_CHANNEL_4, 
        .inited = false,
        .gpio = -1,
    },
    {
        .channel = LEDC_CHANNEL_5, 
        .inited = false,
        .gpio = -1,
    },
};

static int get_free_led_chan(void){
    for(int n = 0; n < 6; n++){
        if(channels[n].inited == false){
            channels[n].inited = true;
            return channels[n].channel;
        }
    }
    // Failed to get good channel
    return -1;
}

static int get_free_led_timer(void){
    for(int n = 0; n < 4; n++){
        if(led_timers[n].inited == false){
            led_timers[n].inited = true;
            return led_timers[n].timer;
        }
    }
    // Failed to get good channel
    return -1;
}

int os_pwm_gpio_init(int gpio, int freq, int resolution){
    if(resolution < 1 || resolution <= LEDC_TIMER_BIT_MAX){
        return OS_RET_INT_ERR;
    }

    int chan = get_free_led_chan();
    if(chan == -1){
        os_println("Failed to get free PWM cycle channel, cannot init pwm pin %d", gpio);
        return OS_RET_INT_ERR;
    }

    int timer = get_free_led_timer();

    if(timer == -1){
        // uninit this
        channels[chan].inited = false;
        return OS_RET_INT_ERR;
    }

    channels[chan].timer = &led_timers[timer];

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = (ledc_timer_bit_t)resolution,
        .timer_num        = (ledc_timer_t)timer,
        .freq_hz          = freq,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .gpio_num       = gpio,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = (ledc_channel_t)chan,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = (ledc_timer_t)timer,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    channels[chan].gpio = gpio;

    return OS_RET_OK;
}

int os_pwm_gpio_set(int gpio, int value){
    for(int n = 0; n < 6; n++){
        if(channels[n].gpio == gpio){
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channels[n].channel, value));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,  channels[n].channel));
            return OS_RET_OK;
        }
    }

    return OS_RET_INT_ERR;
}

int os_pwm_gpio_uninit(int gpio){

    for(int n = 0; n < 6; n++){
        if(channels[n].gpio == gpio){
            if( channels[n].inited){
                channels[n].inited = false;
                channels[n].timer->inited = false;
                return OS_RET_OK;
            }
        }
    }
    return OS_RET_OK;
}