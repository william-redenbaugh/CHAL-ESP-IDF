
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "global_includes.h"

#define EXAMPLE_ADC_UNIT                    ADC_UNIT_1
#define _EXAMPLE_ADC_UNIT_STR(unit)         #unit
#define EXAMPLE_ADC_UNIT_STR(unit)          _EXAMPLE_ADC_UNIT_STR(unit)
#define EXAMPLE_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_ADC_ATTEN                   ADC_ATTEN_DB_2_5
#define EXAMPLE_ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type1.channel)
#define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type1.data)
#else
#define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type2.channel)
#define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type2.data)
#endif

typedef struct dma_adc_handle_t{
    adc_channel_t channel;
    uint32_t speed;
    uint32_t size;
    adc_continuous_handle_t handle;
}dma_adc_handle_t;

adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {};

dma_adc_handle_t* os_init_dma_adc(int adc_channel, uint32_t buffer_size, uint32_t speed){
    
    dma_adc_handle_t *dma_handle = (dma_adc_handle_t*)malloc(sizeof(dma_adc_handle_t));
    if(dma_handle == NULL){
        return NULL;
    }
   
    dma_handle->size = buffer_size;
    dma_handle->channel = (adc_channel_t)adc_channel;
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = buffer_size,
    };

    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &dma_handle->handle));
    adc_continuous_config_t dig_cfg = {
        .pattern_num = 1,
        .adc_pattern = NULL,
        .sample_freq_hz = speed,
        .conv_mode = EXAMPLE_ADC_CONV_MODE,
        .format = EXAMPLE_ADC_OUTPUT_TYPE,
    };

    dig_cfg.pattern_num = 1;
    for (int i = 0; i < 1; i++) {
        adc_pattern[i].atten = EXAMPLE_ADC_ATTEN;
        adc_pattern[i].channel = dma_handle->channel;
        adc_pattern[i].unit = EXAMPLE_ADC_UNIT;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(dma_handle->handle, &dig_cfg));
    return dma_handle;
}

int os_dma_read(dma_adc_handle_t *handle, uint8_t *buffer, size_t size){
    if(size != handle->size){
        return OS_RET_INVALID_PARAM;
    }
    adc_continuous_start(handle->handle);
    uint32_t out_len;
    esp_err_t err = adc_continuous_read(handle->handle, buffer, size, &out_len, 10000);

    adc_continuous_stop(handle->handle);
    return esp_to_os(err);
}
