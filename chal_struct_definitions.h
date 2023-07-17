#ifndef _IDF_CHAL_STRUCT_DEFINITIONS_H
#define _IDF_CHAL_STRUCT_DEFINITIONS_H

#include <driver/i2s.h>

typedef struct os_i2s{
    i2s_pin_config_t pin_config;
    int chip_select_gpio;
    i2s_port_t port;
}os_i2s_host_t;

#endif