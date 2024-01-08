#include "global_includes.h"
#include "esp_mac.h"
#include "esp_efuse.h"

uint64_t os_fetch_unique_id(void){
    uint64_t value;
    esp_read_mac((uint8_t*)&value, ESP_MAC_BASE);

    return value;
}

void burn_fuse(void){
    esp_efuse_set_key_dis_write(EFUSE_BLK1);
}