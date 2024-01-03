#include "global_includes.h"
#include "nvs_flash.h"
#include "nvs.h"

static char my_handle_str[] = "primary_storage";
static nvs_handle_t my_handle;

int kv_store_init(void){
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    int ret = esp_to_os(err);
    if(ret != OS_RET_OK){
        return ret;
    }

    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    err = nvs_open(my_handle_str, NVS_READWRITE, &my_handle);


    Serial.printf("NVS Ret code %d\n", err);
    ret = esp_to_os(err);
    return ret;
}

int os_kv_store_uninit(void){
    nvs_close(my_handle);
    return OS_RET_OK;
}

int os_kv_put_uint32(char* key, uint32_t value) {
    esp_err_t err = nvs_set_u32(my_handle, key, value);
    return esp_to_os(err);
}

int os_kv_put_uint64(char* key, uint64_t value) {
    esp_err_t err = nvs_set_u64(my_handle, key, value);
    return esp_to_os(err);
}

int os_kv_put_string(char* key, char* value) {
    esp_err_t err = nvs_set_str(my_handle, key, value);
    return esp_to_os(err); 
}

int os_kv_get_uint32(char* key, uint32_t* value) {
    esp_err_t err = nvs_get_u32(my_handle, key, value);
    return esp_to_os(err); 
}

int os_kv_get_uint64(char* key, uint64_t* value) {
    esp_err_t err = nvs_get_u64(my_handle, key, value);
    return esp_to_os(err); 
}

int os_kv_get_string(char* key, char* value, size_t *len) {
    esp_err_t err = nvs_get_str(my_handle, key, value, len);
    return esp_to_os(err); 
}

int os_kv_remove(char* key) {
    esp_err_t err = nvs_erase_key(my_handle, key);
    return esp_to_os(err); 
}

int os_kv_flush_data(void){
    esp_err_t err = nvs_commit(my_handle);
    
    return esp_to_os(err);
}