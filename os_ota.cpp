#include "global_includes.h"
#include "esp_ota_ops.h"

static bool active_ota = false;
static esp_partition_t *current_partition = NULL;
static esp_ota_handle_t ota_handle;

int os_ota_begin(void)
{
    current_partition = NULL;
    current_partition = (esp_partition_t *)esp_ota_get_next_update_partition(NULL);
    if (current_partition == NULL)
    {
        return OS_RET_INT_ERR;
    }

    esp_err_t err = esp_ota_begin(current_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    ESP_ERROR_CHECK(err);

    if (err != ESP_OK)
    {
        return esp_to_os(err);
    }

    active_ota = true;

    return OS_RET_OK;
}

int os_ota_write(uint8_t *data, int size)
{
    if (active_ota == false)
    {
        Serial.printf("OTA is not active!\n");
        return OS_RET_NOT_INITIALIZED;
    }
    esp_err_t err = esp_ota_write(ota_handle, data, size);

    return esp_to_os(err);
}

int os_ota_end(void)
{
    if (active_ota == false)
        return OS_RET_NOT_INITIALIZED;

    esp_err_t err = esp_ota_end(ota_handle);

    if (err != ESP_OK)
    {
        return esp_to_os(err);
    }

    err = esp_ota_set_boot_partition(current_partition);
    active_ota = false;

    return esp_to_os(err);
}

int os_ota_halt_mark_invalid(void)
{
    if (active_ota == false)
        return OS_RET_NOT_INITIALIZED;

    esp_err_t err = esp_ota_abort(ota_handle);

    active_ota = false;

    // No need to set next boot partition if we failed to get the OTA
    return esp_to_os(err);
}

void os_ota_mark_functional_boot(void)
{
    esp_ota_mark_app_valid_cancel_rollback();
}