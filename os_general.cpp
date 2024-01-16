#include "chal_struct_definitions.h"
#include "global_includes.h"
#include "nvs.h"

int esp_to_os(esp_err_t err)
{
    switch (err)
    {
    case ESP_OK:
        return OS_RET_OK;
    case ESP_FAIL:
        return OS_RET_INT_ERR;
    case ESP_ERR_NO_MEM:
        return OS_RET_LOW_MEM_ERROR;
        break;
    case ESP_ERR_INVALID_SIZE:
        return OS_RET_INVALID_PARAM;
        break;
    case ESP_ERR_INVALID_ARG:
        return OS_RET_INVALID_PARAM;
        break;
    case ESP_ERR_INVALID_STATE:
        return OS_RET_INT_ERR;
        break;
    case ESP_ERR_NOT_FOUND:
        return OS_RET_NOT_OWNED;
        break;
    case ESP_ERR_NOT_SUPPORTED:
        return OS_RET_UNSUPPORTED_FEATURES;
        break;
    case ESP_ERR_TIMEOUT:
        return OS_RET_TIMEOUT;
        break;
    case ESP_ERR_INVALID_RESPONSE:
        return OS_RET_INT_ERR;
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        return OS_RET_NO_FLASH_KEY;

    default:
        os_printf("ESP Error: %d\n", err);
        return OS_RET_INT_ERR;
    }
}

void os_assert(int ret)
{
    assert(ret);
}