#include "chal_struct_definitions.h"
#include "global_includes.h"

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
    default:
        return OS_RET_INT_ERR;
    }
}

int os_assert(int ret)
{
}