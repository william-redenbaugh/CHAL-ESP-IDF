// hal_bluetooth_serial.c
#include "global_includes.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_bt.h"
#include "driver/uart.h"
#include "string.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

#define FIFO_MAX_SIZE (4096)

#define BLE_SPP_DEBUG

#ifdef BLE_SPP_DEBUG
#define ble_spp_printf(e, ...) Serial.printf(e)
#else
#define ble_spp_printf(e, ...) (void)e
#endif

#define SPP_DATA_MAX_LEN (512)
#define SPP_CMD_MAX_LEN (20)
#define SPP_STATUS_MAX_LEN (20)
#define ESP_SPP_APP_ID 0x56

#define SPP_DATA_BUFF_MAX_LEN (2 * 1024)

/// Attributes State Machine
enum
{
    SPP_IDX_SVC,

    SPP_IDX_SPP_DATA_RECV_CHAR,
    SPP_IDX_SPP_DATA_RECV_VAL,

    SPP_IDX_SPP_DATA_NOTIFY_CHAR,
    SPP_IDX_SPP_DATA_NTY_VAL,
    SPP_IDX_SPP_DATA_NTF_CFG,

    SPP_IDX_SPP_COMMAND_CHAR,
    SPP_IDX_SPP_COMMAND_VAL,

    SPP_IDX_SPP_STATUS_CHAR,
    SPP_IDX_SPP_STATUS_VAL,
    SPP_IDX_SPP_STATUS_CFG,

#ifdef SUPPORT_HEARTBEAT
    SPP_IDX_SPP_HEARTBEAT_CHAR,
    SPP_IDX_SPP_HEARTBEAT_VAL,
    SPP_IDX_SPP_HEARTBEAT_CFG,
#endif

    SPP_IDX_NB,
};

#define SPP_SERVER_NAME "SPP_SERVER"
#define BT_DEVICE_NAME DEVICE_NAME

#define GATTS_TABLE_TAG "GATTS_SPP_DEMO"

#define SPP_PROFILE_NUM 1
#define SPP_PROFILE_APP_IDX 0
#define ESP_SPP_APP_ID 0x56
#define SAMPLE_DEVICE_NAME "ESP_SPP_SERVER" // The Device Name Characteristics in GAP
#define SPP_SVC_INST_ID 0

#define RX_BUFFER_SIZE 4096

static bool enable_data_ntf = false;
static bool is_connected = false;
static esp_bd_addr_t spp_remote_bda = {
    0x0,
};

static uint16_t spp_handle_table[SPP_IDX_NB];

static int handle = -1;
static byte_array_fifo *spp_fifo;

/// SPP Service
static const uint16_t spp_service_uuid = 0xABF0;
/// Characteristic UUID
#define ESP_GATT_UUID_SPP_DATA_RECEIVE 0xABF1
#define ESP_GATT_UUID_SPP_DATA_NOTIFY 0xABF2
#define ESP_GATT_UUID_SPP_COMMAND_RECEIVE 0xABF3
#define ESP_GATT_UUID_SPP_COMMAND_NOTIFY 0xABF4

#ifdef SUPPORT_HEARTBEAT
#define ESP_GATT_UUID_SPP_HEARTBEAT 0xABF5
#endif

static const uint8_t spp_adv_data[23] = {
    /* Flags */
    0x02, 0x01, 0x06,
    /* Complete List of 16-bit Service Class UUIDs */
    0x03, 0x03, 0xF0, 0xAB,
    /* Complete Local Name in advertising */
    0x0F, 0x09, 'R', 'a', 'v', 'e', 'R', 'a', 'v', 'e', 'L', 'i', 'g', 'h', 't'};

struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static uint8_t find_char_and_desr_index(uint16_t handle)
{
    uint8_t error = 0xff;

    for (int i = 0; i < SPP_IDX_NB; i++)
    {
        if (handle == spp_handle_table[i])
        {
            return i;
        }
    }

    return error;
}

static bool store_wr_buffer(esp_ble_gatts_cb_param_t *p_data)
{

    return true;
}

static void print_write_buffer(void)
{
}

static void free_write_buffer(void)
{
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
    {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_READ;

#ifdef SUPPORT_HEARTBEAT
static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
#endif

static uint16_t spp_mtu_size = 23;
static uint16_t spp_conn_id = 0xffff;
static esp_gatt_if_t spp_gatts_if = 0xff;
QueueHandle_t spp_uart_queue = NULL;
static QueueHandle_t cmd_cmd_queue = NULL;

/// SPP Service - data receive characteristic, read&write without response
static const uint16_t spp_data_receive_uuid = ESP_GATT_UUID_SPP_DATA_RECEIVE;
static const uint8_t spp_data_receive_val[20] = {0x00};

/// SPP Service - data notify characteristic, notify&read
static const uint16_t spp_data_notify_uuid = ESP_GATT_UUID_SPP_DATA_NOTIFY;
static const uint8_t spp_data_notify_val[20] = {0x00};
static const uint8_t spp_data_notify_ccc[2] = {0x00, 0x00};

/// SPP Service - command characteristic, read&write without response
static const uint16_t spp_command_uuid = ESP_GATT_UUID_SPP_COMMAND_RECEIVE;
static const uint8_t spp_command_val[10] = {0x00};

/// SPP Service - status characteristic, notify&read
static const uint16_t spp_status_uuid = ESP_GATT_UUID_SPP_COMMAND_NOTIFY;
static const uint8_t spp_status_val[10] = {0x00};
static const uint8_t spp_status_ccc[2] = {0x00, 0x00};

static esp_ble_adv_params_t spp_adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#ifdef SUPPORT_HEARTBEAT
/// SPP Server - Heart beat characteristic, notify&write&read
static const uint16_t spp_heart_beat_uuid = ESP_GATT_UUID_SPP_HEARTBEAT;
static const uint8_t spp_heart_beat_val[2] = {0x00, 0x00};
static const uint8_t spp_heart_beat_ccc[2] = {0x00, 0x00};
#endif

static void bt_spp_recv_cb(uint8_t *data, size_t len)
{
    ble_spp_printf("Sending data to queue\n");
    int ret = enqueue_bytes_bytearray_fifo(spp_fifo, data, len);

    if(ret != OS_RET_OK){
        ble_spp_printf("Failed to enqueue data into the bytearray for the SPP rx driver");
    }
}

int hal_ble_serial_receive(uint8_t *data, size_t len)
{
    return dequeue_bytes_bytearray_fifo(spp_fifo, data, len);
}

int hal_ble_serial_receive_block(uint8_t *data, size_t len)
{
    int ret = block_until_n_bytes_fifo(spp_fifo, len);

    ble_spp_printf("hmm: %d", ret);
    int n = dequeue_bytes_bytearray_fifo(spp_fifo, data, len);
    
    if(n != len){
        ble_spp_printf("Mismatched data rx sise %d, %d", n, len);
        return OS_RET_INVALID_PARAM;
    }
    return 0;
}

int hal_ble_serial_send(uint8_t *data, size_t len)
{
    // Stub
    return OS_RET_OK;
}

/// Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] =
    {
        // SPP -  Service Declaration
        [SPP_IDX_SVC] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(spp_service_uuid), sizeof(spp_service_uuid), (uint8_t *)&spp_service_uuid}},

        // SPP -  data receive characteristic Declaration
        [SPP_IDX_SPP_DATA_RECV_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        // SPP -  data receive characteristic Value
        [SPP_IDX_SPP_DATA_RECV_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_receive_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, SPP_DATA_MAX_LEN, sizeof(spp_data_receive_val), (uint8_t *)spp_data_receive_val}},

        // SPP -  data notify characteristic Declaration
        [SPP_IDX_SPP_DATA_NOTIFY_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

        // SPP -  data notify characteristic Value
        [SPP_IDX_SPP_DATA_NTY_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_notify_uuid, ESP_GATT_PERM_READ, SPP_DATA_MAX_LEN, sizeof(spp_data_notify_val), (uint8_t *)spp_data_notify_val}},

        // SPP -  data notify characteristic - Client Characteristic Configuration Descriptor
        [SPP_IDX_SPP_DATA_NTF_CFG] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc}},

        // SPP -  command characteristic Declaration
        [SPP_IDX_SPP_COMMAND_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        // SPP -  command characteristic Value
        [SPP_IDX_SPP_COMMAND_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_command_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, SPP_CMD_MAX_LEN, sizeof(spp_command_val), (uint8_t *)spp_command_val}},

        // SPP -  status characteristic Declaration
        [SPP_IDX_SPP_STATUS_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

        // SPP -  status characteristic Value
        [SPP_IDX_SPP_STATUS_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_status_uuid, ESP_GATT_PERM_READ, SPP_STATUS_MAX_LEN, sizeof(spp_status_val), (uint8_t *)spp_status_val}},

        // SPP -  status characteristic - Client Characteristic Configuration Descriptor
        [SPP_IDX_SPP_STATUS_CFG] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(spp_status_ccc), (uint8_t *)spp_status_ccc}},

#ifdef SUPPORT_HEARTBEAT
        // SPP -  Heart beat characteristic Declaration
        [SPP_IDX_SPP_HEARTBEAT_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

        // SPP -  Heart beat characteristic Value
        [SPP_IDX_SPP_HEARTBEAT_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_heart_beat_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(spp_heart_beat_val), sizeof(spp_heart_beat_val), (uint8_t *)spp_heart_beat_val}},

        // SPP -  Heart beat characteristic - Client Characteristic Configuration Descriptor
        [SPP_IDX_SPP_HEARTBEAT_CFG] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(spp_data_notify_ccc), (uint8_t *)spp_heart_beat_ccc}},
#endif
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *)param;
    uint8_t res = 0xff;

    ESP_LOGI(GATTS_TABLE_TAG, "event = %x", event);
    Serial.printf("event = %x\n", event);
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "%s %d", __func__, __LINE__);
        esp_ble_gap_set_device_name(DEVICE_NAME);

        ESP_LOGI(GATTS_TABLE_TAG, "%s %d", __func__, __LINE__);
        esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data, sizeof(spp_adv_data));

        ESP_LOGI(GATTS_TABLE_TAG, "%s %d", __func__, __LINE__);
        esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
        break;
    case ESP_GATTS_READ_EVT:
        ble_spp_printf("Received data\n");
        res = find_char_and_desr_index(p_data->read.handle);
        if (res == SPP_IDX_SPP_STATUS_VAL)
        {
            // TODO:client read the status characteristic
        }
        break;
    case ESP_GATTS_WRITE_EVT:
    {
        ble_spp_printf("Data written!");

        res = find_char_and_desr_index(p_data->write.handle);
        if (p_data->write.is_prep == false)
        {
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT : handle = %d", res);
            if (res == SPP_IDX_SPP_COMMAND_VAL)
            {
                uint8_t *spp_cmd_buff = NULL;
                spp_cmd_buff = (uint8_t *)malloc((spp_mtu_size - 3) * sizeof(uint8_t));
                if (spp_cmd_buff == NULL)
                {
                    ESP_LOGE(GATTS_TABLE_TAG, "%s malloc failed", __func__);
                    break;
                }
                memset(spp_cmd_buff, 0x0, (spp_mtu_size - 3));
                memcpy(spp_cmd_buff, p_data->write.value, p_data->write.len);
                xQueueSend(cmd_cmd_queue, &spp_cmd_buff, 10 / portTICK_PERIOD_MS);
            }
            else if (res == SPP_IDX_SPP_DATA_NTF_CFG)
            {
                if ((p_data->write.len == 2) && (p_data->write.value[0] == 0x01) && (p_data->write.value[1] == 0x00))
                {
                    enable_data_ntf = true;
                }
                else if ((p_data->write.len == 2) && (p_data->write.value[0] == 0x00) && (p_data->write.value[1] == 0x00))
                {
                    enable_data_ntf = false;
                }
            }
#ifdef SUPPORT_HEARTBEAT
            else if (res == SPP_IDX_SPP_HEARTBEAT_CFG)
            {
                if ((p_data->write.len == 2) && (p_data->write.value[0] == 0x01) && (p_data->write.value[1] == 0x00))
                {
                    enable_heart_ntf = true;
                }
                else if ((p_data->write.len == 2) && (p_data->write.value[0] == 0x00) && (p_data->write.value[1] == 0x00))
                {
                    enable_heart_ntf = false;
                }
            }
            else if (res == SPP_IDX_SPP_HEARTBEAT_VAL)
            {
                if ((p_data->write.len == sizeof(heartbeat_s)) && (memcmp(heartbeat_s, p_data->write.value, sizeof(heartbeat_s)) == 0))
                {
                    heartbeat_count_num = 0;
                }
            }
#endif
            else if (res == SPP_IDX_SPP_DATA_RECV_VAL)
            {
                bt_spp_recv_cb((p_data->write.value), (p_data->write.len));
            }
            else
            {
                // TODO:
            }
        }
        else if ((p_data->write.is_prep == true) && (res == SPP_IDX_SPP_DATA_RECV_VAL))
        {
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_PREP_WRITE_EVT : handle = %d", res);
            store_wr_buffer(p_data);
        }
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
    {
        ble_spp_printf("Data incoming!");
        if (p_data->exec_write.exec_write_flag)
        {
            print_write_buffer();
            free_write_buffer();
        }
        break;
    }
    case ESP_GATTS_MTU_EVT:
        spp_mtu_size = p_data->mtu.mtu;
        break;
    case ESP_GATTS_CONF_EVT:
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT:
        spp_conn_id = p_data->connect.conn_id;
        spp_gatts_if = gatts_if;
        is_connected = true;
        memcpy(&spp_remote_bda, &p_data->connect.remote_bda, sizeof(esp_bd_addr_t));

        ble_spp_printf("Connected to a device!\n");
#ifdef SUPPORT_HEARTBEAT
        uint16_t cmd = 0;
        xQueueSend(cmd_heartbeat_queue, &cmd, 10 / portTICK_PERIOD_MS);
#endif
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        is_connected = false;
        enable_data_ntf = false;
#ifdef SUPPORT_HEARTBEAT
        enable_heart_ntf = false;
        heartbeat_count_num = 0;
#endif
        esp_ble_gap_start_advertising(&spp_adv_params);
        ble_spp_printf("Disconnected from device\n");
        break;
    case ESP_GATTS_OPEN_EVT:
        break;
    case ESP_GATTS_CANCEL_OPEN_EVT:
        break;
    case ESP_GATTS_CLOSE_EVT:
        break;
    case ESP_GATTS_LISTEN_EVT:
        break;
    case ESP_GATTS_CONGEST_EVT:
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    {
        ESP_LOGI(GATTS_TABLE_TAG, "The number handle =%x", param->add_attr_tab.num_handle);
        if (param->add_attr_tab.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        }
        else if (param->add_attr_tab.num_handle != SPP_IDX_NB)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, SPP_IDX_NB);
        }
        else
        {
            memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
            esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
        }
        break;
    }
    default:
        break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;
    print("GAP_EVT, event %d", event);
    Serial.printf("GAP_EVT, event %d", event);
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&spp_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        // advertising start complete event to indicate advertising start successfully or failed
        if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            print("Advertising start failed: %s", esp_err_to_name(err));
        }
        break;
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "EVT %d, gatts if %d", event, gatts_if);

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
            return;
        }
    }

    do
    {
        int idx;
        for (idx = 0; idx < SPP_PROFILE_NUM; idx++)
        {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gatts_if == spp_profile_tab[idx].gatts_if)
            {
                if (spp_profile_tab[idx].gatts_cb)
                {
                    spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

hal_bt_serial_err_t hal_ble_serial_init(void)
{

    // Generate a fifo to store all the date into
    spp_fifo = create_byte_array_fifo(FIFO_MAX_SIZE);

    if(spp_fifo == nullptr){
        return HAL_BT_SERIAL_BUFFER_OVERFLOW;
    }
    
    // Release all existing ble profiles
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_err_t ret;

    // Initialize the bluetooth controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK)
    {
        print("%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return HAL_BT_SERIAL_ERROR;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BLE)) != ESP_OK)
    {
        print("%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return HAL_BT_SERIAL_ERROR;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK)
    {
        print("%s initialize bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return HAL_BT_SERIAL_ERROR;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK)
    {
        print("%s enable bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return HAL_BT_SERIAL_ERROR;
    }

    if ((ret = esp_ble_gap_register_callback(gap_event_handler)) != ESP_OK)
    {
        print("%s gap register failed: %s", __func__, esp_err_to_name(ret));
        return HAL_BT_SERIAL_ERROR;
    }

    if ((ret = esp_ble_gatts_register_callback(gatts_event_handler)) != ESP_OK)
    {
        print("%s gap register failed: %s", __func__, esp_err_to_name(ret));
        return HAL_BT_SERIAL_ERROR;
    }

    esp_ble_gatts_app_register(ESP_SPP_APP_ID);
    
    return HAL_BT_SERIAL_OK;
}