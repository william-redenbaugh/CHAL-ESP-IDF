// hal_bluetooth_serial.c
#include "global_includes.h"

#ifdef BLE_NIMBLE_OS
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "string.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "driver/uart.h"
#include "host/ble_uuid.h"

#define FIFO_MAX_SIZE (4096)
// #define BLE_SPP_DEBUG

#ifdef BLE_SPP_DEBUG
#define ble_spp_printf(...) os_printf(__VA_ARGS__)
#else
#define ble_spp_printf(...) ((void)0)
#endif

#define SPP_DATA_MAX_LEN (512)
static int ble_spp_server_gap_event(struct ble_gap_event *event, void *arg);
static int ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int ble_svc_gatt_recv_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

static bool conn_handle_subs[CONFIG_BT_NIMBLE_MAX_CONNECTIONS + 1];
static uint8_t own_addr_type;
int gatt_svr_register(void);
static uint16_t ble_spp_svc_gatt_read_val_handle;
static uint16_t ble_spp_svc_gatt_read_val_handle_notify;
uint16_t attribute_handle[CONFIG_BT_NIMBLE_MAX_CONNECTIONS + 1];

static byte_array_fifo *spp_in_fifo;
static byte_array_fifo *spp_out_fifo;
int spp_mtu_size = 512;
static ble_connected_cb_t bluetooth_pair_cb = NULL;
static ble_disconnected_cb_t bluetooth_disc_cb = NULL;
static char adv_name[13] = "LightsLights";

/* 16 Bit SPP Service UUID */
#define BLE_SVC_SPP_UUID16 0xABF0

/* 16 Bit SPP Service Characteristic UUID */
#define BLE_SVC_SPP_CHR_UUID16 0xABF1

/* 16 Bit BLE data notify*/
#define BLE_SVC_NOTIFY_SPP_CHR_UUID16 0xABF2



ble_uuid16_t svc_spp_uuid16[] = {
    {
        .u = {
            .type = BLE_UUID_TYPE_16,
        },
        .value = (BLE_SVC_SPP_UUID16),
    },
};

ble_uuid16_t chr_spp_uuid16[] = {
    {
        .u = {
            .type = BLE_UUID_TYPE_16,
        },
        .value = (BLE_SVC_SPP_CHR_UUID16),
    },
};

ble_uuid16_t chr_spp_notify_uuid16[] = {
    {
        .u = {
            .type = BLE_UUID_TYPE_16,
        },
        .value = (BLE_SVC_NOTIFY_SPP_CHR_UUID16),
    },
};


struct ble_gatt_chr_def characteristics_spp[] =
    {{
         /* Support SPP service */
         .uuid = (ble_uuid_t *)&chr_spp_uuid16,
         .access_cb = ble_svc_gatt_recv_handler,
         .arg = NULL,
         .descriptors = NULL,
         .flags = BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_PROP_WRITE | BLE_GATT_CHR_PROP_WRITE_NO_RSP | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_PROP_NOTIFY,
         .min_key_size = 0,
         .val_handle = &ble_spp_svc_gatt_read_val_handle,
     },
     {
         /* Support SPP service */
         .uuid = (ble_uuid_t *)&chr_spp_notify_uuid16,
         .access_cb = ble_svc_gatt_handler,
         .arg = NULL,
         .descriptors = NULL,
         .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_PROP_NOTIFY | BLE_GATT_CHR_F_NOTIFY,
         .min_key_size = 0,
         .val_handle = &ble_spp_svc_gatt_read_val_handle_notify,
     },
        {
         /* Support SPP service */
         .uuid = (ble_uuid_t *)NULL,
         .access_cb = NULL,
         .arg = NULL,
         .descriptors = NULL,
         .flags = 0,
         .min_key_size = 0,
         .val_handle = NULL,
        }, /* No more characteristics */
     };

int hal_ble_serial_receive(uint8_t *data, size_t len)
{
    return dequeue_bytes_bytearray_fifo(spp_in_fifo, data, len);
}

int hal_ble_serial_receive_block(uint8_t *data, size_t len)
{
    int ret = block_until_n_bytes_fifo(spp_in_fifo, len);

    if (ret != OS_RET_OK)
    {
        return ret;
    }
    // os_printf("\nhmm: %d\n", ret);
    int n = dequeue_bytes_bytearray_fifo(spp_in_fifo, data, len);

    if (n != len)
    {
        ble_spp_printf("\nMismatched data rx sise %d, %d\n", n, len);
        hal_ble_flush_serial();
        return OS_RET_INVALID_PARAM;
    }
    return 0;
}

int hal_ble_serial_receive_block_timeout(uint8_t *data, size_t len, uint32_t timeout_ms)
{

    int ret = block_until_n_bytes_fifo_timeout(spp_in_fifo, len, timeout_ms);
    if (ret != OS_RET_OK)
    {
        return ret;
    }

    // os_printf("\nhmm: %d\n", ret);
    int n = dequeue_bytes_bytearray_fifo(spp_in_fifo, data, len);

    if (n != len)
    {
        ble_spp_printf("\nMismatched data rx sise %d, %d\n", n, len);
        hal_ble_flush_serial();
        return OS_RET_INVALID_PARAM;
    }
    return 0;
}

int hal_ble_flush_serial(void)
{
    fifo_flush(spp_in_fifo);
    return 0;
}

void ble_store_config_init(void);

/**
 * Logs information about a connection to the console.
 */
static void
ble_spp_server_print_conn_desc(struct ble_gap_conn_desc *desc)
{
    os_printf("handle=%d our_ota_addr_type=%d our_ota_addr=",
                  (int)desc->conn_handle, (int)desc->our_ota_addr.type);
    os_printf("%d\n", (int)desc->our_ota_addr.val);
    os_printf(" our_id_addr_type=%d our_id_addr=",
                  (int)desc->our_id_addr.type);
    os_printf("%d\n", (int)desc->our_id_addr.val);
    os_printf(" peer_ota_addr_type=%d peer_ota_addr=",
                  (int)desc->peer_ota_addr.type);
    os_printf("%d\n", (int)desc->peer_ota_addr.val);
    os_printf(" peer_id_addr_type=%d peer_id_addr=",
                  (int)desc->peer_id_addr.type);
    os_printf("%d\n", (int)desc->peer_id_addr.val);
    os_printf(" conn_itvl=%d conn_latency=%d supervision_timeout=%d "
                  "encrypted=%d authenticated=%d bonded=%d\n",
                  (int)desc->conn_itvl, desc->conn_latency,
                  (int)desc->supervision_timeout,
                  (int)desc->sec_state.encrypted,
                  (int)desc->sec_state.authenticated,
                  (int)desc->sec_state.bonded);
}

/**
 * Enables advertising with the following parameters:
 *     o General discoverable mode.
 *     o Undirected connectable mode.
 */
static void
ble_spp_server_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name;
    int rc;

    /**
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info).
     *     o Advertising tx power.
     *     o Device name.
     *     o 16-bit service UUIDs (alert notifications).
     */

    memset(&fields, 0, sizeof fields);
    memset(&adv_params, 0, sizeof adv_params);


    /* Advertise two flags:
     *     o Discoverability in forthcoming advertisement (general)
     *     o BLE-only (BR/EDR unsupported).
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /* Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    fields.uuids16 = svc_spp_uuid16;

    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0)
    {
        os_printf("error setting advertisement data; rc=%d\n", rc);
        return;
    }

    /* Begin advertising. */
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_spp_server_gap_event, NULL);
    if (rc != 0)
    {
        os_printf("error enabling advertisement; rc=%d\n", rc);
        return;
    }
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that forms.
 * ble_spp_server uses the same callback for all connections.
 *
 * @param event                 The type of event being signalled.
 * @param ctxt                  Various information pertaining to the event.
 * @param arg                   Application-specified argument; unused by
 *                                  ble_spp_server.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int
ble_spp_server_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        os_printf("connection %s; status=%d ",
                      event->connect.status == 0 ? "established" : "failed",
                      event->connect.status);
        if (event->connect.status == 0)
        {
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
            ble_spp_server_print_conn_desc(&desc);
        }
        os_printf("\n");
        if (event->connect.status != 0 || CONFIG_BT_NIMBLE_MAX_CONNECTIONS > 1)
        {
            /* Connection failed or if multiple connection allowed; resume advertising. */
            ble_spp_server_advertise();
        }
        if(bluetooth_pair_cb){
            bluetooth_pair_cb(0);
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        os_printf("disconnect; reason=%x ", event->disconnect.reason);
        ble_spp_server_print_conn_desc(&event->disconnect.conn);
        os_printf("\n");
        if(bluetooth_disc_cb){
            bluetooth_disc_cb(0);
        }
        conn_handle_subs[event->disconnect.conn.conn_handle] = false;

        /* Connection terminated; resume advertising. */
        ble_spp_server_advertise();
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        os_printf("connection updated; status=%d ",
                      event->conn_update.status);
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        assert(rc == 0);
        ble_spp_server_print_conn_desc(&desc);
        os_printf("\n");
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        os_printf("advertise complete; reason=%d",
                      event->adv_complete.reason);
        ble_spp_server_advertise();
        return 0;

    case BLE_GAP_EVENT_MTU:
        os_printf("mtu update event; conn_handle=%d cid=%d mtu=%d\n",
                      event->mtu.conn_handle,
                      event->mtu.channel_id,
                      event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        os_printf("subscribe event; conn_handle=%d attr_handle=%d "
                      "reason=%d prevn=%d curn=%d previ=%d curi=%d\n",
                      event->subscribe.conn_handle,
                      event->subscribe.attr_handle,
                      event->subscribe.reason,
                      event->subscribe.prev_notify,
                      event->subscribe.cur_notify,
                      event->subscribe.prev_indicate,
                      event->subscribe.cur_indicate);
        conn_handle_subs[event->subscribe.conn_handle] = true;
        return 0;

    default:
        return 0;
    }
}

static void
ble_spp_server_on_reset(int reason)
{
    os_printf("Resetting state; reason=%d\n", reason);
}

static void
ble_spp_server_on_sync(void)
{
    int rc;

    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0)
    {
        os_printf("error determining address type; rc=%d\n", rc);
        return;
    }

    /* Printing ADDR */
    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);

    os_printf("Device Address: ");
    os_printf("%d\n", (int)addr_val);
    os_printf("\n");
    /* Begin advertising. */
    ble_spp_server_advertise();
}

void ble_spp_server_host_task(void *param)
{
    os_printf("BLE Host Task Started\n");

    /* Set the default device name. */
    int rc = ble_svc_gap_device_name_set(adv_name);
    assert(rc == 0);

    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();
    nimble_port_freertos_deinit();
}

uint16_t copied_len;
uint8_t copy_buff[620];

static int ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    return 0;
}

static int ble_svc_gatt_recv_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    switch (ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        os_printf("Callback for read");
        break;
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
    {
        int len = OS_MBUF_PKTLEN(ctxt->om);
        if (len > 0)
        {
            int rc = ble_hs_mbuf_to_flat(ctxt->om, copy_buff, len, &copied_len);
            if (rc == 0)
            {
                // os_printf("Receved data %d Conn %d\n", len, conn_handle);
                //  Put the data into the bytearray to get consumed later!
                enqueue_bytes_bytearray_fifo(spp_in_fifo, copy_buff, copied_len);
                return 0;
            }
            else
            {
            }
        }
    }
    break;
    default:
        os_printf("\nDefault Callback");
        break;
    }
    return 0;
}

/* Define new custom service */
static const struct ble_gatt_svc_def new_ble_svc_gatt_defs[] = {
{
    /*** Service: SPP */
    .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = (ble_uuid_t *)svc_spp_uuid16,
    .includes = NULL,
    .characteristics = characteristics_spp
},
{
    .type = 0,
    .uuid = (ble_uuid_t *)NULL,
    .includes = NULL,
    .characteristics = NULL
}};

static inline void process_ble_data(uint8_t *arr){
    // Sit n wait for any data to come in
    int ret = block_until_n_bytes_fifo(spp_out_fifo, 1);
    if(ret != OS_RET_OK){
        os_panic(ret);
    }

    int count = fifo_byte_array_count(spp_out_fifo);
    // Max transfer size
    if (count > spp_mtu_size)
        count = spp_mtu_size;

    // os_printf("Sending data out!");
    dequeue_bytes_bytearray_fifo(spp_out_fifo, arr, count);
    for (int i = 0; i <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++)
    {
        if (conn_handle_subs[i])
        {
            struct os_mbuf *txom = ble_hs_mbuf_from_flat(arr, count);
            int rc = ble_gatts_notify_custom(i, ble_spp_svc_gatt_read_val_handle_notify, txom);
            if(rc != 0){
                ble_spp_printf("Failed to notify over BLE %d\n", rc);
            }
        }
    }
}

void hal_ble_serial_send_task(void *parameters)
{
    uint8_t arr[spp_mtu_size];
    for (;;)
    {
        process_ble_data(arr);   
    }
}

int hal_ble_serial_send(uint8_t *data, size_t len)
{
    return enqueue_bytes_bytearray_fifo(spp_out_fifo, data, len);
}

static void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op)
    {
    case BLE_GATT_REGISTER_OP_SVC:
        MODLOG_DFLT(DEBUG, "registered service %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                    ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        MODLOG_DFLT(DEBUG, "registering characteristic %s with "
                           "def_handle=%d val_handle=%d\n",
                    ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                    ctxt->chr.def_handle,
                    ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                    ctxt->dsc.handle);
        break;

    default:
        assert(0);
        break;
    }
}

static int gatt_svr_init(void)
{
    int rc = 0;
    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(new_ble_svc_gatt_defs);

    if (rc != 0)
    {
        return rc;
    }

    rc = ble_gatts_add_svcs(new_ble_svc_gatt_defs);
    if (rc != 0)
    {
        return rc;
    }

    return 0;
}

hal_bt_serial_err_t hal_ble_serial_init(ble_connected_cb_t cb, ble_disconnected_cb_t disc_cb,  char *name, size_t name_len)
{
    for(int n = 0; n < name_len; n++){
        adv_name[n] = name[n];
    }
    bluetooth_pair_cb = cb;
    bluetooth_disc_cb = disc_cb;
    // Generate a fifo to store all the date into
    spp_in_fifo = create_byte_array_fifo(FIFO_MAX_SIZE);
    spp_out_fifo = create_byte_array_fifo(FIFO_MAX_SIZE);
    int rc;
    if (spp_in_fifo == nullptr)
    {
        return HAL_BT_SERIAL_BUFFER_OVERFLOW;
    }

    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK)
    {
        os_printf("Failed to init nimble %d \n", ret);
        return HAL_BT_SERIAL_ERROR;
    }

    /* Initialize connection_handle array */
    for (int i = 0; i <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++)
    {
        conn_handle_subs[i] = false;
    }

    /* Initialize the NimBLE host configuration. */
    ble_hs_cfg.reset_cb = ble_spp_server_on_reset;
    ble_hs_cfg.sync_cb = ble_spp_server_on_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_hs_cfg.sm_io_cap = 3;
#ifdef CONFIG_EXAMPLE_BONDING
    ble_hs_cfg.sm_bonding = 1;
#endif
#ifdef CONFIG_EXAMPLE_MITM
    ble_hs_cfg.sm_mitm = 1;
#endif
#ifdef CONFIG_EXAMPLE_USE_SC
    ble_hs_cfg.sm_sc = 1;
#else
    ble_hs_cfg.sm_sc = 0;
#endif
#ifdef CONFIG_EXAMPLE_BONDING
    ble_hs_cfg.sm_our_key_dist = 1;
    ble_hs_cfg.sm_their_key_dist = 1;
#endif

    /* Register custom service */
    rc = gatt_svr_init();
    assert(rc == 0);

    /* Set the default device name. */
    rc = ble_svc_gap_device_name_set("nimble-ble-spp-svr");
    assert(rc == 0);

    /* XXX Need to have template for store */
    // ble_store_config_init();

    nimble_port_freertos_init(ble_spp_server_host_task);

    xTaskCreate((TaskFunction_t)hal_ble_serial_send_task,
                "SPP BLE RX Task",
                4096,
                NULL,
                0,
                NULL);

    os_printf("Nimble Initialized Successfully\n");
    return HAL_BT_SERIAL_OK;
}
#endif