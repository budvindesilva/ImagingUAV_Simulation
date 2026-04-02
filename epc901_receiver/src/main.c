/*
 * BLE Receiver for EPC901 CCD Line Sensor (1024 pixels)
 * nRF5340 / nRF Connect SDK v3.1.1
 *
 * Triggered capture mode:
 *   - Listens on UART for 0x54 ('T') trigger byte from laptop Python
 *   - Forwards trigger as BLE write to transmitter CMD characteristic
 *   - Receives pixel notification packets and forwards as framed binary to UART
 *
 * UART frame format (to laptop):
 *   [0xAA] [0x55] [len_lo] [len_hi] [data...]
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

/* UUIDs — must match transmitter */
#define BT_UUID_EPC901_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define BT_UUID_EPC901_CHAR_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)
#define BT_UUID_EPC901_CMD_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2)

static struct bt_uuid_128 epc901_char_uuid = BT_UUID_INIT_128(BT_UUID_EPC901_CHAR_VAL);
static struct bt_uuid_128 epc901_cmd_uuid  = BT_UUID_INIT_128(BT_UUID_EPC901_CMD_VAL);

static struct bt_conn *default_conn;
static struct bt_gatt_discover_params  discover_params;
static struct bt_gatt_subscribe_params subscribe_params;
static struct bt_gatt_exchange_params  exchange_params;

static uint16_t epc901_value_handle;
static uint16_t service_end_handle;
static uint16_t found_ccc_handle = 0;
static uint16_t cmd_value_handle = 0;  /* handle to write trigger to */

static void start_discovery(struct bt_conn *conn);

/* --------------------------------------------------------------------------
 * Binary framed UART write to laptop
 * [0xAA][0x55][len_lo][len_hi][data...]
 * -------------------------------------------------------------------------- */
static void uart_write_frame(const uint8_t *data, uint16_t length)
{
    uint8_t header[4] = {
        0xAA,
        0x55,
        (uint8_t)(length & 0xFF),
        (uint8_t)((length >> 8) & 0xFF)
    };
    for (int i = 0; i < 4; i++) printk("%c", header[i]);
    for (int i = 0; i < length; i++) printk("%c", data[i]);
}

/* --------------------------------------------------------------------------
 * Send trigger to transmitter CMD characteristic
 * -------------------------------------------------------------------------- */
static void send_trigger(void)
{
    if (!default_conn || !cmd_value_handle) {
        return;
    }
    static const uint8_t trigger_byte = 0x01;
    bt_gatt_write_without_response(default_conn, cmd_value_handle,
                                   &trigger_byte, 1, false);
}

/* --------------------------------------------------------------------------
 * UART trigger polling thread
 * Reads UART byte by byte. 0x54 ('T') = trigger next capture.
 * -------------------------------------------------------------------------- */
static const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

void uart_poll_thread(void)
{
    while (1) {
        unsigned char c;
        if (uart_poll_in(uart_dev, &c) == 0) {
            if (c == 0x54) {  /* 'T' */
                send_trigger();
            }
        }
        k_sleep(K_MSEC(5));
    }
}

K_THREAD_DEFINE(uart_poll_tid, 1024, uart_poll_thread, NULL, NULL, NULL, 6, 0, 0);

/* --------------------------------------------------------------------------
 * 1. Notification Callback — pixel packets arrive here
 * -------------------------------------------------------------------------- */
static uint8_t notify_func(struct bt_conn *conn,
                           struct bt_gatt_subscribe_params *params,
                           const void *data, uint16_t length)
{
    if (!data) {
        params->value_handle = 0U;
        return BT_GATT_ITER_STOP;
    }

    uart_write_frame((const uint8_t *)data, length);
    return BT_GATT_ITER_CONTINUE;
}

/* --------------------------------------------------------------------------
 * Discovery: descriptor scan
 * -------------------------------------------------------------------------- */
static uint8_t discover_descriptors(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr,
                                    struct bt_gatt_discover_params *params)
{
    if (!attr) {
        if (found_ccc_handle) {
            subscribe_params.notify       = notify_func;
            subscribe_params.value        = BT_GATT_CCC_NOTIFY;
            subscribe_params.value_handle = epc901_value_handle;
            subscribe_params.ccc_handle   = found_ccc_handle;
            bt_gatt_subscribe(conn, &subscribe_params);
        }
        return BT_GATT_ITER_STOP;
    }

    if (attr->uuid->type == BT_UUID_TYPE_16) {
        if (BT_UUID_16(attr->uuid)->val == 0x2902) {
            found_ccc_handle = attr->handle;
        }
    }

    return BT_GATT_ITER_CONTINUE;
}

/* --------------------------------------------------------------------------
 * Discovery: characteristic scan
 * Finds both DATA (notify) and CMD (write) characteristics.
 * -------------------------------------------------------------------------- */
static uint8_t discover_char(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              struct bt_gatt_discover_params *params)
{
    if (!attr) {
        /* All characteristics found — now find CCC descriptor for DATA */
        if (epc901_value_handle) {
            discover_params.uuid         = NULL;
            discover_params.func         = discover_descriptors;
            discover_params.start_handle = epc901_value_handle + 1;
            discover_params.end_handle   = service_end_handle;
            discover_params.type         = BT_GATT_DISCOVER_DESCRIPTOR;
            bt_gatt_discover(conn, &discover_params);
        }
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = attr->user_data;

    /* DATA characteristic */
    if (bt_uuid_cmp(chrc->uuid, &epc901_char_uuid.uuid) == 0) {
        epc901_value_handle = bt_gatt_attr_value_handle(attr);
    }

    /* CMD characteristic */
    if (bt_uuid_cmp(chrc->uuid, &epc901_cmd_uuid.uuid) == 0) {
        cmd_value_handle = bt_gatt_attr_value_handle(attr);
    }

    return BT_GATT_ITER_CONTINUE;
}

/* --------------------------------------------------------------------------
 * Discovery: service
 * -------------------------------------------------------------------------- */
static uint8_t discover_service(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr,
                                struct bt_gatt_discover_params *params)
{
    if (!attr) {
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_service_val *svc = attr->user_data;
    service_end_handle = svc ? svc->end_handle : BT_ATT_LAST_ATTRIBUTE_HANDLE;

    discover_params.uuid         = NULL;
    discover_params.func         = discover_char;
    discover_params.start_handle = attr->handle + 1;
    discover_params.end_handle   = service_end_handle;
    discover_params.type         = BT_GATT_DISCOVER_CHARACTERISTIC;

    bt_gatt_discover(conn, &discover_params);
    return BT_GATT_ITER_STOP;
}

/* --------------------------------------------------------------------------
 * MTU Exchange
 * -------------------------------------------------------------------------- */
static void exchange_func(struct bt_conn *conn, uint8_t att_err,
                          struct bt_gatt_exchange_params *params)
{
    start_discovery(conn);
}

static void start_discovery(struct bt_conn *conn)
{
    found_ccc_handle  = 0;
    cmd_value_handle  = 0;
    epc901_value_handle = 0;

    discover_params.uuid         = BT_UUID_DECLARE_128(BT_UUID_EPC901_SERVICE_VAL);
    discover_params.func         = discover_service;
    discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    discover_params.end_handle   = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    discover_params.type         = BT_GATT_DISCOVER_PRIMARY;

    bt_gatt_discover(conn, &discover_params);
}

/* --------------------------------------------------------------------------
 * Name Parsing
 * -------------------------------------------------------------------------- */
static bool parse_name(struct bt_data *data, void *user_data)
{
    char *name = user_data;
    if (data->type == BT_DATA_NAME_COMPLETE ||
        data->type == BT_DATA_NAME_SHORTENED) {
        int len = MIN(data->data_len, 31);
        memcpy(name, data->data, len);
        name[len] = '\0';
        return false;
    }
    return true;
}

/* --------------------------------------------------------------------------
 * Scan Callback
 * -------------------------------------------------------------------------- */
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                         struct net_buf_simple *ad)
{
    char name[32] = {0};
    bt_data_parse(ad, parse_name, name);

    if (strcmp(name, "EPC901_TX") != 0) {
        return;
    }

    int err = bt_le_scan_stop();
    if (err && err != -EALREADY) {
        return;
    }

    err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
                            BT_LE_CONN_PARAM_DEFAULT, &default_conn);
    if (err) {
        bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
    }
}

/* --------------------------------------------------------------------------
 * Connection Callbacks
 * -------------------------------------------------------------------------- */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        if (default_conn) {
            bt_conn_unref(default_conn);
            default_conn = NULL;
        }
        bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
        return;
    }

    exchange_params.func = exchange_func;
    int mtu_err = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (mtu_err) {
        start_discovery(conn);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    if (default_conn) {
        bt_conn_unref(default_conn);
        default_conn = NULL;
    }
    bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = connected,
    .disconnected = disconnected,
};

/* --------------------------------------------------------------------------
 * Main
 * -------------------------------------------------------------------------- */
int main(void)
{
    int err = bt_enable(NULL);
    if (err) {
        return 0;
    }

    bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
    k_sleep(K_FOREVER);
    return 0;
}