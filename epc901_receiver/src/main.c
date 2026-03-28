/*
 * BLE Receiver for EPC901 CCD Line Sensor (1024 pixels)
 * nRF5340 / nRF Connect SDK v3.1.1
 *
 * Debug: Dump all descriptors after characteristic to find CCC handle.
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#define BT_UUID_EPC901_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

#define BT_UUID_EPC901_CHAR_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)

static struct bt_uuid_128 epc901_char_uuid = BT_UUID_INIT_128(BT_UUID_EPC901_CHAR_VAL);

static struct bt_conn *default_conn;
static struct bt_gatt_discover_params  discover_params;
static struct bt_gatt_subscribe_params subscribe_params;
static struct bt_gatt_exchange_params  exchange_params;

static uint16_t epc901_value_handle;
static uint16_t service_end_handle;
static uint16_t found_ccc_handle = 0;

static void start_discovery(struct bt_conn *conn);

/* --------------------------------------------------------------------------
 * 1. Notification Callback
 * -------------------------------------------------------------------------- */
static uint8_t notify_func(struct bt_conn *conn,
                           struct bt_gatt_subscribe_params *params,
                           const void *data, uint16_t length)
{
    if (!data) {
        printk("[UNSUBSCRIBED]\n");
        params->value_handle = 0U;
        return BT_GATT_ITER_STOP;
    }

    printk("Received %u bytes of pixel data\n", length);
    return BT_GATT_ITER_CONTINUE;
}

/* --------------------------------------------------------------------------
 * 2c. Subscribe using the hardcoded CCC handle found during descriptor dump
 * -------------------------------------------------------------------------- */
static void do_subscribe(struct bt_conn *conn, uint16_t ccc_handle)
{
    printk("Subscribing with value_handle=%u, ccc_handle=%u\n",
           epc901_value_handle, ccc_handle);

    subscribe_params.notify       = notify_func;
    subscribe_params.value        = BT_GATT_CCC_NOTIFY;
    subscribe_params.value_handle = epc901_value_handle;
    subscribe_params.ccc_handle   = ccc_handle;

    int err = bt_gatt_subscribe(conn, &subscribe_params);
    if (err && err != -EALREADY) {
        printk("Subscribe failed (err %d)\n", err);
    } else {
        printk("Subscribed! Waiting for notifications...\n");
    }
}

/* --------------------------------------------------------------------------
 * 2c. Descriptor dump — prints UUID of every descriptor found
 *     We will read the handle printed here and hardcode it.
 * -------------------------------------------------------------------------- */
static uint8_t discover_descriptors(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr,
                                    struct bt_gatt_discover_params *params)
{
    if (!attr) {
        printk("Descriptor scan complete. CCC handle=%u\n", found_ccc_handle);
        if (found_ccc_handle) {
            do_subscribe(conn, found_ccc_handle);
        } else {
            printk("ERROR: CCC handle not found in descriptor scan\n");
        }
        return BT_GATT_ITER_STOP;
    }

    /* Print the UUID type and handle for every descriptor */
    if (attr->uuid->type == BT_UUID_TYPE_16) {
        uint16_t uuid_val = BT_UUID_16(attr->uuid)->val;
        printk("  Descriptor handle=%u uuid=0x%04x\n", attr->handle, uuid_val);

        /* 0x2902 is the 16-bit UUID for CCCD */
        if (uuid_val == 0x2902) {
            found_ccc_handle = attr->handle;
        }
    } else {
        printk("  Descriptor handle=%u uuid=128-bit\n", attr->handle);
    }

    return BT_GATT_ITER_CONTINUE;
}

/* --------------------------------------------------------------------------
 * 2b. Characteristic discovery — find any char, verify UUID, then dump descriptors
 * -------------------------------------------------------------------------- */
static uint8_t discover_char(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              struct bt_gatt_discover_params *params)
{
    if (!attr) {
        printk("No characteristic found in service range\n");
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = attr->user_data;
    if (bt_uuid_cmp(chrc->uuid, &epc901_char_uuid.uuid) != 0) {
        printk("Skipping characteristic handle=%u\n", attr->handle);
        return BT_GATT_ITER_CONTINUE;
    }

    epc901_value_handle = bt_gatt_attr_value_handle(attr);
    printk("EPC901 characteristic matched (decl=%u value=%u)\n",
           attr->handle, epc901_value_handle);
    printk("Scanning all descriptors from handle %u to %u...\n",
           epc901_value_handle + 1, service_end_handle);

    /* Dump ALL descriptors with no UUID filter */
    discover_params.uuid         = NULL;
    discover_params.func         = discover_descriptors;
    discover_params.start_handle = epc901_value_handle + 1;
    discover_params.end_handle   = service_end_handle;
    discover_params.type         = BT_GATT_DISCOVER_DESCRIPTOR;

    int err = bt_gatt_discover(conn, &discover_params);
    if (err) {
        printk("Descriptor scan failed (err %d)\n", err);
    }

    return BT_GATT_ITER_STOP;
}

/* --------------------------------------------------------------------------
 * 2a. Service discovery
 * -------------------------------------------------------------------------- */
static uint8_t discover_service(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr,
                                struct bt_gatt_discover_params *params)
{
    if (!attr) {
        printk("EPC901 service not found\n");
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_service_val *svc = attr->user_data;
    service_end_handle = svc ? svc->end_handle : BT_ATT_LAST_ATTRIBUTE_HANDLE;

    printk("EPC901 service found (handle=%u end=%u)\n",
           attr->handle, service_end_handle);

    discover_params.uuid         = NULL;
    discover_params.func         = discover_char;
    discover_params.start_handle = attr->handle + 1;
    discover_params.end_handle   = service_end_handle;
    discover_params.type         = BT_GATT_DISCOVER_CHARACTERISTIC;

    int err = bt_gatt_discover(conn, &discover_params);
    if (err) {
        printk("Characteristic discovery failed (err %d)\n", err);
    }

    return BT_GATT_ITER_STOP;
}

/* --------------------------------------------------------------------------
 * 3. MTU Exchange
 * -------------------------------------------------------------------------- */
static void exchange_func(struct bt_conn *conn, uint8_t att_err,
                          struct bt_gatt_exchange_params *params)
{
    if (att_err) {
        printk("MTU exchange failed (err %u)\n", att_err);
    } else {
        printk("MTU exchanged: %u bytes\n", bt_gatt_get_mtu(conn));
    }
    start_discovery(conn);
}

static void start_discovery(struct bt_conn *conn)
{
    found_ccc_handle = 0;
    discover_params.uuid         = BT_UUID_DECLARE_128(BT_UUID_EPC901_SERVICE_VAL);
    discover_params.func         = discover_service;
    discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    discover_params.end_handle   = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    discover_params.type         = BT_GATT_DISCOVER_PRIMARY;

    int err = bt_gatt_discover(conn, &discover_params);
    if (err) {
        printk("Service discovery failed (err %d)\n", err);
    }
}

/* --------------------------------------------------------------------------
 * 4. Name Parsing
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
 * 5. Scan Callback
 * -------------------------------------------------------------------------- */
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                         struct net_buf_simple *ad)
{
    char name[32] = {0};
    bt_data_parse(ad, parse_name, name);

    if (strcmp(name, "EPC901_TX") != 0) {
        return;
    }

    printk("Found EPC901_TX (RSSI %d). Connecting...\n", rssi);

    int err = bt_le_scan_stop();
    if (err && err != -EALREADY) {
        printk("Scan stop failed (err %d)\n", err);
        return;
    }

    err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
                            BT_LE_CONN_PARAM_DEFAULT, &default_conn);
    if (err) {
        printk("Connection create failed (err %d)\n", err);
        bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
    }
}

/* --------------------------------------------------------------------------
 * 6. Connection Callbacks
 * -------------------------------------------------------------------------- */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("Connection failed (err %u)\n", err);
        if (default_conn) {
            bt_conn_unref(default_conn);
            default_conn = NULL;
        }
        bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
        return;
    }

    printk("Connected. Negotiating MTU...\n");
    exchange_params.func = exchange_func;
    int mtu_err = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (mtu_err) {
        printk("MTU exchange failed (err %d)\n", mtu_err);
        start_discovery(conn);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason 0x%02x). Restarting scan...\n", reason);
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
        printk("Bluetooth init failed (err %d)\n", err);
        return 0;
    }

    printk("Receiver initialized. Scanning (active)...\n");
    bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);

    k_sleep(K_FOREVER);
    return 0;
}