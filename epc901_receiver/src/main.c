/*
 * BLE Receiver for EPC901 CCD Line Sensor
 * nRF5340 / nRF Connect SDK v3.1.1
 *
 * Protocol:
 *   PC → UART → receiver (0x54 'T') → BLE write → transmitter CMD char
 *   Transmitter captures 1024 samples, packs 10-bit, sends BLE notifications
 *   Receiver forwards each notification payload to PC as:
 *     [0xAA] [0x55] [len_lo] [len_hi] [packed_data...]
 *
 * save_frames.py on PC reassembles packets into full frames and unpacks.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

LOG_MODULE_REGISTER(EPC901_Receiver, LOG_LEVEL_INF);

/* --------------------------------------------------------------------------
 * UUIDs — must match transmitter exactly
 * -------------------------------------------------------------------------- */
#define BT_UUID_EPC901_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define BT_UUID_EPC901_DATA_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)
#define BT_UUID_EPC901_CMD_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2)

#define BT_UUID_EPC901_SERVICE  BT_UUID_DECLARE_128(BT_UUID_EPC901_SERVICE_VAL)
#define BT_UUID_EPC901_DATA     BT_UUID_DECLARE_128(BT_UUID_EPC901_DATA_VAL)
#define BT_UUID_EPC901_CMD      BT_UUID_DECLARE_128(BT_UUID_EPC901_CMD_VAL)

/* --------------------------------------------------------------------------
 * UART device — console UART on nRF5340 DK
 * -------------------------------------------------------------------------- */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_console)
static const struct device *uart_dev;

/* --------------------------------------------------------------------------
 * BLE state
 * -------------------------------------------------------------------------- */
static struct bt_conn *default_conn   = NULL;
static uint16_t data_handle           = 0;
static uint16_t ccc_handle            = 0;
static uint16_t cmd_handle            = 0;

/* --------------------------------------------------------------------------
 * Stats
 * -------------------------------------------------------------------------- */
static struct {
    uint32_t packets_forwarded;
    uint32_t bytes_forwarded;
    uint32_t triggers_sent;
} stats = {0};

/* --------------------------------------------------------------------------
 * Binary-safe UART write
 * Uses uart_poll_out to correctly transmit binary data including null bytes.
 * -------------------------------------------------------------------------- */
static void uart_write_bytes(const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(uart_dev, buf[i]);
    }
}


/* --------------------------------------------------------------------------
 * Forward BLE notification payload to PC with 0xAA 0x55 framing.
 * save_frames.py expects: [0xAA] [0x55] [len_lo] [len_hi] [data...]
 * The data is raw packed 10-bit bytes — save_frames.py unpacks on the PC.
 * -------------------------------------------------------------------------- */
static void forward_packet_to_uart(const uint8_t *data, uint16_t length)
{
    uint8_t header[4];
    header[0] = 0xAA;
    header[1] = 0x55;
    header[2] = (uint8_t)(length & 0xFF);
    header[3] = (uint8_t)((length >> 8) & 0xFF);

    uart_write_bytes(header, sizeof(header));
    uart_write_bytes(data, length);

    stats.packets_forwarded++;
    stats.bytes_forwarded += length;
}

/* --------------------------------------------------------------------------
 * Send capture trigger to transmitter via BLE write to CMD characteristic.
 * Transmitter expects 0x01 to start one frame capture.
 * -------------------------------------------------------------------------- */
static void send_ble_trigger(void)
{
    if (!default_conn || !cmd_handle) {
        LOG_WRN("Cannot trigger — not connected or CMD handle unknown.");
        return;
    }

    uint8_t trigger = 0x01;
    int err = bt_gatt_write_without_response(default_conn, cmd_handle,
                                             &trigger, sizeof(trigger), false);
    if (err) {
        LOG_ERR("BLE trigger write failed (err %d)", err);
    } else {
        stats.triggers_sent++;
        LOG_INF("Trigger sent (%u total).", stats.triggers_sent);
    }
}

/* --------------------------------------------------------------------------
 * UART Trigger Thread
 * Polls UART for 0x54 ('T') from save_frames.py and fires BLE trigger.
 * -------------------------------------------------------------------------- */
static void uart_trigger_thread(void)
{
    LOG_INF("UART trigger thread started. Waiting for 0x54 ('T')...");

    while (1) {
        uint8_t c;
        if (uart_poll_in(uart_dev, &c) == 0) {
            if (c == 0x54) {
                LOG_INF("Trigger byte received from PC.");
                send_ble_trigger();
            }
        }
        k_sleep(K_MSEC(1));
    }
}

K_THREAD_DEFINE(uart_trigger_tid, 1024,
                uart_trigger_thread, NULL, NULL, NULL,
                8, 0, 0);

/* --------------------------------------------------------------------------
 * BLE Notification Handler
 * Forward raw packed payload directly to PC — no unpack/repack needed.
 * -------------------------------------------------------------------------- */
static uint8_t notify_handler(struct bt_conn *conn,
                               struct bt_gatt_subscribe_params *params,
                               const void *data, uint16_t length)
{
    if (!data) {
        LOG_INF("Unsubscribed from notifications.");
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    if (length > 0) {
        forward_packet_to_uart((const uint8_t *)data, length);
    }

    return BT_GATT_ITER_CONTINUE;
}

static struct bt_gatt_subscribe_params subscribe_params = {
    .notify = notify_handler,
    .value  = BT_GATT_CCC_NOTIFY,
};

/* --------------------------------------------------------------------------
 * Subscribe to DATA notifications
 * -------------------------------------------------------------------------- */
static void subscribe_to_data(struct bt_conn *conn)
{
    if (!data_handle || !ccc_handle) {
        LOG_ERR("Cannot subscribe — handles not found.");
        return;
    }

    subscribe_params.value_handle = data_handle;
    subscribe_params.ccc_handle   = ccc_handle;

    int err = bt_gatt_subscribe(conn, &subscribe_params);
    if (err && err != -EALREADY) {
        LOG_ERR("Subscribe failed (err %d)", err);
    } else {
        LOG_INF("Subscribed to DATA notifications (handle %u, ccc %u).",
                data_handle, ccc_handle);
    }
}

/* --------------------------------------------------------------------------
 * GATT Discovery — three passes:
 *   1. Primary service
 *   2. DATA characteristic + CCC descriptor
 *   3. CMD characteristic
 * -------------------------------------------------------------------------- */
static struct bt_gatt_discover_params disc_service_params;
static struct bt_gatt_discover_params disc_char_params;
static struct bt_gatt_discover_params disc_ccc_params;
static struct bt_gatt_discover_params disc_cmd_params;

static uint8_t discover_cmd_func(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 struct bt_gatt_discover_params *params)
{
    if (!attr) {
        LOG_INF("Discovery complete. cmd=%u data=%u ccc=%u",
                cmd_handle, data_handle, ccc_handle);
        return BT_GATT_ITER_STOP;
    }

    if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
        struct bt_gatt_chrc *chrc = (struct bt_gatt_chrc *)attr->user_data;
        if (bt_uuid_cmp(chrc->uuid, BT_UUID_EPC901_CMD) == 0) {
            cmd_handle = chrc->value_handle;
            LOG_INF("Found CMD characteristic (handle %u).", cmd_handle);
        }
    }

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t discover_ccc_func(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 struct bt_gatt_discover_params *params)
{
    if (!attr) {
        /* CCC done — now find CMD characteristic */
        disc_cmd_params.uuid         = NULL;
        disc_cmd_params.type         = BT_GATT_DISCOVER_CHARACTERISTIC;
        disc_cmd_params.start_handle = data_handle + 1;
        disc_cmd_params.end_handle   = 0xFFFF;
        disc_cmd_params.func         = discover_cmd_func;

        int err = bt_gatt_discover(conn, &disc_cmd_params);
        if (err) {
            LOG_ERR("CMD discovery failed (err %d)", err);
        }
        return BT_GATT_ITER_STOP;
    }

    /* Filter for CCC UUID specifically */
    if (bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CCC) == 0) {
        ccc_handle = attr->handle;
        LOG_INF("Found CCC descriptor (handle %u).", ccc_handle);
        subscribe_to_data(conn);
    }

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t discover_char_func(struct bt_conn *conn,
                                  const struct bt_gatt_attr *attr,
                                  struct bt_gatt_discover_params *params)
{
    if (!attr) {
        if (!data_handle) {
            LOG_ERR("DATA characteristic not found.");
            return BT_GATT_ITER_STOP;
        }

        disc_ccc_params.uuid         = NULL;
        disc_ccc_params.type         = BT_GATT_DISCOVER_DESCRIPTOR;
        disc_ccc_params.start_handle = data_handle + 1;
        disc_ccc_params.end_handle   = data_handle + 4;
        disc_ccc_params.func         = discover_ccc_func;

        int err = bt_gatt_discover(conn, &disc_ccc_params);
        if (err) {
            LOG_ERR("CCC discovery failed (err %d)", err);
        }
        return BT_GATT_ITER_STOP;
    }

    if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
        struct bt_gatt_chrc *chrc = (struct bt_gatt_chrc *)attr->user_data;
        if (bt_uuid_cmp(chrc->uuid, BT_UUID_EPC901_DATA) == 0) {
            data_handle = chrc->value_handle;
            LOG_INF("Found DATA characteristic (handle %u).", data_handle);
        }
    }

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t discover_service_func(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
                                     struct bt_gatt_discover_params *params)
{
    if (!attr) {
        LOG_ERR("EPC901 service not found.");
        return BT_GATT_ITER_STOP;
    }

    LOG_INF("Found EPC901 service (handle %u).", attr->handle);

    disc_char_params.uuid         = NULL;
    disc_char_params.type         = BT_GATT_DISCOVER_CHARACTERISTIC;
    disc_char_params.start_handle = attr->handle + 1;
    disc_char_params.end_handle   = 0xFFFF;
    disc_char_params.func         = discover_char_func;

    int err = bt_gatt_discover(conn, &disc_char_params);
    if (err) {
        LOG_ERR("Characteristic discovery failed (err %d)", err);
    }

    return BT_GATT_ITER_STOP;
}

static void start_discovery(struct bt_conn *conn)
{
    LOG_INF("Starting GATT discovery...");

    disc_service_params.uuid         = BT_UUID_EPC901_SERVICE;
    disc_service_params.type         = BT_GATT_DISCOVER_PRIMARY;
    disc_service_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    disc_service_params.end_handle   = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    disc_service_params.func         = discover_service_func;

    int err = bt_gatt_discover(conn, &disc_service_params);
    if (err) {
        LOG_ERR("Service discovery failed (err %d)", err);
    }
}
/* Forward declaration */
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                         struct net_buf_simple *ad);

/* --------------------------------------------------------------------------
 * Connection Callbacks
 * -------------------------------------------------------------------------- */
static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err) {
        LOG_ERR("Connection failed to %s (err %u)", addr, err);
        default_conn = NULL;
        return;
    }

    LOG_INF("Connected to %s", addr);
    default_conn = bt_conn_ref(conn);
    start_discovery(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Disconnected from %s (reason 0x%02x). Restarting scan.",
            addr, reason);

    if (default_conn) {
        bt_conn_unref(default_conn);
        default_conn = NULL;
    }

    data_handle = 0;
    ccc_handle  = 0;
    cmd_handle  = 0;

    bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = connected,
    .disconnected = disconnected,
};

/* --------------------------------------------------------------------------
 * Scan — filter by EPC901 service UUID in advertisement payload
 * -------------------------------------------------------------------------- */
static bool parse_ad_for_uuid(struct bt_data *data, void *user_data)
{
    bool *found = (bool *)user_data;

    if (data->type == BT_DATA_UUID128_ALL ||
        data->type == BT_DATA_UUID128_SOME) {
        static const uint8_t target_uuid[] = {
            BT_UUID_EPC901_SERVICE_VAL
        };
        if (data->data_len >= 16 &&
            memcmp(data->data, target_uuid, 16) == 0) {
            *found = true;
            return false;
        }
    }
    return true;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                         struct net_buf_simple *ad)
{
    if (default_conn) {
        return;
    }

    bool found = false;
    bt_data_parse(ad, parse_ad_for_uuid, &found);
    if (!found) {
        return;
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
    LOG_INF("Found EPC901_TX at %s (RSSI %d). Connecting...", addr_str, rssi);

    int err = bt_le_scan_stop();
    if (err) {
        LOG_ERR("Scan stop failed (err %d)", err);
        return;
    }

    /* Clear any stale connection reference before creating a new one */
    if (default_conn) {
        bt_conn_unref(default_conn);
        default_conn = NULL;
    }

    err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
                            BT_LE_CONN_PARAM_DEFAULT, &default_conn);
    if (err) {
        LOG_ERR("Connection create failed (err %d)", err);
        default_conn = NULL;
        bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
    }
}

/* --------------------------------------------------------------------------
 * Main
 * -------------------------------------------------------------------------- */
int main(void)
{
    LOG_INF("EPC901 Receiver starting...");

    uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready");
        return -1;
    }
    LOG_INF("UART ready.");

    int err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return -1;
    }
    LOG_INF("Bluetooth initialized. Scanning for EPC901_TX...");

    err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
    if (err) {
        LOG_ERR("Scan start failed (err %d)", err);
        return -1;
    }

    while (1) {
        k_sleep(K_SECONDS(10));
        LOG_INF("Status — triggers: %u, packets: %u, bytes: %u",
                stats.triggers_sent,
                stats.packets_forwarded,
                stats.bytes_forwarded);
    }

    return 0;
}