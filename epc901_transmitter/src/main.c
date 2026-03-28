/*
 * BLE Transmitter for EPC901 CCD Line Sensor (1024 pixels)
 * Optimized for nRF5340 Stability (nRF Connect SDK v3.1.1)
 *
 * Fixes applied:
 *  - Sampling moved from timer ISR to workqueue (fixes mutex-in-ISR deadlock)
 *  - Timer removed entirely; sampling now driven by burst thread (fixes ring overflow)
 *  - Sample -> transmit -> sleep 500ms, no race condition possible
 *  - Ring buffer overflow guard added
 *  - BUILD_ASSERT added for packing alignment
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/random/random.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

LOG_MODULE_REGISTER(EPC901_Transmitter, LOG_LEVEL_INF);

/* Custom UUIDs - Shared between TX and RX */
#define BT_UUID_EPC901_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define BT_UUID_EPC901_DATA_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)

#define BT_UUID_EPC901_SERVICE  BT_UUID_DECLARE_128(BT_UUID_EPC901_SERVICE_VAL)
#define BT_UUID_EPC901_DATA     BT_UUID_DECLARE_128(BT_UUID_EPC901_DATA_VAL)

/* Configuration */
#define SAMPLES_PER_BUFFER      1024
#define STORAGE_BUFFER_COUNT    4
#define BURST_INTERVAL_MS       500
#define BLE_PACKET_SIZE         244     /* Optimized for MTU 247 */

/* Packing alignment guard */
BUILD_ASSERT(SAMPLES_PER_BUFFER % 4 == 0, "SAMPLES_PER_BUFFER must be a multiple of 4");

/* Storage ring buffer */
typedef struct {
    uint8_t data[SAMPLES_PER_BUFFER * 10 / 8 + 10];
    uint16_t length;
    uint32_t timestamp_ms;
    bool ready_to_send;
} storage_buffer_t;

static storage_buffer_t storage_buffers[STORAGE_BUFFER_COUNT];
static uint32_t storage_write_index = 0;
static uint32_t storage_read_index  = 0;
static uint32_t buffers_available   = STORAGE_BUFFER_COUNT;

static struct k_mutex storage_mutex;

/* BLE State */
static struct bt_conn *current_conn = NULL;
static bool ble_ready = false;

/* Statistics */
static struct {
    uint32_t samples_captured;
    uint32_t buffers_transmitted;
    uint32_t ble_send_errors;
    uint32_t buffers_dropped;
} stats = {0};

/* Persistent buffers for sampling */
static int16_t  sample_buffer[SAMPLES_PER_BUFFER];
static uint8_t  packed_buffer[SAMPLES_PER_BUFFER * 10 / 8 + 10];

/* --------------------------------------------------------------------------
 * 10-bit Packing Logic
 * -------------------------------------------------------------------------- */
static inline void pack_four_10bit(uint16_t s0, uint16_t s1,
                                   uint16_t s2, uint16_t s3,
                                   uint8_t *out)
{
    s0 &= 0x03FF; s1 &= 0x03FF; s2 &= 0x03FF; s3 &= 0x03FF;
    out[0] = s0 & 0xFF;
    out[1] = ((s0 >> 8) & 0x03) | ((s1 & 0x3F) << 2);
    out[2] = ((s1 >> 6) & 0x0F) | ((s2 & 0x0F) << 4);
    out[3] = ((s2 >> 4) & 0x3F) | ((s3 & 0x03) << 6);
    out[4] = (s3 >> 2) & 0xFF;
}

/* --------------------------------------------------------------------------
 * CCCD Callback
 * -------------------------------------------------------------------------- */
static void epc901_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ble_ready = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Notifications %s.", ble_ready ? "ENABLED" : "DISABLED");
}

/* --------------------------------------------------------------------------
 * GATT Service Definition
 * -------------------------------------------------------------------------- */
BT_GATT_SERVICE_DEFINE(epc901_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_EPC901_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_EPC901_DATA,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, NULL, NULL),
    BT_GATT_CCC(epc901_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* --------------------------------------------------------------------------
 * Buffer Management
 * -------------------------------------------------------------------------- */
static void store_buffer(uint8_t *packed_data, uint16_t length, uint32_t timestamp)
{
    k_mutex_lock(&storage_mutex, K_FOREVER);

    storage_buffer_t *buf = &storage_buffers[storage_write_index];

    if (buffers_available == 0) {
        storage_read_index = (storage_read_index + 1) % STORAGE_BUFFER_COUNT;
        stats.buffers_dropped++;
        LOG_WRN("Ring full, oldest buffer dropped (%u total)", stats.buffers_dropped);
    } else {
        buffers_available--;
    }

    memcpy(buf->data, packed_data, length);
    buf->length        = length;
    buf->timestamp_ms  = timestamp;
    buf->ready_to_send = true;

    storage_write_index = (storage_write_index + 1) % STORAGE_BUFFER_COUNT;

    k_mutex_unlock(&storage_mutex);
}

static storage_buffer_t *get_next_buffer(void)
{
    storage_buffer_t *buf = NULL;

    k_mutex_lock(&storage_mutex, K_FOREVER);

    if (buffers_available < STORAGE_BUFFER_COUNT) {
        buf = &storage_buffers[storage_read_index];
        if (buf->ready_to_send) {
            storage_read_index = (storage_read_index + 1) % STORAGE_BUFFER_COUNT;
            buffers_available++;
        } else {
            buf = NULL;
        }
    }

    k_mutex_unlock(&storage_mutex);
    return buf;
}

/* --------------------------------------------------------------------------
 * Burst Thread: Sample -> Transmit -> Sleep, no timer race possible
 * -------------------------------------------------------------------------- */
void ble_burst_thread(void)
{
    while (1) {
        /* 1. Capture one frame if BLE is ready */
        if (ble_ready) {
            /* Replace with actual EPC901 SPI/parallel readout when connected */
            for (int i = 0; i < SAMPLES_PER_BUFFER; i++) {
                sample_buffer[i] = sys_rand32_get() & 0x3FF;
            }

            size_t out_index = 0;
            for (size_t i = 0; i < SAMPLES_PER_BUFFER; i += 4) {
                pack_four_10bit(sample_buffer[i],     sample_buffer[i + 1],
                                sample_buffer[i + 2], sample_buffer[i + 3],
                                &packed_buffer[out_index]);
                out_index += 5;
            }

            store_buffer(packed_buffer, out_index, k_uptime_get_32());
            stats.samples_captured++;
        }

        /* 2. Transmit everything in the ring */
        if (current_conn && ble_ready) {
            storage_buffer_t *buf;
            while ((buf = get_next_buffer()) != NULL) {
                uint16_t offset = 0;
                bool send_error = false;

                while (offset < buf->length) {
                    uint16_t packet_size = MIN(BLE_PACKET_SIZE, buf->length - offset);
                    int err = bt_gatt_notify(current_conn, &epc901_svc.attrs[1],
                                             &buf->data[offset], packet_size);
                    if (err) {
                        stats.ble_send_errors++;
                        LOG_WRN("Notify error %d at offset %u", err, offset);
                        send_error = true;
                        break;
                    }
                    k_sleep(K_MSEC(2));
                    offset += packet_size;
                }

                buf->ready_to_send = false;
                if (!send_error) {
                    stats.buffers_transmitted++;
                }
            }
        } else {
            /* Not connected — flush stale data */
            k_mutex_lock(&storage_mutex, K_FOREVER);
            storage_read_index = storage_write_index;
            buffers_available  = STORAGE_BUFFER_COUNT;
            k_mutex_unlock(&storage_mutex);
        }

        /* 3. Sleep until next burst */
        k_sleep(K_MSEC(BURST_INTERVAL_MS));
    }
}

K_THREAD_DEFINE(ble_burst_tid, 4096, ble_burst_thread, NULL, NULL, NULL, 7, 0, 0);

/* --------------------------------------------------------------------------
 * Advertising
 * -------------------------------------------------------------------------- */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_EPC901_SERVICE_VAL),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* --------------------------------------------------------------------------
 * Connection Callbacks
 * -------------------------------------------------------------------------- */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }
    current_conn = bt_conn_ref(conn);
    LOG_INF("Connected. Waiting for CCCD subscription...");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (0x%02x). Returning to advertising.", reason);
    ble_ready = false;
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
    bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
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
    k_mutex_init(&storage_mutex);

    if (bt_enable(NULL)) {
        LOG_ERR("Bluetooth init failed");
        return 0;
    }

    bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

    LOG_INF("Transmitter ready. Advertising as EPC901_TX.");
    return 0;
}