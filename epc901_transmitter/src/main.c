/*
 * BLE Transmitter for EPC901 CCD Line Sensor (1024 pixels)
 * nRF5340 / nRF Connect SDK v3.1.1
 *
 * Triggered capture mode:
 *   - Burst thread waits for capture_requested flag
 *   - Flag is set by GATT write to CMD characteristic (UUID ...ef2)
 *   - Receiver forwards trigger from laptop serial → BLE write → here
 *   - One frame captured and transmitted per trigger
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

/* UUIDs */
#define BT_UUID_EPC901_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define BT_UUID_EPC901_DATA_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)
#define BT_UUID_EPC901_CMD_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2)

#define BT_UUID_EPC901_SERVICE  BT_UUID_DECLARE_128(BT_UUID_EPC901_SERVICE_VAL)
#define BT_UUID_EPC901_DATA     BT_UUID_DECLARE_128(BT_UUID_EPC901_DATA_VAL)
#define BT_UUID_EPC901_CMD      BT_UUID_DECLARE_128(BT_UUID_EPC901_CMD_VAL)

/* Configuration */
#define SAMPLES_PER_BUFFER  1024
#define BLE_PACKET_SIZE     244

BUILD_ASSERT(SAMPLES_PER_BUFFER % 4 == 0, "SAMPLES_PER_BUFFER must be a multiple of 4");

/* BLE state */
static struct bt_conn *current_conn = NULL;
static bool ble_ready = false;

/* Trigger flag — set by CMD write, consumed by burst thread */
static volatile bool capture_requested = false;

/* Sample buffers */
static int16_t sample_buffer[SAMPLES_PER_BUFFER];
static uint8_t packed_buffer[SAMPLES_PER_BUFFER * 10 / 8 + 10];

/* Stats */
static struct {
    uint32_t frames_captured;
    uint32_t frames_transmitted;
    uint32_t send_errors;
} stats = {0};

/* --------------------------------------------------------------------------
 * 10-bit Packing
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
 * CMD Characteristic Write Handler
 * Receiver writes 0x01 here to trigger one frame capture.
 * -------------------------------------------------------------------------- */
static ssize_t cmd_write(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr,
                         const void *buf, uint16_t len,
                         uint16_t offset, uint8_t flags)
{
    if (len >= 1 && ((uint8_t *)buf)[0] == 0x01) {
        capture_requested = true;
        LOG_INF("Capture triggered.");
    }
    return len;
}

/* --------------------------------------------------------------------------
 * GATT Service
 * Attribute index map:
 *   [0] Primary service declaration
 *   [1] DATA characteristic declaration
 *   [2] DATA characteristic value      <- notify here
 *   [3] DATA CCCD
 *   [4] CMD characteristic declaration
 *   [5] CMD characteristic value       <- write here
 * -------------------------------------------------------------------------- */
BT_GATT_SERVICE_DEFINE(epc901_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_EPC901_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_EPC901_DATA,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, NULL, NULL),
    BT_GATT_CCC(epc901_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_EPC901_CMD,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE,
                           NULL, cmd_write, NULL),
);

/* --------------------------------------------------------------------------
 * Burst Thread — waits for trigger, captures one frame, transmits it
 * -------------------------------------------------------------------------- */
void ble_burst_thread(void)
{
    while (1) {
        /* Wait for trigger from receiver */
        while (!capture_requested) {
            k_sleep(K_MSEC(10));
        }
        capture_requested = false;

        if (!current_conn || !ble_ready) {
            LOG_WRN("Trigger received but not ready — skipping.");
            continue;
        }

        /* Capture one frame
         * Replace sys_rand32_get() with actual EPC901 readout here */
        for (int i = 0; i < SAMPLES_PER_BUFFER; i++) {
            sample_buffer[i] = sys_rand32_get() & 0x3FF;
        }

        /* Pack 1024 x 10-bit -> 1280 bytes */
        size_t out_index = 0;
        for (size_t i = 0; i < SAMPLES_PER_BUFFER; i += 4) {
            pack_four_10bit(sample_buffer[i],     sample_buffer[i + 1],
                            sample_buffer[i + 2], sample_buffer[i + 3],
                            &packed_buffer[out_index]);
            out_index += 5;
        }
        stats.frames_captured++;

        /* Transmit in 244-byte BLE packets */
        uint16_t offset = 0;
        bool send_error = false;

        while (offset < out_index) {
            uint16_t packet_size = MIN(BLE_PACKET_SIZE, out_index - offset);
            int err = bt_gatt_notify(current_conn, &epc901_svc.attrs[2],
                                     &packed_buffer[offset], packet_size);
            if (err) {
                LOG_WRN("Notify error %d at offset %u", err, offset);
                send_error = true;
                break;
            }
            k_sleep(K_MSEC(2));
            offset += packet_size;
        }

        if (!send_error) {
            stats.frames_transmitted++;
            LOG_INF("Frame %u transmitted.", stats.frames_transmitted);
        }
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
    capture_requested = false;
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
    if (bt_enable(NULL)) {
        LOG_ERR("Bluetooth init failed");
        return 0;
    }

    bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    LOG_INF("Transmitter ready. Advertising as EPC901_TX.");
    return 0;
}