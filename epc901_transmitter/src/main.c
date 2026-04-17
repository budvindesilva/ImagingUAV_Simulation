/*
 * BLE Transmitter for EPC901 CCD Line Sensor (1024 pixels)
 * nRF54L15 / nRF Connect SDK v3.1.1
 *
 * Triggered capture mode:
 *   - Burst thread waits for capture_ready flag
 *   - Flag is set by SAADC EVT_DONE after 1024 samples are collected
 *   - SAADC sampling is started by CMD characteristic write (0x01 from receiver)
 *   - TIMER22 drives SAADC via DPPI — no CPU involvement during sampling
 *   - One frame captured and transmitted per trigger
 *
 * ADC pin: AIN4 = P1.11 (nRF54L15)
 * Timer:   TIMER22 @ 16 MHz, 1 µs compare → 1 Msps
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

/* nrfx drivers for hardware-timed ADC */
#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <helpers/nrfx_gppi.h>
#if defined(DPPI_PRESENT)
#include <nrfx_dppi.h>
#else
#include <nrfx_ppi.h>
#endif

LOG_MODULE_REGISTER(EPC901_Transmitter, LOG_LEVEL_INF);

/* --------------------------------------------------------------------------
 * UUIDs
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
 * Configuration
 * -------------------------------------------------------------------------- */
#define SAMPLES_PER_FRAME        1024
#define BLE_PACKET_SIZE          244   /* preferred, requires MTU 247 */
#define BLE_PACKET_SIZE_FALLBACK 20    /* safe minimum if MTU not yet negotiated */
#define SAADC_SAMPLE_INTERVAL_US 1     /* 1 µs → 1 Msps */

BUILD_ASSERT(SAMPLES_PER_FRAME % 4 == 0, "SAMPLES_PER_FRAME must be a multiple of 4");

/* --------------------------------------------------------------------------
 * ADC / Timer setup (nRF54L15-specific)
 * -------------------------------------------------------------------------- */
#define NRF_SAADC_INPUT_AIN4   NRF_PIN_PORT_TO_PIN_NUMBER(11U, 1)
#define SAADC_INPUT_PIN        NRF_SAADC_INPUT_AIN4
#define TIMER_INSTANCE_NUMBER  22

static nrfx_saadc_channel_t   adc_channel = NRFX_SAADC_DEFAULT_CHANNEL_SE(SAADC_INPUT_PIN, 0);
static const nrfx_timer_t     timer_instance = NRFX_TIMER_INSTANCE(TIMER_INSTANCE_NUMBER);

/* Double-buffer for SAADC */
static int16_t saadc_buf[2][SAMPLES_PER_FRAME];
static uint32_t saadc_current_buffer = 0;

/* --------------------------------------------------------------------------
 * BLE state
 * -------------------------------------------------------------------------- */
static struct bt_conn *current_conn = NULL;
static bool ble_ready = false;

/* --------------------------------------------------------------------------
 * Shared state between SAADC event handler and burst thread
 * -------------------------------------------------------------------------- */
static volatile bool    capture_requested = false;
static volatile bool    capture_ready     = false;
static volatile int16_t *capture_buf      = NULL;

/* Output packed buffer: 1024 * 10 bits = 1280 bytes */
static uint8_t packed_buffer[SAMPLES_PER_FRAME * 10 / 8 + 10];

/* Stats */
static struct {
    uint32_t frames_captured;
    uint32_t frames_transmitted;
    uint32_t send_errors;
} stats = {0};

/* --------------------------------------------------------------------------
 * 10-bit Packing — 4 samples → 5 bytes
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
 * SAADC Event Handler — runs in interrupt context
 * -------------------------------------------------------------------------- */
static void saadc_event_handler(nrfx_saadc_evt_t const *p_event)
{
    nrfx_err_t err;

    switch (p_event->type) {

    case NRFX_SAADC_EVT_READY:
        nrfx_timer_enable(&timer_instance);
        LOG_INF("SAADC ready, timer started.");
        break;

    case NRFX_SAADC_EVT_BUF_REQ:
        err = nrfx_saadc_buffer_set(
            saadc_buf[(saadc_current_buffer++) % 2],
            SAMPLES_PER_FRAME);
        if (err != NRFX_SUCCESS) {
            LOG_ERR("saadc_buffer_set error: 0x%08x", err);
        }
        break;

    case NRFX_SAADC_EVT_DONE:
        /* Buffer full — stop timer, hand buffer to burst thread */
        nrfx_timer_disable(&timer_instance);
        capture_buf   = p_event->data.done.p_buffer;
        capture_ready = true;
        break;

    default:
        LOG_DBG("Unhandled SAADC evt %d", p_event->type);
        break;
    }
}

/* --------------------------------------------------------------------------
 * SAADC Init
 * -------------------------------------------------------------------------- */
static void configure_saadc(void)
{
    nrfx_err_t err;

    IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)),
                DT_IRQ(DT_NODELABEL(adc), priority),
                nrfx_isr, nrfx_saadc_irq_handler, 0);

    err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_init error: 0x%08x", err);
        return;
    }

    adc_channel.channel_config.gain = NRF_SAADC_GAIN1_4;

    err = nrfx_saadc_channels_config(&adc_channel, 1);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_channels_config error: 0x%08x", err);
        return;
    }

    nrfx_saadc_adv_config_t adv_cfg = NRFX_SAADC_DEFAULT_ADV_CONFIG;
    err = nrfx_saadc_advanced_mode_set(BIT(0),
                                       NRF_SAADC_RESOLUTION_10BIT,
                                       &adv_cfg,
                                       saadc_event_handler);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_advanced_mode_set error: 0x%08x", err);
        return;
    }

    err = nrfx_saadc_buffer_set(saadc_buf[0], SAMPLES_PER_FRAME);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("saadc_buffer_set [0] error: 0x%08x", err);
        return;
    }
    err = nrfx_saadc_buffer_set(saadc_buf[1], SAMPLES_PER_FRAME);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("saadc_buffer_set [1] error: 0x%08x", err);
        return;
    }

    err = nrfx_saadc_mode_trigger();
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_mode_trigger error: 0x%08x", err);
        return;
    }

    LOG_INF("SAADC configured (10-bit, AIN4/P1.11).");
}

/* --------------------------------------------------------------------------
 * Timer Init
 * -------------------------------------------------------------------------- */
static void configure_timer(void)
{
    nrfx_err_t err;

    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG(16000000);
    err = nrfx_timer_init(&timer_instance, &timer_cfg, NULL);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_timer_init error: 0x%08x", err);
        return;
    }

    uint32_t ticks = nrfx_timer_us_to_ticks(&timer_instance,
                                             SAADC_SAMPLE_INTERVAL_US);
    nrfx_timer_extended_compare(&timer_instance,
                                NRF_TIMER_CC_CHANNEL0,
                                ticks,
                                NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                false);

    LOG_INF("TIMER22 configured (%u µs interval = %u ticks).",
            SAADC_SAMPLE_INTERVAL_US, ticks);
}

/* --------------------------------------------------------------------------
 * DPPI — wire TIMER COMPARE → SAADC SAMPLE
 *         and SAADC END → SAADC START
 * -------------------------------------------------------------------------- */
static void configure_ppi(void)
{
    nrfx_err_t err;
    uint8_t ppi_sample_ch;
    uint8_t ppi_start_ch;

    err = nrfx_gppi_channel_alloc(&ppi_sample_ch);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("gppi_channel_alloc (sample) error: 0x%08x", err);
        return;
    }

    err = nrfx_gppi_channel_alloc(&ppi_start_ch);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("gppi_channel_alloc (start) error: 0x%08x", err);
        return;
    }

    nrfx_gppi_channel_endpoints_setup(
        ppi_sample_ch,
        nrfx_timer_compare_event_address_get(&timer_instance, NRF_TIMER_CC_CHANNEL0),
        nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE));

    nrfx_gppi_channel_endpoints_setup(
        ppi_start_ch,
        nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_END),
        nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_START));

    nrfx_gppi_channels_enable(BIT(ppi_sample_ch));
    nrfx_gppi_channels_enable(BIT(ppi_start_ch));

    LOG_INF("DPPI configured.");
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
 * Burst Thread
 *
 * Flow per frame:
 *   1. Wait for capture_requested (set by CMD write)
 *   2. Abort any ongoing SAADC, re-arm buffers, re-trigger
 *   3. Wait for capture_ready (set by SAADC EVT_DONE)
 *   4. Pack 1024 x 10-bit → 1280 bytes
 *   5. Wait 100ms for MTU negotiation to complete
 *   6. Transmit in BLE notifications, falling back to 20-byte packets
 *      if the preferred 244-byte MTU is not yet available
 * -------------------------------------------------------------------------- */
void ble_burst_thread(void)
{
    while (1) {
        /* Wait for trigger */
        while (!capture_requested) {
            k_sleep(K_MSEC(10));
        }
        capture_requested = false;

        if (!current_conn || !ble_ready) {
            LOG_WRN("Trigger received but BLE not ready — skipping.");
            continue;
        }

        /* Stop timer and abort any ongoing SAADC conversion */
        nrfx_timer_disable(&timer_instance);
        nrfx_saadc_abort();
        k_sleep(K_MSEC(1));

        /* Re-arm SAADC */
        capture_ready = false;
        saadc_current_buffer = 0;

        nrfx_err_t err = nrfx_saadc_buffer_set(saadc_buf[0], SAMPLES_PER_FRAME);
        if (err != NRFX_SUCCESS) {
            LOG_ERR("Re-arm buffer_set [0] error: 0x%08x", err);
            continue;
        }
        err = nrfx_saadc_buffer_set(saadc_buf[1], SAMPLES_PER_FRAME);
        if (err != NRFX_SUCCESS) {
            LOG_ERR("Re-arm buffer_set [1] error: 0x%08x", err);
            continue;
        }
        err = nrfx_saadc_mode_trigger();
        if (err != NRFX_SUCCESS) {
            LOG_ERR("Re-arm mode_trigger error: 0x%08x", err);
            continue;
        }

        /* Wait for SAADC to fill one frame (1024 samples @ 1Msps = ~1ms) */
        uint32_t timeout = 500;
        while (!capture_ready && timeout > 0) {
            k_sleep(K_MSEC(1));
            timeout--;
        }
        if (!capture_ready) {
            LOG_ERR("SAADC capture timed out.");
            continue;
        }

        /* Pack 1024 x 10-bit → 1280 bytes */
        const int16_t *src = (const int16_t *)capture_buf;
        size_t out_index = 0;
        for (size_t i = 0; i < SAMPLES_PER_FRAME; i += 4) {
            pack_four_10bit((uint16_t)src[i],   (uint16_t)src[i+1],
                            (uint16_t)src[i+2], (uint16_t)src[i+3],
                            &packed_buffer[out_index]);
            out_index += 5;
        }
        stats.frames_captured++;

        /* Allow time for MTU negotiation to complete before sending */
        k_sleep(K_MSEC(100));

        /* Transmit in BLE notifications.
         * Try preferred 244-byte packets first. If MTU hasn't been
         * negotiated yet (-ENOMEM / -ENOBUFS), fall back to 20 bytes. */
        uint16_t tx_offset = 0;
        bool send_error = false;

        while (tx_offset < out_index) {
            uint16_t pkt_size = MIN(BLE_PACKET_SIZE, out_index - tx_offset);

            int ble_err = bt_gatt_notify(current_conn, &epc901_svc.attrs[2],
                                         &packed_buffer[tx_offset], pkt_size);

            if ((ble_err == -ENOMEM || ble_err == -ENOBUFS) &&
                pkt_size > BLE_PACKET_SIZE_FALLBACK) {
                /* MTU not negotiated yet — retry with minimum packet size */
                pkt_size = BLE_PACKET_SIZE_FALLBACK;
                ble_err = bt_gatt_notify(current_conn, &epc901_svc.attrs[2],
                                         &packed_buffer[tx_offset], pkt_size);
            }

            if (ble_err) {
                LOG_WRN("Notify error %d at offset %u", ble_err, tx_offset);
                send_error = true;
                break;
            }

            k_sleep(K_MSEC(2));
            tx_offset += pkt_size;
        }

        if (!send_error) {
            stats.frames_transmitted++;
            LOG_INF("Frame %u transmitted (%u bytes).",
                    stats.frames_transmitted, (unsigned)out_index);
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
    bt_le_adv_start(BT_LE_ADV_CONN_ONE_TIME, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
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
    configure_timer();
    configure_saadc();
    configure_ppi();

    if (bt_enable(NULL)) {
        LOG_ERR("Bluetooth init failed");
        return 0;
    }

    bt_le_adv_start(BT_LE_ADV_CONN_ONE_TIME, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    LOG_INF("Transmitter ready. Advertising as EPC901_TX.");

    return 0;
}