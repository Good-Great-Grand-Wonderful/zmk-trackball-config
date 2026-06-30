/*
 * Copyright (c) 2026 Vincent Franco
 * SPDX-License-Identifier: MIT
 *
 * Central-side config service for both trackballs' Mode (pipeline) and CPI,
 * driven from a host app over a USB CDC-ACM serial link (Web Serial). See
 * CONTEXT.md and docs/adr/0004 (USB transport).
 *
 * Design: the central is the SOLE originator of changes. It drives each ball by
 * invoking the per-side EVENT_SOURCE behaviors with zmk_behavior_invoke_binding():
 *   - right (central) ball -> event.source = LOCAL (255), runs locally;
 *   - left  (peripheral) ball -> event.source = peripheral index, relayed.
 * It keeps an authoritative, central-persisted MIRROR of both balls' state so it
 * can answer reads without a peripheral->central report.
 *
 * Wire protocol (host always initiates; device only replies, no async):
 *   frame = 0xAB | type:u8 | len:u16 LE | payload[len]
 *   0x01 REQ_DESCRIBE          -> 0x81 DESCRIBE (describe blob)
 *   0x02 REQ_STATE             -> 0x82 STATE (6 bytes)
 *   0x03 REQ_CONTROL (5 bytes) -> 0x82 STATE (6 bytes, the new state)
 */
#define DT_DRV_COMPAT gggw_trackball_config

#include <string.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/drivers/uart.h>

#include <zmk/behavior.h>
#include <zmk/events/position_state_changed.h>

#include "zip_pipeline_switch.h"
#include "paw32xx_res_set.h"

LOG_MODULE_REGISTER(trackball_config, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
             "exactly one gggw,trackball-config node is supported");

#if !DT_HAS_CHOSEN(gggw_trackball_config_uart)
#error "gggw,trackball-config-uart chosen node is required (a cdc-acm-uart port)"
#endif

#define TB_NODE DT_DRV_INST(0)

/* Sides / command encoding (must match the host app). */
#define SIDE_LEFT  0
#define SIDE_RIGHT 1
#define KIND_MODE  0
#define KIND_CPI   1
#define FLAG_COMMIT BIT(0)

#define LEFT_SOURCE  ((uint8_t)DT_INST_PROP(0, left_peripheral_source))
#define RIGHT_SOURCE ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL

#define PROTOCOL_VERSION 1

/* --- referenced devices (resolved from devicetree phandles) --- */
static const struct device *const pipeline_dev[2] = {
    DEVICE_DT_GET(DT_INST_PHANDLE(0, left_pipeline)),
    DEVICE_DT_GET(DT_INST_PHANDLE(0, right_pipeline)),
};
static const struct device *const mode_behavior[2] = {
    DEVICE_DT_GET(DT_INST_PHANDLE(0, left_mode_behavior)),
    DEVICE_DT_GET(DT_INST_PHANDLE(0, right_mode_behavior)),
};
static const struct device *const cpi_dev[2] = {
    DEVICE_DT_GET(DT_INST_PHANDLE(0, left_cpi)),
    DEVICE_DT_GET(DT_INST_PHANDLE(0, right_cpi)),
};

static const uint8_t side_source[2] = {LEFT_SOURCE, RIGHT_SOURCE};

/* --- authoritative mirror (persisted on the central) --- */
struct tb_mirror {
    uint8_t mode[2];
    uint16_t cpi[2];
};
static struct tb_mirror mirror;
static bool mirror_loaded;

#define SETTINGS_KEY "tbcfg/state"

static struct k_work_delayable save_work;

static void save_work_cb(struct k_work *work) {
    int err = settings_save_one(SETTINGS_KEY, &mirror, sizeof(mirror));
    if (err < 0) {
        LOG_ERR("Failed to save trackball config %d", err);
    }
}

/* --- behavior invocation (the relay) --- */
static int drive(const struct device *behavior, uint8_t source, uint32_t value, bool persist) {
    if (!device_is_ready(behavior)) {
        return -ENODEV;
    }
    struct zmk_behavior_binding binding = {
        .behavior_dev = behavior->name,
        .param1 = value,
        .param2 = persist ? 1 : 0,
    };
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_LOCAL_IDS_IN_BINDINGS)
    binding.local_id = zmk_behavior_get_local_id(behavior->name);
#endif
    struct zmk_behavior_binding_event event = {
        .layer = 0,
        .position = 0,
        .timestamp = k_uptime_get(),
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = source,
#endif
    };

    int err = zmk_behavior_invoke_binding(&binding, event, true);
    if (err < 0) {
        return err;
    }
    return zmk_behavior_invoke_binding(&binding, event, false);
}

/* --- describe blob, built once at init. Layout:
 *   u8 version
 *   per side (left, right):
 *     u8 mode_count
 *     u16 cpi_min, u16 cpi_max, u16 cpi_step   (little-endian)
 *     per mode: u8 label_len, label bytes
 */
static uint8_t describe_blob[256];
static uint16_t describe_len;

static void build_describe(void) {
    uint16_t o = 0;
    describe_blob[o++] = PROTOCOL_VERSION;

    for (int s = 0; s < 2; s++) {
        uint8_t count = zip_pipeline_switch_count(pipeline_dev[s]);
        describe_blob[o++] = count;
        sys_put_le16(paw32xx_res_set_min(cpi_dev[s]), &describe_blob[o]);
        o += 2;
        sys_put_le16(paw32xx_res_set_max(cpi_dev[s]), &describe_blob[o]);
        o += 2;
        sys_put_le16(paw32xx_res_set_step(cpi_dev[s]), &describe_blob[o]);
        o += 2;

        for (uint8_t m = 0; m < count; m++) {
            const char *label = zip_pipeline_switch_label(pipeline_dev[s], m);
            size_t n = label ? strlen(label) : 0;
            if (n > 255) {
                n = 255;
            }
            if (o + 1 + n > sizeof(describe_blob)) {
                LOG_ERR("describe blob overflow");
                describe_len = o;
                return;
            }
            describe_blob[o++] = (uint8_t)n;
            memcpy(&describe_blob[o], label, n);
            o += n;
        }
    }
    describe_len = o;
}

/* State blob (6 bytes): left_mode u8, left_cpi u16, right_mode u8, right_cpi u16. */
static void build_state(uint8_t out[6]) {
    out[0] = mirror.mode[SIDE_LEFT];
    sys_put_le16(mirror.cpi[SIDE_LEFT], &out[1]);
    out[3] = mirror.mode[SIDE_RIGHT];
    sys_put_le16(mirror.cpi[SIDE_RIGHT], &out[4]);
}

/* Apply a 5-byte control command to a ball + the mirror. Returns 0 or -errno. */
static int apply_control(const uint8_t cmd[5]) {
    uint8_t side = cmd[0];
    uint8_t kind = cmd[1];
    uint16_t value = sys_get_le16(&cmd[2]);
    bool commit = (cmd[4] & FLAG_COMMIT) != 0;

    if (side > SIDE_RIGHT) {
        return -EINVAL;
    }

    int err;
    switch (kind) {
    case KIND_MODE:
        if (value >= zip_pipeline_switch_count(pipeline_dev[side])) {
            return -EINVAL;
        }
        err = drive(mode_behavior[side], side_source[side], value, commit);
        if (!err) {
            mirror.mode[side] = (uint8_t)value;
        }
        break;
    case KIND_CPI: {
        /* Clamp to the side's advertised bounds so the mirror matches what the
         * behavior will apply (the behavior also clamps). */
        uint16_t lo = paw32xx_res_set_min(cpi_dev[side]);
        uint16_t hi = paw32xx_res_set_max(cpi_dev[side]);
        if (value < lo) {
            value = lo;
        } else if (value > hi) {
            value = hi;
        }
        err = drive(cpi_dev[side], side_source[side], value, commit);
        if (!err) {
            mirror.cpi[side] = value;
        }
        break;
    }
    default:
        return -EINVAL;
    }

    if (err) {
        LOG_ERR("control: side %u kind %u value %u failed (%d)", side, kind, value, err);
        return err;
    }

    if (commit) {
        k_work_reschedule(&save_work, K_MSEC(2000));
    }
    return 0;
}

/* --- USB CDC-ACM transport --- */
#define UART_NODE DT_CHOSEN(gggw_trackball_config_uart)
static const struct device *const uart_dev = DEVICE_DT_GET(UART_NODE);

/* Frame: 0xAB | type:u8 | len:u16 LE | payload[len] */
#define FRAME_START 0xAB
#define MSG_REQ_DESCRIBE 0x01
#define MSG_REQ_STATE    0x02
#define MSG_REQ_CONTROL  0x03
#define MSG_DESCRIBE     0x81
#define MSG_STATE        0x82

/* Inbound payloads are tiny (REQ_CONTROL is the largest at 5 bytes). */
#define MAX_REQ_PAYLOAD 8

RING_BUF_DECLARE(rx_rb, 64);
K_SEM_DEFINE(rx_sem, 0, 1);

/* Send one frame. Called only from the rx thread, so no TX locking is needed. */
static void send_frame(uint8_t type, const uint8_t *data, uint16_t len) {
    uint8_t hdr[4] = {FRAME_START, type, (uint8_t)(len & 0xff), (uint8_t)(len >> 8)};
    for (int i = 0; i < (int)sizeof(hdr); i++) {
        uart_poll_out(uart_dev, hdr[i]);
    }
    for (uint16_t i = 0; i < len; i++) {
        uart_poll_out(uart_dev, data[i]);
    }
}

static void dispatch(uint8_t type, const uint8_t *payload, uint16_t len) {
    uint8_t state[6];

    switch (type) {
    case MSG_REQ_DESCRIBE:
        send_frame(MSG_DESCRIBE, describe_blob, describe_len);
        break;
    case MSG_REQ_STATE:
        build_state(state);
        send_frame(MSG_STATE, state, sizeof(state));
        break;
    case MSG_REQ_CONTROL:
        if (len == 5) {
            apply_control(payload);
        } else {
            LOG_WRN("REQ_CONTROL bad length %u", len);
        }
        /* Reply with the (possibly unchanged) authoritative state either way. */
        build_state(state);
        send_frame(MSG_STATE, state, sizeof(state));
        break;
    default:
        LOG_WRN("unknown request type 0x%02x", type);
        break;
    }
}

/* Byte-at-a-time frame parser, fed from the rx ring buffer by the rx thread. */
static void parse_byte(uint8_t b) {
    static enum { S_START, S_TYPE, S_LEN0, S_LEN1, S_PAYLOAD } state = S_START;
    static uint8_t type;
    static uint16_t len, got;
    static uint8_t payload[MAX_REQ_PAYLOAD];

    switch (state) {
    case S_START:
        if (b == FRAME_START) {
            state = S_TYPE;
        }
        break;
    case S_TYPE:
        type = b;
        state = S_LEN0;
        break;
    case S_LEN0:
        len = b;
        state = S_LEN1;
        break;
    case S_LEN1:
        len |= (uint16_t)b << 8;
        got = 0;
        if (len > MAX_REQ_PAYLOAD) {
            LOG_WRN("frame payload too long (%u), dropping", len);
            state = S_START;
        } else if (len == 0) {
            dispatch(type, NULL, 0);
            state = S_START;
        } else {
            state = S_PAYLOAD;
        }
        break;
    case S_PAYLOAD:
        payload[got++] = b;
        if (got == len) {
            dispatch(type, payload, len);
            state = S_START;
        }
        break;
    }
}

static void rx_thread_main(void) {
    for (;;) {
        k_sem_take(&rx_sem, K_FOREVER);
        uint8_t b;
        while (ring_buf_get(&rx_rb, &b, 1) == 1) {
            parse_byte(b);
        }
    }
}

K_THREAD_DEFINE(tbcfg_rx_thread, 1024, rx_thread_main, NULL, NULL, NULL, K_PRIO_PREEMPT(10), 0, 0);

static void uart_cb(const struct device *dev, void *user_data) {
    ARG_UNUSED(user_data);
    if (!uart_irq_update(dev)) {
        return;
    }
    while (uart_irq_rx_ready(dev)) {
        uint8_t tmp[32];
        int n = uart_fifo_read(dev, tmp, sizeof(tmp));
        if (n <= 0) {
            break;
        }
        uint32_t put = ring_buf_put(&rx_rb, tmp, n);
        if (put < (uint32_t)n) {
            LOG_WRN("rx ring buffer full, dropped %d bytes", n - (int)put);
        }
    }
    k_sem_give(&rx_sem);
}

/* --- settings (mirror persistence) --- */
static int tbcfg_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
    const char *next;
    if (settings_name_steq(name, "state", &next) && !next) {
        ssize_t rc = read_cb(cb_arg, &mirror, sizeof(mirror));
        if (rc >= 0) {
            mirror_loaded = true;
            return 0;
        }
        return rc;
    }
    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(trackball_config, "tbcfg", NULL, tbcfg_settings_set, NULL, NULL);

/* --- init: build describe, seed mirror from device defaults unless a persisted
 * value was already loaded by the settings subsystem, then bring up the CDC-ACM
 * transport. --- */
static int trackball_config_init(void) {
    k_work_init_delayable(&save_work, save_work_cb);
    build_describe();

    if (!mirror_loaded) {
        for (int s = 0; s < 2; s++) {
            mirror.mode[s] = zip_pipeline_switch_active(pipeline_dev[s]);
            mirror.cpi[s] = paw32xx_res_set_get(cpi_dev[s]);
        }
    }

    if (!device_is_ready(uart_dev)) {
        LOG_ERR("config UART not ready");
        return -ENODEV;
    }
    int err = uart_irq_callback_user_data_set(uart_dev, uart_cb, NULL);
    if (err < 0) {
        LOG_ERR("Failed to set config UART callback (%d)", err);
        return err;
    }
    uart_irq_rx_enable(uart_dev);

    LOG_INF("trackball config ready: L mode %u cpi %u / R mode %u cpi %u",
            mirror.mode[SIDE_LEFT], mirror.cpi[SIDE_LEFT], mirror.mode[SIDE_RIGHT],
            mirror.cpi[SIDE_RIGHT]);
    return 0;
}

/* APPLICATION priority: after POST_KERNEL devices (pipelines, behaviors, the USB
 * CDC-ACM uart) and around the settings load, so device introspection is valid.
 * The mirror_loaded guard makes us order-independent w.r.t. settings_load(). */
SYS_INIT(trackball_config_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
