/*
 * Copyright (c) 2026 Vincent Franco
 * SPDX-License-Identifier: MIT
 *
 * Central-side custom GATT service for configuring both trackballs' Mode
 * (pipeline) and CPI from a host app, over the air.
 *
 * Design (see plan): the central is the SOLE originator of changes. It drives
 * each ball by invoking the per-side EVENT_SOURCE behaviors with
 * zmk_behavior_invoke_binding():
 *   - right (central) ball -> event.source = LOCAL (255), runs locally;
 *   - left  (peripheral) ball -> event.source = peripheral index, relayed.
 * It keeps an authoritative, central-persisted MIRROR of both balls' state so
 * it can answer reads without a peripheral->central report.
 */
#define DT_DRV_COMPAT gggw_trackball_config

#include <string.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/att.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include <zmk/behavior.h>
#include <zmk/events/position_state_changed.h>

#include "zip_pipeline_switch.h"
#include "paw32xx_res_set.h"

LOG_MODULE_REGISTER(trackball_config, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
             "exactly one gggw,trackball-config node is supported");

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

/* --- GATT --- */
#define TBCFG_UUID(last)                                                                           \
    BT_UUID_128_ENCODE(0x7242b0##last, 0xba11, 0x4c0d, 0x9e00, 0xa11ce0bca11e)

static const struct bt_uuid_128 svc_uuid = BT_UUID_INIT_128(TBCFG_UUID(00));
static const struct bt_uuid_128 describe_uuid = BT_UUID_INIT_128(TBCFG_UUID(01));
static const struct bt_uuid_128 state_uuid = BT_UUID_INIT_128(TBCFG_UUID(02));
static const struct bt_uuid_128 control_uuid = BT_UUID_INIT_128(TBCFG_UUID(03));

/* Describe blob, built once at init. Layout:
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

static ssize_t describe_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                             uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, describe_blob, describe_len);
}

/* State blob (6 bytes): left_mode u8, left_cpi u16, right_mode u8, right_cpi u16. */
static void build_state(uint8_t out[6]) {
    out[0] = mirror.mode[SIDE_LEFT];
    sys_put_le16(mirror.cpi[SIDE_LEFT], &out[1]);
    out[3] = mirror.mode[SIDE_RIGHT];
    sys_put_le16(mirror.cpi[SIDE_RIGHT], &out[4]);
}

static ssize_t state_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                          uint16_t len, uint16_t offset) {
    uint8_t blob[6];
    build_state(blob);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, blob, sizeof(blob));
}

/* Implemented after BT_GATT_SERVICE_DEFINE (needs the static service symbol). */
static void notify_state(void);

static void state_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    LOG_DBG("trackball state notifications %s", value == BT_GATT_CCC_NOTIFY ? "on" : "off");
}

static ssize_t control_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                             uint16_t len, uint16_t offset, uint8_t flags) {
    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    if (len != 5) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    const uint8_t *p = buf;
    uint8_t side = p[0];
    uint8_t kind = p[1];
    uint16_t value = sys_get_le16(&p[2]);
    bool commit = (p[4] & FLAG_COMMIT) != 0;

    if (side > SIDE_RIGHT) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    int err;
    switch (kind) {
    case KIND_MODE:
        if (value >= zip_pipeline_switch_count(pipeline_dev[side])) {
            return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
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
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    if (err) {
        LOG_ERR("control: side %u kind %u value %u failed (%d)", side, kind, value, err);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    if (commit) {
        k_work_reschedule(&save_work, K_MSEC(2000));
    }

    notify_state();
    return len;
}

/* Attribute indices (keep in sync with notify above):
 *   0 primary service
 *   1 describe characteristic decl, 2 describe value
 *   3 state characteristic decl,    4 state value, 5 state CCC
 *   6 control characteristic decl,  7 control value
 */
BT_GATT_SERVICE_DEFINE(
    trackball_svc, BT_GATT_PRIMARY_SERVICE(&svc_uuid),
    BT_GATT_CHARACTERISTIC(&describe_uuid.uuid, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, describe_read,
                           NULL, NULL),
    BT_GATT_CHARACTERISTIC(&state_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, state_read, NULL, NULL),
    BT_GATT_CCC(state_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&control_uuid.uuid, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE_ENCRYPT, NULL,
                           control_write, NULL));

/* attrs[4] = state value attribute (0 svc, 1-2 describe, 3-4 state, 5 ccc, 6-7 control). */
static void notify_state(void) {
    uint8_t blob[6];
    build_state(blob);
    bt_gatt_notify(NULL, &trackball_svc.attrs[4], blob, sizeof(blob));
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

/* --- init: build describe, seed mirror from device defaults unless a
 * persisted value was already loaded by the settings subsystem. --- */
static int trackball_config_init(void) {
    k_work_init_delayable(&save_work, save_work_cb);
    build_describe();

    if (!mirror_loaded) {
        for (int s = 0; s < 2; s++) {
            mirror.mode[s] = zip_pipeline_switch_active(pipeline_dev[s]);
            mirror.cpi[s] = paw32xx_res_set_get(cpi_dev[s]);
        }
    }
    LOG_INF("trackball config ready: L mode %u cpi %u / R mode %u cpi %u",
            mirror.mode[SIDE_LEFT], mirror.cpi[SIDE_LEFT], mirror.mode[SIDE_RIGHT],
            mirror.cpi[SIDE_RIGHT]);
    return 0;
}

/* APPLICATION priority: after POST_KERNEL devices (pipelines, behaviors) and
 * around the settings load, so device introspection is valid. The mirror_loaded
 * guard makes us order-independent w.r.t. settings_load(). */
SYS_INIT(trackball_config_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
