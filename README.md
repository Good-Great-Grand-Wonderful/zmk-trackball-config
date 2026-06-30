# zmk-trackball-config

A central-side custom BLE GATT service that lets a host app configure both
trackballs' **Mode** (input pipeline) and **CPI** over the air, with the
keyboard as the source of truth.

It does not touch ZMK Studio (which stays keymap-only). It drives the existing
`zmk,behavior-pipeline-switch` and `zmk,behavior-paw32xx-res-set` behaviors,
relaying changes for the left (peripheral) ball across the split, and keeps an
authoritative, central-persisted mirror of state so any host reads consistent
values.

## Devicetree

Add one node on the **central** half only (e.g. `crosses_v2_right.dts`):

```dts
/ {
    trackball_config {
        compatible = "gggw,trackball-config";
        left-pipeline       = <&zip_periph_ball>;   // mode introspection (left)
        right-pipeline      = <&zip_central_ball>;  // mode introspection (right)
        left-mode-behavior  = <&pipe_switch>;       // mode invoke (relayed)
        right-mode-behavior = <&pipe_switch_r>;     // mode invoke (local)
        left-cpi            = <&cpi_set_l>;          // CPI introspect + invoke (relayed)
        right-cpi           = <&cpi_set_r>;          // CPI introspect + invoke (local)
        // left-peripheral-source = <0>;            // split index of the left half
    };
};
```

The referenced behaviors/processors must exist on **both** halves (shared dts)
so the relay can resolve them by name; only the `trackball_config` node itself
is central-only. `CONFIG_ZMK_TRACKBALL_CONFIG` auto-enables from the node.

## GATT protocol

Service UUID `7242b000-ba11-4c0d-9e00-a11ce0bca11e`. All multi-byte integers
are **little-endian**. `side`: `0` = left, `1` = right.

### Describe — `7242b001-…` (read)

Static description so the app renders from device truth:

```
u8  version (= 1)
per side, in order [left, right]:
  u8  mode_count
  u16 cpi_min
  u16 cpi_max
  u16 cpi_step
  per mode (mode_count times):
    u8  label_len
    u8[label_len] label (UTF-8, e.g. "Scroll", "Cursor")
```

### State — `7242b002-…` (read + notify)

Current selection (6 bytes); notified on every change:

```
u8  left_mode
u16 left_cpi
u8  right_mode
u16 right_cpi
```

### Control — `7242b003-…` (write, requires encryption/bonding)

5-byte command:

```
u8  side    (0 = left, 1 = right)
u8  kind    (0 = mode, 1 = cpi)
u16 value   (mode: pipeline index; cpi: CPI value)
u8  flags   (bit0 = commit: 1 = apply + persist, 0 = live preview only)
```

CPI values are clamped to `[cpi_min, cpi_max]`; out-of-range mode indices are
rejected. Send `commit = 0` while dragging a slider (RAM-only live preview) and
`commit = 1` once on release to persist.

## Behavior

- The central is the **sole originator** of changes; it relays the left ball's
  changes to the peripheral and applies the right ball's locally.
- The mirror is persisted on the central, so reads are correct across reboots
  and from any host machine without a peripheral→central report. This assumes
  the app is the only thing changing settings (no peripheral-local Mode/CPI
  keys); add those only as central-locality dispatchers if you want them.
