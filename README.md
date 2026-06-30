# zmk-trackball-config

A central-side config service that lets a host app configure both trackballs'
**Mode** (input pipeline) and **CPI** over a USB cable, with the keyboard as the
source of truth.

The host app talks to a dedicated **USB CDC-ACM serial port** via the Web Serial
API (Chrome/Edge desktop). It does not touch ZMK Studio (which stays keymap-only
on its own serial port). It drives the existing `zmk,behavior-pipeline-switch`
and `zmk,behavior-paw32xx-res-set` behaviors, relaying changes for the left
(peripheral) ball across the split, and keeps an authoritative, central-persisted
mirror of state so any host reads consistent values.

> Earlier revisions exposed this over a BLE GATT service. It moved to USB; see
> `docs/adr/0004` for why.

## Devicetree

Add one config node on the **central** half only (e.g. `crosses_v2_right.dts`):

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

The referenced behaviors/processors must exist on **both** halves (shared dts) so
the relay can resolve them by name; only the `trackball_config` node itself is
central-only. `CONFIG_ZMK_TRACKBALL_CONFIG` auto-enables from the node.

The transport needs a CDC-ACM serial port selected via a `chosen` node, on the
central half:

```dts
&zephyr_udc0 {
    tbcfg_acm: tbcfg_acm {
        compatible = "zephyr,cdc-acm-uart";
    };
};

/ {
    chosen {
        gggw,trackball-config-uart = &tbcfg_acm;
    };
};
```

and the USB CDC-ACM stack enabled in the board `.conf`:

```
CONFIG_USB_DEVICE_STACK=y
CONFIG_USB_CDC_ACM=y
CONFIG_SERIAL=y
CONFIG_UART_INTERRUPT_DRIVEN=y
CONFIG_UART_LINE_CTRL=y
```

This CDC-ACM should be the **only** USB serial port. Two CDC-ACM instances (for
example, also enabling ZMK Studio's `studio-rpc-usb-uart`) starve ZMK's legacy
USB device stack - each grabs a READ plus a pending ZLP-WRITE `usb_transfer` slot
at enumeration, and the second port's RX never works reliably (raising
`CONFIG_USB_MAX_NUM_TRANSFERS` does **not** fix it). Run ZMK Studio over BLE
instead.

## Serial protocol

All multi-byte integers are **little-endian**. `side`: `0` = left, `1` = right.

Every message is a frame:

```
0xAB        start byte
u8          type
u16         len   (payload length)
u8[len]     payload
```

The host always initiates; the device only replies (no unsolicited messages).

### Requests (host → device)

| Type | Name | Payload |
|------|------|---------|
| `0x01` | REQ_DESCRIBE | - |
| `0x02` | REQ_STATE | - |
| `0x03` | REQ_CONTROL | 5 bytes (below) |

REQ_CONTROL payload:

```
u8  side    (0 = left, 1 = right)
u8  kind    (0 = mode, 1 = cpi)
u16 value   (mode: pipeline index; cpi: CPI value)
u8  flags   (bit0 = commit: 1 = apply + persist, 0 = live preview only)
```

CPI values are clamped to `[cpi_min, cpi_max]`; out-of-range mode indices are
rejected (the state reply then reflects no change). Send `commit = 0` while
dragging a slider (RAM-only live preview) and `commit = 1` once on release to
persist.

### Responses (device → host)

| Type | Name | Payload |
|------|------|---------|
| `0x81` | DESCRIBE | describe blob (below) |
| `0x82` | STATE | 6 bytes (below) |

A REQ_CONTROL is acknowledged with a STATE frame carrying the new selection.

DESCRIBE blob - static, so the app renders from device truth:

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

STATE - current selection (6 bytes):

```
u8  left_mode
u16 left_cpi
u8  right_mode
u16 right_cpi
```

There are no asynchronous notifications: the app re-reads STATE (REQ_STATE) when
it wants the latest, and gets a STATE reply automatically after each REQ_CONTROL.

## Behavior

- The central is the **sole originator** of changes; it relays the left ball's
  changes to the peripheral and applies the right ball's locally.
- The mirror is persisted on the central, so reads are correct across reboots and
  from any host machine without a peripheral→central report. This assumes the app
  is the only thing changing settings (no peripheral-local Mode/CPI keys); add
  those only as central-locality dispatchers if you want them.
