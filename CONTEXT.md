# Trackball Config

The domain of configuring the two Crosses V2 trackballs (Mode and CPI) from a
host web app over a USB cable - while the keyboard stays connected and usable as
an HID device. The keyboard is the source of truth.

## Topology

**Central**:
The right half. Runs the keymap, owns the USB connection to the host, and is the
**sole originator** of config changes. The config transport and its persisted
state live here only.

**Peripheral**:
The left half. Receives relayed config changes from the central across the split
link. Has no host-facing config surface of its own.

**Side**:
Which trackball a command targets - `0` = left, `1` = right. Distinct from
central/peripheral roles, though right == central and left == peripheral today.

## Transport

**Config link**:
The USB CDC-ACM serial connection between the central (right) and the host web
app (Web Serial API). A second CDC-ACM port, separate from the one ZMK Studio
uses for keymap editing.
_Avoid_: config connection, BLE link (the config link is USB; BLE is not used
for config).

**HID link**:
The keyboard's normal keystroke/pointer connection to the host. Stays live during
configuration, so Mode/CPI changes are felt immediately while using the trackball.
_Avoid_: host connection.

## Protocol

**Frame**:
One message on the config link: `0xAB` start byte + `type` (u8) + `len` (u16 LE)
+ `len` payload bytes. The host always initiates; the device only replies. No
unsolicited messages.

**Describe**:
Request/response (`REQ_DESCRIBE` → `DESCRIBE`) returning a static blob of each
side's mode count, CPI range, and mode labels - so the app renders from device
truth.

**State**:
Request/response (`REQ_STATE` → `STATE`) of the current selection (each side's
mode + CPI). Also returned as the acknowledgement to a Control write. Not pushed
asynchronously - the app reads it when it wants it.
_Avoid_: notification (there is no async notify; State is always polled).

**Control**:
Request (`REQ_CONTROL`, 5-byte payload: side, kind, value, commit flag) that
applies a change; the device replies with the new `STATE`.

**Commit**:
The control flag distinguishing live preview (`commit=0`, RAM-only, e.g. while
dragging a slider) from apply-and-persist (`commit=1`, on release).

## State

**Mirror**:
The central's authoritative, persisted copy of both sides' Mode and CPI, so reads
are consistent across reboots and any host without a peripheral→central report.
