# CAN ID map (MVP: 4 drive wheels, no steering joints)

VESC firmware has a built-in UAVCAN/DroneCAN CAN mode, so **there is no separate bridge MCU**:
the PC's CAN adapter wires directly to a single CAN bus carrying the 4 VESCs, each configured
to speak DroneCAN natively. This file is the source of truth both `rp1_dronecan_bridge`
(`ros2_ws/src/rp1_dronecan_bridge/`) and each VESC's UAVCAN config must agree on.

```
PC (rp1_dronecan_bridge) --DroneCAN over SocketCAN (can0)--> VESC #1..4 (CAN mode: UAVCAN)
```

## Bus (`can0` in this repo's defaults)

| Node                  | DroneCAN node ID |
|-----------------------|------------------|
| PC (`rp1_dronecan_bridge`) | 42 (see `rp1_dronecan_bridge/config/rp1_dronecan_bridge.yaml`) |
| VESC front-left       | TBD -- set via VESC Tool's UAVCAN page, must be unique on the bus |
| VESC front-right      | TBD |
| VESC rear-left        | TBD |
| VESC rear-right       | TBD |

Messages used:
- `uavcan.equipment.esc.RawCommand` (PC -> VESCs): `cmd[esc_index]`, saturated int14
  (-8192..8191). Only indices 0-3 are used for the MVP. Broadcast, not addressed to a node ID --
  every VESC on the bus sees every RawCommand and picks out its own `esc_index`.
- `uavcan.equipment.esc.Status` (VESC -> PC): one message per wheel as it updates
  (`esc_index`, `rpm`, `voltage`, `current`).

## Wheel index convention (`rp1_msgs/WheelCommand`, `rp1_msgs/WheelFeedback`)

| Index | Wheel       | DroneCAN esc_index (set on that VESC via VESC Tool) |
|-------|-------------|-------------------------------------------------------|
| 0     | Front-left  | 0                   |
| 1     | Front-right | 1                   |
| 2     | Rear-left   | 2                   |
| 3     | Rear-right  | 3                   |

`rp1_dronecan_bridge` uses the WheelCommand index directly as the DroneCAN esc_index -- keep
this 1:1 mapping unless there's a strong reason to diverge, to avoid a second translation layer.
Each VESC's `esc_index` (set in VESC Tool, not its DroneCAN node ID) must match this table.

## VESC UAVCAN configuration (one-off, per VESC, via VESC Tool over USB)

Firmware 7.00 (`/home/user/dev/bldc`, confirmed via `conf_general.h`) exposes **two** distinct
UAVCAN CAN modes -- use **VESC+UAVCAN**, not plain **UAVCAN**:

- `UAVCAN` (`CAN_MODE_UAVCAN`) -- pure UAVCAN, exclusive use of the bus. Incoming frames are
  only ever interpreted as UAVCAN (`libcanard/canard_driver.c`'s own thread drains the CAN rx
  queue directly).
- `VESC+UAVCAN` (`CAN_MODE_VESC_UAVCAN`) -- **this is what we want.** UAVCAN and VESC's native
  CAN protocol coexist on the same bus: each incoming frame is tried as a UAVCAN transfer first
  and falls back to the normal VESC CAN handling if it isn't one (`comm/comm_can.c`, the
  `cancom_process_thread` loop). Plain `UAVCAN` mode would prevent VESC Tool / normal VESC CAN
  diagnostics from working on the shared bus.

For each of the 4 VESCs: App Settings -> General -> CAN Mode = **VESC+UAVCAN**, and set:
- **CAN ID** (the normal `controller_id` field, App Settings -> General -> CAN ID, range
  0-253) -- this doubles as the DroneCAN node ID in `VESC+UAVCAN` mode; there is no separate
  UAVCAN-only node ID field. Must be unique on the bus.
- **`esc_index`** (App Settings -> General -> UAVCAN ESC index, the `can_esc_index` param,
  range 0-255) -- the value from the wheel index table above.

This is a bench/setup-time activity done over VESC Tool's own USB link -- the runtime control
path never touches VESC Tool.

`uavcan.equipment.esc.RawCommand`'s `cmd` value maps to VESC's commanded duty cycle: confirmed
in firmware 7.00 as `raw_val = cmd.data[esc_index] / 8192.0` (int14 range -8192..8191 ->
-1.0..1.0 duty), see `libcanard/canard_driver.c` around the `RawCommand` handler. Re-confirm
against `/home/user/dev/bldc` if the firmware version changes.
