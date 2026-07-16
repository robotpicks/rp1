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

For each of the 4 VESCs: App Settings -> General -> CAN Mode = UAVCAN, set a unique DroneCAN
node ID and the `esc_index` from the table above. This is a bench/setup-time activity done
over VESC Tool's own USB link -- the runtime control path never touches VESC Tool.

`uavcan.equipment.esc.RawCommand`'s `cmd` value maps to VESC's commanded duty cycle
(-1.0..1.0 scaled to the int14 range) in UAVCAN CAN mode -- confirm against the specific VESC
firmware version in use and record any deviation here.
