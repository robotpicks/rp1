# CAN ID map (MVP: 4 drive wheels, no steering joints)

This is the source of truth both the PC-side `rp1_dronecan_bridge` node and the bridge MCU
firmware must agree on. Update it as soon as any of these values are pinned down on real
hardware -- the code refers back to this file rather than duplicating the rationale.

## DroneCAN bus (PC <-> bridge MCU), `can0` in this repo's defaults

| Node                  | DroneCAN node ID |
|-----------------------|------------------|
| PC (`rp1_dronecan_bridge`) | 42 (see `rp1_dronecan_bridge/config/rp1_dronecan_bridge.yaml`) |
| Bridge MCU            | TBD -- must differ from the PC's node ID |

Messages used:
- `uavcan.equipment.esc.RawCommand` (PC -> bridge): `cmd[esc_index]`, saturated int14
  (-8192..8191). Only indices 0-3 are used for the MVP.
- `uavcan.equipment.esc.Status` (bridge -> PC): one message per wheel as it updates
  (`esc_index`, `rpm`, `voltage`, `current`).

## Wheel index convention (`rp1_msgs/WheelCommand`, `rp1_msgs/WheelFeedback`)

| Index | Wheel       | DroneCAN esc_index |
|-------|-------------|---------------------|
| 0     | Front-left  | 0                   |
| 1     | Front-right | 1                   |
| 2     | Rear-left   | 2                   |
| 3     | Rear-right  | 3                   |

`rp1_dronecan_bridge` uses the WheelCommand index directly as the DroneCAN esc_index -- keep
this 1:1 mapping unless there's a strong reason to diverge, to avoid a second translation layer.

## VESC-CAN bus (bridge MCU <-> VESC #1-4)

| Wheel       | VESC CAN controller ID |
|-------------|--------------------------|
| Front-left  | TBD |
| Front-right | TBD |
| Rear-left   | TBD |
| Rear-right  | TBD |

Fill in the controller IDs as each VESC is configured via VESC Tool (VESC Tool's own USB/CAN
link is the only place these should be set/changed at runtime -- see project README).

VESC CAN command choice (Set Duty vs Set Current vs Set RPM): **TBD** -- pick one during bridge
firmware bring-up and record the choice + the exact CAN_PACKET_ID used here. Pull the packet ID
values from the VESC firmware source (`comm_can.c` / `datatypes.h`, `CAN_PACKET_ID` enum) rather
than assuming them, since they can shift between firmware versions.
