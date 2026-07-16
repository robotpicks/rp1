# rp1 bridge node firmware

DroneCAN <-> VESC-CAN bridge, sitting between the PC and the 4 VESCs. This is the highest-risk,
most time-consuming item in the MVP -- not yet implemented, this is the plan for it.

## Hardware target

An MCU with **two independent CAN controllers** (e.g. an STM32F4xx part with CAN1 + CAN2), one
transceiver per bus:

- CAN1: DroneCAN to the PC (`rp1_dronecan_bridge`, node ID from `docs/can_id_map.md`).
- CAN2: VESC-CAN master to VESC #1-4.

Two physically separate buses avoid ID-space collisions between the two protocols and keep
message filtering simple, rather than trying to share one bus.

## Software plan

- Base the DroneCAN side on an existing open-source node example (e.g. from
  `DroneCAN/libcanard`'s examples) instead of implementing the DroneCAN stack from scratch --
  only the VESC-CAN master logic and the command/status translation are genuinely new code.
- Responsibilities:
  1. Standard DroneCAN node: allocate a node ID, send Heartbeat, subscribe to
     `uavcan.equipment.esc.RawCommand`.
  2. On each RawCommand, map `esc_index` 0-3 to a VESC controller ID (`docs/can_id_map.md`) and
     issue the corresponding VESC-CAN command (Set Duty or Set Current -- decide and record the
     choice in `docs/can_id_map.md`; pull exact `CAN_PACKET_ID` values from the VESC firmware
     source rather than assuming them).
  3. Read VESC status broadcast frames off CAN2, repackage as `uavcan.equipment.esc.Status`
     back to the PC.

## Bring-up / test plan

Bench-testable independently of ROS2 and of any motors:
1. Flash the DroneCAN-only skeleton, verify Heartbeat/RawCommand reception with
   `dronecan_gui_tool` or a small `dronecan` Python script sending synthetic RawCommand frames.
2. Add VESC-CAN output, verify with a CAN sniffer (`candump` on CAN2) or VESC Tool's CAN view
   that the correct frames go out for a given synthetic RawCommand.
3. Connect one VESC + one motor, confirm correct spin direction/speed for a known command.
4. Add the status feedback path (VESC status -> `esc.Status`), verify on the PC side.

## Status

Not started. Toolchain (PlatformIO vs STM32CubeIDE) and exact MCU/board not yet chosen.
