# CAN ID map (MVP: 4 drive wheels, no steering joints)

VESC firmware has a built-in UAVCAN/DroneCAN CAN mode, so **there is no separate bridge MCU**:
the PC's CAN adapter wires directly to a single CAN bus carrying the 4 VESCs, each configured
to speak DroneCAN natively. This file is the source of truth both `rp1_description`'s URDFs and
each VESC's UAVCAN config must agree on.

The component that speaks the protocol -- `vesc_dronecan_driver`, in the `vesc_dronecan_ros`
submodule -- is robot-agnostic and imposes **no** id convention of its own; it reads whatever
`esc_index` / `actuator_id` a joint declares. Everything allocated below is rp1's choice.

```
PC (vesc_dronecan_driver) --DroneCAN over SocketCAN (can0)--> VESC #1..4 (CAN mode: UAVCAN)
```

The DroneCAN encoding lives in the ros2_control hardware component's `read()`/`write()`, using a
vendored copy of the same libcanard codec the firmware uses (`vendor/libcanard/`). It used to
live in a separate `rp1_dronecan_bridge` node speaking the `dronecan` Python library; that node
is gone. The wire protocol did not change with that move -- only the process that speaks it.

## Bus (`can0` in this repo's defaults)

| Node                  | DroneCAN node ID |
|-----------------------|------------------|
| PC (`vesc_dronecan_driver`) | 42 (the `node_id` hardware param in `urdf/rp1_drive.urdf`) |
| VESC front-left       | TBD -- set via VESC Tool's UAVCAN page, must be unique on the bus |
| VESC front-right      | TBD |
| VESC rear-left        | TBD |
| VESC rear-right       | TBD |

Messages used:
- `uavcan.equipment.esc.RPMCommand` (PC -> VESCs): `rpm[esc_index]`, an 18-bit signed array.
  Only indices 0-3 are used for the MVP. Broadcast, not addressed to a node ID -- every VESC on
  the bus sees every RPMCommand and picks out its own `esc_index`. Firmware routes it to
  `mc_interface_set_pid_speed()`, a closed-loop speed PID, which is what lets ros2_control
  expose an honest **velocity** command interface.
- `uavcan.equipment.esc.Status` (VESC -> PC): one message per wheel as it updates
  (`esc_index`, `rpm`, `voltage`, `current`, `temperature`).

**The two directions do not use the same RPM units.** Confirmed in firmware 7.00
(`libcanard/canard_driver.c`):

| Direction | Code | Units on the wire |
|-----------|------|-------------------|
| VESC -> PC | `sendEscStatus()`: `status.rpm = mc_interface_get_rpm() / (si_motor_poles / 2.0)` | mechanical motor RPM |
| PC -> VESC | `handle_esc_rpm_command()`: `mc_interface_set_pid_speed(rpm_val)`, no scaling | ERPM (electrical) |

So a value read back from `Status` cannot be commanded verbatim -- it is off by the pole-pair
count. `vesc_dronecan_driver` compensates on the command side, gated by the
`command_rpm_is_erpm` hardware parameter (default `true`, matching the firmware as it stands).
If the fork is ever fixed to scale the command side too, set that parameter `false` and the
extra factor drops out. `esc.RawCommand` (duty cycle) is no longer used by the runtime path.

Per-ESC `voltage`/`current`/`temperature` from the same `Status` messages are exported as
ros2_control `<gpio>` state interfaces, which reach `/dynamic_joint_states` through
`joint_state_broadcaster`; `rp1_elrs`'s `esc_telemetry_to_battery` turns them into the handset's
`BatteryState`.

## Wheel index convention

| Index | Wheel       | ros2_control joint  | DroneCAN esc_index (set on that VESC via VESC Tool) |
|-------|-------------|---------------------|------------------------------------------------------|
| 0     | Front-left  | `drive_front_left`  | 0 |
| 1     | Front-right | `drive_front_right` | 1 |
| 2     | Rear-left   | `drive_rear_left`   | 2 |
| 3     | Rear-right  | `drive_rear_right`  | 3 |

Each VESC's `esc_index` (set in VESC Tool, not its DroneCAN node ID) must match this table, and
must match the `esc_index` parameter on the corresponding joint in
`rp1_description/urdf/rp1_drive.urdf`. There is no separate translation layer -- the URDF
joint carries the index directly.

**Drive motor commutation/speed feedback sensor**: each drive VESC has both an AB (2-channel)
encoder and a 3-Hall-sensor setup available to wire up, but only one can be selected as the
active feedback sensor at a time (a per-motor VESC Tool FOC configuration choice). **Decision
(2026-07-30): use the Hall sensors.**
- The drive units are sealed hub motors (`ZLLG16ASM800`) -- Hall sensors are near-certainly
  already factory-wired inside; retrofitting an external AB encoder onto an already-sealed hub
  motor would need mechanical rework that may not even be possible without a custom part.
- The drive path only needs velocity, never position -- `diff_drive_controller` runs with
  `position_feedback: false` and integrates position from velocity already (see below); nothing
  needs the encoder's finer resolution.
- rp1 is an outdoor ag-robot (dust/moisture/vibration) -- Hall sensors are simple digital
  switches with generous alignment tolerance; an encoder needs tighter mechanical alignment and
  is more failure-prone in that environment.
- Hall + sensorless hybrid FOC is VESC's most mature, most widely-used sensor mode, with better
  standstill/low-speed startup behavior than sensorless alone, without encoder-commutation setup
  complexity.

Doesn't change the DroneCAN wire protocol either way (`esc.Status.rpm` reporting is the same
regardless of which sensor feeds the VESC's internal FOC loop) -- this is a VESC Tool
configuration choice for whoever wires up the motors, not a software change. Unrelated to the
steering VESCs' ABZ encoder (see below) -- that's a separate 3-channel encoder for
absolute-ish position feedback, on a different actuator (`actuator_id`, not `esc_index`) with
its own FOC position-control needs (steering does need position, unlike drive).

**Shared VESC sensor port**: on this VESC hardware, Hall and encoder modes use the *same*
physical connector -- its pins are just reinterpreted depending on configuration: Hall mode
reads them as 3 Hall channels + motor temperature; encoder mode reads the same pins as A/B/Z +
temperature. So the drive VESCs (Hall mode) and the steering VESCs (ABZ encoder mode) use an
identical port type, just configured differently -- not two different connectors.

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

## Steering actuator convention (bench, ahead of the MVP's "no steering joints" phasing)

Firmware 7.00 (`add-actuator-arraycommand` branch, `/home/user/dev/bldc` commit `a242b9ae`)
also implements `uavcan.equipment.actuator.ArrayCommand`/`Status` for position-controlled
steering, reusing the *same* `uavcan_esc_index` VESC Tool field as the message's `actuator_id`
(no separate config field) -- so a given VESC is either a drive wheel (`esc.RawCommand`/`Status`
at its `esc_index`) or a steering actuator (`actuator.ArrayCommand`/`Status` at that same
numeric value used as `actuator_id`), depending only on how the PC side addresses it.

**Convention: steering `actuator_id` = drive wheel index + 4.**

| Wheel index | Drive `esc_index` | Steering `actuator_id` |
|-------------|--------------------|--------------------------|
| 0 (Front-left)  | 0 | 4 |
| 1 (Front-right) | 1 | 5 |
| 2 (Rear-left)   | 2 | 6 |
| 3 (Rear-right)  | 3 | 7 |

Only wheels 1 and 2's steering (`actuator_id` 5 and 6) are wired up on the bench so far; 0 and
3's steering (4 and 7) are unconfirmed.

- `Command.command_type` -- only `COMMAND_TYPE_POSITION` (1) is implemented in firmware; the
  other DSDL-defined types (UNITLESS/FORCE/SPEED/PWM) are not handled.
- `Command.command_value` / `Status.position` are DSDL radians. Firmware converts to/from VESC's
  own degrees internally (`mc_interface_set_pid_pos`/`mc_interface_get_pid_pos_now`) -- see
  `libcanard/canard_driver.c`'s `handle_actuator_array_command`/`sendActuatorStatus`.
- Real position control needs an encoder wired to the steering VESC (`mc_interface_set_pid_pos`
  requires FOC position feedback). As of 2026-07-30 the steering VESCs are specified to use an
  ABZ quadrature encoder for this -- a correction to this file's earlier claim that none of the
  bench steering VESCs have one (and that reported positions were meaningless FOC-fighting-
  phantom-feedback). Not yet independently re-verified against the actual bench wiring.
  - The encoder's A/B/Z outputs are **differential** (A+/A-, B+/B-, Z+/Z-), not the single-ended
    levels the VESC's shared sensor port expects in encoder mode (see "Shared VESC sensor port"
    above). An AM26C32-based differential-to-single-ended converter board sits between them
    (spec sheet: `rp1-specs/assets/Diff2single.odt`): 3-channel differential in, single-ended
    0-5V A/B/Z out (not 5V-tolerant micro pins need a 1-3.3kΩ series resistor per the sheet), up
    to 20MHz with no pulse loss, plus a built-in 5V/150mA
    supply for the encoder itself, powered from either 7-35V or a direct 5V rail.
- `vesc_dronecan_driver` handles this through the same `VescDroneCanSystem` component as
  the drive wheels: a joint declaring `actuator_id` is a steering actuator (position command),
  one declaring `esc_index` is a drive wheel (velocity command). The MVP description
  (`urdf/rp1_drive.urdf`) deliberately contains drive joints only -- including steering joints
  there would broadcast an `actuator.ArrayCommand` every cycle carrying whatever an unclaimed
  command interface holds, and firmware `ArrayCommand` support is on a non-mainline branch (see
  above) regardless of encoder state. The bench steering description (`urdf/rp1_steering.urdf`)
  is separate and used on its own.
