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
| VESC front-left       | 1 -- confirmed on the bench via `tools/can_vesc_test.py listen` (2026-08-06) |
| VESC front-right      | 2 |
| VESC rear-left        | 3 -- confirmed on the bench via `tools/can_vesc_test.py listen` (2026-09-06), after fixing a scrambled `uavcan_esc_index` (was reporting 7, the steering range, instead of 3) |
| VESC rear-right       | 4 -- confirmed on the bench via `tools/can_vesc_test.py listen` (2026-09-06), after fixing a scrambled `uavcan_esc_index`; didn't appear on the bus at all before the fix, so its prior (wrong) index value wasn't directly observed |

**Never assign node ID `0`.** In UAVCAN/DroneCAN, `0` is reserved for "anonymous" -- the source
address a node without an assigned ID uses (e.g. during dynamic node ID allocation), not a valid
device address. The `dronecan` Python library enforces this too (`node_id` setter requires
`1 <= value <= 127`, treats `0` as `is_anonymous`).

Messages used:
- `uavcan.equipment.esc.RPMCommand` (PC -> VESCs): `rpm[esc_index]`, an 18-bit signed array.
  Only indices 1-4 are used for the MVP (index 0 is deliberately left unused -- see the wheel
  index convention below). Broadcast, not addressed to a node ID -- every VESC on
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
| 0     | Front-left  | `drive_front_left`  | 1 |
| 1     | Front-right | `drive_front_right` | 2 |
| 2     | Rear-left   | `drive_rear_left`   | 3 |
| 3     | Rear-right  | `drive_rear_right`  | 4 |

**`esc_index`/`actuator_id` 0 is deliberately unused** -- 1-8 covers all 8 VESCs (4 drive + 4
steering) so no VESC is ever left at the field's power-on-default-looking value of 0, keeping
every physically-installed unit's assignment an explicit, deliberate choice. (This is distinct
from the node ID table's `0` rule above, which is a hard UAVCAN protocol reservation --
`esc_index`/`actuator_id` has no such protocol restriction, this is purely rp1's own convention.)

Each VESC's `esc_index` (set in VESC Tool, not its DroneCAN node ID) must match this table, and
must match the `esc_index` parameter on the corresponding joint in
`rp1_description/urdf/rp1_drive.urdf`. There is no separate translation layer -- the URDF
joint carries the index directly.

**Spin direction (`m_invert_direction`, bench-confirmed 2026-09-06):** verified on the bench by
pulsing each wheel individually with a small positive command and observing whether it drives
the robot forward. Left and right side wheels are expected to visually spin in *opposite*
directions for both to drive forward (mirrored mounting) -- judge each wheel by "does it drive
forward," not by matching rotation direction across sides.

| Wheel | `m_invert_direction` |
|-------|------------------------|
| Front-left | 0 (default, already correct) |
| Front-right | **1** -- spun in reverse at the default (0); flipping this VESC-Tool-side flag corrects it without touching phase wiring |
| Rear-left | 0 (default, already correct) |
| Rear-right | **1** -- same fix as front-right |

**Drive motor commutation/speed feedback sensor**: each drive VESC has both an AB (2-channel)
encoder and a 3-Hall-sensor setup available to wire up, but only one can be selected as the
active feedback sensor at a time (a per-motor VESC Tool FOC configuration choice, `foc_sensor_mode`).
**Decision (2026-07-30): use the Hall sensors.**

**Correction (2026-08-06): confirmed on the bench that the Hall sensor connector is NOT plugged
in on any of the 4 drive motors** -- the "near-certainly already factory-wired" assumption below
was wrong, or the connector was disconnected at some point since. All 4 are therefore currently
running on sensorless-only startup, not the intended Hall+sensorless hybrid. This plausibly
explains the elevated current draw all 4 wheels showed on their first full-pipeline `RPMCommand`
spin (a plain duty-cycle `RawCommand` pulse -- see the VESC UAVCAN configuration section's
`s_pid_min_erpm` note above -- doesn't go through the speed-PID's startup ramp the same way, so
`tools/can_vesc_test.py pulse` wouldn't have surfaced this) -- sensorless FOC's open-loop startup
ramp draws more current than a Hall-commutated start, and one wheel (rear-left, esc_index 3)
drew dramatically more than the other three (56A, then 16A after reseating an unrelated loose
phase connector on that unit -- still ~15x the other wheels' ~1A for a comparable commanded
speed). Wiring up the Hall connector on all 4 per the original decision is still the intended
end state and hasn't been done yet -- deliberately deferred for now (2026-08-06 decision:
continue bring-up on sensorless-only startup, since no overheating has been observed and it
does spin all 4 wheels; revisit Hall wiring before trusting this under real driving load/duty
cycle, not just a bench pulse test).

**Superseded (2026-09-06): moved to the AB encoder instead** (`FOC_SENSOR_MODE_ENCODER_AB`) --
confirmed live on the bench VESCs (e.g. Rear-left drive reading `foc_sensor_mode = 9`,
`m_encoder_counts = 16348`), bypassing the Hall-wiring question above entirely rather than
resolving it. Both the original Hall rationale and the 2026-08-06 sensorless-interim rationale
below are kept for history; neither reflects the current bench config.
- ~~The drive units are sealed hub motors (`ZLLG16ASM800`) -- Hall sensors are near-certainly
  already factory-wired inside; retrofitting an external AB encoder onto an already-sealed hub
  motor would need mechanical rework that may not even be possible without a custom part.~~
- The drive path only needs velocity, never position -- `diff_drive_controller` runs with
  `position_feedback: false` and integrates position from velocity already (see below); this
  still holds with the AB encoder, it just supplies a higher-resolution velocity estimate than
  Hall or sensorless would have.
- ~~rp1 is an outdoor ag-robot (dust/moisture/vibration) -- Hall sensors are simple digital
  switches with generous alignment tolerance; an encoder needs tighter mechanical alignment and
  is more failure-prone in that environment.~~
- ~~Hall + sensorless hybrid FOC is VESC's most mature, most widely-used sensor mode, with better
  standstill/low-speed startup behavior than sensorless alone, without encoder-commutation setup
  complexity.~~

This VESC firmware's `enc_abi_init()` skips arming the index-pulse EXTI interrupt entirely in
`FOC_SENSOR_MODE_ENCODER_AB` (no physical index line on a 2-channel AB encoder) -- see
`bldc/encoder/enc_abi.c`. Without that fix, the unconnected index pin's weak pull-up picked up
motor-driver switching noise as spurious index pulses, resetting the position count.

Doesn't change the DroneCAN wire protocol either way (`esc.Status.rpm` reporting is the same
regardless of which sensor feeds the VESC's internal FOC loop) -- this is a VESC Tool
configuration choice for whoever wires up the motors, not a software change. Unrelated to the
steering VESCs' ABZ encoder (see below) -- that's a separate 3-channel encoder for
absolute-ish position feedback, on a different actuator (`actuator_id`, not `esc_index`) with
its own FOC position-control needs (steering does need position, unlike drive).

**Shared VESC sensor port**: on this VESC hardware, Hall and encoder modes use the *same*
physical connector -- its pins are just reinterpreted depending on configuration: Hall mode
reads them as 3 Hall channels + motor temperature; encoder mode reads the same pins as A/B/Z +
temperature. With the drive VESCs now also in AB encoder mode, drive and steering VESCs use an
identical port type and a similar encoder configuration, just wired/tuned per motor -- not two
different connectors.

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
  UAVCAN-only node ID field. Must be unique on the bus, and **must not be 0** -- VESC's own
  field accepts 0-253, but 0 is DroneCAN's reserved anonymous address (see the node ID table
  above). Use 1-127.
- **`esc_index`** (App Settings -> General -> UAVCAN ESC index, the `can_esc_index` param,
  range 0-255) -- the value from the wheel index table above. **Use 1-8, never 0** (see that
  table's note).

This is a bench/setup-time activity done over VESC Tool's own USB link -- the runtime control
path never touches VESC Tool.

`uavcan.equipment.esc.RawCommand`'s `cmd` value maps to VESC's commanded duty cycle: confirmed
in firmware 7.00 as `raw_val = cmd.data[esc_index] / 8192.0` (int14 range -8192..8191 ->
-1.0..1.0 duty), see `libcanard/canard_driver.c` around the `RawCommand` handler. Re-confirm
against `/home/user/dev/bldc` if the firmware version changes.

**Required per drive VESC: lower "Minimum ERPM" (`s_pid_min_erpm`) below the robot's real
operating range.** Discovered 2026-08-06: with all 4 drive VESCs at firmware's default
`s_pid_min_erpm` (900), full-stick teleop (`rp1_teleop`'s `scale_linear: 1.0` m/s) never spins
any wheel, even though `tools/can_vesc_test.py listen` confirms `esc.RPMCommand` frames reach
the bus with the right per-wheel ERPM values (peaking ~310 ERPM at full stick, computed as wheel
rad/s x `kRadPerSecToRpm` x `gear_ratio` x `motor_pole_pairs`, matching what
`vesc_dronecan_driver` actually sends). Root cause is firmware, not this repo:
`mcpwm_foc_set_pid_speed()` (`bldc/motor/mcpwm_foc.c`) only transitions the motor into
`MC_STATE_RUNNING` when `fabsf(rpm) >= motor->m_conf->s_pid_min_erpm` -- below that ERPM the
speed-PID stays disabled and the motor never outputs any current, however long the command is
held. This gate is specific to the closed-loop speed-command path
(`uavcan.equipment.esc.RPMCommand` -> `mc_interface_set_pid_speed()` -> here) -- it does not
exist on the open-loop duty-cycle path (`RawCommand` -> `mc_interface_set_duty()`), which is why
`tools/can_vesc_test.py pulse` spun all 4 wheels individually with no issue while the full
ros2_control pipeline (which only ever sends `RPMCommand`) span none of them. Fix: VESC Tool ->
Motor Settings -> FOC -> Speed Controller -> **Minimum ERPM**, set well below this robot's
commanded range (e.g. 0-50) on all 4 drive VESCs. **Applied 2026-08-06** to all 4 -- confirmed
3 of 4 wheels (front-left, front-right, rear-right) now spin correctly under the full
ros2_control pipeline. Rear-left (esc_index 3) is still not right after the fix -- see the
Hall-sensor correction above and the open issue immediately below.

**Open issue (2026-08-06): rear-left (esc_index 3) still misbehaves after the above fixes.**
It briefly drew 56A during its first `RPMCommand` test (all other wheels ~1A); a loose phase
connector was found and reseated, dropping that to 16A, still ~15x the other wheels for a
comparable commanded speed. An isolated duty-cycle `pulse` test afterward showed the rpm
oscillating (0->30s->0 repeatedly) under a *constant* commanded duty, with current spiking
(up to 8A) and briefly going negative -- the signature of the motor repeatedly losing
commutation lock and restarting its startup ramp, not a clean spin-up. `esc.Status.error_count`
(firmware's live fault code) stayed `0`/`NONE` throughout, which weighs against a hard/detected
ESC fault (over-current, DRV, gate-driver, over-voltage -- those set a nonzero code) but doesn't
rule out subtler hardware degradation (partial MOSFET/gate-driver damage, current-sensor drift)
possibly caused by that initial 56A event. Leading hypothesis: this unit's FOC motor detection
(resistance/inductance/flux-linkage, VESC Tool's Motor Settings -> FOC -> Detect wizard) was run
while the connector was loose and is now stale/wrong -- re-running detection with the connector
properly seated is the next step. If that doesn't resolve it, swap-test VESC 3 and VESC 4
(same motors/wiring, swap which ESC drives which) to tell a bad ESC from a bad motor/wiring:
if the erratic behavior follows the ESC, it's the VESC; if it stays with the rear-left motor,
it isn't. Not yet resolved as of this writing.

## Steering actuator convention (ahead of the MVP's "no steering joints" phasing)

Firmware 7.00 (`add-actuator-arraycommand` branch, `/home/user/dev/bldc` commit `a242b9ae`)
also implements `uavcan.equipment.actuator.ArrayCommand`/`Status` for position-controlled
steering, reusing the *same* `uavcan_esc_index` VESC Tool field as the message's `actuator_id`
(no separate config field) -- so a given VESC is either a drive wheel (`esc.RawCommand`/`Status`
at its `esc_index`) or a steering actuator (`actuator.ArrayCommand`/`Status` at that same
numeric value used as `actuator_id`), depending only on how the PC side addresses it.

**Convention: steering `actuator_id` = drive `esc_index` + 4.**

| Wheel index | Drive `esc_index` | Steering `actuator_id` |
|-------------|--------------------|--------------------------|
| 0 (Front-left)  | 1 | 5 |
| 1 (Front-right) | 2 | 6 |
| 2 (Rear-left)   | 3 | 7 |
| 3 (Rear-right)  | 4 | 8 |

All 8 VESCs (4 drive + 4 steering) are now physically installed on the robot -- this is no
longer a bench-only subset. `tools/can_vesc_test.py listen` confirms all 8 esc_index values
(1-8) present on the bus with no collisions and no anonymous node IDs (`tools/can_vesc_gui.py`
gives the same per-esc_index/node_id/UUID/rpm/voltage/current/temp view as a live-updating
table instead of a fixed-duration snapshot). The 4 drive wheels (esc_index 1-4) are further
confirmed spinning correctly in the right direction via `tools/can_vesc_test.py pulse`, one
wheel at a time, wheels off the ground (2026-08-06) -- `wiring.md`'s bring-up steps 2-3. The 4
steering actuators (actuator_id 5-8) are bus-present but not yet exercised with a real
`actuator.ArrayCommand` -- firmware support for that message is on a non-mainline branch (see
below), so there is no equivalent pulse test for them yet.

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
