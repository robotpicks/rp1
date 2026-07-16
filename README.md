# rp1 — Agricultural Robot

4-wheel agricultural robot. Each wheel is driven by a VESC-based motor controller and (in the
full build) sits on a continuous-rotation (360°) steering joint — a swerve-drive base. This
repo currently targets the **MVP: the 4 drive wheels only, skid-steer, no steering joints yet**.

## Architecture

VESC firmware has a built-in UAVCAN/DroneCAN CAN mode, so there is **no separate bridge MCU**:
the PC talks DroneCAN directly to the 4 VESCs over a single CAN bus.

```
[Xbox Series X controller]
        | (joy)
        v
   ROS2 (Kilted, on Ubuntu 24.04 in this environment; Jazzy-compatible)
   joy_node -> rp1_teleop -> /cmd_vel -> rp1_control -> /wheel_cmd (per wheel)
        |
        v
   rp1_dronecan_bridge (DroneCAN over SocketCAN, e.g. a CANable/candleLight adapter)
        |
        v
   VESC #1..#4 (CAN mode: UAVCAN) -> drive motors
     esc.RawCommand in (per-wheel duty), esc.Status out (rpm/voltage/current)
```

DroneCAN is the runtime transport from the PC to the robot. VESC USB (VESC Tool) is used only
for one-off per-motor configuration (CAN mode, node ID, esc_index, FOC tuning), never in the
runtime control path. See `docs/can_id_map.md` and `docs/wiring.md` for details.

## Control hierarchy

Movement commands flow through the same layers whether they come from a human (Xbox
controller) or, later, an autonomy stack -- everything below `/cmd_vel` doesn't care which one
it was. Today (MVP) there's no steering, only the 4 drive wheels; the steering-joint row below
is the planned extension, not yet implemented.

| Layer | What it decides | Node(s) | Message |
|-------|------------------|---------|---------|
| Input | Human/autonomy intent: "go this fast, turn this fast" | `rp1_teleop` (`teleop_node.py`) | `sensor_msgs/Joy` → `geometry_msgs/Twist` on `/cmd_vel` |
| Kinematics | Robot-frame velocity → what each individual actuator must do | `rp1_control` (`control_node.py`) | `/cmd_vel` → `rp1_msgs/WheelCommand` on `/wheel_cmd` |
| Transport | Per-actuator setpoints → wire format understood by the hardware | `rp1_dronecan_bridge` (or `rp1_sim` for testing) | `WheelCommand` → DroneCAN `esc.RawCommand` |
| Actuator | Actually spins/turns | VESC (drive), future steering actuator | motor duty / (future) joint position |

**The 4 drive wheels (implemented):** `rp1_control` runs skid-steer kinematics --
`v_left = v - w*track_width/2`, `v_right = v + w*track_width/2` -- and publishes one normalized
velocity setpoint per wheel in `WheelCommand`. Drive wheels are **velocity-controlled**: there's
no target angle to hold, so no position feedback loop is needed at this layer; `WheelFeedback`
(rpm/voltage/current, from DroneCAN `esc.Status`) is telemetry, not something `rp1_control`
closes a loop on.

**The 4 steering joints (planned, see `docs/can_id_map.md`):** once added, `rp1_control`'s
kinematics become full swerve inverse kinematics -- each wheel gets an independent (speed,
angle) pair computed from the same `/cmd_vel`, instead of just a speed. Steering joints are
**position-controlled** (continuous/360°, so angle wrap has to be handled explicitly), which is
a different control problem from the drive wheels: each joint needs its own closed loop holding
a commanded angle, fed by DroneCAN `uavcan.equipment.actuator.ArrayCommand`/`Status` instead of
`esc.RawCommand`/`Status`. Whether that loop closes on the VESC itself (position/FOC mode) or on
a separate servo/stepper + encoder is an open decision (see `docs/can_id_map.md`) -- either way,
it sits at the same "Transport"/"Actuator" layers as the drive wheels, just a parallel path
through `rp1_dronecan_bridge` rather than a new layer above `rp1_control`.

## Repo layout

- `ros2_ws/` — ROS2 colcon workspace.
  - `rp1_msgs` — `WheelCommand` / `WheelFeedback` interfaces.
  - `rp1_teleop` — Xbox Series X → `/cmd_vel`.
  - `rp1_control` — `/cmd_vel` → per-wheel skid-steer setpoints.
  - `rp1_dronecan_bridge` — ROS2 ⇄ DroneCAN bridge node (PC side).
  - `rp1_sim` — pure-ROS2 stand-in for `rp1_dronecan_bridge` (no CAN at all): simulates wheel
    dynamics + odometry so the teleop/control graph can be checked with zero hardware.
  - `rp1_bringup` — launch files + params tying the above together.
- `docs/` — CAN id map, wiring notes.
- `simulation/` — bonus: a virtual-CAN DroneCAN node that pretends to be the 4 VESCs, for
  exercising the real `rp1_dronecan_bridge` (not `rp1_sim`) with no hardware. See
  `simulation/README.md`.

## Build

```bash
source /opt/ros/kilted/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run (MVP teleop, once hardware is wired up)

```bash
ros2 launch rp1_bringup rp1_mvp.launch.py
```

## Run without hardware

Check the ROS2 architecture itself (teleop -> control -> odometry), no CAN involved:
```bash
ros2 launch rp1_bringup rp1_mvp_sim.launch.py
```

Same, plus rviz2 pre-configured to show odometry/TF (fixed frame `odom`) so you can watch it
move (drive it via `/cmd_vel` if no controller is attached):
```bash
ros2 launch rp1_bringup rp1_mvp_sim_rviz.launch.py
```

Bonus: exercise the real `rp1_dronecan_bridge` (DroneCAN wire format and all) over a virtual
CAN bus instead — see `simulation/README.md`.

## Status

MVP scaffold in progress — see `docs/wiring.md` for the bring-up sequence (configure each
VESC's UAVCAN settings before wiring it into the shared bus; wheels-off-ground teleop test
before driving on the ground).

## Roadmap (not yet designed)

- **4 steering joints** (swerve) — see "Control hierarchy" above and `docs/can_id_map.md`.
- **ExpressLRS (ELRS)-based RC controller** as an alternative/additional remote control input
  to the Xbox Series X controller — likely valuable for long-range control and as an
  independent failsafe path from the ROS2/DroneCAN control loop.
- **OpenIPC-based IP camera** for vision/FPV.
