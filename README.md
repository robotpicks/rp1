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

Bonus: exercise the real `rp1_dronecan_bridge` (DroneCAN wire format and all) over a virtual
CAN bus instead — see `simulation/README.md`.

## Status

MVP scaffold in progress — see `docs/wiring.md` for the bring-up sequence (configure each
VESC's UAVCAN settings before wiring it into the shared bus; wheels-off-ground teleop test
before driving on the ground).
