# rp1 — Agricultural Robot

4-wheel agricultural robot. Each wheel is driven by a VESC-based motor controller and (in the
full build) sits on a continuous-rotation (360°) steering joint — a swerve-drive base. This
repo currently targets the **MVP: the 4 drive wheels only, skid-steer, no steering joints yet**.
See `/home/user/.claude/plans/merry-sauteeing-storm.md` for the full phased plan.

## Architecture

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
   [Bridge MCU: custom firmware, 2x CAN]
     CAN1: DroneCAN <-> PC        (esc.RawCommand in, esc.Status out)
     CAN2: VESC-CAN master        (Set Duty/Current per VESC; reads VESC status frames)
        |
        v
   VESC #1..#4 -> drive motors
```

DroneCAN is the runtime transport from the PC to the robot. VESC USB/CAN (VESC Tool) is used
only for one-off per-motor configuration/tuning, never in the runtime control path.

## Repo layout

- `firmware/bridge_node/` — DroneCAN ⇄ VESC-CAN bridge firmware (STM32, two CAN controllers).
- `ros2_ws/` — ROS2 colcon workspace.
  - `rp1_msgs` — `WheelCommand` / `WheelFeedback` interfaces.
  - `rp1_teleop` — Xbox Series X → `/cmd_vel`.
  - `rp1_control` — `/cmd_vel` → per-wheel skid-steer setpoints.
  - `rp1_dronecan_bridge` — ROS2 ⇄ DroneCAN bridge node (PC side).
  - `rp1_bringup` — launch files + params tying the above together.
- `docs/` — CAN id map, wiring notes.

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

## Status

MVP scaffold in progress — see the plan doc for the bring-up sequence (bench test the bridge
firmware before connecting ROS2; wheels-off-ground teleop test before driving on the ground).
