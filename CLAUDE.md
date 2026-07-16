# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

rp1 is a 4-wheel agricultural robot. End-state hardware: each wheel has its own VESC-driven
drive motor **and** a continuous-rotation (360°) steering joint (a swerve-drive base). This
repo currently targets the **MVP: the 4 drive wheels only, skid-steer, no steering joints
yet** — see `docs/can_id_map.md`, `docs/wiring.md`, and `firmware/bridge_node/README.md` for
the phased plan and open hardware decisions.

Communication architecture (PC → robot):

```
[Xbox Series X controller] --joy--> ROS2 (rp1_teleop -> /cmd_vel -> rp1_control -> /wheel_cmd)
        -> rp1_dronecan_bridge (DroneCAN over SocketCAN, e.g. a CANable/candleLight adapter)
        -> [Bridge MCU: custom firmware, not yet built] -> VESC-CAN -> VESC #1-4 -> motors
```

- **DroneCAN is the runtime transport** from the PC to the robot (via `rp1_dronecan_bridge`).
- **VESC's own USB/CAN protocol (VESC Tool) is only used for one-off per-motor configuration**
  (setting CAN controller IDs, FOC tuning) — never in the runtime control path.
- The bridge MCU (DroneCAN ⇄ VESC-CAN translator) is planned but **not implemented** —
  `rp1_dronecan_bridge` can run today in `require_can:=false` dry-run mode against the rest of
  the ROS2 pipeline without it.

## Build / run

ROS2 environment in this sandbox is **Kilted** (`/opt/ros/kilted`); the project was designed
against Jazzy but nothing here is Jazzy-specific.

```bash
source /opt/ros/kilted/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Run the full MVP teleop pipeline:
```bash
ros2 launch rp1_bringup rp1_mvp.launch.py
```

Run just the teleop leg (joy -> /cmd_vel) for isolated testing:
```bash
ros2 launch rp1_teleop teleop.launch.py
```

Run the DroneCAN bridge without CAN hardware attached (logs intended frames instead of sending):
```bash
ros2 run rp1_dronecan_bridge bridge_node --ros-args -p require_can:=false
```

There is no test suite yet. Verification so far has been manual: `colcon build`, then running
nodes and publishing synthetic `/cmd_vel` / `/joy` messages and checking `/wheel_cmd`. When
using `ros2 topic echo`/`hz` piped through `head`/`tail` or wrapped in `timeout`, this sandbox
has shown flaky hangs/false failures — redirect to a file and inspect it instead of piping, and
prefer a small standalone `rclpy` subscriber script over `ros2 topic echo` if a CLI check seems
to hang without an obvious code reason.

Python deps for the DroneCAN bridge (`dronecan`, `python-can`) are not ROS/apt packages — they
were installed with `pip install --user --break-system-packages dronecan python-can` (see
`ros2_ws/src/rp1_dronecan_bridge/requirements.txt`). This is a system-managed (PEP 668) Python
environment, so plain `pip install` will refuse; passwordless `sudo` is not available in this
sandbox.

## Architecture

### Workspace layout

- `firmware/bridge_node/` — planned DroneCAN ⇄ VESC-CAN bridge firmware (STM32, two CAN
  controllers: CAN1 to the PC, CAN2 as VESC-CAN master to the 4 VESCs). Not implemented yet;
  see its README for the bring-up plan.
- `ros2_ws/src/` — colcon workspace, one package per pipeline stage (see below).
- `docs/can_id_map.md` — the source of truth for DroneCAN node IDs, the wheel-index ↔
  `esc_index` ↔ VESC controller ID mapping, and the (not-yet-chosen) VESC CAN command type.
  Both the PC-side bridge node and the future MCU firmware must agree with this file.
- `docs/wiring.md` — bus wiring, VESC one-off configuration notes, controller pairing, and the
  hardware bring-up order (bench-test the bridge firmware before ROS2; wheels off the ground
  before driving).

### ROS2 packages (`ros2_ws/src/`) and topic flow

```
/joy --[rp1_teleop]--> /cmd_vel --[rp1_control]--> /wheel_cmd --[rp1_dronecan_bridge]--> DroneCAN
                                                                  DroneCAN --> /wheel_feedback
```

- **`rp1_msgs`** — the only `ament_cmake`/`rosidl` package. Defines `WheelCommand` (per-wheel
  normalized `[-1, 1]` setpoint) and `WheelFeedback` (per-wheel rpm/voltage/current). Both use
  the same wheel-index constants (`WHEEL_FRONT_LEFT=0, WHEEL_FRONT_RIGHT=1, WHEEL_REAR_LEFT=2,
  WHEEL_REAR_RIGHT=3`) — this indexing is used directly as the DroneCAN `esc_index` downstream,
  so don't introduce a second translation layer for it without updating `docs/can_id_map.md`.
- **`rp1_teleop`** (`teleop_node.py`) — Xbox Series X mapping is config, not hardcoded
  (`config/joy_xbox_series_x.yaml`): axis/button indices, scale, deadman button, joy-timeout.
  Only publishes non-zero `Twist` while the deadman button is held; zeroes output if `/joy`
  goes stale. Axis/button numbering is the common Linux `joy_node` (xpad) convention but is
  **unverified against real hardware** — check with `ros2 topic echo /joy` or `jstest` first.
- **`rp1_control`** (`control_node.py`) — skid-steer kinematics only (no steering joints yet):
  `v_left = v - w*track_width/2`, `v_right = v + w*track_width/2`, both sides scaled by
  `max_wheel_speed` into the normalized WheelCommand range. If either side would exceed
  `[-1, 1]`, **both** are scaled down together (see `_publish`) to preserve the requested turn
  ratio rather than clipping independently — preserve this behavior if you touch the scaling.
  Has its own `/cmd_vel` staleness watchdog independent of rp1_teleop's.
- **`rp1_dronecan_bridge`** (`bridge_node.py`) — the PC-side half of the DroneCAN bridge (the
  MCU-side half lives in `firmware/bridge_node/`, not yet built). Uses the `dronecan` python
  library over a SocketCAN interface (`can_iface` param, default `can0`). Pumps
  `dronecan_node.spin(timeout=0)` on a fast ROS2 timer to process incoming CAN frames
  alongside broadcasting `esc.RawCommand` at `command_rate_hz`. `require_can: false` skips
  opening a CAN device entirely and just logs what would be sent — the way to exercise this
  node without the (not-yet-built) bridge MCU or any CAN adapter attached.
- **`rp1_bringup`** — no code, just `launch/rp1_mvp.launch.py` and `config/rp1_mvp.yaml`. Each
  node's parameters are layered: that node's own package `config/*.yaml` defaults first, then
  `rp1_bringup`'s `rp1_mvp.yaml` on top for the handful of robot-specific values (track width,
  CAN interface, deadman button) — add new tunables to the owning package's default config, and
  only add an override here if it's genuinely robot-specific/likely to change per deployment.

### Extending to full swerve (steering joints) — not yet started

When steering joints are added, the intended shape of the change (see `docs/can_id_map.md` and
the firmware README) is: extend the bridge firmware to also handle
`uavcan.equipment.actuator.ArrayCommand`/`Status`, and replace `rp1_control`'s skid-steer
kinematics with full swerve inverse kinematics (per-wheel speed *and* angle, including wheel
angle wrap for continuous joints). The steering actuator type (VESC position/FOC mode vs. a
separate servo/stepper + encoder) is an open decision, deliberately deferred so it doesn't box
in the MVP.
