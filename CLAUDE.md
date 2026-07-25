# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

rp1 is a 4-wheel agricultural robot. End-state hardware: each wheel has its own VESC-driven
drive motor **and** a continuous-rotation (360°) steering joint (a swerve-drive base). This
repo currently targets the **MVP: the 4 drive wheels only, skid-steer, no steering joints
yet** — see `docs/can_id_map.md` and `docs/wiring.md` for the phased plan and open hardware
decisions.

Communication architecture (PC → robot):

```
[Xbox Series X controller] --USB--> joy_node -----\
[ExpressLRS radio] --CRSF/UART--> elrs_driver ----->--joy--> ROS2 (rp1_teleop -> /cmd_vel ->
        rp1_control -> /wheel_cmd)
        -> rp1_dronecan_bridge (DroneCAN over SocketCAN, e.g. a CANable/candleLight adapter)
        -> VESC #1-4 (CAN mode: VESC+UAVCAN) -> motors
```

Two interchangeable RC inputs feed the same `/joy` topic: the Xbox pad via `joy_node`, or an
ExpressLRS radio via `elrs_driver` (CRSF over UART). Pick one per bringup (`rp1_mvp.launch.py` vs
`rp1_mvp_elrs.launch.py`) -- don't run both at once.

`elrs_driver` is **not in this repo**: it was extracted to the robot-agnostic `elrs_ros`
repository (a sibling submodule of the `robotpicks` meta repo). What stays here is `rp1_elrs`, the
rp1-specific glue: the `/wheel_feedback` -> `/battery` telemetry adapter plus rp1's ELRS configs.
Build both together with `robotpicks.sh build rp1`, or point colcon at both trees:
`colcon build --base-paths src ../../elrs_ros`.

- **DroneCAN is the runtime transport** from the PC to the robot (via `rp1_dronecan_bridge`).
- **There is no separate bridge MCU.** VESC firmware has a built-in UAVCAN/DroneCAN CAN mode,
  so the PC's CAN adapter wires directly to a single bus carrying the 4 VESCs. (An earlier plan
  revision assumed a custom-firmware bridge MCU translating DroneCAN↔VESC-CAN; that's been
  dropped now that VESC's native UAVCAN mode covers it — don't resurrect that architecture.)
- **VESC's own USB CAN-config protocol (VESC Tool) is only used for one-off per-motor
  configuration** (CAN mode, DroneCAN node ID, `esc_index`, FOC tuning) — never in the runtime
  control path.
- `rp1_dronecan_bridge` can run today in `require_can:=false` dry-run mode (logs intended
  frames instead of opening a CAN device), and there are two ways to run the whole pipeline
  with zero hardware — see "Simulation" below.
- `elrs_driver` (in the `elrs_ros` submodule) is an **optional alternative RC input**: an
  ExpressLRS receiver's CRSF stream over a UART, republished as `/joy` (so `rp1_teleop` is reused
  unchanged), with battery telemetry pushed back to the handset. It has the same
  `require_serial:=false` dry-run escape hatch as the bridge. Only standard CRSF is implemented;
  **MAVLink-over-ELRS is a deliberately-deferred Phase-2 mode** — the wire protocol lives in
  `elrs_driver/crsf.py` specifically so a MAVLink codec can be added beside it (and a UDP-to-GCS
  sink is an independent later add). Don't build the MAVLink path into the CRSF path.

## Build / run

ROS2 environment in this sandbox is **Lyrical** (`/opt/ros/lyrical`), with `ros2-control`
installed; the project was designed against Jazzy but nothing here is Jazzy-specific. Don't
hardcode the distro — `robotpicks.sh` picks it up from `ROS_DISTRO` or the first `/opt/ros/*`.

```bash
source /opt/ros/lyrical/setup.bash
cd ros2_ws
colcon build --symlink-install --base-paths src ../../elrs_ros   # elrs_driver lives outside src
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

Run the whole pipeline against `rp1_sim` (pure ROS2, no CAN at all — the main way to check the
ROS2 architecture itself):
```bash
ros2 launch rp1_bringup rp1_mvp_sim.launch.py
```

The only automated tests are `elrs_driver`'s pure-python codec/link tests (10 of them, in the
`elrs_ros` submodule); no rp1 package has tests of its own yet. `colcon test` runs them via its
pytest step, which it selects from a `extras_require={'test': ['pytest']}` entry in each
`setup.py` — the older `tests_require=['pytest']` spelling is silently dropped by current
setuptools, which makes colcon fall back to unittest and report every Python package as failed
with "NO TESTS RAN" (exit 5). Keep the `extras_require` form. Beyond that, verification is
manual: `colcon build`, then running nodes and publishing synthetic `/cmd_vel` / `/joy` messages
and checking `/wheel_cmd`. When
using `ros2 topic echo`/`hz` piped through `head`/`tail` or wrapped in `timeout`, this sandbox
has shown flaky hangs/false failures — redirect to a file and inspect it instead of piping, and
prefer a small standalone `rclpy` subscriber script over `ros2 topic echo` if a CLI check seems
to hang without an obvious code reason.

The installed `dronecan` (1.0.27) + `python-can` (4.6.1) pair has two real bugs, both worked
around in `bridge_node.py` and `simulation/sim_vesc_node.py` — don't remove these if you see
them and think they look unnecessary:
- `dronecan`'s python-can driver calls `bus.flush_tx_buffer()` after every send; python-can ≥4
  doesn't implement it for any interface (`NotImplementedError`), which kills the writer thread
  on the first broadcast. Worked around by monkeypatching `can.bus.BusABC.flush_tx_buffer` to a
  no-op (`_patch_python_can_flush_tx_buffer()` in both files).
- `dronecan`'s `receive()` multiplies any `spin(timeout=X)` by 1000 before passing it to
  python-can's `recv()`, which wants seconds, not ms — so `spin(timeout=0.1)` blocks for ~100s
  instead of 100ms. Only `spin(timeout=0)` is safe; both files busy-poll with `timeout=0` plus a
  short `sleep`/ROS2 timer instead of relying on a blocking nonzero timeout.

Python deps for the DroneCAN bridge (`dronecan`, `python-can`) are not ROS/apt packages and are
**not currently installed** — `bridge_node` therefore dies with `ModuleNotFoundError: dronecan`
unless run with `require_can:=false`. Install with
`pip install --user --break-system-packages dronecan python-can` (see
`ros2_ws/src/rp1_dronecan_bridge/requirements.txt`). This is a system-managed (PEP 668) Python
environment, so plain `pip install` will refuse; passwordless `sudo` is not available in this
sandbox. `elrs_driver` needs `pyserial` the same way (`elrs_ros/elrs_driver/requirements.txt`,
already installed); like `dronecan` in the bridge, it's imported lazily so the package still
builds/dry-runs without it.

## Architecture

### Workspace layout

- `ros2_ws/src/` — colcon workspace, one package per pipeline stage (see below).
- `docs/can_id_map.md` — the source of truth for DroneCAN node IDs and the wheel-index ↔
  `esc_index` mapping. `rp1_dronecan_bridge` and each VESC's UAVCAN config (set via VESC Tool)
  must agree with this file.
- `docs/wiring.md` — bus wiring, VESC one-off configuration notes, controller pairing, and the
  hardware bring-up order (configure each VESC's UAVCAN settings before sharing the bus; wheels
  off the ground before driving).
- `simulation/` — bonus DroneCAN-level simulator (virtual CAN + fake VESCs), separate from
  `rp1_sim`. See its own section below.

### ROS2 packages (`ros2_ws/src/`) and topic flow

```
/joy --[rp1_teleop]--> /cmd_vel --[rp1_control]--> /wheel_cmd --[rp1_dronecan_bridge]--> DroneCAN --> VESCs
                                                       (or)--[rp1_sim]--> /odom, /tf          DroneCAN --> /wheel_feedback
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
- **`rp1_elrs`** (`wheel_feedback_to_battery.py`) — rp1's glue for the optional ExpressLRS RC
  input. The driver itself is **`elrs_driver`, in the `elrs_ros` submodule, not here**: it reads
  the CRSF stream from a serial port (`pyserial`, `serial_port`/`baud` params) via the
  ROS-independent codec in its `crsf.py`, unpacks `RC_CHANNELS_PACKED`, and publishes
  `sensor_msgs/Joy` (an arm/switch channel mapped to Joy **button 4** so `rp1_teleop`'s
  `deadman_button: 4` gate works unchanged — see `config/joy_elrs.yaml`); RC-loss failsafe
  publishes a neutral `Joy`. Its structure mirrors `bridge_node.py` (lazy transport import,
  `require_serial:=false` dry-run, timer-pumped serial drain, keep-alive try/except), and it holds
  the repo's only unit tests. What `rp1_elrs` contributes is the reverse direction: the
  `wheel_feedback_to_battery` adapter turns `/wheel_feedback` into the `sensor_msgs/BatteryState`
  on `/battery` that `elrs_driver` forwards to the handset as CRSF `BATTERY_SENSOR` — keeping the
  driver robot-agnostic — plus rp1's ELRS configs (`config/rp1_elrs.yaml`, `config/joy_elrs.yaml`).
  The full pipeline needs **both** nodes (see `rp1_mvp_elrs.launch.py`). **CRSF only; MAVLink is
  Phase-2** (see the "What this is" note above).
- **`rp1_control`** (`control_node.py`) — skid-steer kinematics only (no steering joints yet):
  `v_left = v - w*track_width/2`, `v_right = v + w*track_width/2`, both sides scaled by
  `max_wheel_speed` into the normalized WheelCommand range. If either side would exceed
  `[-1, 1]`, **both** are scaled down together (see `_publish`) to preserve the requested turn
  ratio rather than clipping independently — preserve this behavior if you touch the scaling.
  Has its own `/cmd_vel` staleness watchdog independent of rp1_teleop's.
- **`rp1_dronecan_bridge`** (`bridge_node.py`) — talks DroneCAN directly to the VESCs (no
  separate bridge MCU, see "What this is" above). Uses the `dronecan` python library over a
  SocketCAN interface (`can_iface` param, default `can0`). Pumps `dronecan_node.spin(timeout=0)`
  on a fast ROS2 timer to process incoming CAN frames alongside broadcasting `esc.RawCommand`
  at `command_rate_hz`. `require_can: false` skips opening a CAN device entirely and just logs
  what would be sent.
- **`rp1_sim`** (`sim_bridge_node.py`) — pure-ROS2 drop-in replacement for
  `rp1_dronecan_bridge` with **no CAN/DroneCAN involved at all**: subscribes the same
  `/wheel_cmd`, integrates skid-steer dynamics (must be kept in sync with `rp1_control`'s
  `track_width`/`max_wheel_speed` — see `config/rp1_sim.yaml`), and publishes `/wheel_feedback`
  plus `nav_msgs/Odometry` + an `odom`→`base_link` TF. The rpm/voltage/current numbers are a
  plausible telemetry shape, not a physically accurate motor model. This is the primary way to
  validate the ROS2 graph itself (e.g. with rviz2) before any CAN hardware exists.
- **`rp1_bringup`** — no code, just launch files + `config/rp1_mvp.yaml`. Launch files include
  `rp1_mvp.launch.py` (real hardware, ends in `rp1_dronecan_bridge`), `rp1_mvp_sim.launch.py`
  (identical joy/teleop/control stack, but ends in `rp1_sim` instead), and
  `rp1_mvp_elrs.launch.py` (like `rp1_mvp` but the `/joy` source is `elrs_driver` instead of
  `joy_node`, plus `rp1_elrs`'s `wheel_feedback_to_battery` for the telemetry direction). Each
  node's parameters are layered: that node's own package `config/*.yaml` defaults first, then
  `rp1_bringup`'s `rp1_mvp.yaml` on top for the handful of robot-specific values (track width, CAN
  interface, deadman button) — add new tunables to the owning package's default config, and only
  add an override here if it's genuinely robot-specific/likely to change per deployment. The
  per-node blocks in `rp1_mvp.yaml` are keyed by **node name**, so they must track the names used
  in the launch files (e.g. the ELRS block is `elrs_driver:`, matching the extracted driver's node
  name — a stale key is silently ignored, not an error).

### Simulation (two tiers)

- `rp1_sim` (see above) — pure ROS2, no CAN. Primary tool for checking the ROS2 architecture.
- `simulation/sim_vesc_node.py` (bonus) — a standalone script (no ROS2 dependency, since the
  real VESCs don't have one either) using `python-dronecan` over a Linux `vcan` interface,
  pretending to be the 4 VESCs. Use this to exercise the real `rp1_dronecan_bridge` — actual
  DroneCAN framing over a virtual CAN bus — without any physical hardware. See
  `simulation/README.md` for `vcan0` setup (needs `sudo`, not automatable in a sandboxed
  session).

### Extending to full swerve (steering joints) — not yet started

When steering joints are added, the intended shape of the change (see `docs/can_id_map.md`) is:
extend `rp1_dronecan_bridge` (and each steering actuator's DroneCAN config) to also handle
`uavcan.equipment.actuator.ArrayCommand`/`Status`, and replace `rp1_control`'s skid-steer
kinematics with full swerve inverse kinematics (per-wheel speed *and* angle, including wheel
angle wrap for continuous joints). The steering actuator type (VESC position/FOC mode vs. a
separate servo/stepper + encoder) is an open decision, deliberately deferred so it doesn't box
in the MVP.
