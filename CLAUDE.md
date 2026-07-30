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
[ExpressLRS radio] --CRSF/UART--> elrs_driver ----->--joy--> rp1_teleop -> /cmd_vel (TwistStamped)
        -> diff_drive_controller (skid-steer kinematics + odometry, stock ros2_controllers)
        -> ros2_control velocity command interfaces
        -> vesc_dronecan_driver (DroneCAN over SocketCAN, e.g. a CANable/candleLight adapter)
        -> VESC #1-4 (CAN mode: VESC+UAVCAN) -> motors
```

**The drive path is ros2_control.** `rp1_control` (hand-written skid-steer kinematics) and
`rp1_dronecan_bridge` (a Python DroneCAN node) both existed once and are both gone: the
kinematics, the `/cmd_vel` watchdog and the odometry are `diff_drive_controller`'s, and the
DroneCAN encoding moved into the hardware component's `read()`/`write()`. DroneCAN itself is
unchanged -- only the process that speaks it moved. Don't reintroduce a separate bridge node.

Two interchangeable RC inputs feed the same `/joy` topic: the Xbox pad via `joy_node`, or an
ExpressLRS radio via `elrs_driver` (CRSF over UART). Pick one per bringup (`rp1_mvp.launch.py` vs
`rp1_mvp_elrs.launch.py`) -- don't run both at once.

`elrs_driver` is **not in this repo**: it was extracted to the robot-agnostic `elrs_ros`
repository (a sibling submodule of the `robotpicks` meta repo). What stays here is `rp1_elrs`, the
rp1-specific glue: the `/dynamic_joint_states` -> `/battery` telemetry adapter plus rp1's ELRS
configs.
Build both together with `robotpicks.sh build rp1`, or point colcon at both trees:
`colcon build --base-paths src ../../elrs_ros`.

- **DroneCAN is the runtime transport** from the PC to the robot (via `vesc_dronecan_driver`).
- **There is no separate bridge MCU.** VESC firmware has a built-in UAVCAN/DroneCAN CAN mode,
  so the PC's CAN adapter wires directly to a single bus carrying the 4 VESCs. (An earlier plan
  revision assumed a custom-firmware bridge MCU translating DroneCAN↔VESC-CAN; that's been
  dropped now that VESC's native UAVCAN mode covers it — don't resurrect that architecture.)
- **VESC's own USB CAN-config protocol (VESC Tool) is only used for one-off per-motor
  configuration** (CAN mode, DroneCAN node ID, `esc_index`, FOC tuning) — never in the runtime
  control path.
- The whole pipeline runs with zero hardware via `use_mock:=true`, which swaps the hardware
  plugin for `mock_components/GenericSystem` — see "Simulation" below. (This replaced
  `rp1_dronecan_bridge`'s `require_can:=false` dry run and `rp1_sim`'s `sim_bridge_node`.)
- `elrs_driver` (in the `elrs_ros` submodule) is an **optional alternative RC input**: an
  ExpressLRS receiver's CRSF stream over a UART, republished as `/joy` (so `rp1_teleop` is reused
  unchanged), with battery telemetry pushed back to the handset. It has the same
  `require_serial:=false` dry-run escape hatch. Only standard CRSF is implemented;
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

Run the whole pipeline with no CAN hardware at all (mock hardware component — the main way to
check the ROS2 architecture itself):
```bash
ros2 launch rp1_bringup rp1_mvp.launch.py use_mock:=true
```

Point the real hardware component at a virtual bus, with nothing driving cmd_vel:
```bash
ros2 launch rp1_bringup rp1_mvp.launch.py can_iface:=vcan0 teleop:=false
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

Python deps are declared as rosdep keys in each `package.xml` and installed with
`./robotpicks.sh deps` (rosdep) from the meta repo — don't hand-roll `pip install` lines, and
don't add a dep without also declaring it in the manifest. `python3-can` and `python3-serial` are
stock keys resolving to distro packages; `dronecan` is packaged nowhere and has no upstream key,
so the meta repo's `rosdep/robotpicks.yaml` defines it and maps it to pip. That rule file needs
registering under `/etc/ros/rosdep/sources.list.d/` once (root); `robotpicks.sh deps` prints the
commands. This is a system-managed (PEP 668) Python environment, so a bare `pip install` refuses
and passwordless `sudo` is not available in this sandbox. All three libs are imported lazily, so
every package still builds and dry-runs (`require_can:=false` / `require_serial:=false`) without
them.

## Architecture

### Workspace layout

- `ros2_ws/src/` — colcon workspace, one package per pipeline stage (see below).
- `docs/can_id_map.md` — the source of truth for DroneCAN node IDs, the wheel-index ↔
  `esc_index` mapping, and the firmware's ERPM/mechanical-RPM asymmetry.
  `rp1_description`'s URDF and each VESC's UAVCAN config (set via VESC Tool) must agree
  with this file.
- `docs/wiring.md` — bus wiring, VESC one-off configuration notes, controller pairing, and the
  hardware bring-up order (configure each VESC's UAVCAN settings before sharing the bus; wheels
  off the ground before driving).
- `simulation/` — bonus DroneCAN-level simulator (virtual CAN + fake VESCs), separate from
  `rp1_sim`. See its own section below.

### ROS2 packages (`ros2_ws/src/`) and topic flow

```
/joy --[rp1_teleop]--> /diff_drive_controller/cmd_vel (TwistStamped)
     --[diff_drive_controller]--> velocity command interfaces --[vesc_dronecan_driver]--> DroneCAN --> VESCs
                                  /diff_drive_controller/odom + odom->base_link TF
     VESCs --> DroneCAN --> [vesc_dronecan_driver] --> /joint_states, /dynamic_joint_states
```

- **`rp1_msgs`** — the only `ament_cmake`/`rosidl` package. Defines `WheelCommand`,
  `WheelFeedback`, `SteeringCommand` and `SteeringFeedback`, all sharing the wheel-index
  constants (`WHEEL_FRONT_LEFT=0, WHEEL_FRONT_RIGHT=1, WHEEL_REAR_LEFT=2, WHEEL_REAR_RIGHT=3`).
  **The MVP drive path no longer uses these** — ros2_control carries commands and state over
  interfaces, not topics. They remain because `rp1_sim`'s `swerve_sim_node` is still built on
  them. Don't add them back into the hardware path.
- **`rp1_teleop`** (`teleop_node.py`) — Xbox Series X mapping is config, not hardcoded
  (`config/joy_xbox_series_x.yaml`): axis/button indices, scale, deadman button, joy-timeout.
  Only publishes non-zero `TwistStamped` while the deadman button is held; zeroes output if
  `/joy` goes stale. **TwistStamped, not Twist**: `diff_drive_controller` in this ROS 2 release
  subscribes to `TwistStamped` only, with no `use_stamped_vel` escape hatch. Axis/button numbering is the common Linux `joy_node` (xpad) convention but is
  **unverified against real hardware** — check with `ros2 topic echo /joy` or `jstest` first.
- **`rp1_elrs`** (`esc_telemetry_to_battery.py`) — rp1's glue for the optional ExpressLRS RC
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
- **`vesc_dronecan_driver`** — **not in this repo.** It lives in the `vesc_dronecan_ros`
  submodule (sibling of rp1 under the `robotpicks` meta repo, same arrangement as `elrs_driver`),
  because nothing about it is rp1-specific. `src/vesc_dronecan_system.cpp`, C++: the ros2_control
  `SystemInterface` that speaks DroneCAN to the VESCs, using a **vendored copy of libcanard**
  (`vendor/libcanard/`, the same codec the firmware uses) over a raw non-blocking SocketCAN
  socket. What stays here is `rp1_description`, whose URDFs name the plugin and carry rp1's
  geometry and id assignments. One component
  handles both wheel kinds on one bus and one node ID; which kind a joint is comes from its URDF
  parameters — `esc_index` means a drive wheel (velocity command via `esc.RPMCommand`, feedback
  from `esc.Status`), `actuator_id` means a steering actuator (position command via
  `actuator.ArrayCommand`). Per-ESC voltage/current/temperature is exported through `<gpio>`
  state interfaces. Two things to know before touching it:
  - **The firmware's RPM units are asymmetric** (`esc.Status` is mechanical RPM, `RPMCommand` is
    interpreted as ERPM). The `command_rpm_is_erpm` hardware parameter compensates. See
    `docs/can_id_map.md`.
  - **Drive wheel position is integrated from velocity** — `esc.Status` carries no position —
    which is why `diff_drive_controller` runs with `position_feedback: false`.
  - Only the esc and actuator DSDL are vendored. Adding a message means copying its generated
    sources from `bldc/libcanard/dsdl/` and adding them to the `rp1_canard` CMake target;
    `canard.h`/`canard_internals.h` carry an rp1-specific `CANARD_ENABLE_CANFD=1` fix for a
    64-bit pointer-layout assumption, so preserve that when re-vendoring.
- **`rp1_sim`** (`swerve_sim_node.py`) — pure-ROS2 swerve simulator: 4 drive wheels + 4 steering
  actuators, consuming `rp1_msgs/WheelCommand` + `SteeringCommand` and publishing odometry and
  TF. **Nothing publishes those topics** — there is no swerve inverse-kinematics controller yet,
  so it is driven by hand with `ros2 topic pub`. The MVP skid-steer simulator that used to live
  here (`sim_bridge_node.py`) is gone: `use_mock:=true` covers that case with ros2_control's
  `mock_components/GenericSystem` plus the controller's own odometry.
- **`rp1_bringup`** — no code, just launch files + `config/rp1_mvp.yaml` and
  `config/rp1_controllers.yaml`. `rp1_mvp.launch.py` is the whole MVP (args: `use_mock`,
  `can_iface`, `teleop`, `rviz`); `rp1_mvp_elrs.launch.py` is the same with `elrs_driver` as the
  `/joy` source plus `rp1_elrs`'s telemetry adapter; `rp1_swerve_sim.launch.py` runs the swerve
  simulator standalone. Three things bite here, all commented in the launch files:
  - **controller_manager takes `robot_description` from the TOPIC**, not a parameter. It waits
    on `/robot_description` until `robot_state_publisher` latches it. Passing a different
    description as a parameter is silently ignored — which would mean `use_mock:=true` quietly
    loading the real DroneCAN plugin and opening `can0`.
  - **Spawners need `--param-file`.** A controller node does not inherit the params file given
    to `ros2_control_node`; without it `diff_drive_controller` declares `wheel_separation` at its
    `0.0` default, violating that parameter's own `> 0` range, and fails to load.
  - **Spawn controllers strictly sequentially.** controller_manager's service handling shares a
    thread with its RT update loop (ros-controls/ros2_control#2808); two spawners at once hung it
    outright in this sandbox.
  - `joint_state_broadcaster` needs `use_urdf_to_filter: false`, or the `<gpio>` telemetry
    interfaces are dropped from `/dynamic_joint_states` and the handset battery goes dead
    silently.
  Hardware parameters (`can_iface`, `node_id`, `gear_ratio`, `motor_pole_pairs`) live in the
  URDF, not `rp1_mvp.yaml` — ros2_control reads them from the description only.

### Simulation (two tiers)

- **`use_mock:=true`** — swaps the hardware plugin for `mock_components/GenericSystem`: no CAN,
  no DroneCAN, commands loop straight back to states, and `diff_drive_controller` still produces
  real odometry and TF. Primary tool for checking the ROS2 architecture.
- **`simulation/sim_vesc_node.py`** — a standalone script (no ROS2 dependency, since the real
  VESCs don't have one either) using `python-dronecan` over a Linux `vcan` interface, pretending
  to be the 4 VESCs. Run the real stack against it with `can_iface:=vcan0` to exercise actual
  DroneCAN framing over a virtual bus — this is the only tier that tests the wire protocol. See
  `simulation/README.md` for `vcan0` setup (needs `sudo`, not automatable in a sandboxed
  session). `simulation/sim_actuator_node.py` does the same for steering actuators.

### Extending to full swerve (steering joints) — not yet started

The hardware half is **already done**: `VescDroneCanSystem` handles `actuator.ArrayCommand`/`Status`
and reads its joint list from the URDF, so going from 2 to 4 steering actuators is a URDF edit,
not a code change. What is missing is the controller: **ros2_controllers has no swerve
controller** (`steering_controllers_library` covers Ackermann/bicycle/tricycle, not swerve), so
this needs a custom `controller_interface` plugin claiming 4 velocity + 4 position interfaces and
doing the inverse kinematics (per-wheel speed *and* angle, including wheel angle wrap for
continuous joints). When it exists, merge `rp1_drive.urdf` and `rp1_steering.urdf` into one
description on a single node ID.

One remaining blocker is hardware, not software: firmware `ArrayCommand` support is on the
`add-actuator-arraycommand` branch, not mainline. The steering actuator type is no longer an
open decision as of 2026-07-30: the steering VESCs use an ABZ quadrature encoder for FOC
position feedback (`mc_interface_set_pid_pos`), resolving the earlier "no encoder, reported
positions are meaningless" state -- confirm this is actually wired up on the bench units before
trusting reported steering angle, since this is a correction to the previously-documented
state, not yet independently re-verified against hardware. Each steering axis is also getting
two proximity sensors, at 0° and 90° relative to the robot's direction of travel, to lock/
reference the steering angle -- mechanism and electrical interface not yet designed (see
rp1-specs/system_architecture.md's swerve section).
