# rp1_swerve_controller

`ros2_control` `controller_interface::ControllerInterface` plugin for rp1's 4-corner swerve
base. Started on the `swerve` branch (`master`/main bringup stays 4-wheel-only skid-steer via
`diff_drive_controller` until this replaces it) — see `rp1-specs/requirements.md` ("Why swerve,
and what it needs to do") for the operating modes this needs to support, and
`rp1-specs/software_spec.md` ("Swerve controller — not yet started") for the broader status.

## What's here

- Claims 4 drive velocity command interfaces + 4 steering position command interfaces, named via
  the `drive_joints`/`steering_joints` parameters (each a list of exactly 4 names, in
  front_left/front_right/rear_left/rear_right order — matches `docs/can_id_map.md`'s wheel index
  convention).
- Subscribes `~/cmd_vel` (`geometry_msgs/TwistStamped`, matching `rp1_teleop`'s output and
  `diff_drive_controller`'s convention on this ROS 2 release).
- Lifecycle (`on_init`/`on_configure`/`on_activate`/`on_deactivate`) and `update()` are wired
  end-to-end and safe to load/activate.
- **Real swerve inverse kinematics.** `compute_corner_commands()` decomposes `(vx, vy, wz)` into
  each corner's wheel speed + steering angle at its steering-axis position
  (`half_wheelbase`/`half_track` params), with the angle-flip optimization (rotate ≤90° and
  reverse wheel speed instead of always steering to the literal computed angle) tracked against
  an unwrapped last-commanded-angle per corner. Continuous free-angle swerve only — no discrete
  mode concept yet (see below).
- Verified end-to-end against `rp1_bringup`'s `rp1_swerve_mock.launch.py` (mock hardware, real
  `rp1_swerve.urdf`, no CAN/Gazebo): straight, pure lateral/crab, and turn-in-place all produce
  the expected `/joint_states`, including the angle-flip case actually engaging correctly. See
  that launch file for the "watch it before trusting it on the robot" bringup.

## What's not here yet

- **The 0°/90°-locked, 2-wheel, and full-swerve operating modes** from
  `rp1-specs/requirements.md` — no mode concept exists yet; there's only ever continuous
  free-angle swerve from whatever `cmd_vel` says.
- **No state feedback is consumed** (`state_interface_configuration()` claims none) — nothing
  here reads back steering position, the 0°/90° proximity sensors (`seek_home`/`home_0deg`/
  `home_90deg`, see `rp1_swerve.urdf`), or brake state yet.
- **No odometry.** Unlike `diff_drive_controller`, this doesn't publish `/odom` or an
  `odom→base_link` TF — in `rp1_swerve_mock.launch.py` the robot's joints articulate correctly
  but it doesn't visibly translate through the world.
- Not wired into the real DroneCAN hardware path (`vesc_dronecan_driver` against `can0`/`vcan0`)
  or Gazebo physics yet — only the mock-hardware bringup exists so far.
- No automated tests (the IK has been verified by hand and via live `ros2 topic pub` +
  `/joint_states` inspection, not a gtest suite).

Not chainable (`controller_interface::ControllerInterface`, not `ChainableControllerInterface`)
— unlike this ROS 2 release's `diff_drive_controller`/`mecanum_drive_controller`. Revisit only if
something actually needs to chain into/out of this controller; not needed for anything currently
planned.
