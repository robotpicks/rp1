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
- **Odometry.** `compute_body_twist()` reads back drive velocity + steering position *state*
  (`wheel_radius` param converts angular velocity to linear speed) and solves the same
  rigid-body-twist equations as the IK, in reverse — exact for this controller's always-
  rectangular corner geometry (see the code comment for why it decouples into 3 independent
  averages rather than needing a general least-squares solver). Integrated pose is published as
  `nav_msgs/Odometry` on `~/odom` and broadcast as an `odom_frame_id`→`base_frame_id` TF
  (defaults `odom`/`base_link`, `enable_odom_tf` to disable), matching `diff_drive_controller`'s
  conventions.
- Verified end-to-end against `rp1_bringup`'s `rp1_swerve_mock.launch.py` (mock hardware, real
  `rp1_swerve.urdf`, no CAN/Gazebo): straight, pure lateral/crab, and turn-in-place all produce
  the expected `/joint_states`, including the angle-flip case actually engaging correctly; and
  driving straight for 5s at 1 m/s produced `~/odom` position ≈5.5m and a matching `odom→
  base_link` TF, confirming the robot actually drives across RViz's grid now, not just
  articulates its joints in place. See that launch file for the "watch it before trusting it on
  the robot" bringup.

## What's not here yet

- **The 0°/90°-locked, 2-wheel, and full-swerve operating modes** from
  `rp1-specs/requirements.md` — no mode concept exists yet; there's only ever continuous
  free-angle swerve from whatever `cmd_vel` says.
- **`seek_home`/`home_0deg`/`home_90deg` aren't read** — the 0°/90° proximity sensors and homing
  command from `rp1_swerve.urdf` exist but nothing here consumes them yet.
- Not wired into the real DroneCAN hardware path (`vesc_dronecan_driver` against `can0`/`vcan0`)
  or Gazebo physics yet — only the mock-hardware bringup exists so far. On mock hardware the
  odometry above is computed from state that's just last cycle's command looped back, not real
  sensor feedback -- a meaningfully different trust level once real VESC feedback is in the loop.
- No automated tests (the IK/odometry have been verified by hand and via live `ros2 topic pub` +
  `/joint_states`/`~/odom`/TF inspection, not a gtest suite).

Not chainable (`controller_interface::ControllerInterface`, not `ChainableControllerInterface`)
— unlike this ROS 2 release's `diff_drive_controller`/`mecanum_drive_controller`. Revisit only if
something actually needs to chain into/out of this controller; not needed for anything currently
planned.
