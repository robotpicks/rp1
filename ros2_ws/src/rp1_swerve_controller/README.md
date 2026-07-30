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
  end-to-end and safe to load/activate today — with every corner always commanded to 0
  velocity / 0 angle regardless of `cmd_vel` input.

## What's not here yet

- **Swerve inverse kinematics.** `compute_corner_commands()` in
  `rp1_swerve_controller.cpp` is the deliberate placeholder for this — see the `TODO(swerve)`
  comment there. No per-wheel speed/angle math, no continuous-joint angle-wrap optimization.
- **The 0°/90°-locked, 2-wheel, and full-swerve operating modes** from
  `rp1-specs/requirements.md` — no mode concept exists yet; there's only ever "the" swerve
  command.
- **No state feedback is consumed** (`state_interface_configuration()` claims none) — nothing
  here reads back steering position, the 0°/90° proximity sensors, or brake state yet.
- Not wired into any launch file (`rp1_bringup`) or controller config YAML yet, and not
  registered in any URDF's `<ros2_control>` block.
- No tests.

Not chainable (`controller_interface::ControllerInterface`, not `ChainableControllerInterface`)
— unlike this ROS 2 release's `diff_drive_controller`/`mecanum_drive_controller`. Revisit only if
something actually needs to chain into/out of this controller; not needed for anything currently
planned.
