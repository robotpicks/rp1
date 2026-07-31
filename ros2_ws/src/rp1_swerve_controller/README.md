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
  an unwrapped last-commanded-angle per corner.
- **The four operating modes from `rp1-specs/requirements.md`**, selected via `~/mode`
  (`std_msgs/UInt8`, `SwerveMode` in the header):
  - `0` **FULL_SWERVE** (default) — all 4 corners independently steered, as above.
  - `1` **LOCKED_0** — all 4 wheels held at 0°, driven by projecting the same rigid-body-twist
    vector onto that fixed direction — algebraically identical to standard skid-steer
    differential drive (`vx ∓ wz·half_track` per side). `vy` has no effect (can't strafe with
    wheels locked forward).
  - `2` **LOCKED_90** — all 4 wheels held at 90° (pure crab), same projection trick. `vx` has no
    effect (can't drive fore/aft with wheels locked sideways).
  - `3` **TWO_WHEEL** — 2 corners free-steer via the full IK above, the other 2 locked at 0°
    like `LOCKED_0`. **Which 2 is the `two_wheel_steered_corners` parameter** (a list of exactly
    2 of `front_left`/`front_right`/`rear_left`/`rear_right`), not fixed in code — default is
    the front pair (a conventional front-steered vehicle), but one front + one rear, both on one
    side, or diagonal corners are equally valid. Ackermann-like, though not true Ackermann
    geometry (no explicit turn-radius computation) — a deliberate simplification reusing the
    existing per-corner IK/lock machinery.
  - Locked corners skip the angle-flip optimization on purpose — a "locked" wheel should stay
    visually fixed at its locked angle, not flip to the opposite angle with reversed speed even
    though that's motion-equivalent. Unrecognized `~/mode` values fail safe to `FULL_SWERVE`
    rather than erroring.
  - Verified live against the mock bringup: `LOCKED_0`/`LOCKED_90`/`TWO_WHEEL` all produced
    exact numeric matches to hand-calculated expected wheel speeds/angles (including the
    front/rear or left/right differential split from `wz`), switching back to `FULL_SWERVE` —
    and an out-of-range mode value — both correctly fall back to the unconstrained IK, and
    overriding `two_wheel_steered_corners` to a non-default pair (`front_left`/`rear_left`,
    a front+rear diagonal-ish pair rather than the default front pair) correctly moved which 2
    corners free-steer.
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

- **No verified/homed-position gating on locked modes.** Nothing here checks
  `home_0deg`/`home_90deg` before or after commanding `LOCKED_0`/`LOCKED_90` — a mode switch
  commands the locked angle immediately based on `last_steering_angle_`'s current value, not a
  confirmed physical reference. `requirements.md`'s note that transitioning between locked
  configurations should happen by rotating in place (and presumably being confirmed by the
  proximity switches) isn't implemented as a state machine here.
- **`seek_home`/`home_0deg`/`home_90deg` aren't read** — the 0°/90° proximity sensors and homing
  command from `rp1_swerve.urdf` exist but nothing here consumes them yet.
- **No mode-switch safety handling** — switching modes while the robot is moving takes effect
  on the next control cycle with whatever `cmd_vel` happens to be current; there's no ramping,
  no requirement to be stopped first, no rejection of a switch mid-turn.
- Not wired into the real DroneCAN hardware path (`vesc_dronecan_driver` against `can0`/`vcan0`)
  or Gazebo physics yet — only the mock-hardware bringup exists so far. On mock hardware the
  odometry above is computed from state that's just last cycle's command looped back, not real
  sensor feedback -- a meaningfully different trust level once real VESC feedback is in the loop.

## Tests

`test/test_rp1_swerve_controller.cpp` — 9 gtest cases, run via `colcon test --packages-select
rp1_swerve_controller`. Builds a real controller instance with real `CommandInterface`/
`StateInterface` objects (not a mock hardware component), drives the actual lifecycle
(`init`/`configure`/`assign_interfaces`/`activate`), and delivers `cmd_vel`/`mode` via real
publishers spun through a `SingleThreadedExecutor` (not direct field writes) so the subscription
wiring itself is exercised, not just the math. Covers straight/crab/turn-in-place (including
asserting the angle-flip optimization's exact expected values on the two corners where it should
engage), `LOCKED_0`/`LOCKED_90`, `TWO_WHEEL` with both the default and an overridden
`two_wheel_steered_corners` pair, the out-of-range-mode fallback, and odometry (drives state
interfaces directly, subscribes `~/odom`, checks the published value).

Not chainable (`controller_interface::ControllerInterface`, not `ChainableControllerInterface`)
— unlike this ROS 2 release's `diff_drive_controller`/`mecanum_drive_controller`. Revisit only if
something actually needs to chain into/out of this controller; not needed for anything currently
planned.
