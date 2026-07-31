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
  - Verified (gtest, with the corners' `home_0deg`/`home_90deg` pre-confirmed so the homing gate
    below doesn't mask the math being checked): `LOCKED_0`/`LOCKED_90`/`TWO_WHEEL` all produced
    exact numeric matches to hand-calculated expected wheel speeds/angles (including the
    front/rear or left/right differential split from `wz`), switching back to `FULL_SWERVE` —
    and an out-of-range mode value — both correctly fall back to the unconstrained IK, and
    overriding `two_wheel_steered_corners` to a non-default pair (`front_left`/`rear_left`,
    a front+rear diagonal-ish pair rather than the default front pair) correctly moved which 2
    corners free-steer.
- **Homing-gated locked-mode transitions.** Fixes the gap `rp1-specs/requirements.md` flagged
  ("a mode switch takes effect immediately based on whatever angle the controller last
  commanded, not a verified position"): `compute_corner_commands()` reads each locked corner's
  live `home_0deg`/`home_90deg` `<gpio>` state every cycle (no separate "have we ever homed"
  bookkeeping needed — a corner that later drives off the reference naturally stops reporting it
  and re-gates the next time a locked mode needs it), via the `steering_home_sensors` parameter
  (4 gpio prefix names, defaulting to `rp1_swerve.urdf`'s `steering_sensors_*` naming). While
  *any* corner that needs a fixed reference for the current mode isn't yet confirmed there, every
  drive wheel is held at zero (the whole chassis, not just that corner — a mix of confirmed and
  unconfirmed locked wheels would drag/scrub if the drive wheels moved) and the corner's
  `seek_home` command interface carries `0.0`/`1.0` (edge-triggered, matching
  `vesc_dronecan_driver`'s convention — `NaN` means no active request, written once confirmed or
  when the corner isn't locked at all this mode). The corner's steering position command still
  targets the locked angle as usual, so it visibly rotates in place to seek its sensor even
  though the chassis doesn't translate. `FULL_SWERVE` never gates on anything (no fixed
  reference to verify). Verified both via gtest (an unhomed `LOCKED_0`/`LOCKED_90` switch holds
  all 4 wheels at zero and requests the right `seek_home` target; confirming the sensor resumes
  driving with the exact previously-verified locked-mode numbers; a partially-homed corner still
  holds the whole chassis; `TWO_WHEEL`'s free-steering pair never gates) and live: against
  `rp1_swerve_mock.launch.py` (mock hardware never confirms homing, since nothing simulates the
  proximity sensors, so `LOCKED_0` correctly holds `/joint_states` at zero indefinitely) and
  against `rp1_swerve_dronecan.launch.py` over `vcan0` (same zero-hold, plus a raw DroneCAN
  sniff confirmed exactly one `COMMAND_TYPE_HOME` `actuator.ArrayCommand` entry per corner
  reaches the wire — not resent every cycle — when switching into `LOCKED_0`).
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

- **No mode-switch safety handling beyond the homing gate above** — a switch INTO `LOCKED_0`/
  `LOCKED_90`/`TWO_WHEEL` while already homed at the target angle (or switching between two
  modes needing the *same* angle, e.g. `LOCKED_0`→`TWO_WHEEL`) takes effect immediately with
  whatever `cmd_vel` is current; there's no ramping, no requirement to be stopped first, no
  rejection of a switch mid-turn. The homing gate only helps when the target angle isn't yet
  physically confirmed.
- Not wired into Gazebo physics yet.
- **Real DroneCAN hardware path verified for both drive and steering.**
  `rp1_swerve_dronecan.launch.py` (against `vesc_dronecan_driver`, real wire framing over
  `can0`/`vcan0`) drove straight and produced genuine nonzero drive-wheel velocity/position in
  `/joint_states`; a turn-in-place command produced steering positions matching the
  hand-computed full-swerve angles exactly (`±0.5586 rad` on all 4 corners, alternating sign per
  the angle-flip optimization) — both fed back over actual DroneCAN traffic, a meaningfully
  different trust level than the mock-hardware tier, where odometry is computed from state
  that's just last cycle's command looped back. Getting there required fixing three real bugs
  (all in `vesc_dronecan_ros`'s swerve branch except the dronecan driver-dispatch one): (1)
  `dronecan`'s driver-dispatch bug in the simulator scripts; (2) `pumpRx()` passed a hardcoded
  `timestamp_usec=0` to libcanard, which made every multi-frame transfer (`esc.Status`,
  `actuator.Status`) look "never initialized" on its second frame and get silently dropped —
  single-frame transfers (`esc.RPMCommand`, `actuator.ArrayCommand`) were unaffected, which is
  why this went unnoticed until real feedback was checked; (3) `actuator.Status`'s decoder
  hard-failed the *entire* decode (throwing away position/speed/force too, not just the 2
  extension fields) whenever the private `home_0deg`/`home_90deg` extension bits were absent —
  which they always are from a standard, non-extended sender like `sim_actuator_node.py`. The
  extension bits themselves (and `seek_home`/`COMMAND_TYPE_HOME`) still aren't exercised by
  these simulator scripts, since the stock `dronecan` Python codec doesn't know to populate
  them — only a real bldc-flashed VESC (or a simulator taught the extension) would cover that.

## Tests

`test/test_rp1_swerve_controller.cpp` — 14 gtest cases, run via `colcon test --packages-select
rp1_swerve_controller`. Builds a real controller instance with real `CommandInterface`/
`StateInterface` objects (not a mock hardware component), drives the actual lifecycle
(`init`/`configure`/`assign_interfaces`/`activate`), and delivers `cmd_vel`/`mode` via real
publishers spun through a `SingleThreadedExecutor` (not direct field writes) so the subscription
wiring itself is exercised, not just the math. Covers straight/crab/turn-in-place (including
asserting the angle-flip optimization's exact expected values on the two corners where it should
engage), `LOCKED_0`/`LOCKED_90`, `TWO_WHEEL` with both the default and an overridden
`two_wheel_steered_corners` pair, the out-of-range-mode fallback, odometry (drives state
interfaces directly, subscribes `~/odom`, checks the published value), and the homing gate: an
unhomed `LOCKED_0`/`LOCKED_90` switch holds every drive wheel at zero and issues the right
`seek_home` target, confirming the sensor afterward resumes driving with the same numbers the
non-gated tests check; a partially-homed corner (3 of 4) still holds the whole chassis; `TWO_WHEEL`
gates on its locked pair only, never the free-steering one; and `FULL_SWERVE` never gates
regardless of home state.

Not chainable (`controller_interface::ControllerInterface`, not `ChainableControllerInterface`)
— unlike this ROS 2 release's `diff_drive_controller`/`mecanum_drive_controller`. Revisit only if
something actually needs to chain into/out of this controller; not needed for anything currently
planned.
