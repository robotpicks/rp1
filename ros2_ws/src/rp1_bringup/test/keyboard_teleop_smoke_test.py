#!/usr/bin/env python3
"""Verify keyboard teleop actually drives the robot, without a human at a keyboard.

teleop_twist_keyboard reads raw keypresses via termios, which needs a real controlling tty --
it crashes immediately with termios.error if launched detached/backgrounded (e.g. from CI or a
plain subprocess pipe). This script gives it one anyway, using a pseudo-terminal (Python's
`pty` module), then writes synthetic keypresses to it and confirms /diff_drive_controller/odom
actually moves -- the same thing a human pressing keys in a focused terminal would produce.

Two things this caught during development, worth keeping in mind if this script ever needs
touching:
  - Spawning via `ros2 run teleop_twist_keyboard ...` adds enough CLI-wrapper startup latency
    that synthetic keypresses sent too early are silently lost (the process isn't reading stdin
    yet). Invoking the module's main() directly via `python3 -c` avoids that layer entirely.
  - The pty needs a few seconds to settle (rclpy init, node startup, entering the raw-mode read
    loop) before it's safe to start writing -- see STARTUP_SETTLE_S below.

Usage: start a bringup first (rp1_mvp.launch.py use_mock:=true teleop:=false, or
rp1_gazebo.launch.py teleop:=false), then run this against it:

    python3 keyboard_teleop_smoke_test.py [cmd_vel_topic] [odom_topic]

Exits 0 and prints PASS if position changed after driving; exits 1 and prints FAIL otherwise.
"""
import math
import os
import pty
import select
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

STARTUP_SETTLE_S = 3.0
DRIVE_FORWARD_S = 4.0
MOVE_THRESHOLD_M = 0.05


class OdomWatcher(Node):
    def __init__(self, odom_topic):
        super().__init__('keyboard_teleop_smoke_test')
        self.last = None
        self.create_subscription(Odometry, odom_topic, self._cb, 10)

    def _cb(self, msg):
        p = msg.pose.pose.position
        self.last = (p.x, p.y)

    def read_position(self, spin_seconds):
        end = time.time() + spin_seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.last


def spawn_teleop_under_pty(cmd_vel_topic):
    """Direct module invocation, not `ros2 run` -- see module docstring."""
    py_code = (
        "import sys; sys.argv=['teleop_twist_keyboard','--ros-args',"
        "'-p','stamped:=true','-p','frame_id:=base_link',"
        f"'-r','cmd_vel:={cmd_vel_topic}']; "
        "import teleop_twist_keyboard; teleop_twist_keyboard.main()"
    )
    master_fd, slave_fd = pty.openpty()
    proc = subprocess.Popen(
        [sys.executable, "-c", py_code],
        stdin=slave_fd, stdout=slave_fd, stderr=slave_fd,
        close_fds=True, start_new_session=True,
    )
    os.close(slave_fd)
    os.set_blocking(master_fd, False)
    return proc, master_fd


def drain(master_fd, seconds):
    end = time.time() + seconds
    while time.time() < end:
        r, _, _ = select.select([master_fd], [], [], 0.2)
        if r:
            try:
                os.read(master_fd, 65536)
            except (BlockingIOError, OSError):
                pass


def main():
    cmd_vel_topic = sys.argv[1] if len(sys.argv) > 1 else '/diff_drive_controller/cmd_vel'
    odom_topic = sys.argv[2] if len(sys.argv) > 2 else '/diff_drive_controller/odom'

    rclpy.init()
    watcher = OdomWatcher(odom_topic)

    before = watcher.read_position(2.0)
    if before is None:
        print(f"FAIL: no messages on {odom_topic} -- is a bringup actually running?")
        sys.exit(1)
    print(f"before: x={before[0]:.4f} y={before[1]:.4f}")

    proc, master_fd = spawn_teleop_under_pty(cmd_vel_topic)
    drain(master_fd, STARTUP_SETTLE_S)
    if proc.poll() is not None:
        print(f"FAIL: teleop_twist_keyboard exited early (code {proc.returncode}) -- "
              "likely no controlling tty; see module docstring.")
        sys.exit(1)

    print(f"driving forward ('i') for {DRIVE_FORWARD_S}s...")
    end = time.time() + DRIVE_FORWARD_S
    while time.time() < end:
        os.write(master_fd, b"i")
        time.sleep(0.2)
    os.write(master_fd, b"k")  # stop

    proc.send_signal(2)  # SIGINT, like a real Ctrl-C
    time.sleep(1)
    if proc.poll() is None:
        proc.kill()
    os.close(master_fd)

    after = watcher.read_position(2.0)
    print(f"after:  x={after[0]:.4f} y={after[1]:.4f}")

    watcher.destroy_node()
    rclpy.shutdown()

    moved = math.hypot(after[0] - before[0], after[1] - before[1])
    passed = moved >= MOVE_THRESHOLD_M
    if passed:
        print(f"PASS: moved {moved:.4f}m")
    else:
        print(f"FAIL: only moved {moved:.4f}m (threshold {MOVE_THRESHOLD_M}m)")

    # os._exit, not sys.exit: rclpy.shutdown() above doesn't reliably tear down every
    # background DDS thread promptly, which left plain sys.exit() hanging until an external
    # timeout killed the process even though the check itself had already completed and
    # printed its result. Everything that matters is already flushed by this point.
    sys.stdout.flush()
    os._exit(0 if passed else 1)


if __name__ == '__main__':
    main()
