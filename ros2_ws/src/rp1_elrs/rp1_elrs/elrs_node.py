"""ExpressLRS (CRSF over UART) bridge for the rp1 MVP.

Reads an ELRS receiver's CRSF stream from a serial port and republishes the RC channels as
`sensor_msgs/Joy`, so the existing `rp1_teleop` node handles scaling / deadman / staleness and
ELRS becomes just another `/joy` source alongside the Xbox pad (see rp1/CLAUDE.md pipeline). In
the other direction it subscribes to `rp1_msgs/WheelFeedback` and pushes battery telemetry back
over the same CRSF link so it shows up on the ELRS handset.

Only the standard CRSF path is implemented (`link_mode: crsf`). MAVLink-over-ELRS is a
documented Phase-2 mode -- the wire protocol lives in `crsf.py` precisely so a MAVLink codec can
be added beside it without reworking this node.

Structure deliberately mirrors rp1_dronecan_bridge/bridge_node.py: the transport library
(pyserial here, cf. dronecan/python-can there) is imported lazily so the package still builds
and dry-runs without it; `require_serial: false` is the analogue of the bridge's
`require_can: false` (open no port, log intended telemetry instead of sending); and every serial
read/write is wrapped so a hardware hiccup logs but never kills the node.
"""

import rclpy
from rclpy.node import Node as RosNode
from sensor_msgs.msg import Joy

from rp1_elrs import crsf


class ElrsNode(RosNode):

    def __init__(self):
        super().__init__('elrs_node')

        # --- serial / link -----------------------------------------------------------------
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 420000)
        self.declare_parameter('require_serial', True)
        self.declare_parameter('link_mode', 'crsf')          # 'crsf' now; 'mavlink' is Phase-2
        self.declare_parameter('poll_rate_hz', 200.0)         # serial drain / Joy publish tick

        # --- RC channel -> Joy mapping -----------------------------------------------------
        # Which CRSF channel (0-based) feeds each Joy axis, in axis order. rp1_teleop then picks
        # the axes it wants by index (axis_linear/axis_angular in joy_elrs.yaml). Defaults are
        # AETR (0=Aileron,1=Elevator,2=Throttle,3=Rudder) -- verify against your TX.
        self.declare_parameter('axis_channels', [0, 1, 2, 3])
        # The arm/deadman switch: a CRSF channel mapped to a Joy button so rp1_teleop's
        # deadman_button gate works unchanged. deadman_button_index defaults to 4 to match the
        # existing Xbox convention (LB == button 4 == deadman) in rp1_mvp.yaml.
        self.declare_parameter('deadman_channel', 4)
        self.declare_parameter('deadman_button_index', 4)
        self.declare_parameter('deadman_threshold', 0.5)     # switch "on" when channel unit > this
        self.declare_parameter('rc_timeout_sec', 0.5)        # neutral Joy if RC goes stale

        # --- telemetry back to the handset -------------------------------------------------
        self.declare_parameter('telemetry_rate_hz', 5.0)
        # Pack voltage/current source across the 4 wheels' WheelFeedback: 'mean' or a wheel index.
        self.declare_parameter('battery_source', 'mean')
        self.declare_parameter('battery_remaining_pct', 0)   # SoC not estimated yet (0 = unknown)

        self._require_serial = self.get_parameter('require_serial').value
        self._link_mode = self.get_parameter('link_mode').value
        self._axis_channels = list(self.get_parameter('axis_channels').value)
        self._deadman_channel = self.get_parameter('deadman_channel').value
        self._deadman_button_index = self.get_parameter('deadman_button_index').value
        self._deadman_threshold = self.get_parameter('deadman_threshold').value
        self._rc_timeout = self.get_parameter('rc_timeout_sec').value
        self._battery_source = str(self.get_parameter('battery_source').value)
        self._battery_remaining = int(self.get_parameter('battery_remaining_pct').value)

        if self._link_mode != 'crsf':
            self.get_logger().warn(
                "link_mode='%s' not implemented yet; only 'crsf' is supported (MAVLink-over-ELRS "
                'is a documented Phase-2 mode). Falling back to CRSF.' % self._link_mode)

        self._parser = crsf.CrsfParser()
        self._serial = None
        self._last_rc_time = None

        # latest per-wheel feedback for telemetry (index 0-3); None until first frame arrives
        self._voltage = [None, None, None, None]
        self._current = [None, None, None, None]

        self._joy_pub = self.create_publisher(Joy, 'joy', 10)

        from rp1_msgs.msg import WheelFeedback
        self.create_subscription(WheelFeedback, 'wheel_feedback', self._on_wheel_feedback, 10)

        if self._require_serial:
            self._serial = self._open_serial()
            poll_hz = self.get_parameter('poll_rate_hz').value
            self.create_timer(1.0 / poll_hz, self._poll_serial)
            self.create_timer(0.1, self._rc_watchdog)
        else:
            self.get_logger().warn(
                'require_serial=false: dry-run, no serial port opened; RC input is disabled and '
                'telemetry frames are logged instead of sent')

        telem_hz = self.get_parameter('telemetry_rate_hz').value
        if telem_hz > 0.0:
            self.create_timer(1.0 / telem_hz, self._on_telemetry_tick)

    # -- serial ----------------------------------------------------------------------------

    def _open_serial(self):
        import serial  # lazy: pyserial is a pip dep, not required to import/build the package
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud').value
        # timeout=0 -> non-blocking reads; we drain in_waiting on a fast ROS2 timer instead of
        # blocking the executor (cf. the DroneCAN bridge pumping spin(timeout=0)).
        ser = serial.Serial(port, baudrate=baud, timeout=0)
        self.get_logger().info('ELRS CRSF link up on %s @ %d baud' % (port, baud))
        return ser

    def _poll_serial(self) -> None:
        try:
            waiting = self._serial.in_waiting
            data = self._serial.read(waiting if waiting else 1)
        except Exception as exc:  # noqa: BLE001 - a serial read error must not kill the node
            self.get_logger().error('serial read error: %s' % exc)
            return
        if not data:
            return
        for _addr, frame_type, payload in self._parser.feed(data):
            if frame_type == crsf.FRAMETYPE_RC_CHANNELS_PACKED:
                self._on_rc_channels(payload)
            # LINK_STATISTICS / other telemetry-from-RX frames are ignored for now.

    def _write_frame(self, frame: bytes, what: str) -> None:
        if self._serial is None:
            self.get_logger().debug('dry-run %s: %s' % (what, frame.hex()))
            return
        try:
            self._serial.write(frame)
        except Exception as exc:  # noqa: BLE001 - a dropped telemetry write must not kill the node
            self.get_logger().error('serial write error: %s' % exc)

    # -- RC channels -> Joy ----------------------------------------------------------------

    def _on_rc_channels(self, payload: bytes) -> None:
        channels = crsf.unpack_channels(payload)
        if len(channels) < crsf.NUM_RC_CHANNELS:
            return
        self._last_rc_time = self.get_clock().now()
        self._publish_joy(channels)

    def _publish_joy(self, channels) -> None:
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [crsf.raw_to_unit(channels[c]) if 0 <= c < len(channels) else 0.0
                    for c in self._axis_channels]
        buttons = [0] * (self._deadman_button_index + 1)
        if 0 <= self._deadman_channel < len(channels):
            on = crsf.raw_to_unit(channels[self._deadman_channel]) > self._deadman_threshold
            buttons[self._deadman_button_index] = 1 if on else 0
        msg.buttons = buttons
        self._joy_pub.publish(msg)

    def _rc_watchdog(self) -> None:
        """On RC loss (no valid frame within rc_timeout), publish a neutral Joy: zero axes and
        deadman released, so rp1_teleop zeroes /cmd_vel -- the RC-side failsafe."""
        if self._last_rc_time is None:
            return
        stale = (self.get_clock().now() - self._last_rc_time).nanoseconds / 1e9
        if stale > self._rc_timeout:
            self._publish_joy([crsf.CRSF_CHANNEL_MID] * crsf.NUM_RC_CHANNELS)

    # -- telemetry -> handset --------------------------------------------------------------

    def _on_wheel_feedback(self, msg) -> None:
        if 0 <= msg.wheel_index < 4:
            self._voltage[msg.wheel_index] = float(msg.voltage)
            self._current[msg.wheel_index] = float(msg.current)

    def _pack_value(self, values):
        present = [v for v in values if v is not None]
        if not present:
            return None
        if self._battery_source == 'mean':
            return sum(present) / len(present)
        try:
            idx = int(self._battery_source)
        except ValueError:
            return sum(present) / len(present)
        if 0 <= idx < 4 and values[idx] is not None:
            return values[idx]
        return sum(present) / len(present)

    def _on_telemetry_tick(self) -> None:
        voltage = self._pack_value(self._voltage)
        current = self._pack_value(self._current)
        if voltage is None:
            return  # no WheelFeedback seen yet -- nothing to report
        frame = crsf.build_battery_frame(
            voltage_dv=int(round(voltage * 10)),
            current_da=int(round((current or 0.0) * 10)),
            capacity_mah=0,                       # used-capacity integration not tracked yet
            remaining_pct=self._battery_remaining,
        )
        self._write_frame(frame, 'battery telemetry')


def main(args=None):
    rclpy.init(args=args)
    node = ElrsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
