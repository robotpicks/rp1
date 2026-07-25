"""ROS2 <-> DroneCAN bridge for the rp1 MVP.

Subscribes to `rp1_msgs/WheelCommand` (from rp1_control) and broadcasts it on the DroneCAN bus
as `uavcan.equipment.esc.RawCommand`, at a fixed rate with a command timeout failsafe. Listens
for `uavcan.equipment.esc.Status` frames coming back from the VESCs (which speak DroneCAN
natively via their UAVCAN CAN mode -- no separate bridge MCU) and republishes them as
`rp1_msgs/WheelFeedback`.

The wheel index convention (rp1_msgs.msg.WheelCommand.WHEEL_*) is used directly as the DroneCAN
esc_index, and must match the mapping documented in docs/can_id_map.md and each VESC's UAVCAN
config (set via VESC Tool).

If `require_can` is false, no CAN device is opened and RawCommand frames are logged instead of
sent -- useful for exercising the rest of the ROS2 pipeline before the bridge hardware exists.
"""

import rclpy
from rclpy.node import Node as RosNode
from rp1_msgs.msg import WheelCommand, WheelFeedback

RAW_COMMAND_MAX_MAGNITUDE = 8191  # uavcan.equipment.esc.RawCommand.cmd is saturated int14


def _patch_python_can_flush_tx_buffer() -> None:
    """dronecan's python-can driver calls bus.flush_tx_buffer() after every send, but
    python-can >=4's BusABC no longer implements it for any interface (raises
    NotImplementedError), which otherwise kills the writer thread on the first broadcast.
    """
    import can
    can.bus.BusABC.flush_tx_buffer = lambda self: None


class BridgeNode(RosNode):

    def __init__(self):
        super().__init__('bridge_node')

        self.declare_parameter('can_iface', 'can0')
        self.declare_parameter('dronecan_node_id', 42)
        self.declare_parameter('command_rate_hz', 50.0)
        self.declare_parameter('command_timeout_sec', 0.2)
        self.declare_parameter('require_can', True)

        self._can_iface = self.get_parameter('can_iface').value
        self._dronecan_node_id = self.get_parameter('dronecan_node_id').value
        self._command_timeout = self.get_parameter('command_timeout_sec').value
        self._require_can = self.get_parameter('require_can').value

        self._latest_velocity = [0.0, 0.0, 0.0, 0.0]
        self._last_cmd_time = None

        self._feedback_pub = self.create_publisher(WheelFeedback, 'wheel_feedback', 10)
        self.create_subscription(WheelCommand, 'wheel_cmd', self._on_wheel_cmd, 10)

        self._dronecan_node = None
        if self._require_can:
            self._dronecan_node = self._make_dronecan_node()
            self.create_timer(0.01, self._spin_dronecan)
        else:
            self.get_logger().warning(
                'require_can=false: running in dry-run mode, no CAN device opened')

        rate_hz = self.get_parameter('command_rate_hz').value
        self.create_timer(1.0 / rate_hz, self._on_command_tick)

    def _make_dronecan_node(self):
        import dronecan
        _patch_python_can_flush_tx_buffer()
        # bitrate is required by this dronecan version's python-can driver even for SocketCAN,
        # which actually ignores it (the interface's real bitrate is set via `ip link`).
        node = dronecan.make_node(self._can_iface, node_id=self._dronecan_node_id,
                                   bitrate=1000000)
        node.add_handler(dronecan.uavcan.equipment.esc.Status, self._on_esc_status)
        # Referencing uavcan.equipment.actuator.Status registers its DTID (1011) in dronecan's
        # global type table -- steering actuators sharing the bus otherwise make spin() raise
        # "Unrecognised message type ID 1011" on every actuator.Status broadcast, since this
        # bridge only ever imports the esc.* messages on its own.
        node.add_handler(dronecan.uavcan.equipment.actuator.Status, lambda transfer_event: None)
        self.get_logger().info('DroneCAN node up on %s (node_id=%d)'
                                % (self._can_iface, self._dronecan_node_id))
        return node

    def _spin_dronecan(self) -> None:
        import dronecan
        try:
            # timeout=0 deliberately: this dronecan/python-can version pair multiplies any
            # nonzero timeout by 1000 before handing it to python-can's recv() (which wants
            # seconds, not ms), so e.g. timeout=0.01 would block for ~10s instead of 10ms.
            self._dronecan_node.spin(timeout=0)
        except dronecan.transport.TransferError as exc:
            # Bench steering firmware also broadcasts a high-rate custom message (observed DTID
            # 20601, no public DSDL for it) that isn't part of any standard DroneCAN message set
            # this bridge knows about -- can't be decoded or handled, just silently unrecognised.
            # Don't spam ERROR for that expected case; still surface anything else at debug level.
            if 'Unrecognised' not in str(exc):
                self.get_logger().error('DroneCAN transfer error: %s' % exc)
        except Exception as exc:  # noqa: BLE001 - surface any driver/transport error, keep node alive
            self.get_logger().error('DroneCAN spin error: %s' % exc)

    def _on_wheel_cmd(self, msg: WheelCommand) -> None:
        self._last_cmd_time = self.get_clock().now()
        self._latest_velocity = list(msg.velocity)

    def _on_command_tick(self) -> None:
        velocity = self._latest_velocity
        if self._last_cmd_time is not None:
            stale = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
            if stale > self._command_timeout:
                velocity = [0.0, 0.0, 0.0, 0.0]
        else:
            velocity = [0.0, 0.0, 0.0, 0.0]

        raw_cmd = [self._to_raw(v) for v in velocity]

        if self._dronecan_node is not None:
            import dronecan
            try:
                self._dronecan_node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=raw_cmd))
            except Exception as exc:  # noqa: BLE001 - a dropped command tick must not kill the node
                self.get_logger().error('DroneCAN broadcast error: %s' % exc)
        else:
            self.get_logger().debug('dry-run RawCommand: %s' % raw_cmd)

    @staticmethod
    def _to_raw(normalized: float) -> int:
        clamped = max(-1.0, min(1.0, normalized))
        return int(round(clamped * RAW_COMMAND_MAX_MAGNITUDE))

    def _on_esc_status(self, transfer_event) -> None:
        status = transfer_event.message
        msg = WheelFeedback()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.wheel_index = status.esc_index
        msg.rpm = float(status.rpm)
        msg.voltage = float(status.voltage)
        msg.current = float(status.current)
        self._feedback_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
