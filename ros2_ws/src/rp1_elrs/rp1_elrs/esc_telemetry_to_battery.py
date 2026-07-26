"""Adapter: per-ESC telemetry -> sensor_msgs/BatteryState for the generic elrs_driver.

The ExpressLRS driver (`elrs_driver`, from the elrs_ros submodule) is robot-agnostic: it sends
handset battery telemetry from a standard `sensor_msgs/BatteryState`. rp1's per-wheel voltage and
current come off the VESCs in uavcan.equipment.esc.Status, which rp1_hardware_interface exports as
<gpio> state interfaces; joint_state_broadcaster publishes those on /dynamic_joint_states. This
thin node aggregates them into one pack-level BatteryState.

Previously this read rp1_msgs/WheelFeedback from rp1_dronecan_bridge. That node is gone -- the
DroneCAN traffic is the hardware component's now -- so the source is control_msgs/DynamicJointState
instead. /joint_states carries only position/velocity/effort, which is why this uses the dynamic
topic: it is the one that carries arbitrary interface names like voltage and current.

This is the *only* rp1-specific code in the ELRS path -- everything else (CRSF parsing, RC->Joy,
serial, failsafe, telemetry framing) is the generic elrs_ros package. See rp1/CLAUDE.md.
"""

import rclpy
from control_msgs.msg import DynamicJointState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class EscTelemetryToBattery(Node):

    def __init__(self):
        super().__init__('esc_telemetry_to_battery')

        # 'mean' of the ESCs that have reported, or one specific gpio name.
        self.declare_parameter('battery_source', 'mean')
        self.declare_parameter('publish_rate_hz', 5.0)
        # The <gpio> names in rp1_drive.urdf. Anything else on /dynamic_joint_states is ignored.
        self.declare_parameter('esc_names', [
            'esc_front_left', 'esc_front_right', 'esc_rear_left', 'esc_rear_right'])

        self._source = str(self.get_parameter('battery_source').value)
        self._esc_names = list(self.get_parameter('esc_names').value)

        self._voltage = {}
        self._current = {}

        self._pub = self.create_publisher(BatteryState, 'battery', 10)
        self.create_subscription(
            DynamicJointState, 'dynamic_joint_states', self._on_state, 10)

        rate = self.get_parameter('publish_rate_hz').value
        self.create_timer(1.0 / rate, self._tick)

    def _on_state(self, msg: DynamicJointState) -> None:
        for name, values in zip(msg.joint_names, msg.interface_values):
            if name not in self._esc_names:
                continue
            for interface, value in zip(values.interface_names, values.values):
                if interface == 'voltage':
                    self._voltage[name] = float(value)
                elif interface == 'current':
                    self._current[name] = float(value)

    def _aggregate(self, values):
        if self._source != 'mean':
            if self._source in values:
                return values[self._source]
            # Named ESC has not reported (yet): fall back to the mean rather than going silent.
        present = list(values.values())
        if not present:
            return None
        return sum(present) / len(present)

    def _tick(self) -> None:
        voltage = self._aggregate(self._voltage)
        if voltage is None:
            return  # no ESC telemetry seen yet
        current = self._aggregate(self._current) or 0.0
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = float(voltage)
        # ROS convention is negative-when-discharging; VESC current sign varies (regen can flip
        # it), and elrs_driver reports the magnitude anyway, so pass the pack value through as-is.
        msg.current = float(current)
        msg.percentage = float('nan')   # state-of-charge not estimated yet
        msg.present = True
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EscTelemetryToBattery()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C, or a launch/systemd supervisor sending SIGTERM: rclpy's signal handler
        # shuts the context down and spin() reports it as ExternalShutdownException.
        # That is a normal stop, so swallow it and exit 0 -- otherwise every clean
        # shutdown looks like a crash to whatever is supervising the node.
        pass
    except RuntimeError:
        # The same stop, different symptom: if the signal lands while the executor is building
        # its wait set, rclpy invalidates the context underneath itself and raises RCLError
        # ("the given context is not valid") instead. It is a RuntimeError subclass that only
        # exists in a private module, so match the base and gate on the context actually being
        # gone -- a RuntimeError with a live context is a real fault and must propagate.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        # try_shutdown(), not shutdown(): the context is already down in the
        # ExternalShutdownException case and shutdown() would raise on top of it.
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
