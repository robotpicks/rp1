"""ICM20948 9-DOF IMU driver (accel + gyro + AK09916 magnetometer) over SPI.

UNVERIFIED AGAINST REAL HARDWARE. Written from the public ICM20948/AK09916 register maps
without a physical sensor to test against -- there is no ICM20948 in this development
sandbox. Confirm WHO_AM_I reads back 0xEA and sanity-check accel (should read ~9.81 m/s^2
on the axis pointing down when stationary) and gyro (should read ~0 rad/s when stationary)
against real hardware before trusting this for anything closed-loop. The magnetometer path
(register bank 3's I2C master passing through to the AK09916 at I2C address 0x0C) is the
fiddliest part of this chip's interface and the most likely place for a subtle bug -- if
MagneticField readings look wrong, look there first.

Design conventions match elrs_driver_node.py (elrs_ros submodule) for consistency across
rp1's hardware drivers: the transport lib (spidev) is imported lazily so the package builds
and dry-runs without it; require_spi: false opens no device and logs intended reads instead
(handy for CI / bring-up without hardware); and all SPI I/O is wrapped so a hardware hiccup
logs but never kills the node.

No ROS2 package for this specific chip was available in this distro's apt repos (checked:
only generic/other-chip IMU drivers exist), hence a from-scratch driver rather than
configuring an existing one -- unlike GPS, where the ublox_gps package is used directly.
"""

import struct

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node as RosNode
from sensor_msgs.msg import Imu, MagneticField

# -- ICM20948 register map (bank-relative addresses; every bank shares REG_BANK_SEL) -------
REG_BANK_SEL = 0x7F

# Bank 0
B0_WHO_AM_I = 0x00
B0_USER_CTRL = 0x03
B0_PWR_MGMT_1 = 0x06
B0_PWR_MGMT_2 = 0x07
B0_INT_PIN_CFG = 0x0F
B0_ACCEL_XOUT_H = 0x2D  # 14 contiguous bytes: accel xyz, gyro xyz, temp
WHO_AM_I_EXPECTED = 0xEA

# Bank 2
B2_GYRO_SMPLRT_DIV = 0x00
B2_GYRO_CONFIG_1 = 0x01
B2_ACCEL_SMPLRT_DIV_1 = 0x10
B2_ACCEL_SMPLRT_DIV_2 = 0x11
B2_ACCEL_CONFIG = 0x14

# Bank 3 -- I2C master, used to reach the AK09916 magnetometer behind it
B3_I2C_MST_CTRL = 0x01
B3_I2C_SLV0_ADDR = 0x03
B3_I2C_SLV0_REG = 0x04
B3_I2C_SLV0_CTRL = 0x05
B3_I2C_SLV0_DO = 0x06

# AK09916 (magnetometer), reached via I2C_SLV0 -- I2C_SLV0_ADDR's read bit (0x80) is what
# makes SLV0 a read from this device rather than a write to it.
AK09916_I2C_ADDR = 0x0C
AK09916_READ = 0x80
AK09916_HXL = 0x11  # 6 contiguous bytes: x, y, z (little-endian int16 each)
AK09916_ST2 = 0x18  # must be read after HXL..HZH to latch the next sample (datasheet requirement)
AK09916_CNTL2 = 0x31
AK09916_CNTL3 = 0x32
AK09916_MODE_CONTINUOUS_100HZ = 0x08

# Accel full-scale select -> (register bits, sensitivity in LSB per m/s^2)
_G = 9.80665
ACCEL_FS = {2: (0b00, 16384 / _G), 4: (0b01, 8192 / _G), 8: (0b10, 4096 / _G), 16: (0b11, 2048 / _G)}
# Gyro full-scale select -> (register bits, sensitivity in LSB per rad/s)
GYRO_FS = {250: (0b00, 131.0), 500: (0b01, 65.5), 1000: (0b10, 32.8), 2000: (0b11, 16.4)}
DEG_PER_LSB_TO_RAD = 3.14159265358979 / 180.0
MAG_TESLA_PER_LSB = 0.15e-6  # AK09916: 0.15 uT/LSB


class Icm20948DriverNode(RosNode):

    def __init__(self):
        super().__init__('icm20948_driver')

        self.declare_parameter('spi_bus', 0)
        self.declare_parameter('spi_device', 0)
        self.declare_parameter('spi_max_speed_hz', 1000000)
        self.declare_parameter('require_spi', True)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('accel_fs_g', 8)      # must be a key of ACCEL_FS
        self.declare_parameter('gyro_fs_dps', 1000)  # must be a key of GYRO_FS
        self.declare_parameter('enable_magnetometer', True)

        self._frame_id = self.get_parameter('frame_id').value
        self._require_spi = self.get_parameter('require_spi').value
        self._enable_mag = self.get_parameter('enable_magnetometer').value

        accel_fs_g = self.get_parameter('accel_fs_g').value
        gyro_fs_dps = self.get_parameter('gyro_fs_dps').value
        if accel_fs_g not in ACCEL_FS:
            self.get_logger().error(
                f'accel_fs_g={accel_fs_g} not one of {list(ACCEL_FS)}; using 8')
            accel_fs_g = 8
        if gyro_fs_dps not in GYRO_FS:
            self.get_logger().error(
                f'gyro_fs_dps={gyro_fs_dps} not one of {list(GYRO_FS)}; using 1000')
            gyro_fs_dps = 1000
        self._accel_fs_bits, self._accel_lsb_per_ms2 = ACCEL_FS[accel_fs_g]
        self._gyro_fs_bits, self._gyro_lsb_per_dps = GYRO_FS[gyro_fs_dps]

        self._spi = None
        self._imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self._mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10) if self._enable_mag else None

        if self._require_spi:
            self._spi = self._open_spi()
            self._configure_chip()
            rate_hz = self.get_parameter('rate_hz').value
            self.create_timer(1.0 / rate_hz, self._poll)
        else:
            self.get_logger().warning(
                'require_spi=false: dry-run, no SPI device opened; no data will be published')

    # -- SPI transport -----------------------------------------------------------------------

    def _open_spi(self):
        import spidev  # lazy: not required to import/build the package without hardware
        bus = self.get_parameter('spi_bus').value
        device = self.get_parameter('spi_device').value
        speed = self.get_parameter('spi_max_speed_hz').value
        spi = spidev.SpiDev()
        spi.open(bus, device)
        spi.max_speed_hz = speed
        spi.mode = 0b00  # ICM20948: SPI mode 0 (CPOL=0, CPHA=0)
        self.get_logger().info(f'SPI open: bus={bus} device={device} speed={speed}')
        return spi

    def _select_bank(self, bank: int) -> None:
        self._write_reg(REG_BANK_SEL, bank << 4)

    def _write_reg(self, reg: int, value: int) -> None:
        self._spi.xfer2([reg & 0x7F, value & 0xFF])

    def _read_regs(self, reg: int, n: int) -> bytes:
        result = self._spi.xfer2([reg | 0x80] + [0x00] * n)
        return bytes(result[1:])

    # -- one-time chip configuration ----------------------------------------------------------

    def _configure_chip(self) -> None:
        self._select_bank(0)
        who_am_i = self._read_regs(B0_WHO_AM_I, 1)[0]
        if who_am_i != WHO_AM_I_EXPECTED:
            self.get_logger().error(
                f'ICM20948 WHO_AM_I=0x{who_am_i:02X}, expected 0x{WHO_AM_I_EXPECTED:02X} -- '
                'wrong chip, bad wiring, or wrong SPI bus/device. Continuing anyway; data is '
                'suspect until this is fixed.')

        self._select_bank(0)
        self._write_reg(B0_PWR_MGMT_1, 0x01)   # wake from sleep, auto-select best clock source
        self._write_reg(B0_PWR_MGMT_2, 0x00)   # enable accel + gyro (both axes triplets on)

        self._select_bank(2)
        self._write_reg(B2_GYRO_CONFIG_1, (self._gyro_fs_bits << 1) | 0x01)   # FS + DLPF enabled
        self._write_reg(B2_ACCEL_CONFIG, (self._accel_fs_bits << 1) | 0x01)  # FS + DLPF enabled
        self._write_reg(B2_GYRO_SMPLRT_DIV, 0x00)
        self._write_reg(B2_ACCEL_SMPLRT_DIV_1, 0x00)
        self._write_reg(B2_ACCEL_SMPLRT_DIV_2, 0x00)

        if self._enable_mag:
            self._configure_magnetometer()

        self._select_bank(0)  # leave the chip on bank 0, where the data registers live

    def _configure_magnetometer(self) -> None:
        """Route the AK09916 through the ICM20948's auxiliary I2C master. This is the part of
        the chip's interface most likely to have a subtle bug -- see module docstring."""
        self._select_bank(0)
        self._write_reg(B0_USER_CTRL, 0x20)     # I2C_MST_EN
        self._select_bank(3)
        self._write_reg(B3_I2C_MST_CTRL, 0x07)  # I2C master clock ~345.6kHz (datasheet table)

        # Reset then set the magnetometer to continuous-measurement mode via a one-shot SLV0
        # write, before configuring SLV0 for the ongoing reads used in _read_magnetometer().
        self._i2c_master_write(AK09916_CNTL3, 0x01)  # soft reset
        self._i2c_master_write(AK09916_CNTL2, AK09916_MODE_CONTINUOUS_100HZ)

        # Configure SLV0 for a standing 8-byte read starting at HXL (6 data bytes + ST2, so
        # each poll's read also clears AK09916's data-ready latch per its datasheet).
        self._write_reg(B3_I2C_SLV0_ADDR, AK09916_I2C_ADDR | AK09916_READ)
        self._write_reg(B3_I2C_SLV0_REG, AK09916_HXL)
        self._write_reg(B3_I2C_SLV0_CTRL, 0x80 | 0x08)  # enable, 8 bytes
        self._select_bank(0)

    def _i2c_master_write(self, ak09916_reg: int, value: int) -> None:
        """One-shot write to the AK09916 through SLV0 (used only during setup -- the ongoing
        magnetometer reads in _read_magnetometer() reuse SLV0 configured for reading)."""
        self._write_reg(B3_I2C_SLV0_ADDR, AK09916_I2C_ADDR)  # write, not AK09916_READ
        self._write_reg(B3_I2C_SLV0_REG, ak09916_reg)
        self._write_reg(B3_I2C_SLV0_DO, value)
        self._write_reg(B3_I2C_SLV0_CTRL, 0x80 | 0x01)  # enable, 1 byte

    # -- polling -------------------------------------------------------------------------------

    def _poll(self) -> None:
        try:
            self._select_bank(0)
            raw = self._read_regs(B0_ACCEL_XOUT_H, 14)
        except Exception as exc:  # noqa: BLE001 - an SPI error must not kill the node
            self.get_logger().error(f'SPI read error: {exc}')
            return
        ax, ay, az, gx, gy, gz, _temp = struct.unpack('>7h', raw)
        self._publish_imu(ax, ay, az, gx, gy, gz)

        if self._enable_mag:
            self._poll_magnetometer()

    def _publish_imu(self, ax, ay, az, gx, gy, gz) -> None:
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.linear_acceleration.x = ax / self._accel_lsb_per_ms2
        msg.linear_acceleration.y = ay / self._accel_lsb_per_ms2
        msg.linear_acceleration.z = az / self._accel_lsb_per_ms2
        msg.angular_velocity.x = (gx / self._gyro_lsb_per_dps) * DEG_PER_LSB_TO_RAD
        msg.angular_velocity.y = (gy / self._gyro_lsb_per_dps) * DEG_PER_LSB_TO_RAD
        msg.angular_velocity.z = (gz / self._gyro_lsb_per_dps) * DEG_PER_LSB_TO_RAD
        # No orientation estimate here (no on-chip DMP fusion is configured, and this node does
        # no sensor fusion of its own) -- orientation_covariance[0] = -1 is sensor_msgs/Imu's
        # documented way of saying "orientation not provided." Pair this topic with
        # imu_filter_madgwick or imu_complementary_filter downstream if orientation is needed.
        msg.orientation_covariance[0] = -1.0
        self._imu_pub.publish(msg)

    def _poll_magnetometer(self) -> None:
        try:
            self._select_bank(3)
            self._write_reg(B3_I2C_SLV0_CTRL, 0x80 | 0x08)  # re-trigger the standing SLV0 read
            self._select_bank(0)
            ext_sens_data = self._read_regs(0x3B, 8)  # EXT_SENS_DATA_00.. bank 0
        except Exception as exc:  # noqa: BLE001 - an SPI error must not kill the node
            self.get_logger().error(f'SPI read error (magnetometer): {exc}')
            return
        mx, my, mz = struct.unpack('<3h', ext_sens_data[0:6])
        st2 = ext_sens_data[7]
        if st2 & 0x08:  # HOFL: magnetic sensor overflow, per AK09916 datasheet
            self.get_logger().warning('AK09916 HOFL (magnetic overflow) -- reading discarded')
            return
        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.magnetic_field.x = mx * MAG_TESLA_PER_LSB
        msg.magnetic_field.y = my * MAG_TESLA_PER_LSB
        msg.magnetic_field.z = mz * MAG_TESLA_PER_LSB
        self._mag_pub.publish(msg)

    def destroy_node(self) -> bool:
        if self._spi is not None:
            try:
                self._spi.close()
            except Exception as exc:  # a failed close must not mask the shutdown itself
                self.get_logger().warning(f'error closing SPI device: {exc}')
            self._spi = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Icm20948DriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
