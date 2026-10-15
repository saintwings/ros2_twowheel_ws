#!/usr/bin/env python3
"""Publish the ESP32-S3 / BNO085 binary IMU stream as sensor_msgs/Imu.

Raw accel + gyro only (no fused orientation), which is what IMU
preintegration / ESIKF fusion such as FAST-LIO expects.
"""

import signal
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import Imu

import serial

from two_wheel_robot.bno085_protocol import PacketParser

ACCURACY_NAMES = ("unreliable", "low", "medium", "high")


class SensorClockSync:
    """Map the sensor's microsecond clock onto ROS time.

    Keeps the smallest observed (host - sensor) offset, which removes USB
    latency jitter, and lets it creep upward slowly so clock drift between
    the ESP32 and the host is followed. Resets if the sensor clock jumps.
    """

    def __init__(self, drift_s_per_s=2e-4, reset_threshold_s=0.1):
        self.drift = drift_s_per_s
        self.reset_threshold = reset_threshold_s
        self.offset = None
        self.last_host = None

    def to_host(self, host_s, sensor_us):
        sensor_s = sensor_us * 1e-6
        candidate = host_s - sensor_s
        if self.offset is None or abs(candidate - self.offset) > self.reset_threshold:
            self.offset = candidate
        else:
            self.offset = min(candidate, self.offset + self.drift * (host_s - self.last_host))
        self.last_host = host_s
        return sensor_s + self.offset


class Bno085ImuNode(Node):
    def __init__(self):
        super().__init__("bno085_imu")
        self.port = self.declare_parameter("port", "/dev/ttyACM0").value
        self.baud = self.declare_parameter("baud", 921600).value
        self.frame_id = self.declare_parameter("frame_id", "imu_link").value
        self.use_sensor_timestamp = self.declare_parameter("use_sensor_timestamp", True).value
        accel_var = self.declare_parameter("linear_acceleration_variance", 0.0).value
        gyro_var = self.declare_parameter("angular_velocity_variance", 0.0).value

        self.accel_cov = [accel_var, 0.0, 0.0, 0.0, accel_var, 0.0, 0.0, 0.0, accel_var]
        self.gyro_cov = [gyro_var, 0.0, 0.0, 0.0, gyro_var, 0.0, 0.0, 0.0, gyro_var]

        self.pub = self.create_publisher(Imu, "imu/data_raw", qos_profile_sensor_data)
        self.clock_sync = SensorClockSync()
        self.parser = PacketParser()
        self.last_sequence = None
        self.dropped = 0
        self.last_status = (None, None)

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _open_serial(self):
        while not self._stop.is_set():
            try:
                ser = serial.Serial(self.port, self.baud, timeout=0.1)
                self.get_logger().info(f"Opened {self.port} @ {self.baud}")
                return ser
            except serial.SerialException as exc:
                self.get_logger().warn(f"Cannot open {self.port}: {exc}; retrying",
                                       throttle_duration_sec=5.0)
                self._stop.wait(1.0)
        return None

    def _read_loop(self):
        ser = None
        while not self._stop.is_set():
            if ser is None:
                ser = self._open_serial()
                if ser is None:
                    return
                self.parser = PacketParser()
                self.last_sequence = None
            try:
                raw = ser.read(256)
            except serial.SerialException as exc:
                self.get_logger().error(f"Serial error: {exc}; reconnecting")
                ser.close()
                ser = None
                continue
            if not raw:
                continue
            if not self.context.ok():
                break
            host_ns = self.get_clock().now().nanoseconds
            try:
                for sample in self.parser.feed(raw):
                    self._publish(sample, host_ns)
            except Exception:
                if self.context.ok():
                    raise
                break  # shutting down
        if ser is not None:
            ser.close()

    def _publish(self, s, host_ns):
        if self.last_sequence is not None:
            gap = (s.sequence - self.last_sequence - 1) & 0xFFFF
            if gap:
                self.dropped += gap
                self.get_logger().warn(f"{self.dropped} IMU packets dropped so far",
                                       throttle_duration_sec=5.0)
        self.last_sequence = s.sequence

        status = (s.accel_status, s.gyro_status)
        if status != self.last_status:
            self.get_logger().info(
                f"Accuracy: accel={ACCURACY_NAMES[s.accel_status & 3]} "
                f"gyro={ACCURACY_NAMES[s.gyro_status & 3]}")
            self.last_status = status

        msg = Imu()
        if self.use_sensor_timestamp:
            stamp_s = self.clock_sync.to_host(host_ns * 1e-9, s.timestamp_us)
            msg.header.stamp = Time(nanoseconds=int(stamp_s * 1e9)).to_msg()
        else:
            msg.header.stamp = Time(nanoseconds=host_ns).to_msg()
        msg.header.frame_id = self.frame_id
        msg.orientation_covariance[0] = -1.0  # orientation not provided
        msg.linear_acceleration.x = s.ax
        msg.linear_acceleration.y = s.ay
        msg.linear_acceleration.z = s.az
        msg.linear_acceleration_covariance = self.accel_cov
        msg.angular_velocity.x = s.gx
        msg.angular_velocity.y = s.gy
        msg.angular_velocity.z = s.gz
        msg.angular_velocity_covariance = self.gyro_cov
        self.pub.publish(msg)

    def destroy_node(self):
        self._stop.set()
        self._thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Bno085ImuNode()
    try:
        # Serial work happens on the reader thread; spin with a timeout so
        # Python gets a chance to raise KeyboardInterrupt.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # ros2 launch and the terminal both send SIGINT; don't let the second cut cleanup short.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
