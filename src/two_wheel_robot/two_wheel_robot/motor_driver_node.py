#!/usr/bin/env python3
"""Drive a ZLAC8015D two-wheel base from geometry_msgs/Twist on /cmd_vel.

Differential drive: v_wheel = v_linear +- (omega * wheelbase / 2).
If no command arrives within cmd_timeout the wheels are commanded to 0,
which replaces the deadman behaviour of the old pygame loop.
"""

import signal

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from two_wheel_robot.zlac8015d import ZLAC8015DController


def clamp(value, limit):
    return max(-limit, min(limit, value))


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__("zlac_motor_driver")
        p = self.declare_parameter
        self.port = p("port", "/dev/ttyUSB1").value
        self.device_id = p("device_id", 1).value
        self.baudrate = p("baudrate", 115200).value
        self.wheelbase = p("wheelbase_m", 0.35).value
        self.wheel_diameter = p("wheel_diameter_m", 0.10).value
        self.gear_ratio = p("gear_ratio", 1.0).value
        self.max_wheel_rpm = p("max_wheel_rpm", 100.0).value
        self.max_linear = p("max_linear_mps", 0.6).value
        self.max_angular = p("max_angular_rad_s", 2.0).value
        self.motor_directions = list(p("motor_directions", [1, -1]).value)
        self.cmd_timeout = p("cmd_timeout_s", 0.5).value
        control_period = p("control_period_s", 0.05).value
        feedback_period = p("feedback_period_s", 1.0).value

        self.linear = 0.0
        self.angular = 0.0
        self.last_cmd_time = None
        self.zlac = None
        self.last_connect_attempt = None

        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self.create_timer(control_period, self._control_tick)
        if self.device_id != 0 and feedback_period > 0:
            self.create_timer(feedback_period, self._feedback_tick)
        elif self.device_id == 0:
            self.get_logger().info("Broadcast mode: controller readback is unavailable")

        self._connect()
        self.get_logger().info(
            f"Ready: wheelbase={self.wheelbase} m, wheel_diameter={self.wheel_diameter} m, "
            f"max {self.max_linear} m/s / {self.max_angular} rad/s")

    def _connect(self):
        self.last_connect_attempt = self.get_clock().now()
        try:
            self.zlac = ZLAC8015DController(
                self.port,
                device_id=self.device_id,
                baudrate=self.baudrate,
                wheel_diameter_m=self.wheel_diameter,
                gear_ratio=self.gear_ratio,
                max_rpm=self.max_wheel_rpm,
            )
            self.zlac.set_velocity_mode()
            self.get_logger().info(f"Motor controller initialized on {self.port}")
        except Exception as exc:  # serial or Modbus failure
            self.get_logger().error(f"Motor controller init failed: {exc}; retrying",
                                    throttle_duration_sec=5.0)
            self._disconnect()

    def _disconnect(self):
        if self.zlac is not None:
            try:
                self.zlac.close()
            except Exception:
                pass
        self.zlac = None

    def _on_cmd_vel(self, msg):
        self.linear = clamp(msg.linear.x, self.max_linear)
        self.angular = clamp(msg.angular.z, self.max_angular)
        self.last_cmd_time = self.get_clock().now()

    def _command_is_fresh(self):
        if self.last_cmd_time is None:
            return False
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        return age <= self.cmd_timeout

    def _control_tick(self):
        if self.zlac is None:
            since = (self.get_clock().now() - self.last_connect_attempt).nanoseconds * 1e-9
            if since >= 1.0:
                self._connect()
            if self.zlac is None:
                return

        if self._command_is_fresh():
            left = self.linear - self.angular * self.wheelbase / 2.0
            right = self.linear + self.angular * self.wheelbase / 2.0
        else:
            left = right = 0.0

        try:
            left_rpm, right_rpm = self.zlac.set_velocity_mps(
                self.motor_directions[0] * left,
                self.motor_directions[1] * right,
            )
        except Exception as exc:
            self.get_logger().warn(f"Velocity write failed: {exc}", throttle_duration_sec=2.0)
            return
        self.get_logger().debug(
            f"left {left:.3f} m/s ({left_rpm} RPM), right {right:.3f} m/s ({right_rpm} RPM)")

    def _feedback_tick(self):
        if self.zlac is None:
            return
        try:
            device_id = self.zlac.read_device_id()
            left_rpm, right_rpm = self.zlac.read_velocity_command()
        except Exception as exc:
            self.get_logger().warn(f"Controller read failed: {exc}", throttle_duration_sec=5.0)
            return
        self.get_logger().info(
            f"[FEEDBACK] device_id={device_id} target_left={left_rpm} RPM "
            f"target_right={right_rpm} RPM")

    def stop(self):
        if self.zlac is None:
            return
        try:
            self.zlac.set_velocity(0, 0)
            self.get_logger().info("Motors stopped")
        except Exception as exc:
            self.get_logger().error(f"Failed to stop motors: {exc}")
        self._disconnect()


def main(args=None):
    # Handle Ctrl-C ourselves so the context is still valid while stopping the motors.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # ros2 launch and the terminal both send SIGINT; don't let the second cut cleanup short.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
