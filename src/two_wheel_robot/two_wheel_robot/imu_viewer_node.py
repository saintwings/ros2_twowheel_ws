#!/usr/bin/env python3
"""Live plot (and optional CSV log) of sensor_msgs/Imu.

ROS counterpart of esp32-bno085/tools/serial_gui.py: shows the last few
seconds of accel/gyro and the observed rate. Set the csv_path parameter
to also log every sample. For full-fidelity recording use `ros2 bag`.
"""

import csv
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
import rclpy
from matplotlib.animation import FuncAnimation
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

GUI_REFRESH_MS = 50
RATE_WINDOW_S = 1.0


class ImuViewerNode(Node):
    def __init__(self):
        super().__init__("imu_viewer")
        self.window_s = self.declare_parameter("plot_window_s", 5.0).value
        csv_path = self.declare_parameter("csv_path", "").value

        self.lock = threading.Lock()
        self.t = deque()
        self.series = {k: deque() for k in ("ax", "ay", "az", "gx", "gy", "gz")}
        self.rate_times = deque()
        self.total = 0
        self.t0 = time.monotonic()

        self.csv_file = None
        self.csv_writer = None
        if csv_path:
            self.csv_file = open(csv_path, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(
                ["host_time_s", "stamp_s", "ax", "ay", "az", "gx", "gy", "gz"])
            self.get_logger().info(f"Logging to {csv_path}")

        self.create_subscription(Imu, "imu/data_raw", self._on_imu, qos_profile_sensor_data)

    def _on_imu(self, msg):
        host_t = time.monotonic() - self.t0
        a, g = msg.linear_acceleration, msg.angular_velocity
        values = (a.x, a.y, a.z, g.x, g.y, g.z)
        with self.lock:
            self.total += 1
            self.t.append(host_t)
            for series, v in zip(self.series.values(), values):
                series.append(v)
            while self.t and host_t - self.t[0] > self.window_s:
                self.t.popleft()
                for series in self.series.values():
                    series.popleft()
            self.rate_times.append(host_t)
            while self.rate_times and host_t - self.rate_times[0] > RATE_WINDOW_S:
                self.rate_times.popleft()
        if self.csv_writer:
            stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.csv_writer.writerow((host_t, stamp_s) + values)

    def snapshot(self):
        with self.lock:
            t = list(self.t)
            data = {k: list(v) for k, v in self.series.items()}
            if len(self.rate_times) >= 2:
                span = self.rate_times[-1] - self.rate_times[0]
                hz = (len(self.rate_times) - 1) / span if span > 0 else 0.0
            else:
                hz = 0.0
            return t, data, hz, self.total

    def close(self):
        if self.csv_file:
            self.csv_file.close()


def main(args=None):
    rclpy.init(args=args)
    node = ImuViewerNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, (ax_accel, ax_gyro) = plt.subplots(2, 1, sharex=True, figsize=(10, 7))
    fig.canvas.manager.set_window_title("BNO085 IMU Viewer")
    ax_accel.set_title(f"Accelerometer (m/s^2) - last {node.window_s:.0f}s")
    ax_accel.set_ylabel("m/s^2")
    ax_gyro.set_title("Gyroscope (rad/s)")
    ax_gyro.set_xlabel("time (s)")
    ax_gyro.set_ylabel("rad/s")
    lines = {k: ax_accel.plot([], [], label=k)[0] for k in ("ax", "ay", "az")}
    lines.update({k: ax_gyro.plot([], [], label=k)[0] for k in ("gx", "gy", "gz")})
    ax_accel.legend(loc="upper right")
    ax_gyro.legend(loc="upper right")
    status = fig.text(0.01, 0.005, "Waiting for /imu/data_raw ...")
    fig.tight_layout(rect=(0, 0.03, 1, 1))

    def update(_frame):
        t, data, hz, total = node.snapshot()
        if not t:
            return
        xs = [x - t[0] for x in t]
        for k, line in lines.items():
            line.set_data(xs, data[k])
        xmax = max(node.window_s, xs[-1])
        for axis in (ax_accel, ax_gyro):
            axis.set_xlim(0, xmax)
            axis.relim()
            axis.autoscale_view(scalex=False, scaley=True)
        status.set_text(f"{total} samples | Rate: {hz:6.1f} Hz")

    _anim = FuncAnimation(fig, update, interval=GUI_REFRESH_MS, cache_frame_data=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
