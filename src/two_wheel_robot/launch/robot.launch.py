"""Bring up IMU, motor driver and joystick teleop.

    ros2 launch two_wheel_robot robot.launch.py
    ros2 launch two_wheel_robot robot.launch.py imu_port:=/dev/ttyACM0 motor_port:=/dev/hubmotor
    ros2 launch two_wheel_robot robot.launch.py use_joy:=false viewer:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("two_wheel_robot")
    robot_cfg = os.path.join(share, "config", "robot.yaml")
    joy_cfg = os.path.join(share, "config", "joy_teleop.yaml")

    imu_port = LaunchConfiguration("imu_port")
    motor_port = LaunchConfiguration("motor_port")
    use_joy = LaunchConfiguration("use_joy")
    use_imu = LaunchConfiguration("use_imu")
    use_motors = LaunchConfiguration("use_motors")
    viewer = LaunchConfiguration("viewer")

    return LaunchDescription([
        DeclareLaunchArgument("imu_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("motor_port", default_value="/dev/hubmotor"),
        DeclareLaunchArgument("use_imu", default_value="true"),
        DeclareLaunchArgument("use_motors", default_value="true"),
        DeclareLaunchArgument("use_joy", default_value="true"),
        DeclareLaunchArgument("viewer", default_value="false",
                              description="Open the live IMU plot window"),

        Node(
            package="two_wheel_robot", executable="imu_node", name="bno085_imu",
            parameters=[robot_cfg, {"port": imu_port}],
            condition=IfCondition(use_imu), output="screen",
        ),
        Node(
            package="two_wheel_robot", executable="motor_driver_node", name="zlac_motor_driver",
            parameters=[robot_cfg, {"port": motor_port}],
            condition=IfCondition(use_motors), output="screen",
        ),
        Node(
            package="joy", executable="joy_node", name="joy_node",
            parameters=[joy_cfg], condition=IfCondition(use_joy),
        ),
        Node(
            package="teleop_twist_joy", executable="teleop_node", name="teleop_twist_joy_node",
            parameters=[joy_cfg], condition=IfCondition(use_joy),
        ),
        Node(
            package="two_wheel_robot", executable="imu_viewer", name="imu_viewer",
            condition=IfCondition(viewer), output="screen",
        ),
    ])
