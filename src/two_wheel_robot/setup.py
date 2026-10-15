from glob import glob

from setuptools import setup

package_name = "two_wheel_robot"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="saintwings",
    maintainer_email="wisanu.jutharee@gmail.com",
    description="Two-wheel ZLAC8015D base with joystick teleop and ESP32-S3/BNO085 IMU.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "imu_node = two_wheel_robot.imu_node:main",
            "motor_driver_node = two_wheel_robot.motor_driver_node:main",
            "imu_viewer = two_wheel_robot.imu_viewer_node:main",
        ],
    },
)
