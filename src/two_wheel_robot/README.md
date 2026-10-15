# two_wheel_robot (ROS 2 Humble)

Combines `twoWheelControl` (joystick → ZLAC8015D differential drive) and
`esp32-bno085` (BNO085 IMU stream + viewer) into one ROS 2 package.

```
joy_node ──/joy──> teleop_twist_joy ──/cmd_vel──> zlac_motor_driver ──Modbus RTU──> ZLAC8015D
ESP32-S3/BNO085 ──USB binary 250 Hz──> bno085_imu ──/imu/data_raw──> imu_viewer (optional), FAST-LIO, ...
```

| Node | Replaces | Notes |
|---|---|---|
| `bno085_imu` (`imu_node`) | serial reader in `tools/serial_gui.py` | Publishes `sensor_msgs/Imu` on `/imu/data_raw` (accel m/s², gyro rad/s, no orientation). Same packet/CRC decoding; stamps from the sensor clock mapped to ROS time; warns on dropped packets; auto-reconnects. |
| `zlac_motor_driver` (`motor_driver_node`) | `main.py` loop + `motor_controller.py` | Subscribes `/cmd_vel`. Same kinematics, RPM conversion/clamp and motor directions. Zeros wheels if `/cmd_vel` is silent for `cmd_timeout_s` (0.5 s). Logs `[FEEDBACK]` readback every second. Stops motors on exit. |
| `joy_node` + `teleop_twist_joy` | pygame joystick code | Same mapping: hold button 5 (R1/RB), axis 3 forward/back, axis 0 turn, deadzone 0.05. |
| `imu_viewer` | plot/log GUI in `tools/serial_gui.py` | Live accel/gyro plot + rate; optional CSV via `csv_path` param. |

The ESP32 firmware is unchanged — keep flashing it from `../esp32-bno085` with PlatformIO.

## Setup

```bash
pip install minimalmodbus          # no rosdep key; pyserial/matplotlib come from apt
sudo apt install ros-humble-joy ros-humble-teleop-twist-joy
sudo usermod -aG dialout $USER     # serial port access (log out/in)

cd ~/ros2_twowheel_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
# everything (IMU + motors + joystick)
ros2 launch two_wheel_robot robot.launch.py imu_port:=/dev/ttyACM0 motor_port:=/dev/hubmotor

# with the live IMU plot
ros2 launch two_wheel_robot robot.launch.py viewer:=true

# subsets
ros2 launch two_wheel_robot robot.launch.py use_imu:=false               # drive only
ros2 launch two_wheel_robot robot.launch.py use_motors:=false use_joy:=false viewer:=true   # IMU only

# individual nodes
ros2 run two_wheel_robot imu_node --ros-args -p port:=/dev/ttyACM0
ros2 run two_wheel_robot imu_viewer --ros-args -p csv_path:=imu_log.csv
ros2 run two_wheel_robot motor_driver_node --ros-args -p port:=/dev/ttyUSB1

# drive without a joystick
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```

Recording (replaces CSV logging): `ros2 bag record /imu/data_raw /cmd_vel`.

## Configuration

- `config/robot.yaml` — ports, wheel geometry, speed limits, motor directions, timeouts, IMU frame/covariances.
- `config/joy_teleop.yaml` — joystick buttons/axes/scales.

Both USB adapters can swap names between boots; consider udev rules
(e.g. `/dev/imu`, `/dev/hubmotor`) and point the launch args at them.

### Joystick sign note
ROS `joy_node` reports axes with the opposite sign to pygame, so the scales in
`joy_teleop.yaml` are sign-adjusted to reproduce the original behaviour
exactly. That includes the original turn direction: stick **left** gives a
negative `angular.z` (clockwise turn). If you want stick-left = turn-left
(the ROS convention), change `scale_angular.yaw` to `2.0`.

## Tests

```bash
python3 -m pytest src/two_wheel_robot/test
```
