# PAVbot E-Stop ROS Node

This repository contains the ROS 2 interface node for the PAVbot
emergency stop system using an Arduino Nano.

## Function
- Reads E-stop state from Arduino over serial
- Publishes `/safety/estop` (`std_msgs/Bool`)
- Fail-safe: if serial data is lost, estop is ACTIVE

## Topic Interface
| Topic | Type | Description |
|------|------|-------------|
| /safety/estop | std_msgs/Bool | true = STOP |

## Parameters
- port (string): Serial device (default: /dev/ttyUSB0)
- baud (int): Baud rate (default: 115200)
- timeout_ms (int): Fail-safe timeout

## Run
```bash
cd ros2_ws
colcon build
source install/setup.bash
ros2 launch pav_estop estop.launch.py
