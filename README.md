# msd700_hardware

This package handle msd700 hardware. This includes handling serial communication with firmware (Arduino Mega) and RPLidar A1.

This package also handle additional feature:
- IMU Filter
- Odometry (from Motor's Encoder, IMU data)

## Available launches:
| Launch | Description |
|-|-|
| msd700_hardware.launch | Main Launch.<br>Launches everything from serial_arduino, lidar_scanner, odometry |
| lidar_scanner.launch | Launch communicatoin to LiDAR |
| serial_arduino.launch | Launch UART communication to firmware (Arduino Mega) and HardwareState parsing |
| odometry.launch | Calculate odometry based on IMU, Mag, and motor_pulse data |
