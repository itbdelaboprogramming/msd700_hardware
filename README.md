# msd700_hardware

This package handle msd700 hardware. This includes handling serial communication with firmware (Arduino Mega) and RPLidar A1.

This package also handle additional feature:
- IMU Filter
- Odometry (from Motor's Encoder, IMU data)

## Node Descriptions
### hardware_state_node.py
| Published Topic | Type | Description |
|-|-|-|
| `/imu/data_raw` | sensor_msgs/Imu | Imu data |
| `/imu/mag`  | sensor_msgs/MagneticField | Magnetic field data|
| `/hardware/motor_pulse` | std_msgs/Float32MultiArray | Array: [left_motor_pulse, right_motor_pulse] |
| `/hardware/uwb`  | std_msgs/Float32MultiArray | Array: [dist, deviation, rho, theta] |

| Subscribed Topic | Type | Description | 
|-|-|-|
| `/hardware_state` | msd700_msgs/HardwareState | Topic published by firmware |

### hardware_command_node.py
| Published Topic | Type | Description |
|-|-|-|
| `/hardware_command` | HardwareCommand  | Topic subscribed by firmware |

| Subscribed Topic | Type | Description | 
|-|-|-|
| `/hardware/cam_angle_command`| std_msgs/UInt8 | Cam angle 0-255 data |
| `/joint_states` | sensor_msgs/JointState | Joint State with "right_motor_speed" and "left_motor_speed" data |

### odom_node.py
| Published Topic | Type | Description |
|-|-|-|
| /odom | sensor_msgs/Odometry      | Odometry data (for navigation, etc) |
| /pose | geometry_msgs/PoseStamped | Accumulative move pose (for debugging, etc) |

| Subscribed Topic | Type | Description | 
|-|-|-|
| /imu/data_raw | sensor_msgs/Imu | Imu data for odometry calculation |
| /hardware/motor_pulse | std_msgs/Float32MultiArray | Motor pulse for odometry calculation |

### twist_to_rpm.py
This node is alternative of [diff_drive_controller from ros-control](http://wiki.ros.org/diff_drive_controller).

| Published Topic | Type | Description |
|-|-|-|
| /joint_states | sensor_msgs/JointState | Joint State with "right_motor_speed" and "left_motor_speed" data |

| Subscribed Topic | Type | Description | 
|-|-|-|
| /cmd_vel | geometry_msgs/Twist | Robot movement twist |

