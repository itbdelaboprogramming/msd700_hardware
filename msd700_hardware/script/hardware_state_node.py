#!/usr/bin/env python3
# --- hardware_state_parser.py ---
# Parse the HardwareState msg and publish the datas

# Libraries
import rospy
import tf
import numpy as np

# Msgs
from msd700_msgs.msg import HardwareState
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Float32MultiArray

class HardwareStateParser:
    def __init__(self) -> None:
        # Parameters
        self.az_offset      = rospy.get_param('msd700_hardware/az_offset', 0.0)
        self.imu_frame      = rospy.get_param('msd700_hardware/imu_frame', 'base_link')
        self.mag_frame      = rospy.get_param('msd700_hardware/mag_frame', 'base_link')

        # Variables
        self.right_motor_pulse_delta = 0.0
        self.left_motor_pulse_delta = 0.0
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0
        self.acc_x, self.acc_y, self.acc_z = 0.0, 0.0, 0.0
        self.gyro_x, self.gyro_y, self.gyro_z = 0.0, 0.0, 0.0
        self.mag_x, self.mag_y, self.mag_z = 0.0, 0.0, 0.0
        self.uwb_dist, self.uwb_rho, self.uwb_dist, self.uwb_deviation = 0.0, 0.0, 0.0, 0.0

        self.az_offset = 0.0

        # Publisher
        self.imu_raw_pub        = rospy.Publisher('/imu/data_raw'           , Imu               , queue_size=10)
        self.mag_pub            = rospy.Publisher('/imu/mag'                , MagneticField     , queue_size=10)
        self.motor_pulse_pub    = rospy.Publisher('/hardware/motor_pulse'   , Float32MultiArray , queue_size=10)
        self.uwb_pub            = rospy.Publisher('/hardware/uwb'           , Float32MultiArray , queue_size=10)

        # Subscriber
        self.hardware_state_sub = rospy.Subscriber('/hardware_state', HardwareState, self.hardware_state_callback, queue_size=10)

    def hardware_state_callback(self, msg: HardwareState) -> None:
        # Update raw values
        self.right_motor_pulse_delta = msg.right_motor_pulse_delta
        self.left_motor_pulse_delta = msg.left_motor_pulse_delta

        self.roll = np.radians(msg.roll)
        self.pitch = np.radians(msg.pitch)

        self.acc_x = msg.acc_x
        self.acc_y = msg.acc_y
        self.acc_z = msg.acc_z + self.az_offset

        self.gyr_x = np.radians(msg.gyr_y)
        self.gyr_y = np.radians(msg.gyr_x)
        self.gyr_z = np.radians(msg.gyr_z)

        self.mag_x = msg.mag_x / 1e6
        self.mag_y = msg.mag_y / 1e6
        self.mag_z = msg.mag_z / 1e6

        self.uwb_dist = msg.uwb_dist
        self.uwb_dev = msg.uwb_deviation
        self.uwb_rho = msg.uwb_rho
        self.uwb_theta = msg.uwb_theta

        # Publish
        self.publish_imu_raw(self.roll, self.pitch, self.yaw
                            , self.acc_x, self.acc_y, self.acc_z
                            , self.gyr_x, self.gyr_y, self.gyr_z)
        self.publish_mag(self.mag_x, self.mag_y, self.mag_z)
        self.publish_motor_pulse(self.right_motor_pulse_delta, self.left_motor_pulse_delta)
        self.publish_uwb(self.uwb_dist, self.uwb_dev, self.uwb_rho, self.uwb_theta)

    def publish_imu_raw(self, roll: float, pitch: float, yaw: float
                        , acc_x: float, acc_y: float, acc_z: float
                        , gyr_x: float, gyr_y: float, gyr_z: float) -> None:
        imu_raw_msg = Imu()
        imu_raw_msg.header.stamp = rospy.Time.now()
        imu_raw_msg.header.frame_id = self.imu_frame

        imu_raw_msg.orientation = Quaternion(*tf.transformations.quaternion_from_euler(roll, pitch, yaw))
        imu_raw_msg.angular_velocity = Vector3(gyr_x, gyr_y, gyr_z)
        imu_raw_msg.linear_acceleration = Vector3(acc_x, acc_y, acc_z)

        imu_raw_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        imu_raw_msg.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        imu_raw_msg.linear_acceleration_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        self.imu_raw_pub.publish(imu_raw_msg)
    
    def publish_mag(self, mag_x: float, mag_y: float, mag_z: float) -> None:
        mag_msg = MagneticField()
        mag_msg.header.stamp = rospy.Time.now()
        mag_msg.header.frame_id = self.mag_frame
        mag_msg.magnetic_field = Vector3(mag_x, mag_y, mag_z)
        self.mag_pub.publish(mag_msg)
    
    def publish_motor_pulse(self, right_motor_pulse_delta: float, left_motor_pulse_delta: float) -> None:
        motor_pulse_msg = Float32MultiArray()
        motor_pulse_msg.data = [right_motor_pulse_delta, left_motor_pulse_delta]
        self.motor_pulse_pub.publish(motor_pulse_msg)
    
    def publish_uwb(self, uwb_dist: float, uwb_dev: float, uwb_rho: float, uwb_theta: float) -> None:
        uwb_msg = Float32MultiArray()
        uwb_msg.data = [uwb_dist, uwb_dev, uwb_rho, uwb_theta]
        self.uwb_pub.publish(uwb_msg)

if __name__ == '__main__':
    rospy.init_node("hardware_state_parser", anonymous=True)
    HardwareStateParser()
    rospy.spin()