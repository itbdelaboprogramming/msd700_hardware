#!/usr/bin/env python3
# [odom_node.py]
# Publishes odometry data (topic and tf) from motor pulses and IMU data

import os, sys
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
import numpy as np

class OdomNode:
    def __init__(self):
        # Publisher
        self.odom_pub   = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.pose_pub   = rospy.Publisher('/pose', PoseStamped, queue_size=1)

        # TF Broadcaster
        self.odom_broadcaster   = tf.TransformBroadcaster()

        # Subscriber
        self.imu_sub            = rospy.Subscriber('/hardware/imu', Imu, self.imu_callback)
        self.motor_pulse_sub    = rospy.Subscriber('/hardware/motor_pulse', Float32MultiArray, self.motor_pulse_callback)

        # Publisher
        self.pose_pub           = rospy.Publisher('/hardware/pose', PoseStamped, queue_size=1)

        # Parameters
        self.compute_period     = rospy.get_param('msd700_odom/compute_period', 30)         # ms
        self.encoder_ppr        = rospy.get_param('msd700_odom/encoder_ppr', 12)            # ppr
        self.wheel_distance     = rospy.get_param('msd700_odom/wheel_distance', 230)/100.0  # cm
        self.wheel_radius       = rospy.get_param('msd700_odom/wheel_radius', 23)/100.0     # cm
        self.use_imu            = rospy.get_param('msd700_odom/use_imu', True)

        # Variables
        # Odometry data
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        # Motor Pulses
        self.right_motor_pulse_delta = 0.0
        self.left_motor_pulse_delta = 0.0
        # IMU data
        self.acc = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        self.rpy = [0.0, 0.0, 0.0]
        # Timing
        self.last_time = None
        self.last_debug = rospy.Time.now()

        rospy.Timer(rospy.Duration(self.compute_period/1000), self.update_odom)

    # --- Sensor Callbacks ---
    def imu_callback(self, msg: Imu) -> None:
        # -- IMU DATA --
        # Get acc
        self.acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]

        # Get gyro
        self.gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]

    def motor_pulse_callback(self, msg: Float32MultiArray):
        # Get motor pulses
        self.left_motor_pulse_delta = msg.data[0]
        self.right_motor_pulse_delta = msg.data[1]
    
    def update_odom(self, event) -> None:
        # Reference:
        # - https://medium.com/@nahmed3536/wheel-odometry-model-for-differential-drive-robotics-91b85a012299
        
        if self.last_time is None:
            self.last_time = rospy.Time.now()
            return
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Convert the motor pulses to distance (meter)
        pulses_to_distance = (2 * np.pi * self.wheel_radius) / self.encoder_ppr
        d_right = self.right_motor_pulse_delta * pulses_to_distance
        d_left = self.left_motor_pulse_delta * pulses_to_distance

        # Compute the delta dist (d) and theta (dtheta)
        d = (d_right + d_left) / 2.0
        dtheta = (d_right - d_left) / self.wheel_distance

        # Update pose
        self.x += d * np.cos(self.theta)
        self.y += d * np.sin(self.theta)

        if self.use_imu:
            # Acceleration-based tilt estimation (pitch and roll)
            accel_pitch = np.arctan2(-self.acc[0], np.sqrt(self.acc[1]**2 + self.acc[2]**2))
            accel_roll = np.arctan2(self.acc[1], self.acc[2])

            # Gyroscope integration
            gyro_pitch = self.rpy[1] + self.gyro[0] * dt
            gyro_roll = self.rpy[0] + self.gyro[1] * dt
            gyro_yaw = self.rpy[2] + self.gyro[2] * dt

            # Complementary filter: 0.98 factor
            self.rpy[0] = 0.98 * gyro_roll + 0.02 * accel_roll  # Roll
            self.rpy[1] = 0.98 * gyro_pitch + 0.02 * accel_pitch  # Pitch
            self.rpy[2] = gyro_yaw # Yaw

            # Update heading with IMU yaw
            self.theta = self.rpy[2]
        else:
            self.theta += dtheta

        # Publish
        self.publish_odom()
    
    def publish_odom(self) -> None:
        # --- Publish Odom ---
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position = Point(self.x, self.y, 0)
        odom_msg.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, self.theta))
        odom_msg.twist.twist.linear = Vector3(self.linear_velocity, 0, 0)
        odom_msg.twist.twist.angular = Vector3(0, 0, self.angular_velocity)
        self.odom_pub.publish(odom_msg)

        # --- Publish TF ---
        self.odom_broadcaster.sendTransform((self.x, self.y, 0), tf.transformations.quaternion_from_euler(0, 0, self.theta), rospy.Time.now(), 'base_link', 'odom')
    
if __name__ == '__main__':
    rospy.init_node('odom_node')
    odom_node = OdomNode()
    rospy.spin()