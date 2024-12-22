#!/usr/bin/env python3
# --- odom_node.py ---
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
        self.imu_sub            = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.motor_pulse_sub    = rospy.Subscriber('/hardware/motor_pulse', Float32MultiArray, self.motor_pulse_callback)

        # Parameters
        self.compute_period     = rospy.get_param('msd700_odom/compute_period', 30)     # ms
        self.encoder_ppr        = rospy.get_param('msd700_odom/encoder_ppr', 1024)      # ppr
        self.wheel_distance     = rospy.get_param('msd700_odom/wheel_distance', 230)/100   # cm
        self.wheel_radius       = rospy.get_param('msd700_odom/wheel_radius', 23)/100     # cm
        self.use_imu            = rospy.get_param('msd700_odom/use_imu', True)
        self.debug              = rospy.get_param('msd700_odom/debug', False)

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
        self.rpy = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        # Timing
        self.last_time = None

        rospy.Timer(rospy.Duration(self.compute_period/1000), self.update_odom)

    # --- Sensor Callbacks ---
    def imu_callback(self, msg: Imu) -> None:
        # -- IMU DATA --
        # Get roll pitch yaw
        ort_q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(ort_q)
        self.rpy = [roll, pitch, yaw]

        # Get gyro
        self.gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]

    def motor_pulse_callback(self, msg: Float32MultiArray):
        # Get motor pulses
        self.right_motor_pulse_delta = msg.data[0]
        self.left_motor_pulse_delta = msg.data[1]
    
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
        self.theta += dtheta

        if self.use_imu:
            self.theta = self.rpy[2]

        # Debug
        if self.debug:
            rospy.loginfo(f"")
            rospy.loginfo(f'dt: {dt:.3f}, d: {d:.3f}, dtheta: {dtheta:.3f}')
            rospy.loginfo(f'x: {self.x:.3f}, y: {self.y:.3f}, theta: {self.theta:.3f}')
            rospy.loginfo(f"")
    
if __name__ == '__main__':
    rospy.init_node('odom_node')
    odom_node = OdomNode()
    rospy.spin()