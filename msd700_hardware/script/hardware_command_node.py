#!/usr/bin/env python

"""
* This node is Hardware Interface to control MSD700's robot hardware.
It publishes HardwareCommand message to /hardware_command.

* Subscribed Topics:
- /hardware/cam_angle_command (UInt8): Camera angle command.
- /joint_states (JointState): Joint states of the robot.

* Published Topics: /hardware_command (HardwareCommand)
HardwareCommand   | Type      | Topic
-----------------------------------------------------------------
movement_command  | UInt8     | 0
cam_angle_command | UInt8     | /hardware/cam_angle_command
right_motor_speed | Float32   | /joint_states
left_motor_speed  | Float32   | /joint_states
"""

import rospy
from std_msgs.msg import UInt8
from sensor_msgs.msg import JointState
from msd700_msgs.msg import HardwareCommand
import math

class HardwareCommandNode:
    def __init__(self) -> None:
        # Parameter
        self.compute_period         = rospy.get_param("~compute_period"         , 0.1)
        self.cam_angle_topic        = rospy.get_param("~cam_angle_topic"        , "/hardware/cam_angle_command")
        self.joint_state_topic      = rospy.get_param("~joint_state_topic"      , "/joint_states")
        self.hardware_command_topic = rospy.get_param("~hardware_command_topic" , "/hardware_command")
        self.left_joint_name        = rospy.get_param("~left_joint_name"        , "left_wheel_joint")
        self.right_joint_name       = rospy.get_param("~right_joint_name"       , "right_wheel_joint")

        # Variables
        self.movement_command = 0
        self.cam_angle_command = 0
        self.right_motor_speed = 0.0
        self.left_motor_speed = 0.0

        # Publisher and Subscriber
        self.command_pub    = rospy.Publisher(self.hardware_command_topic, HardwareCommand, queue_size=10)
        self.cam_angle_sub  = rospy.Subscriber(self.cam_angle_topic, UInt8, self.cam_angle_callback)
        self.joint_sub      = rospy.Subscriber(self.joint_state_topic, JointState, self.joint_state_callback)

        # Timer
        self.mainTimer = rospy.Timer(rospy.Duration(self.compute_period), self.publish_command)

    def cam_angle_callback(self, msg: UInt8) -> None:
        """ Update cam angle command. """
        self.cam_angle_command = msg.data

    def joint_state_callback(self, msg: JointState) -> None:
        """ Update motor speed based on joint_states topic. """

        # Check if right_motor_joint and left_motor_joint are in the joint_states
        if self.right_joint_name not in msg.name or self.left_joing_name not in msg.name:
            rospy.logwarn(f"{self.right_joint_name} or {self.left_joint_name} not found in {self.joint_state_topic}")
            return

        # Update value
        right_index = msg.name.index(self.right_joint_name)
        left_index  = msg.name.index(self.left_joint_name)
        right_motor_speed_rad_s = msg.velocity[right_index] # rad/s
        left_motor_speed_rad_s  = msg.velocity[left_index]  # rad/s
        self.right_motor_speed  = right_motor_speed_rad_s * 60 / (2 * math.pi) # rpm
        self.left_motor_speed   = left_motor_speed_rad_s * 60 / (2 * math.pi)  # rpm
    
    def publish_command(self, event) -> None:
        """ Publish hardware command to hardware_command topic """
        command = HardwareCommand()
        command.movement_command = self.movement_command
        command.cam_angle_command = self.cam_angle_command
        command.right_motor_speed = self.right_motor_speed
        command.left_motor_speed = self.left_motor_speed
        self.command_pub.publish(command)

if __name__ == "__main__":
    rospy.init_node("hardware_command_node", anonymous=True)
    hardware_command_node = HardwareCommandNode()
    rospy.spin()
