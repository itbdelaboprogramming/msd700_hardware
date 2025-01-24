import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import yaml
from typing import Tuple

class TwistToRPM:
    def __init__(self) -> None:
        # Parameters
        self.wheel_distance     = rospy.get_param('msd700_odom/wheel_distance', 230)/100   # cm
        self.wheel_radius       = rospy.get_param('msd700_odom/wheel_radius', 23)/100     # cm
        self.debug              = rospy.get_param('msd700_odom/debug', False)

        # Publisher and Subscriber
        self.rpm_pub_           = rospy.Publisher('/hardware/rpm', Float32MultiArray, queue_size=1)
        self.twist_sub_         = rospy.Subscriber('/cmd_vel', Twist, self.twist_callback)
    
    def twist_to_rpm(self, linear_x:float, angular_z:float) -> Tuple[float, float]:
        # Wheel (left and right) velocity
        v_left = linear_x - angular_z * self.wheel_distance / 2
        v_right = linear_x + angular_z * self.wheel_distance / 2

        # Linear velocity (v) to angular velocity (w)
        w_left = v_left / self.wheel_radius
        w_right = v_right / self.wheel_radius

        # Angular velocity (w) to RPM
        rpm_left = w_left * 60 / (2 * 3.141592)
        rpm_right = w_right * 60 / (2 * 3.141592)

        return [rpm_left, rpm_right]

    def twist_callback(self, msg:Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        rpm_left, rpm_right = self.twist_to_rpm(linear_x, angular_z)

        rpm_msg = Float32MultiArray()
        rpm_msg.data = [rpm_left, rpm_right]
        self.rpm_pub_.publish(rpm_msg)

        if(self.debug):
            rospy.loginfo(f"--Twist--\n{msg}")
            rospy.loginfo(f"--RPM--\n{rpm_msg.data}")

if __name__ == "__main__":
    rospy.init_node('twist_to_rpm')
    twist_to_rpm = TwistToRPM()
    rospy.spin()



        

        