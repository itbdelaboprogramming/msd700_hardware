import rospy
from nav_msgs.msg import Odometry
import math

class OdomTuneNode:
    def __init__(self):
        rospy.init_node('odom_tune_node')

        # Parameters
        self.compute_period = rospy.get_param('msd700_odom/compute_period', 30)         # ms
        self.current_ppr = rospy.get_param('msd700_odom/encoder_ppr', 12)            # ppr

        # Variables
        self.odom_dist      = 0.0
        self.real_dist  = 0.0

        # Subscriber
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.calculate_real_ppr()
    
    def odom_callback(self, msg: Odometry) -> None:
        self.odom_dist = msg.pose.pose.position.x

    def calculate_real_ppr(self):
        rospy.loginfo(f"Move the robot forward for a certain distance.")
        self.real_dist = float(input(f"Please enter the actual distance:\n"))
        real_ppr = self.real_dist * self.current_ppr / self.odom_dist
        rospy.loginfo(f"|----------------------------------------|")
        rospy.loginfo(f"| Current PPR         : {self.current_ppr}")
        rospy.loginfo(f"| Odometry Distance   : {self.odom_dist}")
        rospy.loginfo(f"| Actual Distance     : {self.real_dist}")
        rospy.loginfo(f"| -> Real PPR         : {real_ppr}")
        rospy.loginfo(f"|----------------------------------------|")
