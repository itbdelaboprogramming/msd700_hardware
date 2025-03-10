#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import tkinter as tk


class OdomTuneGUI:
    def __init__(self, root, initial_ppr):
        self.root = root

        self.calc_dist = 0.0
        self.meas_dist = 0.0
        self.calc_ppr = initial_ppr
        self.meas_ppr = 0.0

        # StringVars for values
        self.var_calc_ppr       = tk.StringVar()
        self.var_calc_dist      = tk.StringVar()
        self.var_meas_dist      = tk.StringVar()
        self.var_meas_ppr       = tk.StringVar()

        # First column (labels)
        self.lbl_calc_ppr       = tk.Label(root, text="PPR for Calculation")
        self.lbl_calc_dist      = tk.Label(root, text="Dist from Calculation")
        self.lbl_meas_dist      = tk.Label(root, text="Measured Dist")
        self.lbl_meas_ppr       = tk.Label(root, text="New PPR Value")

        # Second column (values)
        self.lbl_calc_ppr_val   = tk.Label(root, textvariable=self.var_calc_ppr)
        self.lbl_calc_dist_val  = tk.Label(root, textvariable=self.var_calc_dist)
        self.entry_meas_dist    = tk.Entry(root, textvariable=self.var_meas_dist)
        self.lbl_meas_ppr_val   = tk.Label(root, textvariable=self.var_meas_ppr)
        self.var_meas_dist.trace_add("write", self.measured_changed)

        # Grid layout
        self.lbl_calc_ppr.grid(row=0, column=0, padx=5, pady=5, sticky="W")
        self.lbl_calc_ppr_val.grid(row=0, column=1, padx=5, pady=5, sticky="W")

        self.lbl_calc_dist.grid(row=1, column=0, padx=5, pady=5, sticky="W")
        self.lbl_calc_dist_val.grid(row=1, column=1, padx=5, pady=5, sticky="W")

        self.lbl_meas_dist.grid(row=2, column=0, padx=5, pady=5, sticky="W")
        self.entry_meas_dist.grid(row=2, column=1, padx=5, pady=5, sticky="W")

        self.lbl_meas_ppr.grid(row=3, column=0, padx=5, pady=5, sticky="W")
        self.lbl_meas_ppr_val.grid(row=3, column=1, padx=5, pady=5, sticky="W")

        self.update_values()

    def measured_changed(self, *args):
        self.meas_dist = float(self.var_meas_dist.get())
        self.update_values()

    def update_odom_value(self, calc_dist):
        self.calc_dist = calc_dist
        self.update_values()

    def update_values(self):
        # Update new PPR
        if(self.calc_dist != 0):
            self.meas_ppr = self.meas_dist * self.calc_ppr / self.calc_dist

        # Update gui values
        self.var_calc_ppr.set(self.calc_ppr)
        self.var_calc_dist.set(self.calc_dist)
        self.var_meas_dist.set(self.meas_dist)
        self.var_meas_ppr.set(self.meas_ppr)

class OdomTuneNode:
    def __init__(self):
        # Parameters
        self.compute_period = rospy.get_param('msd700_odom/compute_period', 30)         # ms
        self.current_ppr = rospy.get_param('msd700_odom/encoder_ppr', 12)            # ppr

        # Variables
        self.calc_dist = 0.0
        self.calc_ppr = self.current_ppr
        self.last_time = rospy.Time.now()

        # GUI
        self.gui = OdomTuneGUI(tk.Tk(), self.current_ppr)

        # Subscriber
        self.odom_sub = rospy.Subscriber('/sensors/odom', Odometry, self.odom_callback)

    
    def odom_callback(self, msg: Odometry) -> None:
        self.calc_dist = msg.pose.pose.position.x
        self.gui.update_odom_value(self.calc_dist)

if __name__ == '__main__':
    rospy.init_node('odom_tune_node')
    odom_node = OdomTuneNode()
    odom_node.gui.root.mainloop()
    rospy.spin()