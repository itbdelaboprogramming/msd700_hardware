#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <msd700_msgs/HardwareCommand.h>
#include <std_msgs/Float32MultiArray.h>

class MSD700HWInterface : public hardware_interface::RobotHW {
public:
    MSD700HWInterface(ros::NodeHandle nh) : nh_(nh) {
        // Register joint state interface
        // This handles the current state of the robot.
        // We can update these values in the read() function.
        // In simulation, these values are updated by the gazebo plugin
        hardware_interface::JointStateHandle fl_handle("fl_wheel_joint", &pos_[0], &vel_[0], &eff_[0]);
        hardware_interface::JointStateHandle bl_handle("bl_wheel_joint", &pos_[1], &vel_[1], &eff_[1]);
        hardware_interface::JointStateHandle fr_handle("fr_wheel_joint", &pos_[2], &vel_[2], &eff_[2]);
        hardware_interface::JointStateHandle br_handle("br_wheel_joint", &pos_[3], &vel_[3], &eff_[3]);

        jnt_state_interface.registerHandle(fl_handle);
        jnt_state_interface.registerHandle(bl_handle);
        jnt_state_interface.registerHandle(fr_handle);
        jnt_state_interface.registerHandle(br_handle);

        registerInterface(&jnt_state_interface);

        // Register joint velocity interface
        // This is where controllers write their commands.
        // We can convert these commands to actual motor commands in the write() function.

        hardware_interface::JointHandle fl_vel_handle(jnt_state_interface.getHandle("fl_wheel_joint"), &cmd_[0]);
        hardware_interface::JointHandle bl_vel_handle(jnt_state_interface.getHandle("bl_wheel_joint"), &cmd_[0]);
        hardware_interface::JointHandle fr_vel_handle(jnt_state_interface.getHandle("fr_wheel_joint"), &cmd_[1]);
        hardware_interface::JointHandle br_vel_handle(jnt_state_interface.getHandle("br_wheel_joint"), &cmd_[1]);

        jnt_vel_interface.registerHandle(fl_vel_handle);
        jnt_vel_interface.registerHandle(bl_vel_handle);
        jnt_vel_interface.registerHandle(fr_vel_handle);
        jnt_vel_interface.registerHandle(br_vel_handle);

        registerInterface(&jnt_vel_interface);

        // Initial variables
        for (int i = 0; i < 4; i++) {
            pos_[i] = 0.0;
            vel_[i] = 0.0;
            eff_[i] = 0.0;
        }
        cmd_[0] = cmd_[1] = 0.0;

        hardware_command_pub_ = nh_.advertise<msd700_msgs::HardwareCommand>("/hardware_command", 1);

    }

    void read(){
        // No process
    }

    void write(){
        // --- Publish HardwareCommand to Firmware based on cmd_ ---
        // The cmd_ controlled by controller_manager (diff_drive_controller)

        // rpm = (rad/s) * (60 / (2*pi))
        double left_rpm = cmd_[0] * (60.0 / (2.0 * M_PI));
        double right_rpm = cmd_[1] * (60.0 / (2.0 * M_PI));
        
        // Publish the msg
        msd700_msgs::HardwareCommand cmd_msg;
        cmd_msg.movement_command    = 0;            // Not used
        cmd_msg.cam_angle_command   = 0;            // Not used
        cmd_msg.right_motor_speed   = right_rpm;
        cmd_msg.left_motor_speed    = left_rpm;

        hardware_command_pub_.publish(cmd_msg);
    }

private:
    // ROS NodeHandle
    ros::NodeHandle nh_;
    ros::Publisher hardware_command_pub_;
    ros::Subscriber motor_pulse_sub;

    // Parameters
    double wheel_radius;
    double wheel_separation;
    double motor_ppr;

    // Interfaces
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    
    // Variables
    double pos_[4];    // positions for [fl, bl, fr, br]
    double vel_[4];    // velocities for [fl, bl, fr, br]
    double eff_[4];    // efforts for [fl, bl, fr, br]
    double cmd_[2];    // commands for [left, right]

};

int main(int argc, char **argv) {
    // https://docs.ros.org/en/noetic/api/hardware_interface/html/c++/index.html
    // Initialize the node
    ros::init(argc, argv, "msd700_hw_interface");
    ros::NodeHandle nh;

    // Create the hardware interface and controller manager
    MSD700HWInterface hw(nh);
    controller_manager::ControllerManager cm(&hw, nh);

    // Set an update frequency (e.g., 50 Hz)
    ros::Rate loop_rate(50);
    ros::Time last_time = ros::Time::now();

    while (ros::ok())
    {
        ros::Time current_time = ros::Time::now();
        ros::Duration elapsed_time = current_time - last_time;
        last_time = current_time;

        hw.read();
        cm.update(current_time, elapsed_time);
        hw.write();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}