#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <msd700_msgs/HardwareCommand.h>

class MSD700HWInterface : public hardware_interface::RobotHW {
public:
    MSD700HWInterface(ros::NodeHandle nh) : nh(nh) {
        // Register joint state interface
        // (Read the current state of the robot)
        hardware_interface::JointStateHandle left_wheel_handle("left_wheel_joint", &pos[0], &vel[0], &eff[0]);
        jnt_state_interface.registerHandle(left_wheel_handle);
        hardware_interface::JointStateHandle right_wheel_handle("right_wheel_joint", &pos[1], &vel[1], &eff[1]);
        jnt_state_interface.registerHandle(right_wheel_handle);
        registerInterface(&jnt_state_interface);

        // Register joint velocity interface
        // (Write the velocity command to the robot)
        hardware_interface::JointHandle left_wheel_vel_handle(jnt_state_interface.getHandle("left_wheel_joint"), &cmd[0]);
        jnt_vel_interface.registerHandle(left_wheel_vel_handle);
        hardware_interface::JointHandle right_wheel_vel_handle(jnt_state_interface.getHandle("right_wheel_joint"), &cmd[1]);
        jnt_vel_interface.registerHandle(right_wheel_vel_handle);
        registerInterface(&jnt_vel_interface);

    }

    void write(){
        // rpm = (rad/s) * (60 / (2*pi))
        double left_rpm = cmd[0] * (60.0 / (2.0 * M_PI));
        double right_rpm = cmd[1] * (60.0 / (2.0 * M_PI));
        
        msd700_msgs::HardwareCommand cmd_msg;
        cmd_msg.movement_command    = 0;            // Not used
        cmd_msg.cam_angle_command   = 0;            // Not used
        cmd_msg.right_motor_speed   = right_rpm;
        cmd_msg.left_motor_speed    = left_rpm;
        
        // Publish the message to your motor controller topic
        hardware_command_pub.publish(cmd_msg);
    }

private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    
    double cmd[2];
    double pos[2];
    double vel[2];
    double eff[2];

    ros::NodeHandle nh;
    ros::Publisher hardware_command_pub = nh.advertise<msd700_msgs::HardwareCommand>("/hardware_command", 1);

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "msd700_hw_interface");
    ros::NodeHandle nh;

    MSD700HWInterface msd700_hw_interface(nh);
    controller_manager::ControllerManager cm(&msd700_hw_interface, nh);

    ros::Rate loop_rate(10);
    ros::Time last_time = ros::Time::now();
    while (ros::ok()) {
        ros::Time current_time = ros::Time::now();
        ros::Duration period = current_time - last_time;

        cm.update(current_time, period);
        msd700_hw_interface.write();

        last_time = current_time;
        loop_rate.sleep();
    }

    return 0;
}