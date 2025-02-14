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
        // (Read the current state of the robot)
        hardware_interface::JointStateHandle left_wheel_handle("left_wheel_joint", &pos_[0], &vel_[0], &eff_[0]);
        jnt_state_interface.registerHandle(left_wheel_handle);
        hardware_interface::JointStateHandle right_wheel_handle("right_wheel_joint", &pos_[1], &vel_[1], &eff_[1]);
        jnt_state_interface.registerHandle(right_wheel_handle);
        registerInterface(&jnt_state_interface);

        // Register joint velocity interface
        // (Write the velocity command to the robot)
        hardware_interface::JointHandle left_wheel_vel_handle(jnt_state_interface.getHandle("left_wheel_joint"), &cmd_[0]);
        jnt_vel_interface.registerHandle(left_wheel_vel_handle);
        hardware_interface::JointHandle right_wheel_vel_handle(jnt_state_interface.getHandle("right_wheel_joint"), &cmd_[1]);
        jnt_vel_interface.registerHandle(right_wheel_vel_handle);
        registerInterface(&jnt_vel_interface);

        // Parameters
        nh.param("wheel_radius"     , wheel_radius, 0.1);
        nh.param("wheel_separation" , wheel_separation, 0.2);
        nh.param("motor_ppr"        , motor_ppr, 2400.0);
        nh.param("enable_odom"      , enable_odom, false);

        // Initial variables
        pos_[0] = pos_[1] = 0.0;
        vel_[0] = vel_[1] = 0.0;
        eff_[0] = eff_[1] = 0.0;
        cmd_[0] = cmd_[1] = 0.0;

        ros::Publisher hardware_command_pub = nh_.advertise<msd700_msgs::HardwareCommand>("/hardware_command", 1);

        if(enable_odom){
            ros::Subscriber motor_pulse_sub = nh_.subscribe("/hardware/motor_pulse", 10, &MSD700HWInterface::motor_pulse_callback, this);
        }

    }

    void read(){
        // No process
    }

    void write(){
        // rpm = (rad/s) * (60 / (2*pi))
        double left_rpm = cmd_[0] * (60.0 / (2.0 * M_PI));
        double right_rpm = cmd_[1] * (60.0 / (2.0 * M_PI));
        
        msd700_msgs::HardwareCommand cmd_msg;
        cmd_msg.movement_command    = 0;            // Not used
        cmd_msg.cam_angle_command   = 0;            // Not used
        cmd_msg.right_motor_speed   = right_rpm;
        cmd_msg.left_motor_speed    = left_rpm;

        // Publish the message to your motor controller topic
        hardware_command_pub.publish(cmd_msg);
    }

private:

    void motor_pulse_callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        if(!enable_odom){
            return;
        }

        double now = ros::Time::now().toSec();
        double delta_time = now - last_time;

        // read pulse
        double delta_left_pulse = msg->data[0];
        double delta_right_pulse = msg->data[1];

        // rpm = (pulse) * (60 / ppr)
        double delta_left_rpm = delta_left_pulse * (60.0 / motor_ppr);
        double delta_right_rpm = delta_right_pulse * (60.0 / motor_ppr);

        // m = (rpm) * (2*pi / 60)
        double delta_left_m = delta_left_rpm * (2.0 * M_PI / 60.0);
        double delta_right_m = delta_right_rpm * (2.0 * M_PI / 60.0);

        // update time
        last_time = now;

        return;
    }

    // Parameters
    double wheel_radius;
    double wheel_separation;
    double motor_ppr;
    bool enable_odom;

    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    
    // Variables
    double cmd_[2];
    double pos_[2];
    double vel_[2];
    double eff_[2];

    double last_time;

    ros::NodeHandle nh_;
    ros::Publisher hardware_command_pub;
    ros::Subscriber motor_pulse_sub;

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