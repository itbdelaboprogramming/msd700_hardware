#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <msd700_msgs/HardwareCommand.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

struct OdomState {
    double x;
    double y;
    double theta;
    double linear_velocity;
    double angular_velocity;

    OdomState() : x(0.0), y(0.0), theta(0.0),
                    linear_velocity(0.0), angular_velocity(0.0) {}
};

struct MotorPulseState {
    double left_motor_pulse;
    double right_motor_pulse;

    MotorPulseState() : left_motor_pulse(0.0), right_motor_pulse(0.0) {}
};

struct ImuState {
    double acc[3];
    double gyro[3];
    double rpy[3];
    ImuState() {
        for (int i = 0; i < 3; i++) {
            acc[i] = 0.0;
            gyro[i] = 0.0;
            rpy[i] = 0.0;
        }
    }
};

class MSD700HWInterface : public hardware_interface::RobotHW {
public:
    MSD700HWInterface(ros::NodeHandle nh) : nh_(nh) {
        // Pamameters
        nh_.param("encoder_ppr", encoder_ppr_, 2048.0);
        nh_.param("wheel_distance", wheel_distance_, 0.5);
        nh_.param("wheel_radius", wheel_radius_, 0.1);
        nh_.param("compute_period", compute_period_, 0.1);
        nh_.param("use_imu", use_imu_, false);

        // Register joint state interface
        // This handles the current state of the robot.
        // We can update these values in the read() function.
        hardware_interface::JointStateHandle bl_handle("bl_wheel_joint", &pos_[0], &vel_[0], &eff_[0]);
        hardware_interface::JointStateHandle br_handle("br_wheel_joint", &pos_[1], &vel_[1], &eff_[1]);
        hardware_interface::JointStateHandle fr_handle("fr_wheel_joint", &pos_[2], &vel_[2], &eff_[2]);
        hardware_interface::JointStateHandle fl_handle("fl_wheel_joint", &pos_[3], &vel_[3], &eff_[3]);
        jnt_state_interface.registerHandle(bl_handle);
        jnt_state_interface.registerHandle(br_handle);
        jnt_state_interface.registerHandle(fl_handle);
        jnt_state_interface.registerHandle(fr_handle);
        registerInterface(&jnt_state_interface);

        // Register joint velocity interface
        // This is where controllers write their commands.
        // We can convert these commands to actual motor commands in the write() function.
        hardware_interface::JointHandle bl_vel_handle(jnt_state_interface.getHandle("bl_wheel_joint"), &cmd_[0]);
        hardware_interface::JointHandle br_vel_handle(jnt_state_interface.getHandle("br_wheel_joint"), &cmd_[1]);
        hardware_interface::JointHandle fl_vel_handle(jnt_state_interface.getHandle("fl_wheel_joint"), &cmd_[2]);
        hardware_interface::JointHandle fr_vel_handle(jnt_state_interface.getHandle("fr_wheel_joint"), &cmd_[3]);
        jnt_vel_interface.registerHandle(bl_vel_handle);
        jnt_vel_interface.registerHandle(br_vel_handle);
        jnt_vel_interface.registerHandle(fl_vel_handle);
        jnt_vel_interface.registerHandle(fr_vel_handle);
        registerInterface(&jnt_vel_interface);

        // Initialize Joint States
        for (int i = 0; i < 4; i++) {
            pos_[i] = 0.0; vel_[i] = 0.0; eff_[i] = 0.0; cmd_[i] = 0.0;
        }

        // Initialize Odometry Variables
        odom_state_ = OdomState();
        motor_state_ = MotorPulseState();
        imu_state_ = ImuState();

        last_odom_time_ = ros::Time::now();
        last_read_time_ = ros::Time::now();
        pulse2rad_ = 2.0 * M_PI / encoder_ppr_;

        // Publisher
        hardware_command_pub_   = nh_.advertise<msd700_msgs::HardwareCommand>("/hardware_command", 10);
        odom_pub_               = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
        // Subscriber
        motor_pulse_sub_        = nh_.subscribe("/hardware/motor_pulse", 10, &MSD700HWInterface::motor_pulse_callback, this);
        imu_sub_                = nh_.subscribe("/hardware/imu", 10, &MSD700HWInterface::imu_callback, this);

    }


    void read(){
        // Handle Time
        ros::Time now = ros::Time::now();
        double dt = (now - last_read_time_).toSec();

        // Update joint states
        double delta_left = motor_state_.left_motor_pulse * pulse2rad_;
        double delta_right = motor_state_.right_motor_pulse * pulse2rad_;
        pos_[0] += delta_left;
        pos_[1] += delta_right;
        pos_[2] = pos_[0];
        pos_[3] = pos_[1];

        vel_[0] = delta_left / dt;
        vel_[1] = delta_right / dt;
        vel_[2] = vel_[0];
        vel_[3] = vel_[1];

        update_odom();
        publish_odom();

        last_read_time_ = now;
    }

    void write(){
        // Read cmd_ and send to hardware
        double left_rpm = cmd_[0] * 60.0 / (2.0 * M_PI);
        double right_rpm = cmd_[1] * 60.0 / (2.0 * M_PI);
        publish_hardware_command(0, 0, left_rpm, right_rpm);
    }

    void motor_pulse_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
        motor_state_.left_motor_pulse += msg->data[0];
        motor_state_.right_motor_pulse += msg->data[1];
    }

    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
        imu_state_.acc[0] = msg->linear_acceleration.x;
        imu_state_.acc[1] = msg->linear_acceleration.y;
        imu_state_.acc[2] = msg->linear_acceleration.z;

        imu_state_.gyro[0] = msg->angular_velocity.x;
        imu_state_.gyro[1] = msg->angular_velocity.y;
        imu_state_.gyro[2] = msg->angular_velocity.z;

        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        imu_state_.rpy[0] = roll;
        imu_state_.rpy[1] = pitch;
        imu_state_.rpy[2] = yaw;
    }

    void publish_odom() {
        ros::Time now = ros::Time::now();
        nav_msgs::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = odom_state_.x;
        odom.pose.pose.position.y = odom_state_.y;
        odom.pose.pose.position.z = 0.0;

        tf::Quaternion q;
        q.setRPY(0, 0, odom_state_.theta);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x = odom_state_.linear_velocity;
        odom.twist.twist.angular.z = odom_state_.angular_velocity;
        odom_pub_.publish(odom);

        tf_broadcaster_.sendTransform(
            tf::StampedTransform(tf::Transform(q, tf::Vector3(odom_state_.x, odom_state_.y, 0.0)),
                                 now, "odom", "base_link"));
    }

    void publish_hardware_command(int move, int cam, double left_rpm, double right_rpm){
        // Publish the msg
        msd700_msgs::HardwareCommand cmd_msg;
        cmd_msg.movement_command    = move;
        cmd_msg.cam_angle_command   = cam;
        cmd_msg.right_motor_speed   = right_rpm;
        cmd_msg.left_motor_speed    = left_rpm;

        hardware_command_pub_.publish(cmd_msg);
    }

    void update_odom(){
        // Reference:
        // - https://medium.com/@nahmed3536/wheel-odometry-model-for-differential-drive-robotics-91b85a012299

        // Update time
        ros::Time now = ros::Time::now();
        double dt = (now - last_odom_time_).toSec();
        if (dt == 0.0) return;

        // distance traveled by each wheel
        double d_left = motor_state_.left_motor_pulse * pulse2rad_ * wheel_radius_;
        double d_right = motor_state_.right_motor_pulse * pulse2rad_ * wheel_radius_;
        double d = (d_left + d_right) / 2.0;

        double d_theta = 0.0;
        if(use_imu_){
            // calculate from accel
            double accel_pitch = std::atan2(-imu_state_.acc[0],
                                   std::sqrt(imu_state_.acc[1]*imu_state_.acc[1] + imu_state_.acc[2]*imu_state_.acc[2]));
            double accel_roll  = std::atan2(imu_state_.acc[1], imu_state_.acc[2]);

            // calculate from gyro
            double gyro_roll  = imu_state_.rpy[0] + imu_state_.gyro[1] * dt;
            double gyro_pitch = imu_state_.rpy[1] + imu_state_.gyro[0] * dt;
            double gyro_yaw   = imu_state_.rpy[2] + imu_state_.gyro[2] * dt;

            // Complementary filter (with a factor of 0.98 for gyro).
            imu_state_.rpy[0] = 0.98 * gyro_roll  + 0.02 * accel_roll;
            imu_state_.rpy[1] = 0.98 * gyro_pitch + 0.02 * accel_pitch;
            imu_state_.rpy[2] = gyro_yaw;
            odom_state_.theta = imu_state_.rpy[2];
        }else{
            d_theta = (d_right - d_left) / wheel_distance_;
            odom_state_.theta += d_theta;
        }

        // Update odometry
        odom_state_.x += d * std::cos(odom_state_.theta);
        odom_state_.y += d * std::sin(odom_state_.theta);

        // Update linear and angular velocity
        odom_state_.linear_velocity = d / dt;
        odom_state_.angular_velocity = use_imu_ ? imu_state_.gyro[2] : d_theta / dt;

        // Reset motor pulse
        motor_state_.left_motor_pulse = 0.0;
        motor_state_.right_motor_pulse = 0.0;

        last_odom_time_ = now;
    }

private:
    // ROS NodeHandle
    ros::NodeHandle nh_;
    ros::Publisher hardware_command_pub_;
    ros::Publisher odom_pub_;
    ros::Subscriber motor_pulse_sub_;
    ros::Subscriber imu_sub_;
    tf::TransformBroadcaster tf_broadcaster_;

    // Parameters
    double encoder_ppr_;
    double wheel_distance_;
    double wheel_radius_;
    double compute_period_;
    bool use_imu_;

    // Variables
    // - Motor pulse
    MotorPulseState motor_state_;
    // - IMU
    ImuState imu_state_;
    // - Odometry
    OdomState odom_state_;
    // - Time
    ros::Time last_read_time_;
    ros::Time last_odom_time_;
    // - Other
    double pulse2rad_;

    // Ros-control Interfaces
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    double pos_[4];    // positions for [bl, br, fr, fl]
    double vel_[4];    // velocities for [bl, br, fr, fl]
    double eff_[4];    // efforts for [bl, br, fr, fl]
    double cmd_[4];    // commands for [bl, br, fr, fl]


};

int main(int argc, char **argv) {
    // https://docs.ros.org/en/noetic/api/hardware_interface/html/c++/index.html
    // Initialize the node
    ros::init(argc, argv, "msd700_hw_interface");
    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");

    // Create the hardware interface and controller manager
    MSD700HWInterface hw(nh);
    controller_manager::ControllerManager cm(&hw, nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();

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