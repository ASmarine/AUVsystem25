// Copyright [2023] <Your Name or Organization>
#include "control/hosmc_controller.h"

class HOSMCNode {
 private:
    ros::NodeHandle nh;
    ros::Subscriber depth;
    ros::Subscriber IMU;
    ros::Subscriber state;
    ros::Subscriber set_target;
    ros::Publisher control_pub;
    ros::ServiceServer set_service;
    ros::Timer control_timer;

    HOSMCController controller;
    Eigen::VectorXd eta_real;
    Eigen::VectorXd eta_dot;
    Eigen::VectorXd eta_desired;
    bool eta_set;
    bool depth_received;
    bool imu_received;
    bool state_received;

    void depthCallback(const sensor_msgs::FluidPressure::ConstPtr &msg) {
        if (eta_real.size() > 0) {
            eta_real(0) = (msg->fluid_pressure*100 - 101325.0) / (9.8 * 997.0);
            depth_received = true;
        } else {
            ROS_WARN("eta_real is not initialized correctly!");
        }
    }

    void IMUCallback(const sensor_msgs::Imu::ConstPtr &msg) {
        // Extract orientation quaternion
        tf::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        // Convert quaternion to RPY
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        // Set eta_real = [z, roll, pitch]
        eta_real(1) = roll;
        eta_real(2) = pitch;
        eta_real(3) = yaw;

        imu_received = true;
    }

    // void targetCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    //     // Extract target position
    //     double z_target = msg->pose.position.z;

    //     // Extract target orientation quaternion
    //     tf::Quaternion q(
    //         msg->pose.orientation.x,
    //         msg->pose.orientation.y,
    //         msg->pose.orientation.z,
    //         msg->pose.orientation.w);
    //     // Convert quaternion to RPY
    //     double roll, pitch, yaw;
    //     tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    //     // Set eta_desired = [z, roll, pitch]
    //     eta_desired(0) = z_target;
    //     eta_desired(1) = 0.0;  // Roll
    //     eta_desired(2) = 0.0;  // Pitch
    //     eta_desired(3) = yaw;  // Yaw

    //     eta_set = true;
    // }

    void stateCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Extract position z
        double z = msg->pose.pose.position.z;

        // Extract orientation quaternion
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        // Convert quaternion to RPY
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // Set eta_real = [z, roll, pitch]
        eta_real(0) = z;
        eta_real(1) = roll;
        eta_real(2) = pitch;
        eta_real(3) = yaw;

        // Extract velocities
        double z_dot     = msg->twist.twist.linear.z;
        double roll_dot  = msg->twist.twist.angular.x;
        double pitch_dot = msg->twist.twist.angular.y;
        double yaw_dot   = msg->twist.twist.angular.z;

        // Set eta_dot = [vz, roll_rate, pitch_rate]
        eta_dot(0) = z_dot;
        eta_dot(1) = roll_dot;
        eta_dot(2) = pitch_dot;
        eta_dot(3) = yaw_dot;

        // Print the received values
        state_received = true;
    }

    bool setDesired(control::SetDesired::Request &req,  // NOLINT
                        control::SetDesired::Response &res) {  // NOLINT
        ROS_INFO("Received z_desired: [%f]", req.z);

        eta_desired(0) = req.z;
        eta_desired(1) = 0.0;
        eta_desired(2) = 0.0;
        eta_desired(3) = req.yaw;

        res.success = true;  // Indicate success
        eta_set = true;
        return true;
    }
    void controlLoopCallback(const ros::TimerEvent& event) {
        if (eta_set && ((depth_received && imu_received) || state_received)) {
            Eigen::VectorXd f_ver = controller.stabilize(eta_real, eta_desired);

            geometry_msgs::WrenchStamped msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "base_link";

            msg.wrench.force.x = 0.0;
            msg.wrench.force.y = 0.0;
            msg.wrench.force.z = f_ver(0);

            msg.wrench.torque.x = f_ver(1);
            msg.wrench.torque.y = f_ver(2);
            msg.wrench.torque.z = f_ver(3);

            control_pub.publish(msg);
        } else {
            ROS_WARN_THROTTLE(5, "Waiting for eta_desired and sensor data...");
        }
    }

 public:
HOSMCNode()
        : nh(),
          controller(nh),
          eta_real(Eigen::VectorXd::Zero(4)),
          eta_dot(Eigen::VectorXd::Zero(4)),
          eta_desired(Eigen::VectorXd::Zero(4)),
          eta_set(false), depth_received(false), imu_received(false), state_received(false) {
        state = nh.subscribe<nav_msgs::Odometry>("odom", 10, &HOSMCNode::stateCallback, this);
        depth = nh.subscribe<sensor_msgs::FluidPressure>("/pressure", 10, &HOSMCNode::depthCallback, this);
        IMU = nh.subscribe<sensor_msgs::Imu>("/imu", 10, &HOSMCNode::IMUCallback, this);
        // set_target = nh.subscribe<geometry_msgs::PoseStamped>("/target", 10, &HOSMCNode::targetCallback, this);
        control_pub = nh.advertise<geometry_msgs::WrenchStamped>("/HOSMC/Wrench", 10);
        set_service = nh.advertiseService("set_desired",
                                      &HOSMCNode::setDesired,
                                      this);
        control_timer = nh.createTimer(ros::Duration(0.04), &HOSMCNode::controlLoopCallback, this);
    }

    void spin() {
        ros::spin();  // Timer will handle control loop
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "hosmc_node");
    HOSMCNode node;
    node.spin();
    return 0;
}

