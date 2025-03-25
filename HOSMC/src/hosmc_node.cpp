// Copyright [2023] <Your Name or Organization>
#include "hosmc_controller.h"  // NOLINT

class HOSMCNode {
 private:
    ros::NodeHandle nh;
    ros::Subscriber depth;
    ros::Subscriber IMU;
    ros::Publisher control_pub;
    ros::ServiceServer set_Z_service;
    HOSMCController controller;
    Eigen::VectorXd eta_real;
    Eigen::VectorXd eta_desired;
    bool eta_set;
    bool depth_received;
    bool imu_received;

    void depthCallback(const std_msgs::Float64::ConstPtr &msg) {
        if (eta_real.size() > 0) {
            eta_real(0) = msg->data;
            depth_received = true;
        } else {
            ROS_WARN("eta_real is not initialized correctly!");
        }
    }


    void IMUCallback(const geometry_msgs::Vector3::ConstPtr &msg) {
        eta_real(1) = msg->x;
        eta_real(2) = msg->y;
        imu_received = true;
    }

    bool setZDesired(HOSMC::SetZDesired::Request &req,  // NOLINT
                        HOSMC::SetZDesired::Response &res) {  // NOLINT
        ROS_INFO("Received z_desired: [%f]", req.z);

        eta_desired(0) = req.z;
        eta_desired(1) = 0.0;
        eta_desired(2) = 0.0;

        res.success = true;  // Indicate success
        eta_set = true;
        return true;
    }

 public:
HOSMCNode()
        : nh(),
          eta_real(Eigen::VectorXd::Zero(3)),
          eta_desired(Eigen::VectorXd::Zero(3)),
          eta_set(false), depth_received(false), imu_received(false) {
        depth = nh.subscribe<std_msgs::Float64>("depth", 10, &HOSMCNode::depthCallback, this);
        IMU = nh.subscribe<geometry_msgs::Vector3>("imu", 10, &HOSMCNode::IMUCallback, this);
        control_pub = nh.advertise<std_msgs::Float64MultiArray>("control_output", 10);
        set_Z_service = nh.advertiseService("set_Z_desired",
                                      &HOSMCNode::setZDesired,
                                      this);
    }

    void spin() {
        ros::Rate rate(100);
        while (ros::ok()) {
            if (eta_set && depth_received && imu_received) {
                Eigen::VectorXd f_ver = controller.stabilize(eta_real, eta_desired);
                std_msgs::Float64MultiArray msg;
                msg.data.assign(f_ver.data(), f_ver.data() + f_ver.size());
                control_pub.publish(msg);
            } else {
                ROS_WARN_THROTTLE(5, "Waiting for eta_desired and sensor data...");
            }
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "hosmc_node");
    HOSMCNode node;
    node.spin();
    return 0;
}
