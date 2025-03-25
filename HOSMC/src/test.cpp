// Copyright [2023] <Your Name or Organization>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <Eigen/Dense>

class AUVModel {
 private:
    ros::NodeHandle nh;
    ros::Subscriber control_sub;
    ros::Publisher depth_pub;
    ros::Publisher imu_pub;
    double eta_z;
    double velocity_z;
    double acceleration_z;
    double mass;
    double damping;
    ros::Time last_time;
    bool received_control;
    std_msgs::Float64 depth_msg;
    geometry_msgs::Vector3 eta_msg;

    void controlCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
        if (msg->data.size() != 4) return;
        Eigen::Matrix<double, 1, 4> j;
        // cppcheck-suppress constStatement
        j << -1, -1, -1, -1;
        Eigen::Vector4d u;
        u << msg->data[0], msg->data[1], msg->data[2], msg->data[3];
        double thrust_z = j * u;

        ros::Time current_time = ros::Time::now();
        double dT = (last_time.isZero()) ? 0.01 : (current_time - last_time).toSec();
        last_time = current_time;

        // AUV dynamics: mass * acceleration + damping * velocity = thrust
        acceleration_z = (thrust_z - damping * velocity_z) / mass;
        velocity_z += acceleration_z * dT;
        eta_z += velocity_z * dT;

        eta_msg.x = 0.0;
        eta_msg.y = 0.0;
        eta_msg.z = 0.0;
        imu_pub.publish(eta_msg);
        depth_msg.data = eta_z;
        depth_pub.publish(depth_msg);
        ROS_INFO("send z: [%f]", eta_z);
        received_control = true;
    }

 public:
    AUVModel() {
        eta_z = 0.0;
        velocity_z = 0.0;
        acceleration_z = 0.0;
        mass = 10.0;  // Assume mass = 10kg
        damping = 2.0;  // Linear damping

        control_sub = nh.subscribe("control_output", 10, &AUVModel::controlCallback, this);
        depth_pub = nh.advertise<std_msgs::Float64>("depth", 10);
        imu_pub = nh.advertise<geometry_msgs::Vector3>("imu", 10);
    }

    void spin() {
        ros::Rate rate(100);
        while (ros::ok()) {
            if (!received_control) {
                eta_msg.x = 0.0;
                eta_msg.y = 0.0;
                eta_msg.z = 0.0;
                imu_pub.publish(eta_msg);
                depth_msg.data = 0.0;
                depth_pub.publish(depth_msg);
            }
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "auv_model_node");
    AUVModel auv;
    auv.spin();
    return 0;
}
