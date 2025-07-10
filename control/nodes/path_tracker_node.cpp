// Copyright 2024 Mahmoud

#include "control/PathTracker.h"

class PathTrackerNode {
 private:
    ros::NodeHandle nh_;
    ros::Publisher surge_pub_;
    ros::Publisher yaw_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber path_sub_;
    ros::Subscriber sonar_sub_;
    ros::Timer control_timer_;
    ros::ServiceServer set_service;

    PathTracker tracker_;
    double dt_;
    double desired_x_;
    double current_x;

    Eigen::VectorXd state;
    bool has_odom_;
    bool has_path_;
    bool has_x;
    bool has_x_desired;

 public:
    PathTrackerNode() : nh_(),
                        tracker_(nh_),
                        state(Eigen::VectorXd::Zero(3)),
                        has_x(false), has_x_desired(false), has_odom_(false), has_path_(false) {
        // Load parameters
        nh_.param("dt", dt_, 0.1);

        // Initialize path tracker
        tracker_.setDt(dt_);

        // Publishers
        surge_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/PID/Wrench", 1);
        yaw_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("target", 1);

        // Subscribers
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 1, &PathTrackerNode::odomCallback, this);
        path_sub_ = nh_.subscribe<nav_msgs::Path>("path", 1, &PathTrackerNode::pathCallback, this);
        sonar_sub_ = nh_.subscribe<std_msgs::Float64>("sonar", 1, &PathTrackerNode::sonarCallback, this);

        // service
        set_service = nh_.advertiseService("set_desired",
                                      &PathTrackerNode::setDesired,
                                      this);

        // Control timer
        control_timer_ = nh_.createTimer(ros::Duration(dt_),
                                       &PathTrackerNode::controlTimerCallback, this);

        ROS_INFO("Path Tracker Node initialized with control rate: %.1f Hz", 1.0 / dt_);
    }

    bool setDesired(control::SetDesired::Request &req,  // NOLINT
                        control::SetDesired::Response &res) {  // NOLINT
        ROS_INFO("Received z_desired: [%f]", req.z);

        desired_x_ = req.x;

        res.success = true;  // Indicate success
        has_x_desired = true;
        return true;
    }

    void sonarCallback(const std_msgs::Float64::ConstPtr& msg) {
        current_x = msg->data;
        has_x = true;
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        std::vector<Eigen::Vector3d> path;
        for (const auto& pose : msg->poses) {
            path.emplace_back(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        }

        if (!path.empty()) {
            tracker_.setPath(path);
            has_path_ = true;
            ROS_INFO("Received new path with %zu points", path.size());
        } else {
            ROS_WARN("Received empty path");
            has_path_ = false;
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Store latest odometry
        // Position
        state(0) = msg->pose.pose.position.x;
        state(1) = msg->pose.pose.position.y;

        // Orientation (yaw)
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        state(2) = yaw;
        has_odom_ = true;
    }

    void controlTimerCallback(const ros::TimerEvent& event) {
        if ((!has_odom_ || !has_path_) && (!has_x || !has_x_desired)) {
            if (!has_odom_ && !has_path_ && !has_x && !has_x_desired) ROS_WARN_THROTTLE(1.0, "Waiting for data...");
            if (!has_odom_ && has_path_) ROS_WARN_THROTTLE(1.0, "Waiting for odometry data...");
            if (!has_path_ && has_odom_) ROS_WARN_THROTTLE(1.0, "Waiting for path data...");
            if (!has_x && has_x_desired) ROS_WARN_THROTTLE(1.0, "Waiting for x position data...");
            if (!has_x_desired && has_x) ROS_WARN_THROTTLE(1.0, "Waiting for x desired data...");
            return;
        }
        if (has_x && has_x_desired) {
            double u_opt = tracker_.trackPathPID(current_x, desired_x_);

            // Publish surge command
            geometry_msgs::WrenchStamped surge_msg;
            surge_msg.header.stamp = ros::Time::now();
            surge_msg.header.frame_id = "base_link";
            surge_msg.wrench.force.x = u_opt;  // Surge force
            surge_msg.wrench.force.y = 0.0;    // Sway force
            surge_msg.wrench.force.z = 0.0;    // Heave force
            surge_msg.wrench.torque.x = 0.0;   // Roll torque
            surge_msg.wrench.torque.y = 0.0;   // Pitch torque
            surge_msg.wrench.torque.z = 0.0;   // Yaw torque
            surge_pub_.publish(surge_msg);
            return;
        }

        try {
            // Backstepping for heading reference
            Eigen::Vector4d ref = tracker_.trackPathBackstepping(state);

            // PID control for surge
            double u_opt = tracker_.trackPathPID(state(0), ref(0));

            // Publish surge command
            geometry_msgs::WrenchStamped surge_msg;
            surge_msg.header.stamp = ros::Time::now();
            surge_msg.header.frame_id = "base_link";
            surge_msg.wrench.force.x = u_opt;  // Surge force
            surge_msg.wrench.force.y = 0.0;    // Sway force
            surge_msg.wrench.force.z = 0.0;    // Heave force
            surge_msg.wrench.torque.x = 0.0;   // Roll torque
            surge_msg.wrench.torque.y = 0.0;   // Pitch torque
            surge_msg.wrench.torque.z = 0.0;   // Yaw torque
            surge_pub_.publish(surge_msg);

            // Publish desired yaw
            geometry_msgs::PoseStamped yaw_msg;
            yaw_msg.header.stamp = ros::Time::now();
            yaw_msg.header.frame_id = "base_link";
            yaw_msg.pose.position.x = ref(0);  // Target x
            yaw_msg.pose.position.y = ref(1);  // Target y
            yaw_msg.pose.position.z = ref(3);  // Target z

            tf::Quaternion q_des;
            q_des.setRPY(0, 0, ref(3));  // Desired yaw
            yaw_msg.pose.orientation.x = q_des.x();
            yaw_msg.pose.orientation.y = q_des.y();
            yaw_msg.pose.orientation.z = q_des.z();
            yaw_msg.pose.orientation.w = q_des.w();

            yaw_pub_.publish(yaw_msg);

            ROS_DEBUG_THROTTLE(1.0, "Control update - Surge: %.2f, Yaw: %.2f", u_opt, ref(3));
        } catch (const std::exception& e) {
            ROS_WARN_THROTTLE(1.0, "Path tracking error: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_tracker_node");
    PathTrackerNode node;
    ros::spin();
    return 0;
}
