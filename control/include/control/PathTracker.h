// Copyright 2024 <Copyright Owner>
#ifndef _HOME_MAHMOUD_AUV_2025_SRC_CONTROL_INCLUDE_CONTROL_PATHTRACKER_H_
#define _HOME_MAHMOUD_AUV_2025_SRC_CONTROL_INCLUDE_CONTROL_PATHTRACKER_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <vector>
#include <utility>
#include <tuple>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include "control/SetDesired.h"


class PathTracker {
 public:
    PathTracker(const ros::NodeHandle& nh);  // NOLINT

    void setPath(const std::vector<Eigen::Vector3d>& path_points);
    void setDt(double dt);

    // PID tracking controller for x-position
    double trackPathPID(const double x_current, const double x_desired);

    // Backstepping path tracking controller
    Eigen::Vector4d trackPathBackstepping(const Eigen::VectorXd& x_current);

 private:
    std::vector<Eigen::Vector3d> path_points_;
    size_t last_closest_idx_;
    double lookahead_;
    double dt_;

    // PID parameters
    double Kp_, Ki_, Kd_;
    double integral_err_;
    double prev_error_;

    // Helper functions
    std::tuple<double, size_t, double> computeSerretFrenetErrors(const Eigen::VectorXd& x_current);
};
#endif  // _HOME_MAHMOUD_AUV_2025_SRC_CONTROL_INCLUDE_CONTROL_PATHTRACKER_H_
