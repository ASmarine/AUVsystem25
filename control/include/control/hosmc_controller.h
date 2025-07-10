// Copyright [2023] [Your Name or Your Organization]

#ifndef _HOME_MAHMOUD_AUV_2025_SRC_CONTROL_INCLUDE_CONTROL_HOSMC_CONTROLLER_H_
#define _HOME_MAHMOUD_AUV_2025_SRC_CONTROL_INCLUDE_CONTROL_HOSMC_CONTROLLER_H_

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <XmlRpcValue.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>
#include "control/SetDesired.h"


class HOSMCController {
 public:
    HOSMCController(ros::NodeHandle& nh);  // NOLINT
    Eigen::VectorXd stabilize(
        const Eigen::VectorXd &eta_real,
        const Eigen::VectorXd &eta_desired);

 private:
    Eigen::Vector4d model_free_control(
        const Eigen::VectorXd &eta,
        const Eigen::VectorXd &eta_d);
    Eigen::Vector4d normal_differentiator(const Eigen::Vector4d &reading, double dT);
    Eigen::Vector4d S_eta_int;
    Eigen::Vector4d Sd;
    Eigen::Vector4d prev_reading_;
    Eigen::Vector4d j0;
    Eigen::Vector4d j1;
    ros::Time t0;
    ros::Time last_time;
    double last_desired;
    double tb;
    double alpha_0;
    double alpha_c;
    double delta;
    double vel;
    double seg;
    Eigen::Matrix4d K_d;
    Eigen::Matrix4d K_i;
    double Thrustlim;
};

#endif  // _HOME_MAHMOUD_AUV_2025_SRC_CONTROL_INCLUDE_CONTROL_HOSMC_CONTROLLER_H_

