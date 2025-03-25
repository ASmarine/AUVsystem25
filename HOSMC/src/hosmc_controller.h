// Copyright [2023] [Your Name or Your Organization]

#ifndef _HOME_MAHMOUD_AUV_2025_SRC_HOSMC_SRC_HOSMC_CONTROLLER_H_
#define _HOME_MAHMOUD_AUV_2025_SRC_HOSMC_SRC_HOSMC_CONTROLLER_H_

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>
#include "HOSMC/SetZDesired.h"

class HOSMCController {
 public:
    HOSMCController();
    Eigen::VectorXd stabilize(const Eigen::VectorXd &eta_real, const Eigen::VectorXd &eta_desired);

 private:
    Eigen::Vector3d model_free_control(const Eigen::VectorXd &eta, const Eigen::VectorXd &eta_d);
    Eigen::Vector3d exact_differentiator(const Eigen::Vector3d &reading, double dT);
    Eigen::Matrix<double, 4, 3> j_inv;
    Eigen::Vector3d S_eta_int;
    Eigen::Vector3d Sd;
    Eigen::Vector3d j0;
    Eigen::Vector3d j1;
    ros::Time t0;
    ros::Time last_time;
    double last_desired;
    double tb;
    double alpha_0;
    double alpha_c;
    double delta;
    double vel;
    double seg;
    Eigen::Matrix3d K_d;
    Eigen::Matrix3d K_i;
    double Thrustlim;
};

#endif  // _HOME_MAHMOUD_AUV_2025_SRC_HOSMC_SRC_HOSMC_CONTROLLER_H_
