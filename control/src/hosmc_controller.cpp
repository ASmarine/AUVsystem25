// Copyright [2023] <Your Name or Organization>
#include "control/hosmc_controller.h"

HOSMCController::HOSMCController(ros::NodeHandle& nh) {
    if (!nh.getParam("tb", tb)) ROS_ERROR("Failed to get param 'tb'");
    nh.getParam("tb", tb);
    nh.getParam("alpha_0", alpha_0);
    nh.getParam("alpha_c", alpha_c);
    nh.getParam("delta", delta);
    nh.getParam("vel", vel);
    nh.getParam("seg", seg);
    nh.getParam("Thrustlim", Thrustlim);

    std::vector<double> kd_list, ki_list;
    nh.getParam("K_d", kd_list);
    nh.getParam("K_i", ki_list);

    K_d = Eigen::Map<Eigen::Matrix4d>(kd_list.data());
    K_i = Eigen::Map<Eigen::Matrix4d>(ki_list.data());

    S_eta_int = Eigen::Vector4d::Zero();
    Sd = Eigen::Vector4d::Zero();
    last_desired = 0.0;
}


Eigen::VectorXd HOSMCController::stabilize(
    const Eigen::VectorXd &eta_real,
    const Eigen::VectorXd &eta_desired) {
    Eigen::Vector4d f_ver = model_free_control(eta_real, eta_desired);
    // Eigen::VectorXd f_ver = j_inv * u;
    for (int i = 0; i < f_ver.size(); i++) {
        f_ver(i) = std::max(-Thrustlim, std::min(Thrustlim, f_ver(i)));
    }
    return f_ver;
}

Eigen::Vector4d HOSMCController::model_free_control(
    const Eigen::VectorXd &eta, const Eigen::VectorXd &eta_d) {
    Eigen::Vector4d eta_tilde = eta - eta_d;
    ros::Time current_time = ros::Time::now();
    if (std::abs(last_desired) < 1e-3) {
        t0 = current_time;
    }
    double t = (current_time - t0).toSec();
    double dT = (last_time.isZero()) ? 0.01 : (current_time - last_time).toSec();
    last_time = current_time;

    Eigen::Vector4d eta_dot = normal_differentiator(eta_tilde, dT);
    Eigen::Vector4d eta_dot_tilde = eta_dot;

    if (t < 1e-6) {
        tb = std::abs(eta_tilde(0)) / vel;
    }

    double denominator = std::max(seg * tb, 1e-6);
    double alpha = (t <= tb) ? alpha_c + (alpha_0 - alpha_c) *
                    (1 - std::pow(t / denominator, 2.0)) : alpha_c;

    Eigen::Vector4d S = eta_dot_tilde + alpha * eta_tilde;

    if (t < 1e-6) {
        Sd = S;
    }

    Eigen::Vector4d S_d = Sd / (1 + 1000 * t);
    Eigen::Vector4d S_eta = S - S_d;
    S_eta_int += (S_eta.array().sign() * dT).matrix();
    Eigen::Vector4d S_r = S_eta + K_i * S_eta_int;

    Eigen::Vector4d tau_eta = -K_d * S_r;

    last_desired = eta_d(0);
    return tau_eta;
}

// HOSMCController.cpp
Eigen::Vector4d HOSMCController::normal_differentiator(const Eigen::Vector4d &reading, double dT) {
    if (dT <= 1e-6) {
        return Eigen::Vector4d::Zero();  // avoid division by zero
    }

    Eigen::Vector4d derivative = (reading - prev_reading_) / dT;
    prev_reading_ = reading;

    return derivative;
}


