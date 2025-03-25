// Copyright [2023] <Your Name or Organization>
// hosmc_controller.cpp
#include "hosmc_controller.h"  // NOLINT

HOSMCController::HOSMCController() {
    tb = 0.0;
    alpha_0 = 500;
    alpha_c = 1;
    delta = 0.1;
    vel = 1;
    seg = 1;
    K_d << 100, 0, 0,
           0, 5, 0,
           // cppcheck-suppress constStatement
           0, 0, 5;
    K_i << 0.001, 0, 0,
           0, 0.0001, 0,
           // cppcheck-suppress constStatement
           0, 0, 0.0001;
    Thrustlim = 10.0;
    j_inv << 0.25,  1.0638, -1.7857,
               0.25, -1.0638, -1.7857,
               0.25, -1.0638,  1.7857,
               // cppcheck-suppress constStatement
               0.25,  1.0638,  1.7857;

    Eigen::Vector3d j0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d j1 = Eigen::Vector3d::Zero();
    S_eta_int = Eigen::Vector3d::Zero();
    Sd = Eigen::Vector3d::Zero();
    last_desired = 0.0;
}

Eigen::VectorXd HOSMCController::stabilize(const Eigen::VectorXd &eta_real, const Eigen::VectorXd &eta_desired) {
    Eigen::Vector3d u = model_free_control(eta_real, eta_desired);
    Eigen::VectorXd f_ver = j_inv * u;
    for (int i = 0; i < f_ver.size(); i++) {
        f_ver(i) = std::max(-Thrustlim, std::min(Thrustlim, f_ver(i)));
    }
    return f_ver;
}

Eigen::Vector3d HOSMCController::model_free_control(const Eigen::VectorXd &eta, const Eigen::VectorXd &eta_d) {
    Eigen::Vector3d eta_tilde = eta - eta_d;
    ros::Time current_time = ros::Time::now();
    if (std::abs(last_desired - eta_d(0)) > 1e-3) {
        t0 = current_time;
    }
    double t = (current_time - t0).toSec();
    double dT = (last_time.isZero()) ? 0.01 : (current_time - last_time).toSec();
    last_time = current_time;
    Eigen::Vector3d eta_dot = exact_differentiator(eta_tilde, dT);
    Eigen::Vector3d eta_dot_tilde = eta_dot;

    if (t < 1e-6) {
        tb = std::abs(eta_tilde(0)) / vel;
    }

    double denominator = std::max(seg * tb, 1e-6);
    double alpha = (t <= tb) ? alpha_c + (alpha_0 - alpha_c) *
                    (1 - std::pow(t / denominator, 2.0)) : alpha_c;

    Eigen::Vector3d S = eta_dot_tilde + alpha * eta_tilde;

    if (t < 1e-6) {
        Sd = S;
    }

    Eigen::Vector3d S_d = Sd / (1 + 1000 * t);
    Eigen::Vector3d S_eta = S - S_d;
    S_eta_int += (S_eta.array().sign() * dT).matrix();
    Eigen::Vector3d S_r = S_eta + K_i * S_eta_int;

    Eigen::Vector3d tau_eta = K_d * S_r;

    last_desired = eta_d(0);
    return tau_eta;
}

Eigen::Vector3d HOSMCController::exact_differentiator(const Eigen::Vector3d &reading, double dT) {
    double lambda1 = 0.5;
    double lambda2 = 0.1;

    Eigen::Vector3d e = reading - j0;
    j0 += (j1 + e.cwiseSqrt().cwiseProduct(e.cwiseSign()) * lambda1) * dT;
    j1 += e.cwiseSign() * lambda2 * dT;

    return j1;
}
