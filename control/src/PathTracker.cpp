// Copyright 2024 Mahmoud

#include "control/PathTracker.h"

PathTracker::PathTracker(const ros::NodeHandle& nh) {
    // Initialize with default parameters
    last_closest_idx_ = 0;
    integral_err_ = 0.0;
    prev_error_ = 0.0;

    // Load parameters from ROS parameter server
    if (!nh.getParam("dt", dt_)) ROS_ERROR("Failed to get param 'dt'");
    nh.param("lookahead_distance", lookahead_, lookahead_);
    nh.param("dt", dt_);
    nh.param("Kp", Kp_);
    nh.param("Ki", Ki_);
    nh.param("Kd", Kd_);
}

void PathTracker::setPath(const std::vector<Eigen::Vector3d>& path_points) {
    if (path_points.empty()) {
        throw std::invalid_argument("Path points cannot be empty");
    }

    path_points_ = path_points;
    last_closest_idx_ = 0;
    integral_err_ = 0.0;
    prev_error_ = 0.0;
}

void PathTracker::setDt(double dt) {
    if (dt <= 0.0) {
        throw std::invalid_argument("Time step dt must be positive");
    }
    dt_ = dt;
}

double PathTracker::trackPathPID(const double x_current, const double x_desired) {
    // PID for x-position tracking
    double x_error = x_desired - x_current;

    // Update PID terms
    integral_err_ += x_error * dt_;
    double derivative = (x_error - prev_error_) / dt_;
    prev_error_ = x_error;

    // PID control law (surge acceleration)
    return Kp_ * x_error + Ki_ * integral_err_ + Kd_ * derivative;
}

Eigen::Vector4d PathTracker::trackPathBackstepping(const Eigen::VectorXd& x_current) {
    if (path_points_.empty()) {
        throw std::runtime_error("Path not set. Call setPath() first.");
    }

    // Find the closest point on the path and compute errors
    auto [y1, closest_idx, z] = computeSerretFrenetErrors(x_current);
    last_closest_idx_ = closest_idx;

    // Backstepping: Desired yaw rate (LOS guidance)
    double delta = -atan2(y1, lookahead_);

    // Get path tangent angle (projected to x-y plane)
    Eigen::Vector2d tangent_vec;
    if (closest_idx < path_points_.size() - 1) {
        tangent_vec = path_points_[closest_idx + 1].head<2>() - path_points_[closest_idx].head<2>();
    } else {
        tangent_vec = path_points_[closest_idx].head<2>() - path_points_[closest_idx - 1].head<2>();
    }

    double psi_path = atan2(tangent_vec.y(), tangent_vec.x());
    double psi_des = psi_path + delta;

    return Eigen::Vector4d(
        path_points_[closest_idx].x(),  // Target x
        path_points_[closest_idx].y(),  // Target y
        z,                              // Target z (original height)
        psi_des);                         // Desired heading
}

std::tuple<double, size_t, double> PathTracker::computeSerretFrenetErrors(const Eigen::VectorXd& x_current) {
    // Calculate distances to all points (projected to x-y plane)
    std::vector<double> distances;
    Eigen::Vector2d current_pos(x_current(0), x_current(1));
    for (const auto& pt : path_points_) {
        distances.push_back((pt.head<2>() - current_pos).norm());
    }

    // Start search from last closest index (for efficiency)
    size_t start_idx = (last_closest_idx_ > 5) ? last_closest_idx_ - 5 : 0;
    size_t end_idx = std::min(last_closest_idx_ + 50, path_points_.size() - 1);

    // Find all points ahead that are beyond lookahead distance
    std::vector<size_t> candidate_indices;
    for (size_t i = start_idx; i <= end_idx; ++i) {
        if (distances[i] >= lookahead_) {
            candidate_indices.push_back(i);
        }
    }

    size_t target_idx;
    if (!candidate_indices.empty()) {
        // Among candidates, find the one closest to the vehicle
        auto min_it = std::min_element(candidate_indices.begin(), candidate_indices.end(),
            [&distances](size_t a, size_t b) { return distances[a] < distances[b]; });
        target_idx = *min_it;
    } else {
        // If no points are beyond lookahead, use the last point
        target_idx = path_points_.size() - 1;
    }

    // Ensure we don't go backward (always move forward along path)
    target_idx = std::max(target_idx, last_closest_idx_);

    Eigen::Vector3d target_pt = path_points_[target_idx];

    // Get path tangent at target point
    Eigen::Vector2d tangent;
    if (target_idx < path_points_.size() - 1) {
        tangent = (path_points_[target_idx + 1].head<2>() - target_pt.head<2>());
    } else {
        tangent = (target_pt.head<2>() - path_points_[target_idx - 1].head<2>());
    }

    // Normalize tangent vector
    if (tangent.norm() > 1e-3) {
        tangent.normalize();
    } else {
        tangent = Eigen::Vector2d(1.0, 0.0);  // Default if points are coincident
    }

    // Path angle (theta)
    double theta = atan2(tangent.y(), tangent.x());

    // Cross-track error (y1) - positive to left of path
    Eigen::Matrix2d Rot;
    Rot << cos(theta), sin(theta),
          -sin(theta), cos(theta);

    Eigen::Vector2d relative_pos = current_pos - target_pt.head<2>();
    double y1 = (Rot.row(1) * relative_pos);

    return {y1, target_idx, target_pt.z()};  // Now includes z coordinate
}
