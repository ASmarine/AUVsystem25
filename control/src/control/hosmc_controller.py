#!/usr/bin/env python
import rospy
import numpy as np

class HOSMCController:
    def __init__(self):
        self.tb = rospy.get_param("tb")
        self.alpha_0 = rospy.get_param("alpha_0")
        self.alpha_c = rospy.get_param("alpha_c")
        self.delta = rospy.get_param("delta")
        self.vel = rospy.get_param("vel")
        self.seg = rospy.get_param("seg")
        self.Thrustlim = rospy.get_param("Thrustlim")

        K_d = np.array(rospy.get_param("K_d")).reshape((4, 4))
        K_i = np.array(rospy.get_param("K_i")).reshape((4, 4))

        self.K_d = K_d
        self.K_i = K_i

        self.S_eta_int = np.zeros(4)
        self.Sd = np.zeros(4)
        self.last_desired = 0.0
        self.prev_reading = np.zeros(4)
        self.t0 = rospy.Time.now()
        self.last_time = rospy.Time(0)
        self.eta_dot_filtered = np.zeros(4)

    def stabilize(self, eta_real, eta_desired):
        f_ver = self.model_free_control(eta_real, eta_desired)
        f_ver = np.clip(f_ver, -self.Thrustlim, self.Thrustlim)
        return f_ver

    def model_free_control(self, eta, eta_d):
        eta_tilde = eta - eta_d
        current_time = rospy.Time.now()

        if abs(self.last_desired) < 1e-3:
            self.t0 = current_time

        t = (current_time - self.t0).to_sec()
        dT = 0.01 if self.last_time.is_zero() else (current_time - self.last_time).to_sec()
        self.last_time = current_time

        eta_dot = self.normal_differentiator(eta_tilde, dT)
        eta_dot_tilde = eta_dot

        if t < 1e-6:
            self.tb = abs(eta_tilde[0]) / self.vel

        denominator = max(self.seg * self.tb, 1e-6)
        alpha = self.alpha_c + (self.alpha_0 - self.alpha_c) * (1 - (t / denominator) ** 2) if t <= self.tb else self.alpha_c

        S = eta_dot_tilde + alpha * eta_tilde

        if t < 1e-6:
            self.Sd = S

        S_d = self.Sd / (1 + 1000 * t)
        S_eta = S - S_d
        self.S_eta_int += np.sign(S_eta) * dT
        S_r = S_eta + self.K_i @ self.S_eta_int

        tau_eta = -self.K_d @ S_r
        self.last_desired = eta_d[0]

        return tau_eta

    def normal_differentiator(self, reading, dT):
        if dT <= 1e-6:
            return np.zeros(4)
        raw_derivative = (reading - self.prev_reading) / dT
        self.prev_reading = reading
        alpha = 0.1  # tune this (0 < alpha < 1), smaller = more smoothing
        self.eta_dot_filtered = alpha * raw_derivative + (1 - alpha) * self.eta_dot_filtered
        return self.eta_dot_filtered
