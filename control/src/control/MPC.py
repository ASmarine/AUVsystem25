import rospy
import casadi as ca
import numpy as np
from scipy.interpolate import CubicSpline, interp1d
from scipy.linalg import block_diag
from collections import deque

class ControlMPC:
    def __init__(self, sens, dT):
        # AUV parameters
        self.dT = dT  # Controller sampling time (sec)
        self.Tlim = 10.0  # Thruster output limit (N)
        
        # Initialize state and previous values
        self.taw = np.zeros(8)
        self.n = np.array([sens['orbX'], sens['orbY'], sens['ps'], 
                          sens['imu_euler'][0], sens['imu_euler'][1], sens['imu_euler'][2]])
        self.v = np.zeros(6)
        self.s_prev = np.zeros(3)
        self.U_prev = None
        self.X_prev = None
        self.prev_beta = None
        self.prev_delta = None
        
        # BLUE AUV parameters
        self.m = 13.5       # Mass (kg)
        self.Iz = 0.37      # Yaw inertia (kg*m^2)
        self.Xu_dot = 6.357
        self.Yv_dot = 7.121
        self.Nr_dot = 0.2215
        self.Xu = 13.7      # Surge drag coefficient
        self.Yv = 0.0       # Sway drag coefficient
        self.Nr = 0.0       # Yaw drag coefficient
        self.Du = 141.0
        self.Dv = 217.0
        self.Dr = 1.5
        
        # MPC parameters
        self.T = 1.5        # Prediction horizon (s)
        self.N = int(np.ceil(self.T / dT))  # Control intervals
        
        # Input constraints
        self.Tx_max = 20.0   # Max thrust (N)
        self.Ty_max = 5.0
        self.Mz_max = 0.1   # Max moment (Nm)
        
        # Weighting matrices
        self.Q = np.diag([50.0, 50.0, 5.0, 0.1, 0.1, 0.01])  # State weights
        self.R = np.diag([0.01, 0.01, 0.01])                  # Input weights
        self.P = 10 * self.Q  # Terminal cost weight
        
        # Path tracking parameters
        self.lookahead = 0.8  # Lookahead distance (m)
        self.k_r = 1.0        # Heading convergence rate
        self.path_points = None
        self.path_Z = None
        self.path_spline = None
        self.max_s = 1.0
        self.current_s = 0.0
        self.idx = 0
        
        # Setup CasADi
        self.setup_casadi()

    def setup_casadi(self):
        # States (x, y, psi, u, v, r) and inputs (Tx, Ty, Mz)
        x = ca.SX.sym('x', 6)
        u = ca.SX.sym('u', 3)
        n_states = x.size1()
        n_controls = u.size1()
        
        # Extract states
        psi = x[2]
        v_body = x[3:6]  # [u, v, r]
        
        # Corrected rotation matrix construction
        Rot = ca.SX(2, 2)
        Rot[0, 0] = ca.cos(psi)
        Rot[0, 1] = -ca.sin(psi)
        Rot[1, 0] = ca.sin(psi)
        Rot[1, 1] = ca.cos(psi)

        # Kinematics: [ẋ, ẏ, ψ̇]
        eta_dot = Rot @ v_body[0:2]
        psi_dot = v_body[2]
        
        # Inertia matrix
        Mu = self.m + self.Xu_dot
        Mv = self.m + self.Yv_dot
        Mr = self.Iz + self.Nr_dot
        M = ca.SX.zeros(3, 3)
        M[0, 0] = Mu
        M[1, 1] = Mv
        M[2, 2] = Mr
        
        # Coriolis matrix
        C = ca.SX.zeros(3,3)
        C[0,2] = -Mv * v_body[1]
        C[1,2] = Mu * v_body[0]
        C[2,0] = Mv * v_body[1]
        C[2,1] = -Mu * v_body[0]
        
        # Damping matrix (linear + quadratic)
        D = ca.SX.zeros(3,3)
        D[0,0] = -self.Xu * v_body[0] + self.Du * ca.fabs(v_body[0])
        D[1,1] = -self.Yv * v_body[1] + self.Dv * ca.fabs(v_body[1])
        D[2,2] = -self.Nr * v_body[2] + self.Dr * ca.fabs(v_body[2])
        
        # Dynamics: v̇ = M⁻¹ (u - C*v_body - D*v_body)
        v_dot = ca.solve(M, u - C@v_body - D@v_body)
        
        # Full state derivative
        xdot = ca.vertcat(eta_dot, psi_dot, v_dot)
        
        # RK4 integrator
        k1 = xdot
        k2 = xdot + (self.dT/2)*k1
        k3 = xdot + (self.dT/2)*k2
        k4 = xdot + self.dT*k3
        x_next = x + (self.dT/6)*(k1 + 2*k2 + 2*k3 + k4)
        self.F_rk4 = ca.Function('F_rk4', [x, u], [x_next])
        
        # Multiple shooting variables
        X_mpc = ca.SX.sym('X_mpc', n_states, self.N+1)
        U_mpc = ca.SX.sym('U_mpc', n_controls, self.N)
        
        # Parameters: initial state and reference
        x_init = ca.SX.sym('x_init', n_states)
        x_ref = ca.SX.sym('x_ref', n_states)
        
        # Cost and constraints
        cost = 0
        g = []
        g.append(X_mpc[:,0] - x_init)  # Initial state constraint
        
        # Build cost and dynamics constraints
        for i in range(self.N):
            state_error = X_mpc[:,i] - x_ref
            cost += state_error.T @ self.Q @ state_error
            cost += U_mpc[:,i].T @ self.R @ U_mpc[:,i]
            
            # Dynamics constraint
            x_next_pred = self.F_rk4(X_mpc[:,i], U_mpc[:,i])
            g.append(X_mpc[:,i+1] - x_next_pred)
        
        # Terminal cost
        terminal_error = X_mpc[:,-1] - x_ref
        cost += terminal_error.T @ self.P @ terminal_error
        
        # NLP variables and constraints
        OPT_vars = ca.vertcat(U_mpc.reshape((-1,1)), X_mpc.reshape((-1,1)))
        g = ca.vertcat(*g)
        
        # Input bounds
        lbu = [-self.Tx_max, -self.Ty_max, -self.Mz_max] * self.N
        ubu = [self.Tx_max, self.Ty_max, self.Mz_max] * self.N
        lbx = [-ca.inf] * (n_states*(self.N+1))
        ubx = [ca.inf] * (n_states*(self.N+1))
        lbx = lbu + lbx
        ubx = ubu + ubx
        
        # NLP problem
        nlp = {
            'x': OPT_vars,
            'f': cost,
            'g': g,
            'p': ca.vertcat(x_init, x_ref)
        }
        
        # Solver options
        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.tol': 1e-2,
            'ipopt.acceptable_tol': 1e-2,
            'ipopt.acceptable_obj_change_tol': 1e-3,
            'ipopt.max_iter': 50
        }

        self.solver = ca.nlpsol('solver', 'ipopt', nlp, opts)
        
        # Store bounds
        self.lbx = lbx
        self.ubx = ubx
        self.lbg = [0] * g.size1()
        self.ubg = [0] * g.size1()

    def solve_mpc(self, x0, x_ref):
        # Initial guess
        if self.U_prev is None:
            X_guess = np.tile(x0, (self.N+1, 1)).T
            U_guess = np.zeros((3, self.N))
        else:
            X_guess = self.X_prev
            U_guess = self.U_prev
        
        # Solve NLP
        sol = self.solver(
            x0=np.concatenate([U_guess.T.flatten(), X_guess.T.flatten()]),
            lbx=self.lbx,
            ubx=self.ubx,
            lbg=self.lbg,
            ubg=self.ubg,
            p=np.concatenate([x0, x_ref])
        )
        
        # Extract solution
        opt_vars = sol['x'].full().flatten()
        U_opt = opt_vars[:3*self.N].reshape(3, self.N, order='F')
        X_opt = opt_vars[3*self.N:].reshape(6, self.N+1, order='F')
        
        # Store for next iteration
        self.U_prev = U_opt
        self.X_prev = X_opt

        return U_opt[:,0], X_opt

    def exact_differentiator(self, reading, prev_reading):
        return (reading - prev_reading) / self.dT

    def track(self, pose_des, pose_curr):
        # Solve MPC for tracking
        u_opt, _ = self.solve_mpc(pose_curr, pose_des)
        f_hor = np.clip(u_opt, -self.Tlim, self.Tlim)
        return f_hor

    def set_path(self, path3D):
        # Create parametric spline for XY path
        K = np.linspace(0, 1, path3D.shape[0])
        self.path_points = path3D[:, :2]
        self.path_Z = path3D[:, 2]
        self.path_spline = CubicSpline(K, self.path_points)
        self.max_s = K[-1]

    def find_closest_point(self, x_current):
        # Objective function: distance to current position
        def dist(s):
            pt = self.path_spline(s)
            return np.linalg.norm(pt - x_current[:2])
        
        # Minimize distance within valid s range
        from scipy.optimize import minimize_scalar
        res = minimize_scalar(
            dist, 
            bounds=(self.current_s, min(self.current_s + 0.2, self.max_s)),
            method='bounded'
        )
        s = res.x
        closest_pt = self.path_spline(s)
        return closest_pt, s

    def find_lookahead_point(self, x_current):
        # Binary search for lookahead point
        s_low = self.current_s
        s_high = min(self.current_s + 0.5, self.max_s)
        
        for _ in range(20):  # Max 20 iterations
            s_mid = (s_low + s_high) / 2
            pt = self.path_spline(s_mid)
            dist = np.linalg.norm(pt - x_current[:2])
            
            if dist < self.lookahead:
                s_low = s_mid
            else:
                s_high = s_mid
            if s_high - s_low < 1e-4:
                break
        
        s = (s_low + s_high) / 2
        return self.path_spline(s), s

    def track_path(self, x_current):
        # Find lookahead point
        lookahead_pt, s = self.find_lookahead_point(x_current)
        self.current_s = s
        
        # Path tangent and heading
        tangent = self.path_spline(s, 1)
        tangent /= np.linalg.norm(tangent)
        psi_path = np.arctan2(tangent[1], tangent[0])
        
        # Cross-track error
        normal = np.array([-tangent[1], tangent[0]])
        rel_pos = x_current[:2] - lookahead_pt
        y1 = normal @ rel_pos
        
        # Heading error
        psi_e = np.arctan2(np.sin(x_current[2]-psi_path), np.cos(x_current[2]-psi_path))
        
        # Path curvature
        ddt = self.path_spline(s, 2)
        c_c = (tangent[0]*ddt[1] - tangent[1]*ddt[0]) / (np.linalg.norm(tangent)**3)
        
        # LOS guidance
        delta = -np.arctan2(y1, self.lookahead)
        
        # Sideslip angle
        beta = np.arctan2(x_current[4], x_current[3])
        
        # Numerical derivatives
        if self.prev_beta is None:
            self.prev_beta = beta
            self.prev_delta = delta
        beta_dot = (beta - self.prev_beta) / self.dT
        delta_dot = (delta - self.prev_delta) / self.dT
        self.prev_beta = beta
        self.prev_delta = delta
        
        # Path speed
        s_dot = x_current[3]*np.cos(psi_e) + x_current[4]*np.sin(psi_e)
        
        # Desired yaw rate
        r_des = delta_dot - beta_dot - self.k_r*(psi_e - delta) + c_c*s_dot
        
        # Reference state
        x_ref = np.array([
            lookahead_pt[0], lookahead_pt[1],  # Position
            psi_path + delta,                  # Heading
            0.0,                               # Surge velocity
            0.1 * np.sign(y1),                 # Sway velocity
            r_des                              # Yaw rate
        ])
        
        # Solve MPC
        u_opt, _ = self.solve_mpc(x_current, x_ref)
        return np.clip(u_opt, -self.Tlim, self.Tlim)

    def actuate(self, sens, n_des):
        # Current state
        s = np.array([sens['orbX'], sens['orbY'], sens['imu_euler'][2]])
        s_dot = self.exact_differentiator(s, self.s_prev)
        self.s_prev = s
        
        # Form state vector [x, y, psi, u, v, r]
        x_current = np.concatenate([s, s_dot])

        # Extract ONLY position/yaw from n_des (ignore velocities)
        x_des = np.array([n_des[0], n_des[1], n_des[2], 0, 0, 0])  # Velocities=0
        
        # Choose tracking method
        if self.path_points is None:
            f_hor = self.track(x_des, x_current)
        else:
            f_hor = self.track_path(x_current)
        
        return f_hor
