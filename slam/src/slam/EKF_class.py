#!/usr/bin/env python3
import numpy as np
import math

class EKF:
    """
    This class represents an EKF computating object.

    Attributes:
        - Mu(numpy array): mean vector (x, y, z, yaw,
        V_x, V_y, V_z, w_yaw) taking into consideration
        that the pose states are relative to the global frame.
        - Mu_dot(numpy array): vector of mean rates.
        - COV_matrix(numpy array): Covariance matrix.
        - AUV_mass(float): mass of auv in kg.
        - AUV_added_mass(float): added mass effect in kg.
        - I_matrix(numpy array): AUV inertial matrix.
        - Tau_g(numpy array): AUV weight vector in N.
        - Tau_b(numpy array): AUV buoyancy vector in N.
        - Drag_term(numpy array): Drag co-efficients matrix.
        - Lift_term(numpy array): Lift co-efficients matrix.
        - C_matrix(numpy array): Total Coriolis matrix.
        - U(numpy array): Control action vector.
        - G(numpy array): Jacobian matrix.
        - Time(float): Last recorded time in seconds.
        - R(numpy array): motion model noise matrix.
        - C(numpy array): output matrix.
        - Q(numpy array): Sensory data noise matrix.     

    Developer/s:
        Samer A. Mohamed, Hossam M. Elkeshky.
    """
    def __init__(self, AUV_mass, AUV_added_mass, I_matrix, Tau_g, Tau_b, Drag_term, Lift_term, Time, R, C):
        """
        Class constructor: initializes some attributes regarding our
        AUV's inertial parameters, hydrostatics and hydrodynamics
    
        Args:
            - AUV_mass(float): mass of auv in kg
            - AUV_added_mass(float): added mass effect in kg
            - I_matrix(numpy array): AUV inertial matrix
            - Tau_g(numpy array): AUV weight vector in N.
            - Tau_b(numpy array): AUV buoyancy vector in N.
            - Drag_term(numpy array): Drag co-efficients matrix.
            - Lift_term(numpy array): Lift co-efficients matrix.
            - Time(float): Time of class creation.
            - R(numpy array): motion model noise matrix.
            - C(numpy array): output matrix.
    
        Returns:
            N/A.
    
        Developer/s:
            Samer A. Mohamed, Hossam M. Elkeshky.

        """
        # Initialize mean vector
        self.Mu = np.zeros((8,1)) # Creates an 8x1 column vector

        # Initialize vector of mean rates and covariance matrix
        self.Mu_dot = np.zeros((8,1)) # 8x1 column vector for rate of change
        self.COV_matrix = np.zeros((8,8)) # 8x8 covariance matrix

        # Set AUV's inertial parameters, hydrostatics & 
        # hydrodynamic forces co-efficients 
        self.AUV_mass = AUV_mass
        self.AUV_added_mass = AUV_added_mass
        self.I_matrix = I_matrix
        self.Tau_g = Tau_g
        self.Tau_b = Tau_b
        self.Drag_term = -1*Drag_term
        self.Lift_term = Lift_term
        
        # Initialize control action & jacobian
        self.U = np.zeros((4,1))
        self.G = np.identity(8)

        # Initialize Time
        self.Time = Time
        print(Time)
        # Set motion model noise matrix
        self.R = R

        # Set output matrix
        self.C = C

        # Initialize sensory data noise matrix
        self.Q = np.zeros((8,8))

        # Set noise to infinite for states with unavailable 
        # correction data (V_x, V_y, V_z, w_yaw)
        for i in range(4,8):
            self.Q[i,i] = 1.0e+10
        
    def predict(self, U, Time):
        """
        predict function: Executes prediction step in EKF algorithm
        after aquiring new control action updating the mean and
        covariance matrix.

        Args:
            - U(numpy array): Latest control action.
            - Time(float): Latest control action time stamp in seconds.

        Returns:
            N/A
    
        Developer/s:
            Samer A. Mohamed, Hossam M. Elkeshky.
        """
        # Update control action for usage with update step
        # on receiving sensory data
        self.U = U

        # Calculate drag and lift terms
        if self.Mu[4,0] < 0.0:              # Surge (up/-ve direction)
            self.I_matrix[0,0] = 256.5592
            self.Drag_term[0,0] = -385.2905
        else:                               # Surge (up/+ve direction)
            self.I_matrix[0,0] = 364.7894
            self.Drag_term[0,0] = -290.209 

        if self.Mu[6,0] < 0.0:              # Heave (up/-ve direction)
            self.I_matrix[2,2] = 96.4141
            self.Drag_term[2,2] = -2233.0651
        else:                               # Heave (up/+ve direction)
            self.I_matrix[2,2] = 104.8941
            self.Drag_term[2,2] = -362.5386

        Inverse_I_matrix = np.linalg.inv(self.I_matrix)
        # Prediction step:
        # 1 - Full dynamic model equation
        V_local = self.Mu[4:8,0].reshape(-1,1)
        """" converted the + sign to a - sign """
        """" lift force term needs to be revised """
        M_V_dot = U + (self.Drag_term @ (V_local * abs(V_local))) 

        # 3 - calculate local V_dot
        """"Edited change the matrix multiplication to @"""
        V_local_dot = Inverse_I_matrix @ M_V_dot

        # # 4 - Update mean rate of change
        self.Mu_dot[4:8,0] = V_local_dot[0:4,0] # Acceeleration local
        self.Mu_dot[0,0] = (V_local[0,0] * math.cos(self.Mu[3,0])) - (V_local[1,0] * math.sin(self.Mu[3,0]))
        self.Mu_dot[1,0] = (V_local[0,0] * math.sin(self.Mu[3,0])) + (V_local[1,0] * math.cos(self.Mu[3,0]))
        self.Mu_dot[2,0] = V_local[2,0]
        self.Mu_dot[3,0] = V_local[3,0]
        
        # 5 - Jacobian calculation
        """" (Time - self.Time) this will alwas be a value """
        # 5-a- first row "x"
        self.G[0,3] = -1.0 * (Time - self.Time) * ((V_local[0,0] * math.sin(self.Mu[3,0])) + (V_local[1,0] * math.cos(self.Mu[3,0])))
        self.G[0,4] = (Time - self.Time) * math.cos(self.Mu[3,0])
        self.G[0,5] = -1.0 * (Time - self.Time) * math.sin(self.Mu[3,0])

        # 5-b- second row "y"
        self.G[1,3] = (Time - self.Time) * ((V_local[0,0] * math.cos(self.Mu[3,0])) - (V_local[1,0] * math.sin(self.Mu[3,0])))
        self.G[1,4] = (Time - self.Time) * math.sin(self.Mu[3,0])
        self.G[1,5] = (Time - self.Time) * math.cos(self.Mu[3,0])

        # 5-c- third row "z"
        self.G[2,6] = Time - self.Time

        # 5-d- fourth row "yaw"
        self.G[3,7] = Time - self.Time
        
        # 5-e- fifth row "Vx"
        self.G[4,4] = self.G[4,4] + ((Time - self.Time) * Inverse_I_matrix[0,0] * 2.0 * self.Drag_term[0, 0] * V_local[0,0])

        # 5-f- sixth row "Vy"
        self.G[5,5] = self.G[5,5] + ((Time - self.Time) * 2.0 * Inverse_I_matrix[1,1] * self.Drag_term[1,1] * V_local[1,0])

        # 5-g- seventh row "Vz"
        self.G[6,4] = Inverse_I_matrix[2,2] * 2.0 * self.Lift_term[2,0] * V_local[0,0] * (Time - self.Time)
        self.G[6,5] = Inverse_I_matrix[2,2] * 2.0 * self.Lift_term[2,1] * V_local[1,0] * (Time - self.Time)
        self.G[6,6] = self.G[6,6] + ((Time - self.Time) * Inverse_I_matrix[2,2] * 2.0 * self.Drag_term[2,2] * V_local[2,0])

        # 5-h- eighth row "w_yaw"
        self.G[7,7] = self.G[7,7] + ((Time - self.Time) * 2.0 * self.Drag_term[3,3] * V_local[3,0] * Inverse_I_matrix[3,3])

        # 6 - Update mean
        self.Mu[0:8,0] = self.Mu[0:8,0] + ((Time - self.Time) * self.Mu_dot[0:8,0])
        
        # 7 - Update Covariance matrix
        self.COV_matrix[0:8,0:8] = self.G @ self.COV_matrix[0:8,0:8] @ np.transpose(self.G) + self.R

        # 8 - Update Time
        self.Time = Time

    def correct(self, z, Q):
        """
        correct function: Executes correction step in EKF algorithm
        after aquiring new sensory readings updating the mean and
        covariance matrix.

        Args:
            - z(numpy array): Latest sensory data.
            - Q(numpy array): Latest sensory data noise matrix 
              according to which sensory data is being handled.

        Returns:
            N/A
    
        Developer/s:
            Samer A. Mohamed, Hossam M. Elkeshky.
        """
        # Update corresponding noise matrix
        for i in range(0,4):
            self.Q[i,i] = Q[i]
        # Correction step:
        # 1 - Calculate Kalman gain
        K = self.COV_matrix[0:8,0:8] @ np.transpose(self.C) @ np.linalg.inv(((self.C @ self.COV_matrix[0:8,0:8] @ np.transpose(self.C)) + self.Q[0:8,0:8]))
        # 2 - Update mean vector
        """Edited --> reshaped z to 2D array"""
        K = np.identity(8)
        self.Mu[0:8,0] = np.ones(8)
        print("before correction",self.Mu[0:8,0])
        print("z",z)
        self.Mu[0:8,0] = self.Mu[0:8,0] + (K @ (z.reshape(-1) - (self.C @ self.Mu[0:8,0])))
        print("after correction",self.Mu[0:8,0])
        # 3 - Update covariance matrix
        self.COV_matrix[0:8,0:8] = (np.identity(8) - (K @ self.C)) @ self.COV_matrix[0:8,0:8]

    def correct_landmarks(self, z_landmarks, Q_landmarks):
        """
        correct_landmarks function: Executes correction step in
        EKF algorithm after aquiring new landmarks sensory
        readings updating the total mean and covariance matrix.

        Args:
            - z_landmarks(numpy array): Latest landmark sensory data.
            - Q_landmarks(numpy array): Latest landmark data noise matrix 
              according to which sensory data is being handled.

        Returns:
            N/A
    
        Developer/s:
            Samer A. Mohamed, Hossam M. Elkeshky.
        """
        # Update mean, covariance matrix & sensor noise matrix
        # size according to number of landmarks
        if np.size(z_landmarks) > np.size(self.Mu) - 8:
            old_Mu_size = np.size(self.Mu)
            self.Mu = np.concatenate((self.Mu, np.zeros(np.size(z_landmarks) - (old_Mu_size - 8)).reshape(-1,1)))
            i = np.identity(np.size(self.Mu))
            i[0:old_Mu_size,0:old_Mu_size] = self.COV_matrix
            for j in range(old_Mu_size,np.size(self.Mu)):
                i[j,j] = 1.0e+10
            self.COV_matrix = i
            self.Q = np.identity(np.size(self.Mu))
        z = np.zeros(np.size(self.Mu)).reshape(-1,1)
        z[8:np.size(self.Mu),0] = z_landmarks
        for i in range(0,np.size(self.Mu)):
            if i < 8:
                self.Q[i,i] = 1.0e+10
            else:
                self.Q[i,i] = Q_landmarks[i-8,i-8]
        # Calculate non linear model related to landmark readings
        # and its jacobian (h(Mu) & H)
        h = np.zeros(np.size(self.Mu)).reshape(-1,1)
        H = np.identity(np.size(self.Mu))
        for i in range(0, (np.size(self.Mu)-8)/3):
            h[8+(i*3),0] = (self.Mu[8+(i*3),0] * math.cos(self.Mu[3,0])) + (self.Mu[9+(i*3),0] * math.sin(self.Mu[3,0])) \
                           - (self.Mu[0,0] * math.cos(self.Mu[3,0])) - (self.Mu[1,0] * math.sin(self.Mu[3,0]))
            h[9+(i*3),0] = (-1.0 * self.Mu[8+(i*3),0] * math.sin(self.Mu[3,0])) + (self.Mu[9+(i*3),0] * math.cos(self.Mu[3,0])) \
                           + (self.Mu[0,0] * math.sin(self.Mu[3,0])) - (self.Mu[1,0] * math.cos(self.Mu[3,0]))
            h[10+(i*3),0] = self.Mu[10+(i*3),0] - self.Mu[2,0]

            H[8+(i*3),0] = -1.0 * math.cos(self.Mu[3,0])
            H[8+(i*3),1] = -1.0 * math.sin(self.Mu[3,0])
            H[8+(i*3),3] = (-1.0 * math.sin(self.Mu[3,0]) * self.Mu[8+(i*3),0]) + (math.cos(self.Mu[3,0]) * self.Mu[9+(i*3),0]) \
                           + (self.Mu[0,0] * math.sin(self.Mu[3,0])) - (self.Mu[1,0] * math.cos(self.Mu[3,0]))
            H[8+(i*3),8+(i*3)] = math.cos(self.Mu[3,0])
            H[8+(i*3),9+(i*3)] = math.sin(self.Mu[3,0])

            H[9+(i*3),0] = math.sin(self.Mu[3,0])
            H[9+(i*3),1] = -1.0 * math.cos(self.Mu[3,0])
            H[9+(i*3),3] = (-1.0 * math.cos(self.Mu[3,0]) * self.Mu[8+(i*3),0]) - (math.sin(self.Mu[3,0]) * self.Mu[9+(i*3),0]) \
                           + (self.Mu[0,0] * math.cos(self.Mu[3,0])) + (self.Mu[1,0] * math.sin(self.Mu[3,0]))
            H[9+(i*3),8+(i*3)] = -1.0 * math.sin(self.Mu[3,0])
            H[9+(i*3),9+(i*3)] = math.cos(self.Mu[3,0])

            H[10+(i*3), 2] = -1.0
            H[10+(i*3), 10+(i*3)] = 1.0
        # Correction step:
        # 1 - Calculate Kalman gain
        K = self.COV_matrix @ np.transpose(H) @ np.linalg.inv(((H @ self.COV_matrix @ np.transpose(H)) + self.Q))
        # 2 - Update mean vector
        self.Mu = self.Mu + (K @ (z - h))
        # 3 - Update covariance matrix
        self.COV_matrix = (np.identity(np.size(self.Mu)) - (K @ H)) @ self.COV_matrix
    
    def get_control(self):
        """
        get_control function: Returns latest control action

        Args:
            N/A

        Returns:
            self.U
    
        Developer/s:
            Samer A. Mohamed, Hossam M. Elkeshky.
        """
        return self.U
    
    def get_mean(self):
        """
        get_mean function: Returns latest mean vector

        Args:
            N/A

        Returns:
            self.Mu
    
        Developer/s:
            Samer A. Mohamed, Hossam M. Elkeshky.
        """
        return self.Mu
    
    def get_COV(self):
        """
        get_COV function: Returns latest covariance matrix

        Args:
            N/A

        Returns:
            self.COV_matrix
    
        Developer/s:
            Samer A. Mohamed, Hossam M. Elkeshky.
        """
        return self.COV_matrix