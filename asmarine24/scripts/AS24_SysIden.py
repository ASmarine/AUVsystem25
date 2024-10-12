#!/usr/bin/env python3
import sys
import numpy as np
import rospy
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Imu, FluidPressure
from tf.transformations import euler_from_quaternion

class sys_iden_class:
    """
    AUV System Identification Class.

    This class initiates a ROS node for system identification.
    It subscribes to processed IMU (Inertial Measurement Unit) and fluid pressure sensor topics, 
    processes the sensor data, performs PID control calculations, 
    and publishes a wrench to excite and stabilize an AUV.

    Attributes:
        pub (ROS publisher): publishes excitation/stabilization wrench.
        excite_index (int): An index representing the system's excitation state, based on user input ('x', 'y', 'z', 'k', 'm', or 'n').
        pid_mask (numpy.ndarray): Masking matrix that hides unstabilized states from PID calculations.
        pid_time (float): The current timestamp.
        pid_I (numpy.ndarray): An array that stores the integral errors for the PID controller.
        FSM (numpy.ndarray): A finite state machine representing the excitation states and transition triggers.

    Methods:
        callback_imu(self, imu): Callback function for processing IMU sensor data.
        callback_pressure(self, pressure): Callback function for processing fluid pressure sensor data.
        pid(self): Perform a PID control calculation to determine the control signal 'u'.
        run(self): Start the main loop of the ROS node.
    
    Authors:
        [Samer A. Mohamed,
        Abdulrhamn Ragab, 
        Ahmed Adel, 
        Ahmed Amr, 
        Eman Mohammed, 
        Kirolos Thabet, 
        Marwan Hatem, 
        Omar Eltoutongy]

    Revised by:
        Hossam Moataz ElKeshky

    Creation Date:
        [4/11/2023]    

    Revision Date:
        [12/11/2023]
    """

    def __init__(self):
        """
        Initializes the ROS node (system identification node).

        Args:
            None
            
        Returns:
            None
        """
        # Initialize the ROS node
        rospy.init_node('sys_iden', anonymous=True)

        # Create publishers, and subscribers
        self.pub = rospy.Publisher("/Wrench", WrenchStamped, queue_size=1) 
        rospy.Subscriber("/imu_processed", Imu, self.callback_imu)
        rospy.Subscriber("/pressure_processed", FluidPressure, self.callback_pressure)

        # Initialize class members
        try:
            # Map the excitation state argument to an index
            state = rospy.get_param("/sys_iden/excitation_state") 
            if state in ['x', 'y', 'z', 'k', 'm', 'n']:
                self.excite_index = ['x', 'y', 'z', 'k', 'm', 'n'].index(\
                    state)
            else:
                 raise ValueError("user state input must be one of the following." \
                                  "characters: x, y, z, k, m, n")
            
            # Create the class PID members
            self.pid_gains = np.array(rospy.get_param("/C_Gains")) # Control gains
            if self.pid_gains.shape != (4, 3):
                raise ValueError("gain matrix must be of the size (4,3).")
            stab_idx = np.array([[1, 2],  # X --stabilizes-> M, N
                                [0, 2],  # Y --stabilizes-> K, N
                                [0, 1],  # Z --stabilizes-> K, M
                                [1, 3],  # K --stabilizes-> M, Z
                                [0, 3],  # M --stabilizes-> K, Z
                                [0, 1]])[self.excite_index, :]  # N --stabilizes-> K, M
            self.pid_mask = np.zeros((4,3)); self.pid_mask[stab_idx,:] = True # Masking matrix
            self.pid_time = rospy.get_time() # PID clock
            self.pid_feedback = np.zeros(4) # Sensory feedback
            self.pid_I = np.zeros(4) # Integral term

            # Initialize the excitation finite state machine (FSM)
            self.FSM_start_time = self.pid_time # Excitation start time
            self.FSM = [np.array([[-20.0], [15.0]]),
                np.array([[10.0], [15.0]]),
                np.array([[-25.0], [30.0]]),
                np.array([[0.0], [20.0]]),
                np.array([[3.0], [20.0]]),
                np.array([[3.0], [15.0]])][self.excite_index] # FSM table (row 1: Force/moment value, row 2: Transition time)
        except ValueError as e:
            # Handle an invalid user state input
            print(f"ERROR: {str(e)}")
            sys.exit(1)

        # Excitation start notification
        rospy.loginfo(f"Excitation along/about {state}-axis initiated.")

    def callback_imu(self, imu):
        """
        Callback function for processing IMU sensor data.

        Args:
            imu (Imu): The received IMU sensor data.

        This method is called when new IMU sensor data is received.
        It calculates the Euler angles from the received quaternion data and updates the sensor readings.
        It then calls a PID controller.

        Returns:
            None
        """
        # Process received messages in this callback function
        # Calculates Euler angles from Quaternion
        imu_euler = euler_from_quaternion([imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w])

        # Initialize control setpoints
        if not hasattr(self, 'pid_SP'):
            self.pid_SP = np.array([0.0, 0.0, imu_euler[2]]) # Use initial yaw angle as a yaw setpoint 
        elif self.pid_SP.shape[0] < 4:
            self.pid_SP = np.array([0.0, 0.0, imu_euler[2], self.pid_SP[0]])
        else:
            # Update sensor readings with IMU data
            feedback = np.array([imu_euler[0], imu_euler[1], imu_euler[2], self.pid_feedback[3]])

            # Call the pid function
            self.pid(feedback)

    def callback_pressure(self, pressure):
        """
        Callback function for processing fluid pressure sensor data.

        Args:
            pressure (FluidPressure): The received FluidPressure sensor data.

        This method is called when new fluid pressure sensor data is received. It calculates the depth from the fluid pressure
        measurement and updates the sensor readings. It then calls a PID controller.

        Returns:
            None
        """ 
        # Calculate the depth from the fluid pressure
        depth = (pressure.fluid_pressure-101325) / 9810.0
        
        # Initialize control setpoints
        if not hasattr(self, 'pid_SP'):
            self.pid_SP = np.array([depth]) # Use initial depth as a depth setpoint 
        elif self.pid_SP.shape[0] < 4:
            self.pid_SP = np.array([self.pid_SP[0], self.pid_SP[1], self.pid_SP[2], depth])
        else:
            # Update sensor readings with pressure sensor data
            feedback = np.array([self.pid_feedback[0], self.pid_feedback[1], self.pid_feedback[2], depth])

            # Call the pid function
            self.pid(feedback)

    def pid(self, sensor_feed):
        """
        Perform a PID (Proportional-Integral-Derivative) control calculation to determine the control signal 'u'.

        This method computes the control signal 'u' for the system using a PID control algorithm. It takes into account
        the gains, current sensor readings, previous sensor readings, integral errors, and the time difference.

        Args:
            sensor_feed (numpy.ndarray): fresh sensory feedback.

        Returns:
            None
        """
        # Initializing excitation wrench
        u = np.zeros(6)

        # Stabilization control law 
        dT = rospy.get_time() - self.pid_time; self.pid_time = dT + self.pid_time
        u[[3, 4, 5, 2]] = np.sum(self.pid_mask.T * self.pid_gains.T * \
                   np.reshape(np.concatenate((self.pid_SP - sensor_feed, (dT/2) * (self.pid_SP - sensor_feed), \
                   (2/dT) * (self.pid_feedback - sensor_feed))), (3,-1)), axis=0) + self.pid_I # Stabilization control action
        self.pid_feedback = sensor_feed # Update feedback record
        self.pid_I = np.clip(self.pid_I + self.pid_mask[:, 1] * self.pid_gains[:, 1] * (dT/2) * (self.pid_SP - sensor_feed), -5, 5) # Update integral term

        # Excitation
        force_idx = np.argmax(self.FSM[1,:] > self.pid_time - self.FSM_start_time) \
            if np.any(self.FSM[1,:] > self.pid_time - self.FSM_start_time) else -1
        if force_idx != -1:
            u[self.excite_index] = self.FSM[0,force_idx]
        else:
            u = np.zeros(6)
            rospy.loginfo("Excitation process is concluded.")

        # Publish a new wrench
        msg = WrenchStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "AUV_local" #---> hena ya keshky
        msg.wrench.force.x = u[0]; msg.wrench.force.y = u[1]; msg.wrench.force.z = u[2]
        msg.wrench.torque.x = u[3]; msg.wrench.torque.y = u[4]; msg.wrench.torque.z = u[5]
        self.pub.publish(msg)

    def run(self):
        """
        Start the main loop of the ROS node.

        Args:
            None

        Returns:
            None
        """
        # Spin
        rospy.spin()


if __name__ == '__main__':
    my_node = sys_iden_class()
    try:
        my_node.run()
    except rospy.ROSInterruptException:
        pass
