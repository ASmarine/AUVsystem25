#!/usr/bin/env python3
import sys
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu, FluidPressure
from tf.transformations import euler_from_quaternion


class sys_iden_class:
    """
    A class that represents a System Identification Node for AUV.

    This class initializes a ROS node for system identification.
    It subscribes to IMU (Inertial Measurement Unit) and fluid pressure sensor topics, 
    processes the sensor data, performs PID control calculations, 
    and publishes a wrench vector as a Float32MultiArray message to control an AUV.

    Attributes:
        excite_index (int): An index representing the system's excitation state, based on user input ('x', 'y', 'z', 'k', 'm', or 'n').
        Gains (numpy.ndarray): A numpy array of size (4,3) representing the PID control gains for the system.
        stable_1 (int): The index of the first stabilized state.
        stable_2 (int): The index of the second stabilized state.
        sensor_readings (numpy.ndarray): An array that stores the current sensor readings.
        sensor_readings_prev (numpy.ndarray): An array that stores the previous sensor readings.
        dT (float): The time difference between sensor readings.
        I (numpy.ndarray): An array that stores the integral errors for the PID controller.
        SP (numpy.ndarray): An array that stores the set points for the system.
        pid_flag (int): A flag to track PID control execution.
        t (float): The current timestamp.
        FSM (numpy.ndarray): A finite state machine representing the excitation states and transition triggers.

    Methods:
        timer_callbck(self, data): Callback function for handling timer events.
        callback_imu(self, imu): Callback function for processing IMU sensor data.
        callback_pressure(self, pressure): Callback function for processing fluid pressure sensor data.
        handle_callbacks(self): Handle callback events and trigger the PID control.
        wrench_calc(self, u): Calculate the wrench vector based on the control signal 'u' and the Finite State Machine (FSM).
        pid(self): Perform a PID control calculation to determine the control signal 'u'.
        run(self): Start the main loop of the ROS node.
    
    Authors:
        [Abdulrhamn Ragab, 
        Ahmed Adel, 
        Ahmed Amr, 
        Eman Mohammed, 
        Kirolos Thabet, 
        Marwan Hatem, 
        Omar Eltoutongy]

    Date:
        [4/11/2023]    
    """

    def __init__(self, state, gain_matrix):
        """
    Initialize the ROS node (system identification node).

    Args:
        state (str): The excitation state character ('x', 'y', 'z', 'k', 'm', or 'n').
        gain_matrix (np.ndarray): A numpy array of size (4,3) representing gain values.
        
    Returns:
        None
    """
        # Initialize the ROS node
        rospy.init_node('Sys_Idnt_Node', anonymous=True)
        # Log information about the state being excited
        rospy.loginfo(f"Exciting {state}")
        
        # Initialize ROS timers, publishers, and subscribers
        rospy.Timer(rospy.Duration(1), self.timer_callbck)
        self.pub = rospy.Publisher("/thruster_controller/wrench_vector", Float32MultiArray, queue_size=10)
        rospy.Subscriber("/imu1", Imu, self.callback_imu)
        rospy.Subscriber("/pressure1", FluidPressure, self.callback_pressure)

        # Test the validity of user inputs
        try:
            # Map the user state input to an index in the list
            self.excite_index = ['x', 'y', 'z', 'k', 'm', 'n'].index(state)
        except ValueError:
            # Handle an invalid user state input
            print(f"ERROR1: user state input must be one of the following " + \
                "characters: x, y, z, k, m, n")
            sys.exit(1)

        # Test the validity of the gain matrix
        if not (isinstance(gain_matrix, np.ndarray)) or gain_matrix.shape != (4, 3):
            # Handle an invalid gain matrix
            sys.exit(1)

        # Initialize time-related variables
        self.Time = rospy.get_time()
        self.test_time = 0

        # Member initialization
        # Determine the indices of stabilized states
        Table = np.array([[1, 2],  # X --> M, N
                        [0, 2],  # Y --> K, N
                        [0, 1],  # Z --> K, M
                        [1, 3],  # K --> M, Z
                        [0, 3],  # M --> K, Z
                        [0, 1]])  # N --> K, M
        self.st_idx = Table[self.excite_index, :]

        self.stable_1 = self.st_idx[0]
        self.stable_2 = self.st_idx[1]

        # Initialize the error vectors for sensor readings
        self.sensor_readings = np.array([0.0, 0.0, 0.0, 0.0])
        self.sensor_readings_prev = np.array([0.0, 0.0, 0.0, 0.0])
        self.dT = 0
        # Initialize the integral term
        self.I = np.array([0.0, 0.0, 0.0, 0.0])
        # Initialize the set points
        self.SP = np.array([0.0, 0.0, 0.0, 0.0])
        # Assign gains from the provided gain matrix
        self.Gains = gain_matrix
        # Initialize the PID flag
        self.pid_flag = 0
        self.t = 0
        # Initialize the excitation finite state machine (FSM)
        self.FSM = [
            np.array([[1, 0, -1], [10.0, 20.0, 30.0]]),
            np.array([[1], [10.0]]),
            np.array([[1, -1], [6.0, 12.0]]),
            np.array([[1], [20.0]]),
            np.array([[1], [20.0]]),
            np.array([[1], [5.0]])
        ][self.excite_index]  # FSM table (1st row: Thrust value, 2nd value: Transition trigger)

    

    def timer_callbck(self, data):
            """
            Callback function for handling timer events.

            Args:
                data: Data associated with the timer event (not used).

            This method is called periodically based on a timer event. It increments the `test_time` attribute by 1.

            Returns:
                None
            """
            # Increment the test_time by 1 with each timer event
            self.test_time += 1



    def callback_imu(self, imu):
            """
        Callback function for processing IMU sensor data.

        Args:
            imu (Imu): The received IMU sensor data.

        This method is called when new IMU sensor data is received.
        It calculates the Euler angles from the received quaternion data and updates the sensor readings.
        It then calls a common function to handle callbacks, which may trigger the PID controller.

        Returns:
            None
        """
            # Update the timestamp
            self.t = rospy.get_time()

            # Process received messages in this callback function
            # Calculates Euler angles from Quaternion
            imu_euler = euler_from_quaternion([imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w])
            
            # Log the received data
            rospy.loginfo("Received: %s", imu_euler)
            
            # Update sensor readings with IMU data
            self.sensor_readings[0] = imu_euler[0]
            self.sensor_readings[1] = imu_euler[1]
            self.sensor_readings[2] = imu_euler[2]

            # Call the common function to handle callbacks
            self.handle_callbacks()

                

    def callback_pressure(self, pressure):
            """
        Callback function for processing fluid pressure sensor data.

        Args:
            pressure (FluidPressure): The received FluidPressure sensor data.

        This method is called when new fluid pressure sensor data is received. It calculates the depth from the fluid pressure
        measurement and updates the sensor readings. It then calls a common function to handle callbacks, which may trigger the PID controller.

        Returns:
            None
        """
            # Process received messages in this callback function
            self.t = rospy.get_time()
            
            # Calculate the depth from the fluid pressure
            depth = (pressure.fluid_pressure-101325) / 9810.0
            
            # Update the sensor readings for depth
            self.sensor_readings[3] = depth
            
            # Call the common function to handle callbacks
            self.handle_callbacks()
            
            rospy.loginfo("Received: %s", depth)

    
    def handle_callbacks(self):
            """
        Handle callback events and trigger the PID control.

        This method manages callback events from sensor data and ensures that the PID control is executed every two callback calls.
        It increments a flag to track the number of callback events and triggers the PID controller when the flag reaches 2.

        Returns:
            None
        """    
            # Increment the PID flag    
            self.pid_flag += 1
            # If the PID flag reaches 2, reset it and call the PID function
            if self.pid_flag == 2:
                self.pid_flag = 0
                self.pid()


    def wrench_calc (self, u):
            """
        Calculate the wrench vector based on the control signal 'u' and the Finite State Machine (FSM).

        This method computes the wrench vector based on the control signal 'u', taking into account the current
        system state, the FSM, and time-related information. The wrench vector is published as a Float32MultiArray message.

        Args:
            u (numpy.ndarray): The control signal calculated by the PID controller.

        Returns:
            None
        """
            # Log test time information
            rospy.logwarn(f"Actual: {self.test_time}")

            # Initialize the weight matrix
            weight = np.zeros((6,2))

            # Determine weight values based on stabilized states
            if self.stable_1 !=3 and self.stable_2 !=3:  # not z
                weight[3 + self.stable_1][0]= 1
                weight[3 + self.stable_2][1]= 1
            else:
                weight[3 + self.stable_1][0]= 1
                weight[self.stable_2 - 1][1]= 1
            
            # Calculate the wrench
            Wrench_msg =  weight @ u.reshape((2,1))

            # Determine the index using the Finite State Machine (FSM)
            index = 5 #initialize the index with any value other than (0,1,2,3)
            for i in range (len(self.FSM[0])):
                if index == 5:
                    if(self.test_time <= self.FSM[1][i]):
                        index = i

            # Update the wrench message based on the FSM
            if index == 5:
                rospy.loginfo("Shutting Down my_node...")
                rospy.signal_shutdown('Experiment finished')
            else:
                Wrench_msg[self.excite_index]= self.FSM[0][index] * 5.0 # action (force,moment) gain equals 5

            # Create and publish the vector message    
            vector_msg = Float32MultiArray()
            vector_msg.data =  Wrench_msg
            self.pub.publish(vector_msg)

            rospy.loginfo(Wrench_msg)




    def pid(self):
            """
        Perform a PID (Proportional-Integral-Derivative) control calculation to determine the control signal 'u'.

        This method computes the control signal 'u' for the system using a PID control algorithm. It takes into account
        the gains, current sensor readings, previous sensor readings, integral errors, and the time difference.
        Additionally, it calls the 'wrench_calc' method to calculate and publish the wrench vector based on the control signal.

        Args:
            None

        Returns:
            None
        """
            # Extract PID coefficients and sensor readings
            coff_matrix = np.array([self.Gains[self.stable_1], self.Gains[self.stable_2]])
            readings = np.array([self.sensor_readings[self.stable_1],self.sensor_readings[self.stable_2]])
            readings_prev = np.array([self.sensor_readings_prev[self.stable_1],self.sensor_readings_prev[self.stable_2]])
            ierr = np.array([self.I[self.stable_1],self.I[self.stable_2]])
            
            # Calculate the time step
            self.dT = self.t - self.Time
            dT = self.dT

            # Calculate the control signal 'u' using the PID algorithm
            u = np.sum(coff_matrix.T * np.reshape(np.concatenate((readings, (dT / 2) * readings, (2 / dT) * (readings - readings_prev))),(3, -1)), axis=0) + ierr

            # Update recursive terms (past error, integral term, and time stamp)
            self.sensor_readings_prev = self.sensor_readings
            ierr = np.clip(ierr + coff_matrix[:, 1] * (dT / 2) * readings_prev, -20, 20)
            self.Time = self.t

            # Call the wrench calculation
            self.wrench_calc(u)

    def run(self):
        """
        Start the main loop of the ROS node.

        This method sets the operating rate of the node, enters a loop, and keeps the node running as long as
        the ROS system is not shutting down.
        It allows ROS to process callbacks and timers.

        Args:
            None

        Returns:
            None
        """
        # Set the rate at which the node will run
        self.rosRate = 1
        rate = rospy.Rate(self.rosRate)  # 1 Hz (1 message per second)

        # Keep running the node as long as ROS is not shutting down
        while not rospy.is_shutdown():
            # Allow ROS to process callbacks and timers
            rospy.spin()


if __name__ == '__main__':
    gain_matrix = np.array(rospy.get_param("/C_Gains")) #Control gains
    # Read excitation state from user input or ROS parameter, e.g., rospy.get_param("/excitation_state")
    excitation_state = rospy.get_param("/sys_idn_node/excitation_state")  # get excitation state
    my_node = sys_iden_class(excitation_state, gain_matrix)
    try:
        my_node.run()
    except rospy.ROSInterruptException:
        pass
