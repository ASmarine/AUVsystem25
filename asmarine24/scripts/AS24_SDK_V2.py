#!/usr/bin/env python3
"""
AS24 SDK Node

This Python script is an SDK for an Autonomous Underwater Vehicle (AUV). The script is designed to run as a ROS (Robot Operating System) node,
 and it performs various tasks related to the AUV's sensor data processing. It processes sensor data, receives transformations from desired yaml file,
 and actuates the AUV's thrusters. The node acts as a software-hardware abstraction layer.

The main functionalities of this script include:
1. Transformation of incoming IMU data into the desired AUV configuration.
2. Transformation of incoming pressure sensor data, accounting for AUV configuration and sensor position.
3. Assigning AUV thrust forces given the navigation's desired wrench vector.

Usage:
    rosrun [pkg_name] AS24_SDK.py [-h] [-f | --frames ]

Options:
    -h, --help                  Show this help message and exit.
    -f, --frames                Shows the frames received from yaml file.

Authors:
    [Abdulrhamn Ragab, 
    Ahmed Adel, 
    Ahmed Amr, 
    Eman Mohammed, 
    Kirolos Thabet, 
    Marwan Hatem, 
    Omar Eltoutongy]

Revised by:
    Samer A. Ahmed

Creation Date:
    [24/3/2024]

Revision Date:
    [**/3/2024]
"""

import sys
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu, FluidPressure
from geometry_msgs.msg import WrenchStamped
from tf.transformations import quaternion_matrix, translation_matrix, concatenate_matrices, \
    euler_from_quaternion, euler_matrix, euler_from_matrix, quaternion_from_euler

class MySDKNode:
    """
        The SDK node processes the raw sensory information and assings individual thrust values. The
        node acts as a high-level/low-level abstraction layer.

        Members:
            - my_tf (dictionary): Dictionary of vehicle homogeneous transformations.
            - imu_msg1 (Imu): Processed imu message object.
            - pressure_msg1 (FluidPressure): Processed pressure sensor message.
            - imu1_pub (publisher object): Processed imu message publisher.
            - ps1_pub (publisher object): Processed pressure sensor message publisher.
    """
    def __init__(self):
        """
        Class constructor.
         
        Initializes class members.

        Args:
            None

        Returns:
            None
        """
        # Node initialization
        rospy.init_node('sdk_node', anonymous=True)

        # Node publishers
        self.imu1_pub = rospy.Publisher('/imu_processed', Imu, queue_size=1)
        self.ps1_pub = rospy.Publisher('/pressure_processed', FluidPressure, queue_size=1)
        self.thrust_pub = rospy.Publisher('/thrust_forces', Float32MultiArray, queue_size=1)

        # Node subscribers/listeners
        #rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        rospy.Subscriber('/imu_raw', Imu, self.callback_IMU0)
        rospy.Subscriber('/pressure_raw', FluidPressure, self.callback_PSensor0)
        rospy.Subscriber('/Wrench', WrenchStamped, self.wrenchCallbck)

        # Transformations and mapping matrices used in computations
        self.my_tf = {} 
        self.M = None; self.Minv = None

        # Message objects
        self.imu_msg1 = Imu() 
        self.pressure_msg1 = FluidPressure()

        # Receive AUV transformations and mapping matrices
        Frames = np.array(rospy.get_param("/Frames"))
        tf = np.array(rospy.get_param("/tf"))
        for i in range(10):
            # Extract translation and rotation from received data
            translation = tf[i,0:3]
            rotation = tf[i,3:7]
            # Create homogeneous transformation matrices for translation and rotation
            translation_matrix_ = translation_matrix(translation)
            rotation_matrix = quaternion_matrix(rotation)
            # Concatenate the translation and rotation matrices to get the homogeneous transformation matrix
            self.my_tf[Frames[0,i]] = concatenate_matrices(translation_matrix_, rotation_matrix)
            rospy.loginfo(f"Received TF transform for {Frames[0,i]}")
            # Check for '-f' or '--frames' in sys.argv to print specific frames
            if '-f' in sys.argv or '--frames' in sys.argv:
                # Print the frame
                print("\033[1;34m" + Frames[0,i] + " homogeneous transformation:  " + \
                    f"{self.my_tf[Frames[0,i]]}" + "\033[0m")
        # Initialize thrust mapping matrices
        self.my_tf = dict(sorted(self.my_tf.items(), key=lambda item: item[0]))
        self.M = np.array([np.append(value[0:3,0:3], self.skew(value[0:3,3])@value[0:3,0:3], axis=0)[:,0] \
            for key, value in self.my_tf.items() if str(key).startswith('T')]).T
        self.Minv = np.linalg.pinv(self.M)
        rospy.loginfo(self.Minv)
    def skew(self, v):
        """
        Create a skew-symmetric matrix from a given 3D vector.

        This function takes a 3D vector as input and constructs a 3x3 skew-symmetric matrix that represents
        the cross product operation. The skew-symmetric matrix is used to perform vector cross products in 3D space.

        Args:
            v (numpy.ndarray or list): A 3D vector [x, y, z].

        Returns:
            numpy.ndarray: A 3x3 skew-symmetric matrix corresponding to the input vector.

        """
        return np.array([[0, -v[2], v[1]],
                        [v[2], 0, -v[0]],
                        [-v[1], v[0], 0]])
    
    def callback_IMU0(self, imu_msg0):
        """
        Transforms incoming IMU raw sensor message to adjust for AUV configuration.

        This callback function receives an IMU sensor message and processes it. It transforms the orientation from the IMU frame
        to the AUV hull frame. The adjusted IMU message is then published to the relevant topic.

        Args:
            imu_msg0 (Imu): The incoming IMU sensor message.

        Returns:
            None
        """
        if 'IMU' in self.my_tf:
            # Angular position transformation
            raw_euler = euler_from_quaternion([imu_msg0.orientation.x, imu_msg0.orientation.y, \
                                                   imu_msg0.orientation.z, imu_msg0.orientation.w]) 
            T_raw = euler_matrix(raw_euler[0], raw_euler[1], raw_euler[2]) # Raw rotation matrix
            self.my_tf['AUV_global_orient'] = self.my_tf['IMU'] @ T_raw @ self.my_tf['IMU'] # Processed transformation matrix
            processed_euler = euler_from_matrix(self.my_tf['AUV_global_orient'])
            processed_quat = quaternion_from_euler(processed_euler[0], processed_euler[1], processed_euler[2]) 
            self.imu_msg1.orientation.x = processed_quat[0]
            self.imu_msg1.orientation.y = processed_quat[1]
            self.imu_msg1.orientation.z = processed_quat[2]
            self.imu_msg1.orientation.w = processed_quat[3]
            self.imu_msg1.orientation_covariance = imu_msg0.orientation_covariance

            # Angular velocity transformation
            processed_ang_vel = self.my_tf['IMU'][0:3,0:3] @ np.array([imu_msg0.angular_velocity.x, imu_msg0.angular_velocity.y, \
                                                                        imu_msg0.angular_velocity.z])
            self.imu_msg1.angular_velocity.x = processed_ang_vel[0]        
            self.imu_msg1.angular_velocity.y = processed_ang_vel[1]
            self.imu_msg1.angular_velocity.z = processed_ang_vel[2]                                                
            self.imu_msg1.angular_velocity_covariance = imu_msg0.angular_velocity_covariance
            processed_lin_acc = self.my_tf['IMU'][0:3,0:3] @ np.array([imu_msg0.linear_acceleration.x, imu_msg0.linear_acceleration.y, \
                                                                           imu_msg0.linear_acceleration.z])
            self.imu_msg1.linear_acceleration.x = processed_lin_acc[0]
            self.imu_msg1.linear_acceleration.y = processed_lin_acc[1]
            self.imu_msg1.linear_acceleration.z = processed_lin_acc[2]
            self.imu_msg1.linear_acceleration_covariance = imu_msg0.linear_acceleration_covariance
            self.imu_msg1.header.stamp = rospy.Time.now()  # Set the timestamp
            
            # Publish processed IMU message
            self.imu1_pub.publish(self.imu_msg1)

    def callback_PSensor0(self, pressure_msg0):
        """
        Process incoming sensor message, adjusting for AUV configuration and sensor position.

        This callback function receives a pressure sensor message and processes it. It transforms the orientation from 
        the sensor frame to the AUV hull frame. The adjusted pressure message is then published to the relevant topic.

        Args:
            pressure_msg0 (FluidPressure): The incoming sensor message containing fluid pressure data.

        Returns:
            None
        """
        if 'AUV_global_orient' in self.my_tf and 'pressure sensor' in self.my_tf:
            # Add shift due to AUV rotation to raw reading
            shift = self.my_tf['AUV_global_orient'][0:3,0:3] @ self.my_tf['pressure sensor'][0:3,3]
            self.pressure_msg1.fluid_pressure = pressure_msg0.fluid_pressure + (9.8 * 997.0 * shift[2])
            self.pressure_msg1.variance = pressure_msg0.variance
            self.pressure_msg1.header.stamp = rospy.Time.now()  # Set the timestamp
            self.ps1_pub.publish(self.pressure_msg1)

    def wrenchCallbck(self, wrench_msg):
        """
        Process incoming wrench data and publish the corresponding thrust vector.

        This function takes incoming data, representing a wrench vector, converts it into a NumPy array, reshapes it
        into a 6x1 vector, and then calls the `thruster_vector_func` function to compute the thrust vector. The computed
        thrust vector is then prepared as a message and published to the 'thrust_pub' topic.

        Args:
            wrench_msg (WrenchStamped): The incoming data representing a stamped wrench vector.

        Returns:
            None
        """
        if self.Minv is not None:
            # Convert the incoming data to a NumPy array and reshape it to a vector
            Taw = np.array([wrench_msg.wrench.force.x, wrench_msg.wrench.force.y, wrench_msg.wrench.force.z, \
                            wrench_msg.wrench.torque.x, wrench_msg.wrench.torque.y, wrench_msg.wrench.torque.z])
            # Convert wrench to individual thrust forces
            F = self.Minv @ Taw

            # Prepare the message to be published containing the thrust vector
            thrust_msg = Float32MultiArray()
            thrust_msg.data = F

            # Publish the message to the thrust_pub topic
            self.thrust_pub.publish(thrust_msg)

    def run(self):
        rospy.loginfo('AS24 LOG: SDK node initiated..')
        rospy.spin()

if __name__ == '__main__':
    # Check for help in command-line arguments
    if '-h' in sys.argv or '--help' in sys.argv:
        print("""
        This script uses command-line arguments (CLi)

        Usage:
            rosrun sys_iden thrust_control_node.py [-h] [-f | --frames ]

        Options:
            -h, --help                  Show this help message and exit.
            -f, --frames                Shows the frames recieved from yaml file.
        """)
        sys.exit(0)

    # Initiate the ROS node
    try:
        my_node = MySDKNode()
        my_node.run()
    except rospy.ROSInterruptException:
        pass

