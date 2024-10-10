#!/usr/bin/env python3


"""
AUV Thrust Control Node

This Python script is part of a control system for an Autonomous Underwater Vehicle (AUV). The script is designed to run as a ROS (Robot Operating System) node, and it performs various tasks related to the AUV's control and sensor data processing. It processes sensor data, calculates transformations, and controls the AUV's thrusters.

The main functionalities of this script include:
1. Transformation of incoming IMU data into the desired AUV configuration.
2. Transformation of incoming pressure sensor data, accounting for AUV configuration and sensor position.
3. Processing and reshaping of incoming position data for further use.
4. Calculation of thruster forces based on a given wrench vector.
5. Combining transformation matrices to compute the combined transformation matrix.
6. Collecting transformation frames from Arduino and organizing them.
7. Initialization of ROS topics, publishers, and subscribers for data exchange.

This script is intended to be used as part of an AUV control system and should be executed within a ROS environment.

Usage:
    rosrun [pkg_name] thrust_controller_node.py [-h] [-b | --blocked | -f | --frames | -r | --resultant_forces ]

Options:
    -h, --help                  Show this help message and exit.
    -b, --blocked               Show the transformation blocked frames.
    -f, --frames                Shows the frames received from Arduino.
    -r, --resultant_forces      Shows the forces transferred to the thrusters.

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

Note:
    This script relies on various ROS topics and assumes that it is running in a ROS environment. 
"""


import sys
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu, FluidPressure
import tf
from tf.transformations import euler_from_quaternion
from tf.transformations import euler_matrix
from tf.transformations import euler_from_matrix
from tf.transformations import quaternion_from_euler

# Check for help in command-line arguments
if '-h' in sys.argv or '--help' in sys.argv:
    print("""
    This script uses command-line arguments (CLi)

    Usage:
        rosrun sys_iden thrust_control_node.py [-h] [-b | --blocked | -f | --frames | -r | --resultant_forces | g | --graph]

    Options:
        -h, --help                  Show this help message and exit.
        -b, --blocked               Show the transformation blocked frames.
        -f, --frames                Shows the frames recieved from arduino.
        -r, --resultant_forces      Shows the forces transfered to the thrusters
        -g, --graph                 Show the graphs of force over the 8 thrusters
    """)
    sys.exit(0)

# rotation matrix (AUV to IMU and vice versa)
# R_AUV_IMU = [[0.0, 0.0, 1.0],
#              [0.0, -1.0, 0.0],
#              [1.0, 0.0, 0.0]]

R_AUV_IMU = 0

# Transformation matrix (Psensor to AUV)
# T_PS_to_AUV = [[1.0, 0.0, 0.0, -0.25],
#             [0.0, 1.0, 0.0, 0.0],
#             [0.0, 0.0, 1.0, 0.0],
#             [0.0, 0.0, 0.0, 1.0]]

T_PS_to_AUV = 0

# Initialize Transformation matrix for AUV to world
R_AUV_to_W = np.zeros(16)
R_AUV_to_W = R_AUV_to_W.reshape((4,4))

# Initialize Transformation matrix for AUV thrusters and sensor to the center of hull --(local frame)--
global thruster_tf,  pressure_AUVlocal_local,  imu_AUVlocal_local

# Initialize new sensory ROS messages 
imu_msg1 = Imu()
pressure_msg1 = FluidPressure()
T_orn = 0
thrust_vector = 0
T_orn_list = []  # List to store individual Thrusters_Orientation arrays
M_combined = 0   #edit

frame_IDs= ["T1", "T2", "T3", "T4", "T5", "T6", "T7", "T8","pressure_raw","imu_raw"]


def callback_IMU0(imu_msg0):
    """
    Process incoming IMU sensor message and adjust for AUV configuration.

    This callback function receives an IMU sensor message and processes it. It transforms the orientation from the IMU frame
    to the AUV hull frame. The adjusted IMU message is then published to the relevant topic.

    Args:
        imu_msg0 (Imu): The incoming IMU sensor message.

    Returns:
        None

    Global Variables:
        - R_AUV_IMU (numpy.ndarray): A global transformation matrix for AUV to IMU frame configuration.
        - R_AUV_to_W (numpy.ndarray): A global transformation matrix for AUV to world configuration.
        - imu_msg1 (Imu): A global variable to store the adjusted IMU message.

    Note:
        - This function uses global transformation matrices for configuration adjustments.
        - It performs the necessary transformations to adjust the IMU data to the AUV frame.
    """

    Reading_euler = euler_from_quaternion([imu_msg0.orientation.x, imu_msg0.orientation.y, imu_msg0.orientation.z, imu_msg0.orientation.w])
    Reading_R = euler_matrix(Reading_euler[0], Reading_euler[1], Reading_euler[2])
    out_matrix = np.dot(R_AUV_IMU, np.dot(Reading_R[0:3,0:3], R_AUV_IMU)) # AUV orientation in Rmatrix
    R_AUV_to_W[0:3,0:3] = out_matrix # setup transformation matrix for processing input pressure reading 
    out_euler = euler_from_matrix(out_matrix) # AUV orientation in euler
    out_quat = quaternion_from_euler(out_euler[0], out_euler[1], out_euler[2]) # AUV orientation in quat
    imu_msg1.orientation.x = out_quat[0]
    imu_msg1.orientation.y = out_quat[1]
    imu_msg1.orientation.z = out_quat[2]
    imu_msg1.orientation.w = out_quat[3]
    imu_msg1.orientation_covariance = imu_msg0.orientation_covariance
    # The following equations are the result of the following:
    # R_AUV_IMU * angular velocity vector
    # R_AUV_IMU * linear acceleration vector
    imu_msg1.angular_velocity.x = imu_msg0.angular_velocity.z
    imu_msg1.angular_velocity.y = -1.0 * imu_msg0.angular_velocity.y
    imu_msg1.angular_velocity.z = imu_msg0.angular_velocity.x
    imu_msg1.angular_velocity_covariance = imu_msg0.angular_velocity_covariance
    imu_msg1.linear_acceleration.x = imu_msg0.linear_acceleration.z
    imu_msg1.linear_acceleration.y = -1.0 * imu_msg0.linear_acceleration.y
    imu_msg1.linear_acceleration.z = imu_msg0.linear_acceleration.x
    imu_msg1.linear_acceleration_covariance = imu_msg0.linear_acceleration_covariance
    imu_msg1.header.stamp = rospy.Time.now()  # Set the timestamp
    IMU1_pub.publish(imu_msg1)

def callback_PSensor0(pressure_msg0):
    """
    Process incoming sensor message, adjusting for AUV configuration and sensor position.

    This callback function receives a sensor message and processes it to account for the shift in pressure
    caused by the position of the pressure sensor relative to the center of hull.
    It calculates the shift and adds it to the received fluid pressure value. The adjusted fluid pressure value 
    is then published to the relevant topic.

    Args:
        pressure_msg0 (FluidPressure): The incoming sensor message containing fluid pressure data.

    Returns:
        None

    Global Variables:
        - R_AUV_to_W (numpy.ndarray): A global transformation matrix for AUV to world configuration.
        - T_PS_to_AUV (numpy.ndarray): A global transformation matrix for the position of the pressure sensor
          relative to the AUV center of mass.
        - pressure_msg1 (FluidPressure): A global variable to store the adjusted pressure sensor message.

    Note:
        - This function uses global variables for transformation matrices and the adjusted pressure sensor message.
        - It performs the necessary adjustments to the fluid pressure based on the AUV's configuration and sensor position.
    """

    #add shift due to AUV rotation
    shift = np.dot(R_AUV_to_W, T_PS_to_AUV)
    pressure_msg1.fluid_pressure = pressure_msg0.fluid_pressure + (9.8 * 1000.0 * shift[2,3])
    pressure_msg1.variance = pressure_msg0.variance
    pressure_msg1.header.stamp = rospy.Time.now()  # Set the timestamp
    Psensor1_pub.publish(pressure_msg1)



def posCallback(data):
    """
    Process position data, reshape it, and create global variables for further use.

    This function is responsible for processing incoming position data. It reshapes the data into a 2D array based on
    the specified number of columns (n_cols) and then creates a list of lists (T_orn_list), where each element represents
    a row from the reshaped data. The function also dynamically creates global variables for each row, starting with 'T1_orn'
    and incrementing the variable names for each row. Optionally, it can print the contents of individual frames and
    the IMU and pressure frames based on command-line arguments.

    Args:
        data (numpy.ndarray): The incoming position data as a NumPy array.

    Returns:
        None

    Global Variables:
        - Thrusters_Orientation: A global variable that stores the reshaped position data.
        - T_orn: A list containing the reshaped position data.
        - T_orn_list: A list of lists, where each element represents a row from the reshaped data.
        - imu_AUVlocal_local: A global variable representing the IMU frame data.
        - pressure_AUVlocal_local: A global variable representing the pressure frame data.
        
    Note:
        - The number of columns (n_cols) is used to reshape the data into a 2D array.
        - The function dynamically creates global variables for each row of position data.
    """
    # Declare global variables for later use
    global Thrusters_Orientation, T_orn, T_orn_list
    n_cols = 6 
    n_rows = len(data) // n_cols         # Calculate the number of rows based on the length of data
    
    # Reshape the data into a 2D array with dimensions (n_rows, n_cols)
    Thrusters_Orientation = data.reshape(n_rows, n_cols)
    T_orn = Thrusters_Orientation.tolist()

    # Create a list of lists T_orn_list, with each element representing a row from Thrusters_Orientation
    T_orn_list = [Thrusters_Orientation[i, :] for i in range(Thrusters_Orientation.shape[0])]

    # Access individual T_orn arrays using T_orn_list and dynamically create global variables
    for i, T_orn_array in enumerate(T_orn_list):
        globals()[f'T{1+i}_orn'] = T_orn_array

    # Check for '-f' or '--frames' in sys.argv to print specific frames
    if '-f' in sys.argv or '--frames' in sys.argv:
        # Print the contents of individual frames using dynamically created global variables
        for i in range(1, 9):
            var_name = f'T{i}_orn'  # Construct the variable name
            if var_name in globals():
                print("\033[1;32m" + f"Thruster #{i} frame: {globals()[var_name]}"+ "\033[0m")

        # Print the IMU frame and pressure frame
        print("\033[1;34m" + "the IMU frame:  " + f"{imu_AUVlocal_local}" + "\033[0m")
        print("\033[1;34m" + "the pressure frame:  " + f"{pressure_AUVlocal_local}" + "\033[0m")

    

def wrenchCallbck(data):
    """
    Process incoming wrench data and publish the corresponding thrust vector.

    This function takes incoming data, representing a wrench vector, converts it into a NumPy array, reshapes it
    into a 6x1 vector, and then calls the `thruster_vector_func` function to compute the thrust vector. The computed
    thrust vector is then prepared as a message and published to the 'thrust_pub' topic.

    Args:
        data (Float32MultiArray): The incoming data representing a wrench vector.

    Returns:
        None

    Global Variables:
        - thrust_vector: A global variable that stores the computed thrust vector.
        - thrust_pub: A Publisher object for publishing thrust vector messages.

    Note:
        - The `thruster_vector_func` function is used to compute the thrust vector from the input wrench data.
    """
    # Convert the incoming data to a NumPy array and reshape it to a 6x1 vector
    wrench_vector = np.array(data.data).reshape(6, 1)

    # Call the thruster_vector_func with the wrench_vector as input
    thruster_vector_func(wrench_vector)

    # Prepare the message to be published containing the thrust vector
    msg = Float32MultiArray()
    msg.data = thrust_vector

    # Publish the message to the thrust_pub topic
    thrust_pub.publish(msg)


def trans_matrix():
    """
    Compute the combined transformation matrix (M_combined) from orientation arrays.

    This function calculates the combined transformation matrix (M_combined) by processing orientation arrays (T_orn_array)
    for the thrusters and using them to create individual transformation matrices (T_orn). These individual transformation
    matrices are then used to compute M matrices for each thruster. The M matrices are stacked to create M_combined,
    which represents the transformation from thruster forces to the AUV body frame.

    Args:
        None

    Returns:
        None

    Global Variables:
        - M_combined: A global variable that stores the combined transformation matrix (8x6).

    Note:
        - The function relies on the orientation arrays (T_orn_array) for each thruster.
        - The pseudoinverse of M_combined is used to compute thruster forces from a wrench vector.
    """
    # Declare M_combined as a global variable
    global M_combined
    M_matrices = []

    # Iterate through each T_orn_array and compute the corresponding transformation matrices
    for i, T_orn_array in enumerate(T_orn_list):
        # Create a transformation matrix T_orn from the orientation array
        T_orn = np.block([[eul2rotm(np.flip(T_orn_array[3:6])), np.reshape(T_orn_array[0:3], (3, 1))], [0, 0, 0, 1]])
        globals()[f'T{1+i}_orn'] = T_orn

        # Compute the M matrix from the T_orn matrix
        M = np.vstack((T_orn[0:3, 0:3], skew(T_orn[0:3, 3]).dot(T_orn[0:3, 0:3])))
        globals()[f'M{1+i}'] = M
        M_matrices.append(M)  # Store each M matrix in the list

    # Stack the first column of each M matrix to form the combined M matrix
    M_combined = np.column_stack([M[:, 0] for M in M_matrices])

    # Print the combined M matrix
    #print(M_combined)



def thruster_vector_func(taw):
    """
    Compute the thruster forces from a wrench vector using pseudoinverse.

    This function computes the thruster forces based on a given wrench vector using the pseudoinverse of the combined
    transformation matrix (M_combined). The wrench vector represents external forces and torques acting on the AUV,
    and the function calculates the corresponding thruster forces that need to be applied.

    Args:
        taw (numpy.ndarray): A 6x1 vector representing the external forces and torques acting on the AUV.
            The order of elements in the vector is [Fx, Fy, Fz, Tx, Ty, Tz].

    Returns:
        None

    Global Variables:
        - thrust_vector: A global variable that stores the computed thruster forces (8x1 vector).

    Example:
        Given a wrench vector taw = [10, 0, -5, 0, 0, 0], where Fx = 10 N, Fy = 0 N, Fz = -5 N,
        and all torques are 0, the function calculates the thruster forces required for the AUV to maintain equilibrium.

    Note:
        - The function uses the pseudoinverse of the combined transformation matrix (M_combined) to compute thruster forces.

    """

    # Uncomment the line below if you want to send a row vector
    # taw = taw.T

    # Declare thrust_vector as a global variable
    global thrust_vector

    # Compute the thruster vector using the pseudoinverse of M_combined
    # The shape of M_combined is 8x6 and the shape of taw is 6x1
    # The shape of thrust_vector will be 8x1
    thrust_vector = np.linalg.pinv(M_combined) @ taw

    # Print the computed thrust vector
    print("\033[1;32m" + "=="*25 + "\033[0m")
    print("\033[1;32m" + f"||       Thrust #      ||      thrust value" + "\033[0m")
    print("\033[1;32m" + "=="*25 + "\033[0m")

    for i, thrust in enumerate(thrust_vector):
        print(f"||      Thrust {i+1}      ||      {thrust[0]}")

def eul2rotm(rpy):
    """
    Convert Euler angles (roll, pitch, yaw) to a 3x3 rotation matrix.

    Args:
        rpy (list or numpy.ndarray): A list or numpy array containing the Euler angles [roll, pitch, yaw] in radians.

    Returns:
        numpy.ndarray: A 3x3 rotation matrix representing the rotation described by the given Euler angles.

    """

    psi, theta, phi = rpy
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(phi), -math.sin(phi)],
                    [0, math.sin(phi), math.cos(phi)]])
    
    R_y = np.array([[math.cos(theta), 0, math.sin(theta)],
                    [0, 1, 0],
                    [-math.sin(theta), 0, math.cos(theta)]])
    
    R_z = np.array([[math.cos(psi), -math.sin(psi), 0],
                    [math.sin(psi), math.cos(psi), 0],
                    [0, 0, 1]])
    
    R = np.dot(R_z, np.dot(R_y, R_x))
    return R


def skew(v):
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

def topics_init():
    """
    Initialize ROS topics, publishers, and subscribers for the Thruster Node.

    It logs the initiation of the Thruster Node,sets up publishers for forces control,
    subscribes to sensory messages from IMU and pressure sensors, and subscribes
    to the wrench vector topic from the SLAM node for controlling thrusters.

    Args:
        None

    Returns:
        None

    Global Variables:
        - thrust_pub: A ROS Publisher for publishing forces to control thrusters.
        - IMU1_pub: A ROS Publisher for IMU data in the AUV configuration.
        - Psensor1_pub: A ROS Publisher for pressure sensor data in the AUV configuration.
        - sub_IMU0: A ROS Subscriber for receiving IMU data.
        - sub_Psensor0: A ROS Subscriber for receiving pressure sensor data.
        - wrench_vector_sub: A ROS Subscriber for receiving the wrench vector from the SLAM node.

    Note:
        - This function is a part of the Thruster Node initialization process and sets up the necessary communication channels
          for the control and transmission of data between ROS nodes.
    """

    # Log the initiation of the Thruster Node
    rospy.loginfo('Thruster Node has started')

    # Declare global variables for the publishers
    global thrust_pub, IMU1_pub, Psensor1_pub

    # Initialize a Publisher to publish forces to the Arduino to control thrusters
    thrust_pub = rospy.Publisher('/thrust_forces', Float32MultiArray, queue_size=10)

    # Initialize Publishers and Subscribers for sensory messages
    sub_IMU0 = rospy.Subscriber("/imu_raw", Imu, callback_IMU0)
    sub_Psensor0 = rospy.Subscriber("/pressure_raw", FluidPressure, callback_PSensor0)
    IMU1_pub = rospy.Publisher("/imu1", Imu, queue_size=1)
    Psensor1_pub = rospy.Publisher("/pressure1", FluidPressure, queue_size=1)

    # Initialize a Subscriber for the Wrench Vector from the SLAM node
    wrench_vector_sub = rospy.Subscriber("/thruster_controller/wrench_vector", Float32MultiArray, wrenchCallbck)



def euler_to_rotation_matrix(roll, pitch, yaw):
    """
    Calculate a 3x3 rotation matrix from Euler angles (roll, pitch, yaw).

    Args:
        roll (float): Rotation angle around the X-axis (in radians).
        pitch (float): Rotation angle around the Y-axis (in radians).
        yaw (float): Rotation angle around the Z-axis (in radians).

    Returns:
        numpy.ndarray: 3x3 rotation matrix.
    """
    # Create the individual rotation matrices
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(roll), -math.sin(roll)],
                    [0, math.sin(roll), math.cos(roll)]])

    R_y = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                    [0, 1, 0],
                    [-math.sin(pitch), 0, math.cos(pitch)]])

    R_z = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                    [math.sin(yaw), math.cos(yaw), 0],
                    [0, 0, 1]])

    # Combine the individual rotation matrices in ZYX order
    rotation_matrix = np.dot(R_z, np.dot(R_y, R_x))
    #rospy.loginfo(f"rotation_matrix of imu\n {rotation_matrix}")

    return rotation_matrix

def transform_from_pose(pose):
    """
    Create a 4x4 transformation matrix from position (x, y, z) and Euler angles (roll, pitch, yaw).

    Args:
        pose (list or numpy.ndarray): A 6-element array [x, y, z, roll, pitch, yaw] in meters and radians.

    Returns:
        numpy.ndarray: 4x4 transformation matrix.
    """
    if len(pose) != 6:
        raise ValueError("The 'pose' array must contain 6 elements: [x, y, z, roll, pitch, yaw]")

    x, y, z, roll, pitch, yaw = pose

    # Create a 3x1 translation matrix
    translation_matrix = np.array([[x], [y], [z]])

    # Create a 3x3 rotation matrix from Euler angles
    rotation_matrix = euler_to_rotation_matrix(roll, pitch, yaw)

    # Combine translation and rotation into a 4x4 transformation matrix
    transformation_matrix = np.eye(4)
    transformation_matrix[0:3, 0:3] = rotation_matrix
    transformation_matrix[0:3, 3] = translation_matrix.flatten()

    #rospy.loginfo(f"transformation_matrix of pressure sensor\n {transformation_matrix}")

    return transformation_matrix


def frames_transmission_function():
    """
    This function collects transformation frames from Arduino and organizes them for further processing.

    This function retrieves transformation frames (e.g., thruster positions and orientations) from Arduino
    by listening for TF (Transform) data and stores them in a dictionary. It collects frames in a specific order,
    removing each frame from the list as it's received to avoid duplicates. The collected frames are organized in
    ascending order, removing the frames for the IMU and pressure sensor and extracting their data separately.
    The resulting data includes transformation matrices for the AUV to IMU rotation and PressureSensor to AUV
    transformation.

    Args:
        None

    Returns:
        None

    Global Variables:
        - thruster_tf: A numpy array containing the transformation data for thrusters.
        - pressure_AUVlocal_local: A list containing the position and orientation data for the Pressure Sensor frame.
        - imu_AUVlocal_local: A list containing the position and orientation data for the IMU frame.
        - frame_IDs: A list of frame names to be received from Arduino.
        - tf_listener: A TF listener to listen for transformation data.

    Note:
        - This function depends on global variables and uses a while loop to collect frames in the specified order.
        - The collected data is organized into transformation matrices and stored in global variables for further use.
    """
    global thruster_tf,  pressure_AUVlocal_local,  imu_AUVlocal_local, R_AUV_IMU, T_PS_to_AUV
    recieved_frames = {}
    # As we have a list of frames required to be transmitted we will continue in this loop 
    # knowing that we delete every frame name we get from the list till we get all of them which make this list empty
    while frame_IDs:
        # Log a warning message indicating the waiting state for Arduino
        if '-b' in sys.argv or '--blocked' in sys.argv:
            rospy.logwarn("WAITING FOR ARDUINO")
        # In this for loop we will get all the frames from arduino (10 frames: 8 thrusters, IMU and Pressure in this case)
        for thruster_name in frame_IDs:
            try:
                # Lookup the transformation data from 'AUV_local' frame to the current frame
                (trans, rot) = tf_listener.lookupTransform("AUV_local", thruster_name, rospy.Time(0))
                # Convert quaternion to Euler angles for rotation
                angle = euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])
                # Acknowledge the user with the recieved frames
                rospy.loginfo(f"Received TF transform for {thruster_name}:")
                
                # Create a key for the thruster to store the data in the dictionary
                thruster_key = thruster_name
                recieved_frames[thruster_key] = [trans[0], trans[1], trans[2], angle[0], angle[1], angle[2]]
                # Remove the recieved frame from the list to prevent duplicates
                frame_IDs.remove(thruster_name)
            
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                # Acknowledge the user with the recieved frames
                if '-b' in sys.argv or '--blocked' in sys.argv:
                    rospy.logwarn(f"TF transform lookup failed for {thruster_name}: {e}")
                else: pass
        
    # After collecting all thruster data
    print("\033[1;32m" + "RECIEVED" + "\033[0m")

    # Organize the data in ascending order T1, T2, ....... T8, IMU sensor, Pressure sensor
    sorted_recieved_frames = dict(sorted(recieved_frames.items()))
    # Extract the Euler frames of imu sensor and pressure senor recieved from arduino
    pressure_AUVlocal_local = sorted_recieved_frames["pressure_raw"]
    imu_AUVlocal_local = sorted_recieved_frames["imu_raw"]

    R_AUV_IMU = euler_to_rotation_matrix(imu_AUVlocal_local[3],imu_AUVlocal_local[4],imu_AUVlocal_local[5])
    T_PS_to_AUV = transform_from_pose(pressure_AUVlocal_local)

    rospy.loginfo(f"R_AUV_IMU:\n{R_AUV_IMU}")

    rospy.loginfo(f"T_PS_to_AUV:\n{T_PS_to_AUV}")

    #removing the frames of IMU and pressure 
    sorted_recieved_frames.pop("pressure_raw")
    sorted_recieved_frames.pop("imu_raw")

    # Intialize a list to store all frame values
    thruster_tf_list=[]
    for item in sorted_recieved_frames:
        thruster_tf_list.append(sorted_recieved_frames[item])    
    # Make it a matrix to be send
    thruster_tf = np.array(thruster_tf_list).reshape(-1)

if __name__ == '__main__':
    
    rospy.init_node('Thruster_Controller_Node')
    
    # Here, we create a tf.TransformListener object. Once the listener is created,
    # it starts receiving tf transformations over the wire, and buffers them for up to 10 seconds.
    tf_listener = tf.TransformListener()
    
    rate = rospy.Rate(1)
    
    # Here are some functions calls to setup the environment 
    frames_transmission_function()
    posCallback(thruster_tf)
    trans_matrix()     #edit
    topics_init()

    while not rospy.is_shutdown():
        rospy.spin()        

