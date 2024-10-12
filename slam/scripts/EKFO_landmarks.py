#!/usr/bin/env python3
"""
EKFSLAM main node

Developer/s:
Samer A. Mohamed, Hossam M. Elkeshky.
"""
# Import relevant libraries
import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from sensor_msgs.msg import FluidPressure
from tf.transformations import euler_from_quaternion
from slam.EKF_class import EKF
# Perception message
#from pub_sub.msg import Multi_instance

# Define AUV's inertial parameters
# Mass
AUV_mass = np.array(rospy.get_param("/SLAM_node/AUV_mass"))
AUV_added_mass = np.array(rospy.get_param("/SLAM_node/AUV_added_mass"))
# Inertial matrix including added mass by system identification
I_matrix = np.array(rospy.get_param("/SLAM_node/I_matrix"))
# Define AUV hydrostatics
Tau_g = np.array(rospy.get_param("/SLAM_node/Tau_g"))
Tau_b = np.array(rospy.get_param("/SLAM_node/Tau_b"))
# Define AUV hydrodynamic parameters by system identification
Drag_co = np.array(rospy.get_param("/SLAM_node/Drag_co"))
Lift_co = np.array(rospy.get_param("/SLAM_node/Lift_co"))
# Define motion model noise matrix
R = np.array(rospy.get_param("/SLAM_node/R"))
# Define output matrix "C"
C = np.array(rospy.get_param("/SLAM_node/C"))

# Define tracking loss flag
Tracking_lost = 0
# Define reference (with its belief) for ORB_SLAM3 pose message 
x_ref = 0.0
y_ref = 0.0
x_ref_sigma = 0.0
y_ref_sigma = 0.0
# set ORB_SLAM3 variance
ORB_sigma = rospy.get_param("/SLAM_node/ORB_sigma")
# Define array which holds coordinates of received landmarks
landmarks_coord = np.array([])
# Define dictionary which saves coordinates of landmarks witnessed during run
landmarks_dict = {}
# Define array of variances corresponding to received readings 
variance = np.array([])
# Define error matrix for landmark readings
Q_landmarks = np.array([])
# Note: Z variance (in perception message) was generalized for corresponding X & Y  

# Define a 3x3 rotation matrix from camera frame to AUV frame
Cam_to_AUV_R_M = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])

def calc_variance(updated):
    """
    calc_variance function: Assigns variance values to the
    received landmarks data according to the collected 
    perception data.

    Args:
    - updated(list): landmarks detected in the current message.

    Returns:
    N/A
    
    Developer/s:
    Samer A. Mohamed, Hossam M. Elkeshky, Kirollos Thabet.
    """
    global variance
    variance = np.zeros(landmarks_coord.size)
    v = 0
    for i, landmark_id in enumerate(landmarks_dict.keys()):
        x_index = (i*3)   

        if landmark_id in updated:
            x_value = landmarks_coord[x_index]
        
            if 1 <= x_value < 2:
                v = 1.09e-3
            elif 2 <= x_value < 3:
                v = 0.035
            elif 3 <= x_value < 4:
                v = 0.055
            elif 4 <= x_value < 5:
                v = 0.044
            elif 5 <= x_value < 6:
                v = 0.052
            elif x_value >=6:
                v = 0.07
            elif 1 < x_value:
                v = 1.09e-3  
        else:
            v = 1.0e+10
        variance[i*3:3] = v

def landmarks_callback(msg):
    global landmarks_coord
    global landmarks_dict
    global Q_landmarks
    # list for storing updated locations sent 
    # from detection node 
    updated = []  

    for i, msg in enumerate(msg.data):
        ID = msg.ID    # First element is the ID
        x = msg.x      # Second element is x coordinate
        y = msg.y      # Third element is y coordinate
        z = msg.z      # Fourth element is z coordinate
        if ID != '':
            received = np.array([x, y, z])
            # Fill dictionary according to order of
            # landmark detection during whole run
            landmarks_dict[ID] = np.dot(Cam_to_AUV_R_M, received)
            # Fill list with landmarks witnessed in this msg
            updated.append(ID)
    # Extract the values from landmarks_dict
    # and convert them to a numpy array
    landmarks_coord = np.array(list(landmarks_dict.values()))
    # Reshape to n*1 array
    landmarks_coord = landmarks_coord.reshape(-1, 1)
    # Calculate the variance from X reading
    calc_variance(updated)
    # Setup Q for received landmarks redings
    Q_landmarks = np.diag(variance.flatten())
    Bolt_SLAM.predict(Bolt_SLAM.get_control(), msg.header.stamp.secs)
    Bolt_SLAM.correct_landmarks(landmarks_coord, Q_landmarks)
    
def callback_wrench(wrench_msg):
    # Execute prediction with each new control action
    """Edited --> changed the time stamp from integer seconds to float seconds"""
    Bolt_SLAM.predict([[wrench_msg.wrench.force.x], [wrench_msg.wrench.force.y], [wrench_msg.wrench.force.z],\
                                 [wrench_msg.wrench.torque.z]], float(wrench_msg.header.stamp.secs + wrench_msg.header.stamp.nsecs*1.0e-9)) 
    
def callback_imu(imu_msg):
# Get yaw value in radians
    orientation_euler = euler_from_quaternion([imu_msg.orientation.x, imu_msg.orientation.y,\
                                              imu_msg.orientation.z, imu_msg.orientation.w])
    # Execute prediction with most recent control action then yaw correction 
    Bolt_SLAM.predict(Bolt_SLAM.get_control(), imu_msg.header.stamp.secs)
    """Edited --> imu_msg.orientation_covariance[8] was added to the yaw correction reshaped z to a 2D array no need to change Q to 2D array"""
    Bolt_SLAM.correct(np.array([[0.0], [0.0], [0.0], [orientation_euler[2]], [0.0], [0.0], [0.0], [0.0]]).reshape(-1,1),\
                      (np.array([1.0e+10, 1.0e+10, 1.0e+10, imu_msg.orientation_covariance[8]])))

def callback_psensor(psensor_msg):
    # Convert pressure to underwater depth using  (P_fluid = rho * g * h) 
    Depth = (psensor_msg.fluid_pressure - 101325.0) / (9.8 * 997.0)
    # Execute prediction with most recent control action then depth correction 
    Bolt_SLAM.predict(Bolt_SLAM.get_control(), psensor_msg.header.stamp.secs)
    """Edited --> reshaped z to 2D array"""
    Bolt_SLAM.correct(np.array([[0.0], [0.0], [Depth], [0.0], [0.0], [0.0], [0.0], [0.0]]).reshape(-1,1),\
                      np.array([1.0e+10, 1.0e+10, psensor_msg.variance, 1.0e+10]))

def callback_ORBSLAM(pose_msg):
    global Tracking_lost
    if pose_msg.pose.position.x == 0.0 and pose_msg.pose.position.y == 0.0:
        # Raise Tracking loss flag 
        Tracking_lost = 1
    else:
        if Tracking_lost == 0:
            # ORB_SLAM is operational.
            # Execute prediction with most recent control action then x & y correction.
            Bolt_SLAM.predict(Bolt_SLAM.get_control(), pose_msg.header.stamp.secs)
            Bolt_SLAM.correct(np.array([[x_ref + pose_msg.pose.position.x], [y_ref + pose_msg.pose.position.y],\
                                        [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]]),\
                              np.array([x_ref_sigma + ORB_sigma, y_ref_sigma + ORB_sigma, 1.0e+10, 1.0e+10]))
        else:
            # ORB_SLAM returned from tracking loss.
            # Lower Tracking_lost flag.
            # Execute prediction with most recent control action then x & y correction
            # after reference update
            Tracking_lost = 0
            Bolt_SLAM.predict(Bolt_SLAM.get_control(), pose_msg.header.stamp.secs)
            x_ref = Bolt_SLAM.get_mean()[0,0]
            y_ref = Bolt_SLAM.get_mean()[1,0]
            x_ref_sigma = Bolt_SLAM.get_COV()[0,0]
            y_ref_sigma = Bolt_SLAM.get_COV()[1,1]
            Bolt_SLAM.correct(np.array([[x_ref + pose_msg.pose.position.x], [y_ref + pose_msg.pose.position.y],\
                                        [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]]),\
                              np.array([x_ref_sigma + ORB_sigma, y_ref_sigma + ORB_sigma, 1.0e+10, 1.0e+10]))

# Main function which initialises node, setups subscribers,
# publishers and executes SLAM algorithm
if __name__ == '__main__':
    #node initialization
    rospy.init_node('EKFO_landmarks', anonymous=True)
    rospy.loginfo("SLAM begins......")
    # Set up SLAM object
    Bolt_SLAM = EKF(AUV_mass, AUV_added_mass, I_matrix, Tau_g, Tau_b, Drag_co, Lift_co, rospy.get_time(), R, C)
    # Setting up publishers and subscribers
    sub_wrench = rospy.Subscriber("/Wrench", WrenchStamped, callback_wrench)
    sub_imu = rospy.Subscriber("/imu_processed", Imu, callback_imu)
    sub_psensor = rospy.Subscriber("pressure_processed", FluidPressure, callback_psensor)
    # sub_ORBSLAM = rospy.Subscriber("orb_slam3/camera_pose", PoseStamped, callback_ORBSLAM)
    #sub_perception = rospy.Subscriber('/landmarks', Multi_instance, landmarks_callback)

    while not rospy.is_shutdown():
        rospy.spin()
