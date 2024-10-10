#!/usr/bin/env python3

# A class concerned with the conversion of thrust values to PWM values is to be created
# Arduino ROS message covariance & edits including IMU
"""
This class represents an AUV exciter object.

This is the main system identification node responsible
for setting up the ROS environment to start implementing
the required experiment 

Developer/s:
Samer A. Mohamed, Hossam M. Elkeshky.
"""
# Import relevant libraries
from sys_iden_exec_SA import Exciter 
import rospy
import math
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from sensor_msgs.msg import FluidPressure

# initialize choice of experiment inserted when the node runs
exp = rospy.get_param("/sys_iden_node/exp")

#create thrust ROS message
Thrust_msg = Float32MultiArray()

#create a flag to terminate this node when experiment ends 
Tflag = False

#Import relevant experiment configuration parameters 
lh = rospy.get_param("/L_hor") #Horizontal thrust arm in meters
lv = rospy.get_param("/L_ver") #Vertical thrust arm in meters
la = rospy.get_param("/L_arm") #Center to center thrust arm in meters
lim = rospy.get_param("/Lim") #Thrust limit per thruster in Newton
G = np.array(rospy.get_param("/C_Gains")) #Control gains

#   Convert a quaternion into euler angles (roll, pitch, yaw)
#    roll is rotation around x in radians (counterclockwise)
#   pitch is rotation around y in radians (counterclockwise)
#    yaw is rotation around z in radians (counterclockwise) 
def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return np.array([roll_x, pitch_y, yaw_z]) # in radians

# IMU callback function which receives sensor message,
# processes it into desired format and passes it to 
# the function responsible for updating control actions
def callback_IMU1(imu_msg1):
    global Tflag
    orientation = euler_from_quaternion(imu_msg1.orientation.x, imu_msg1.orientation.y, imu_msg1.orientation.z, imu_msg1.orientation.w)
    Thrust, Tflag = bolt_iden.update(orientation, rospy.get_time())
    Thrust_msg.data = Thrust
    pub_thrust.publish(Thrust_msg)

# Psensor callback function which receives sensor message,
# processes it into desired format and passes it to 
# the function responsible for updating control actions
def callback_PSensor1(pressure_msg1):
    global Tflag
    # Convert pressure to underwater depth using  (P_fluid = rho * g * h) 
    Depth = (pressure_msg1.fluid_pressure - 101325.0) / (9.8 * 1000.0)
    Thrust, Tflag = bolt_iden.update(Depth, rospy.get_time())
    Thrust_msg.data = Thrust
    pub_thrust.publish(Thrust_msg)

# Main function which initialises node, setups subscribers,
# publishers and controlling class and terminates the node
if __name__ == '__main__':
    #node initialization
    rospy.init_node('sys_iden_node', anonymous=True)
    rospy.loginfo("node started, please select your experiment\n")
    print(exp)
    print("experiment\n")
    #exp = input("Enter your experiment of choice\n [x, y, z, k, m, n]\n")
    # Setting up publishers, subscribers and controlling class object
    bolt_iden = Exciter(exp, G, rospy.get_time(), lh, lv, la, lim)
    pub_thrust = rospy.Publisher("/pwm", Float32MultiArray, queue_size = 1)
    sub_IMU1 = rospy.Subscriber("/imu1", Imu, callback_IMU1)
    sub_Psensor1 = rospy.Subscriber("/pressure1", FluidPressure, callback_PSensor1)
    # Node termination condition
    while not rospy.is_shutdown():
        if Tflag:
            print('Experiment is concluded')
            rospy.signal_shutdown('Experiment is concluded')

