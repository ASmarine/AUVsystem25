#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from tf.transformations import quaternion_from_euler

thrusts = Float32MultiArray()
rospy.init_node('trial_node', anonymous=True)
pub_T = rospy.Publisher("thrust_forces", Float32MultiArray, queue_size = 1)
counter = 1.0
    
if __name__ == '__main__':
    while not rospy.is_shutdown():
        thrusts.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, counter]
        pub_T.publish(thrusts)
        counter = counter + 1.0
        if counter > 20.0 :
        	counter = 20.0 
        rospy.sleep(1)
        

