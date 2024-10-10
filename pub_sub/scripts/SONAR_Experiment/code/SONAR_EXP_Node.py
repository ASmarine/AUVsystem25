#!/usr/bin/env python3
# Depth experiment node
# Collect dataframe experiment
# pressure, sonar data (z Ground truth) , x (camera) , y (camera) , z (camera) 
# Author:
# - Mohab Ahmed

#importing libraries

import pandas as pd 
import rospy 
from pub_sub.msg import instance
from pub_sub.msg import Multi_instance
from std_msgs.msg import int

def cam_callback(cam_msg):
    print(cam_msg.data)    


#Node initialization

rospy.init_node('depth_exp')

rospy.Subscriber('/landmarks', Multi_instance, cam_callback)
rospy.Subscriber('/distance0',int, depth_logger)
exp_data = pd.Dataframe(columns=['z_gt','x_camera','y_camera','z_camera'])




#Loop
while not rospy.is_shutdown():
	exp_data.append(new_row,sort=True)
	
		
rospy.spin()

#on termination
rospy.on_shutdown(explogger.save_to_csv())


	
def cam_callback(msg,depth_buffer):
	if msg.data[i].ID == :
		new_row = {
 		    "z_gt": depth_buffer[-1]
 		    "x_camera": msg.data.[i].x
		    "y_camera": msg.data.[i].y
		    "z_camera": msg.data.[i].z
		    }
	msg.data.clear()
	depth_buffer.clear()	 

def depth_logger(msg):
	#add arduino readings
	#append to new row
	global depth_buffer.append(msg.data)
	
	
def save_to_csv(df):
		df.to_csv(path='/home/jetsonx/catkin_ws/src/pub_sub/scripts/depth_exp.csv',index=True ,header = True)
		print('experiment concluded, file saved")
		
