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
from std_msgs.msg import UInt32


global exp_data 
exp_data = pd.DataFrame()
global rows
rows = []
global depth_buffer
depth_buffer = []   

global target
target = 0


def cam_callback(msg):
	print('cam msg')
	i = 0

	for i in range(len(msg.data)):
		if msg.data[i].ID == "badge":
			global target
			target = target + 1 
			print('target accquired ',target)
			new_row = {
 		    	"z_gt": (depth_buffer[-1])/1000,
 		    	"x_camera": msg.data[i].x,
 		    	"y_camera": msg.data[i].y,
 		    	"z_camera": msg.data[i].z,
 		    	'ID': msg.data[i].ID,
 		    	'u_camera': msg.data[i].u,
 		    	'v_camera': msg.data[i].v,
 		    	'old_camera':msg.data[i].confidence
 		    	
 		    
		    }
			rows.append(new_row)
			print(new_row)
			

def depth_logger(msg): 
	print("depth: ",msg.data)
	#add arduino readings
	#append to new row
	
	depth_buffer.append(msg.data)
	
	
def Shutdown_Callback():
		exp_data = pd.DataFrame(rows)
		print(exp_data)
		exp_data.to_csv(path_or_buf ='/home/jetsonx/catkin_ws/src/pub_sub/scripts/depth_exp.csv',index=True ,header = True)
		print('experiment concluded, file saved')

#Node initialization

rospy.init_node('depth_exp')
rospy.Subscriber('/landmarks', Multi_instance, cam_callback)
rospy.Subscriber('/distance',UInt32, depth_logger)




rospy.on_shutdown(Shutdown_Callback)
#Loop
while not rospy.is_shutdown():
	rospy.spin()

