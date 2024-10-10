#!/usr/bin/env python3

# libraries Include


import pyzed.sl as sl
import cv2
import joblib
import rospy
import numpy as np
from pub_sub.msg import instance
from pub_sub.msg import Multi_instance
from ultralytics import YOLO
import torch
import time
import threading
import signal
import os
import datetime
import pickle

##latest mohab - Sep 2024

#set cuda backend to utilize the gpu for inference
torch.cuda.set_device(0)
device= torch.device('cuda')

rospy.init_node('perc23')    # Ros Node                                               
pub = rospy.Publisher('/landmarks', Multi_instance, queue_size = 10) # Publisher 
# Custom message
perc23_msg = Multi_instance()  


# Define the Labels by taking the instance Number and return the Instance itself:
def getInstanceName(instance_number):
    labels =['gate','path marker','badge','gun','box','hand','barrel','note','phone','bottle','gman','bootlegger','axe','dollar','beer']
    return labels[instance_number] 


#model_weights_path = "/home/jetsonx/trials/pub_sub/scripts/best.pt"    
# This line assigns the file path to the variable.
model_weights_path = "/home/jetsonx/catkin_ws/src/pub_sub/scripts/best.pt"
model = YOLO(model_weights_path)                                                           
# passing the weights file to YOLO Model.
model.to(device=device)

#initialize the camera
zed = sl.Camera()
init_params= sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use ULTRA depth mode
init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use meter units (for depth measurements)
init_params.camera_fps = 15  # The framerate is lowered to avoid any USB3 bandwidth issues
runtime = sl.RuntimeParameters()
err = zed.open(init_params)
if (err!=sl.ERROR_CODE.SUCCESS):
    print('cam failed')
    exit(-1)

image = sl.Mat()
depth = sl.Mat()
point_cloud = sl.Mat()



print("Running...")

while not rospy.is_shutdown():
	
    if(zed.grab() == sl.ERROR_CODE.SUCCESS):
    # A new image is available if grab() returns SUCCESS
        zed.retrieve_image(image, sl.VIEW.LEFT) 
        # Get the left image
        zed.retrieve_measure(depth, sl.MEASURE.DEPTH) 
        # Retrieve depth Mat. Depth is aligned on the left image
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

    zed_is_detected = 0
    zed_u_mid_left = 0
    zed_v_mid_left = 0
    zed_u_mid_right = 0
    zed_v_mid_right = 0
    zed_x = 0
    zed_y = 0
    zed_z = 0 
    cvt_img=cv2.cvtColor(image.get_data(),cv2.COLOR_RGBA2RGB)
    print("shape:",cvt_img.shape)
    result = model.predict(source= cvt_img, show = False, conf=0.55)
    num_of_instances = result[0].boxes.data.size()[0]                    
    zed_is_detected = 1
    print(num_of_instances)
    for i in range(num_of_instances):
        zed_x_top_left_left = (int)(result[0].boxes.data[0][0].item())
        zed_y_top_left_left = (int)(result[0].boxes.data[0][1].item())
        zed_x_bottom_right_left = (int)(result[0].boxes.data[0][2].item())
        zed_y_bottom_right_left = (int)(result[0].boxes.data[0][3].item())
        zed_u_mid_left = (int)((zed_x_top_left_left + zed_x_bottom_right_left)/2.0)
        zed_v_mid_left = (int)((zed_y_top_left_left + zed_y_bottom_right_left)/2.0) 
        zed_P = point_cloud.get_value(zed_u_mid_left, zed_v_mid_left)
        instance_type = getInstanceName((int)(result[0].boxes.data[i][5].item()))
        confidence_level = result[0].boxes.data[i][4].item()
        #print(zed_P)
        #print(zed_P[1][0])
        #print(zed_P[1][1])
        print(zed_P[1][2])
        X_depth = zed_P[1][0]
        Y_depth = zed_P[1][1]
        Z_depth = zed_P[1][2]
        perc23_msg.data.append(instance())        
        if np.isnan(X_depth) or np.isnan(Y_depth) or np.isnan(Z_depth):
        	X_depth = 50
        	Y_depth = 50 
        	Z_depth = 50 
        # This line appends a new instance of an object to the data list within the perc23_msg message object
        perc23_msg.data[i].ID = instance_type     
        # assigns the value of instance_type to the ID attribute of the i-th element in the data list. 
        #It sets the ID of the instance to the value representing the type or class of the detected object.
        #if u want to display center pixels of bounding boxes ---> UNCOMMENT the following two lines
        perc23_msg.data[i].u = zed_u_mid_left
        # It sets the x-coordinate of the center point of the bounding box for the instance.
        perc23_msg.data[i].v = zed_v_mid_left                  
        #  It sets the y-coordinate of the center point of the bounding box for the instance.
        correction_svm_model = joblib.load('/home/jetsonx/catkin_ws/src/pub_sub/scripts/depth_correction_model.pkl')
        corrected_z = correction_svm_model.predict(np.array([X_depth/1000,Y_depth/1000,Z_depth/1000]).reshape(1,-1))

        perc23_msg.data[i].x = X_depth/1000                   # It sets the X-depth of the instance.
        perc23_msg.data[i].y = Y_depth/1000              # It sets the Y-depth of the instance.
        perc23_msg.data[i].z = corrected_z                   # It sets the Z-depth of the instance.
        #perc23_msg.data[i].confidence = confidence_level *100  
        perc23_msg.data[i].confidence = Z_depth/1000
        # #  It represents the confidence level of the detected object, scaled by a factor of 100.


    pub.publish(perc23_msg)
    perc23_msg.data.clear()
    rospy.sleep(2)

