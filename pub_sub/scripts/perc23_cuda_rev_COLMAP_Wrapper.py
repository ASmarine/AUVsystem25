#!/usr/bin/env python3

# Libraries Include

import rospy
import joblib
from joblib import parallel_backend
import numpy as np
from pub_sub.msg import instance
from pub_sub.msg import Multi_instance
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch
import time
import signal
import os
import datetime
import cv2
import pickle

# Set CUDA backend to utilize the GPU for inference
torch.cuda.set_device(0)
device = torch.device('cuda')

# ROS Node Initialization
rospy.init_node('perc23')
pub = rospy.Publisher('/landmarks', Multi_instance, queue_size=10)

# Custom message object
perc23_msg = Multi_instance()

# Initialize the CvBridge for image conversion
bridge = CvBridge()

# Define the labels by instance number
def getInstanceName(instance_number):
    labels = ['gate', 'path marker', 'badge', 'gun', 'box', 'hand', 'barrel', 'note', 'phone', 'bottle', 'gman', 'bootlegger', 'axe', 'dollar', 'beer']
    return labels[instance_number]

# Load the YOLO model
model_weights_path = "/home/jetsonx/catkin_ws/src/pub_sub/scripts/best.pt"
model = YOLO(model_weights_path)
model.to(device=device)

# Load the correction model for depth
with parallel_backend('threading'):
	correction_svm_model = joblib.load('/home/jetsonx/catkin_ws/src/pub_sub/scripts/depth_correction_model.pkl')

# Initialize callback variables
image_data = None
depth_data = None
point_cloud_data = None

# Callback to handle image messages
def image_callback(msg):
    global image_data
    image_data = bridge.imgmsg_to_cv2(msg, "bgr8")

# Callback to handle depth messages
def depth_callback(msg):
    global depth_data
    depth_data = msg

# Callback to handle point cloud messages
def point_cloud_callback(msg):
    global point_cloud_data
    point_cloud_data = msg

# Subscribe to ROS topics
rospy.Subscriber('/zed/zed_node/left/image_rect_color', Image, image_callback)
rospy.Subscriber('/zed/zed_node/depth/depth_registered', PointCloud2, depth_callback)
rospy.Subscriber('/zed/zed_node/point_cloud/cloud_registered', PointCloud2, point_cloud_callback)

print("Running...")

while not rospy.is_shutdown():
    if image_data is not None and point_cloud_data is not None:
        # Convert the image and run YOLO inference
        cvt_img = cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
        print("Image shape:", cvt_img.shape)
        
        result = model.predict(source=cvt_img, show=False, conf=0.55)
        num_of_instances = result[0].boxes.data.size()[0]
        print("Number of instances:", num_of_instances)
        
        for i in range(num_of_instances):
            zed_x_top_left_left = int(result[0].boxes.data[i][0].item())
            zed_y_top_left_left = int(result[0].boxes.data[i][1].item())
            zed_x_bottom_right_left = int(result[0].boxes.data[i][2].item())
            zed_y_bottom_right_left = int(result[0].boxes.data[i][3].item())
            
            zed_u_mid_left = int((zed_x_top_left_left + zed_x_bottom_right_left) / 2.0)
            zed_v_mid_left = int((zed_y_top_left_left + zed_y_bottom_right_left) / 2.0)
            
            # Extract point cloud data at the center of the bounding box
            for point in pc2.read_points(point_cloud_data, skip_nans=True, uvs=[[zed_u_mid_left, zed_v_mid_left]]):
                X_depth, Y_depth, Z_depth = point[:3]
                break
            
            instance_type = getInstanceName(int(result[0].boxes.data[i][5].item()))
            confidence_level = result[0].boxes.data[i][4].item()

            if np.isnan(X_depth) or np.isnan(Y_depth) or np.isnan(Z_depth):
                X_depth, Y_depth, Z_depth = 50, 50, 50

            # Depth correction
            corrected_z = correction_svm_model.predict(np.array([X_depth, Y_depth, Z_depth]).reshape(1, -1))

            # Append new instance data to the message
            perc23_msg.data.append(instance())
            perc23_msg.data[i].ID = instance_type
            perc23_msg.data[i].u = zed_u_mid_left
            perc23_msg.data[i].v = zed_v_mid_left
            perc23_msg.data[i].x = X_depth
            perc23_msg.data[i].y = Y_depth
            perc23_msg.data[i].z = corrected_z[0]
            perc23_msg.data[i].confidence = Z_depth

        # Publish the message
        pub.publish(perc23_msg)
        perc23_msg.data.clear()

    rospy.sleep(0.5)

