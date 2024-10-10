#!/usr/bin/env python3

# libraries Include
import rospy
import numpy as np
from pub_sub.msg import instance
from pub_sub.msg import Multi_instance
from ultralytics import YOLO
import torch
import time
import pyzed.sl as sl
import cv2
import threading
import signal
import os
import datetime
import rospy
import joblib
import numpy as np


#set cuda backend to utilize the gpu for inference
torch.cuda.set_device(0)
device= torch.device('cuda')


# Define the Labels by taking the instance Number and return the Instance itself:
def getInstanceName(instance_number):
    labels =
    ['gate','path marker','badge','gun','box','hand','barrel','note','phone','bottle','gman','bootlegger','axe','dollar','beer']
    return labels[instance_number] 


#model_weights_path = "/home/jetsonx/trials/pub_sub/scripts/best.pt"    
# This line assigns the file path to the variable.
model_weights_path = "/home/jetsonx/catkin_ws/src/pub_sub/scripts/best.pt"
model = YOLO(model_weights_path)                                                           
# passing the weights file to YOLO Model.
model.to(device=device)
zed_list = []
left_list = []
right_list = []
depth_list = []
point_cloud_list = []
timestamp_list = []
thread_list = []
stop_signal = False


def signal_handler(signal, frame):
    global stop_signal
    stop_signal=True
    time.sleep(0.5)
    exit()

def grab_run(index):
    global stop_signal
    global zed_list
    global timestamp_list
    global left_list
    global right_list
    global point_cloud_list
    global depth_list

    runtime = sl.RuntimeParameters()
    while not stop_signal:
        err = zed_list[index].grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            zed_list[index].retrieve_image(left_list[index], sl.VIEW.LEFT)
            zed_list[index].retrieve_image(right_list[index], sl.VIEW.RIGHT)
            zed_list[index].retrieve_measure(depth_list[index], sl.MEASURE.DEPTH)
            zed_list[index].retrieve_measure(point_cloud_list[index], sl.MEASURE.XYZRGBA)
            timestamp_list[index] = zed_list[index].get_timestamp(sl.TIME_REFERENCE.CURRENT).data_ns
        time.sleep(0.001) #1ms
    zed_list[index].close()
	

print("Perception Running...")
init = sl.InitParameters()
init.camera_resolution = sl.RESOLUTION.HD720
init.depth_mode = sl.DEPTH_MODE.ULTRA # Use ULTRA depth mode
init.coordinate_units = sl.UNIT.MILLIMETER # Use millimeter units (for depth measurements)
init.camera_fps = 15  # The framerate is lowered to avoid any USB3 bandwidth issues

cameras = sl.Camera.get_device_list()
index = 0
for cam in cameras:
    init.set_from_serial_number(cam.serial_number)
    name_list.append("ZED {}".format(cam.serial_number))
    print("Opening {}".format(name_list[index]))
    zed_list.append(sl.Camera())
    left_list.append(sl.Mat())
    right_list.append(sl.Mat())
    depth_list.append(sl.Mat())
    point_cloud_list.append(sl.Mat())
    timestamp_list.append(0)
    last_ts_list.append(0)
    status = zed_list[index].open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        zed_list[index].close()
    index = index +1

#Start camera threads
for index in range(0, len(zed_list)):
    if zed_list[index].is_opened():
        thread_list.append(threading.Thread(target=grab_run, args=(index,)))
        thread_list[index].start()

while not rospy.is_shutdown():  # for 'q' key

    
    zed_is_detected = 0
    zed_u_mid_left = 0
    zed_v_mid_left = 0
    zed_u_mid_right = 0
    zed_v_mid_right = 0
    zed_x = 0
    zed_y = 0
    zed_z = 0
    
    
    for index in range(0, len(zed_list)):
        if zed_list[index].is_opened():
            if (timestamp_list[index] > last_ts_list[index]):               
                
                ############################
                
                
                if(name_list[index] == 'ZED 16133'):
                    zed_left_image_detect = cv2.cvtColor(left_list[index].get_data(),cv2.COLOR_RGBA2RGB)
                    zed_left_result = left_detect_model.predict(source= zed_left_image_detect, show = False, conf=0.7)
                    zed_left_num_of_instances = zed_left_result[0].boxes.data.size()[0]
                                        
                            zed_is_detected = 1
                            zed_x_top_left_left = (int)(zed_left_result[0].boxes.data[0][0].item())
                            zed_y_top_left_left = (int)(zed_left_result[0].boxes.data[0][1].item())
                            zed_x_bottom_right_left = (int)(zed_left_result[0].boxes.data[0][2].item())
                            zed_y_bottom_right_left = (int)(zed_left_result[0].boxes.data[0][3].item())
                            zed_u_mid_left = (int)((zed_x_top_left_left + zed_x_bottom_right_left)/2.0)
                            zed_v_mid_left = (int)((zed_y_top_left_left + zed_y_bottom_right_left)/2.0)
                            
                            zed_point3D = point_cloud_list[index].get_value(zed_u_mid_left, zed_v_mid_left)
                            zed_x = zed_point3D[1][0]
                            zed_y = zed_point3D[1][1]
                            zed_z = 120.0*800.0/(zed_u_mid_left - zed_u_mid_right)
                            
                            

      perc23_msg.data.append(instance())        # This line appends a new instance of an object to the data list within the perc23_msg message object
      perc23_msg.data[i].ID = instance_type     # assigns the value of instance_type to the ID attribute of the i-th element in the data list. It sets the ID of the instance to the value representing the type or class of the detected object.
      
      #if u want to display center pixels of bounding boxes ---> UNCOMMENT the following two lines
      
      perc23_msg.data[i].u = u                  # It sets the x-coordinate of the center point of the bounding box for the instance.
      perc23_msg.data[i].v = v                  # It sets the y-coordinate of the center point of the bounding box for the instance.
      correction_svm_model = joblib.load('depth_correction_model.pkl')
      corrected_z = correction_svm_model.predict(np.array([zed_x/1000,zed_y/1000,zed_z/1000]).reshape(1,-1))

      perc23_msg.data[i].x = X_depth                   # It sets the X-depth of the instance.
      perc23_msg.data[i].y = Y_depth                   # It sets the Y-depth of the instance.
      perc23_msg.data[i].z = Z_depth                   # It sets the Z-depth of the instance.
      
      perc23_msg.data[i].confidence = confidence_level *100   #  It represents the confidence level of the detected object, scaled by a factor of 100.
       
    
#Stop the threads
stop_signal = True
for index in range(0, len(thread_list)):
    thread_list[index].join()



