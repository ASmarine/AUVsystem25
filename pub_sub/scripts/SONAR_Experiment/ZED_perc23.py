#!/usr/bin/env python3
# -- coding: utf-8 --
"""
Created on Sun Apr 23 15:08:30 2023

@author: philo
"""

import pyzed.sl as sl
import cv2
import numpy as np
import threading
import time
import signal
import pandas as pd
import os
import datetime
import rospy
# using time module
import time
from pub_sub.msg import instance
from pub_sub.msg import Multi_instance


rospy.init_node('ZED_perc23')    # Ros Node                                               
pub = rospy.Publisher('/landmarks', Multi_instance, queue_size = 10) # Publisher 
# Custom message
perc23_msg = Multi_instance()  

timestamp = datetime.now().strftime("%Y%m%d_%H%M%S") +str("pyzed")
csv_file_path = f"{timestamp}.csv"

df = pd.DataFrame({'u_left_uw': pd.Series(dtype='int'),
                   'v_left_uw': pd.Series(dtype='int'),
                   'u_right_uw': pd.Series(dtype='int'),
                   'v_right_uw': pd.Series(dtype='int'),
                   'X_depth': pd.Series(dtype='float'),
                   'Y_depth': pd.Series(dtype='float'),
                   'Z_depth': pd.Series(dtype='float')})


check_file_exist = os.path.isfile(csv_file_path)
if(check_file_exist == False):
    df.to_csv(csv_file_path,mode='a',index=False, header = True) 


from ultralytics import YOLO

detect_model_weights_path = "/home/jetsonx/catkin_ws/src/pub_sub/scripts/best.pt"
left_detect_model = YOLO(detect_model_weights_path)
right_detect_model = YOLO(detect_model_weights_path)


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
	

stop_signal = False
zed_list = []
left_list = []
right_list = []
depth_list = []
point_cloud_list = []
timestamp_list = []
thread_list = []
signal.signal(signal.SIGINT, signal_handler)

print("Running...")
init = sl.InitParameters()
init.camera_resolution = sl.RESOLUTION.HD720
init.depth_mode = sl.DEPTH_MODE.ULTRA # Use ULTRA depth mode
init.coordinate_units = sl.UNIT.MILLIMETER # Use millimeter units (for depth measurements)
init.camera_fps = 15  # The framerate is lowered to avoid any USB3 bandwidth issues

#List and open cameras
name_list = []
last_ts_list = []
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

i = 0
#Display camera images
key = ''
while not rospy.is_shutdown():  # for 'q' key

    
    zed2_is_detected = 0
    zed2_u_mid_left = 0
    zed2_v_mid_left = 0
    zed2_u_mid_right = 0
    zed2_v_mid_right = 0
    zed2_x = 0
    zed2_y = 0
    zed2_z = 0
    
    
    for index in range(0, len(zed_list)):
        if zed_list[index].is_opened():
            if (timestamp_list[index] > last_ts_list[index]):               
                
                ############################
                
                
                if(name_list[index] == 'ZED 16133'):
                    zed2_left_image_detect = cv2.cvtColor(left_list[index].get_data(),cv2.COLOR_RGBA2RGB)
                    zed2_right_image_detect = cv2.cvtColor(right_list[index].get_data(),cv2.COLOR_RGBA2RGB)
                    zed2_left_result = left_detect_model.predict(source= zed2_left_image_detect, show = True, conf=0.7)
                    zed2_right_result = right_detect_model.predict(source= zed2_right_image_detect, show = True, conf=0.7)
                    zed2_left_num_of_instances = zed2_left_result[0].boxes.data.size()[0]
                    zed2_right_num_of_instances = zed2_right_result[0].boxes.data.size()[0]
                    
                    if((zed2_left_num_of_instances == 1)and(zed2_right_num_of_instances==1)):
                        if(((int)(zed2_left_result[0].boxes.data[i][5].item()) == 10) and ((int)(zed2_right_result[0].boxes.data[i][5].item()) == 10)): #10 gman     11 bootleger
                            zed2_is_detected = 1
                            zed2_x_top_left_left = (int)(zed2_left_result[0].boxes.data[0][0].item())
                            zed2_y_top_left_left = (int)(zed2_left_result[0].boxes.data[0][1].item())
                            zed2_x_bottom_right_left = (int)(zed2_left_result[0].boxes.data[0][2].item())
                            zed2_y_bottom_right_left = (int)(zed2_left_result[0].boxes.data[0][3].item())
                            zed2_u_mid_left = (int)((zed2_x_top_left_left + zed2_x_bottom_right_left)/2.0)
                            zed2_v_mid_left = (int)((zed2_y_top_left_left + zed2_y_bottom_right_left)/2.0)

                            zed2_x_top_left_right = (int)(zed2_right_result[0].boxes.data[0][0].item())
                            zed2_y_top_left_right = (int)(zed2_right_result[0].boxes.data[0][1].item())
                            zed2_x_bottom_right_right = (int)(zed2_right_result[0].boxes.data[0][2].item())
                            zed2_y_bottom_right_right = (int)(zed2_right_result[0].boxes.data[0][3].item())
                            zed2_u_mid_right = (int)((zed2_x_top_left_right + zed2_x_bottom_right_right)/2.0)
                            zed2_v_mid_right = (int)((zed2_y_top_left_right + zed2_y_bottom_right_right)/2.0)
                            
                            zed2_point3D = point_cloud_list[index].get_value(zed2_u_mid_left, zed2_v_mid_left)
                            zed2_x = zed2_point3D[1][0]
                            zed2_y = zed2_point3D[1][1]
                            zed2_z = 120.0*800.0/(zed2_u_mid_left - zed2_u_mid_right)
                            
                            
                
                ###########################
                last_ts_list[index] = timestamp_list[index]
    


    if zed2_is_detected == 1:
        zed2_is_detected =0
        readings = {'u_left_uw': [zed2_u_mid_left],
                    'v_left_uw': [zed2_v_mid_left],
                    'u_right_uw': [zed2_u_mid_right],
                    'v_right_uw': [zed2_v_mid_right],
                    'X_depth': [zed2_x],
                    'Y_depth': [zed2_y],
                    'Z_depth': [zed2_z]
                    }
        
        df = pd.DataFrame(readings)

        df.to_csv(csv_file_path, mode='a', index = False, header=False)
    key = cv2.waitKey(5)        
    

    #time.sleep(1)
cv2.destroyAllWindows()

#Stop the threads
stop_signal = True
for index in range(0, len(thread_list)):
    thread_list[index].join()

print("\nFINISH")

