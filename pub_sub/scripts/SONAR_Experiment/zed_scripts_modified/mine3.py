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

# using time module
import time


csv_file_path = 'test.csv'
df = pd.DataFrame({'u2': pd.Series(dtype='int'),
                   'v2': pd.Series(dtype='int'),
                   'x2': pd.Series(dtype='float'),
                   'y2': pd.Series(dtype='float'),
                   'z2': pd.Series(dtype='float')})


check_file_exist = os.path.isfile(csv_file_path)
if(check_file_exist == False):
    df.to_csv('test.csv',mode='a',index=False, header = True) 







from ultralytics import YOLO

face_model_weights_path = "C:/Users/mohab/Downloads/best.pt"
		     
face_model = YOLO(face_model_weights_path)


detect_model_weights_path = "C:/Users/mohab/Downloads/best.pt"
detect_model = YOLO(detect_model_weights_path)
detect_model_COCO = YOLO('yolov8n.pt')


zed_list = []
left_list = []
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
    global point_cloud_list
    global depth_list

    runtime = sl.RuntimeParameters()
    while not stop_signal:
        err = zed_list[index].grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            zed_list[index].retrieve_image(left_list[index], sl.VIEW.LEFT)
            zed_list[index].retrieve_measure(depth_list[index], sl.MEASURE.DEPTH)
            zed_list[index].retrieve_measure(point_cloud_list[index], sl.MEASURE.XYZRGBA)
            timestamp_list[index] = zed_list[index].get_timestamp(sl.TIME_REFERENCE.CURRENT).data_ns
        time.sleep(0.001) #1ms
    zed_list[index].close()
	
def main():
    global stop_signal
    global zed_list
    global left_list
    global depth_list
    global point_cloud_list
    global timestamp_list
    global thread_list
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
    while key != 113:  # for 'q' key
    
        
        zed2_is_detected = 0
        zed2_u_mid = 0
        zed2_v_mid = 0
        zed2_x = 0
        zed2_y = 0
        zed2_z = 0
        
        
        for index in range(0, len(zed_list)):
            if zed_list[index].is_opened():
                if (timestamp_list[index] > last_ts_list[index]):               
                    
                    ############################
                    
                    '''
                    if(name_list[index] == 'ZED 16133'):
                        zed1_image_detect = cv2.cvtColor(left_list[index].get_data(),cv2.COLOR_RGBA2RGB)
                        zed1_result = face_model.predict(source= zed1_image_detect, show = True)
                        zed1_num_of_instances = zed1_result[0].boxes.boxes.size()[0]
                        
                        #if(zed1_num_of_instances == 1):
                            
                            zed1_x_top_left = (int)(zed1_result[0].boxes.boxes[0][0].item())
                            zed1_y_top_left = (int)(zed1_result[0].boxes.boxes[0][1].item())
                            zed1_x_bottom_right = (int)(zed1_result[0].boxes.boxes[0][2].item())
                            zed1_y_bottom_right = (int)(zed1_result[0].boxes.boxes[0][3].item())
                            zed1_u_mid = (int)((zed1_x_top_left + zed1_x_bottom_right)/2.0)
                            zed1_v_mid = (int)((zed1_y_top_left + zed1_y_bottom_right)/2.0)
                            

                            #zed1_x = depth_list[index].get_value(zed1_u_mid, zed1_v_mid)
                            #zed1_point3D = point_cloud_list[index].get_value(zed1_u_mid, zed1_v_mid)
                            #zed1_x = zed1_point3D[1][0]
                            zed1_y = zed1_point3D[1][1]
                            zed1_z = zed1_point3D[1][2]

                            zed1_color  = zed1_point3D[1][3]

                            zed1_is_detected = 1
                    ############################
                    '''
                    
                    
                    if(name_list[index] == 'ZED 29976112'):
                        zed2_image_detect = cv2.cvtColor(left_list[index].get_data(),cv2.COLOR_RGBA2RGB)
                        zed2_result = detect_model.predict(source= zed2_image_detect, show = True, conf=0.7)
                        zed2_num_of_instances = zed2_result[0].boxes.data.size()[0]
                        
                        if(zed2_num_of_instances == 1 ):
                            if((int)(zed2_result[0].boxes.data[i][5].item()) == 10): #10 gman     11 bootleger
                                zed2_x_top_left = (int)(zed2_result[0].boxes.data[0][0].item())
                                zed2_y_top_left = (int)(zed2_result[0].boxes.data[0][1].item())
                                zed2_x_bottom_right = (int)(zed2_result[0].boxes.data[0][2].item())
                                zed2_y_bottom_right = (int)(zed2_result[0].boxes.data[0][3].item())
                                zed2_u_mid = (int)((zed2_x_top_left + zed2_x_bottom_right)/2.0)
                                zed2_v_mid = (int)((zed2_y_top_left + zed2_y_bottom_right)/2.0)
                                
                                ##add here the depth estimate function
                                #zed1_x = depth_list[index].get_value(zed1_u_mid, zed1_v_mid)
                                zed2_point3D = point_cloud_list[index].get_value(zed2_u_mid, zed2_v_mid)
                                zed2_x = zed2_point3D[1][0]
                                zed2_y = zed2_point3D[1][1]
                                zed2_z = zed2_point3D[1][2]
                                
                                zed2_color  = zed2_point3D[1][3]

                                
                                zed2_is_detected = 1
                    
                    ###########################
                    last_ts_list[index] = timestamp_list[index]
        
        if zed2_is_detected == 1:
            readings = {'u2': [zed2_u_mid],
                        'v2': [zed2_v_mid],
                        'x2': [zed2_x],
                        'y2': [zed2_y],
                        'z2': [zed2_z]}
            
            df = pd.DataFrame(readings)
            ts = time.time()
            filename = 'C:/Users/mohab/Downloads/left/(LEFT('+str(ts)+')('+str(zed2_u_mid)+')('+str(zed2_v_mid)+')('+str(zed2_x)+')('+str(zed2_y)+')('+str(zed2_z)+')).png'
            cv2.imwrite(filename,zed2_image_detect)
            print('image written to '+filename)
            # index += 1

            
            df.to_csv(csv_file_path, mode='a', index = False, header=False)
        key = cv2.waitKey(5)        
        

        #time.sleep(1)
    cv2.destroyAllWindows()

    #Stop the threads
    stop_signal = True
    for index in range(0, len(thread_list)):
        thread_list[index].join()

    print("\nFINISH")

if __name__ == "__main__":
    main()
