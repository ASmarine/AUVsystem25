#!/usr/bin/env python3
import sensor_msgs.point_cloud2 as pc2
import pyzed.sl as sl # type: ignore
import rospy
import std_msgs
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2, PointField
import cv2
from cv_bridge import CvBridge
import numpy as np
import time

rospy.init_node('camera_node')

image_pub = rospy.Publisher('/zed/image_raw', Image, queue_size=10)
point_cloud_pub = rospy.Publisher('/zed/PC', PointCloud2, queue_size=5)
zed = sl.Camera()
rate = rospy.Rate(15)  # 15 Hz
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.VGA
init_params.depth_mode = sl.DEPTH_MODE.ULTRA
init_params.coordinate_units = sl.UNIT.MILLIMETER
init_params.camera_fps = 15
init_params.camera_disable_self_calib = True
init_params.optional_settings_path="/usr/local/zed/settings/SN16133.conf"
fields = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1)
]


runtime_params = sl.RuntimeParameters()
err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    print('Camera initialization failed')
    exit(-1)

image = sl.Mat()
point_cloud = sl.Mat()
depth = sl.Mat()
bridge = CvBridge()

print("Camera Node Running...")

# Inside your loop where you grab data from the ZED camera
while not rospy.is_shutdown():
    calibration_params_raw = zed.get_camera_information().camera_configuration.calibration_parameters_raw
    calibration_params_rect = zed.get_camera_information().camera_configuration.calibration_parameters

    #print("init",zed.get_init_parameters().camera_disable_self_calib)
    #print("Res",zed.get_camera_information().camera_configuration.resolution.width,"x",zed.get_camera_information().camera_configuration.resolution.height)
    #print("init",zed.get_init_parameters().optional_settings_path)
    #print(zed.get_init_parameters().optional_settings_path)

    # Focal length of the left eye in pixels
    fx_raw = calibration_params_raw.left_cam.fx
    fy_raw = calibration_params_raw.left_cam.fy
    
    fx_rect = calibration_params_rect.left_cam.fx
    fy_rect = calibration_params_rect.left_cam.fy
    # First radial distortion coefficient
    #k1 = calibration_params.left_cam.disto[0]
    # Translation between left and right eye on x-axis
    #tx = calibration_params.stereo_transform.get_translation().get()[0]
    # Horizontal field of view of the left eye in degrees
    #h_fov = calibration_params.left_cam.h_fov
    print("RAW",fx_raw,fy_raw)
    print("Rect",fx_rect,fy_rect)
    print(init_params.camera_disable_self_calib)
    rate.sleep()

