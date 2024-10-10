#!/usr/bin/env python3

import pyzed.sl as sl
import rospy
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float32MultiArray
import cv2
from cv_bridge import CvBridge

rospy.init_node('camera_node')


image_pub = rospy.Publisher('/zed/image_raw', Image, queue_size=10)
point_cloud_pub = rospy.Publisher('/zed/point_cloud', PointCloud2, queue_size=10)
depth_pub = rospy.Publisher('/zed/depth', Float32MultiArray, queue_size=10)


zed = sl.Camera()
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.depth_mode = sl.DEPTH_MODE.ULTRA
init_params.coordinate_units = sl.UNIT.MILLIMETER
init_params.camera_fps = 15

runtime_params = sl.RuntimeParameters()
err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    print('Camera initialization failed')
    exit(-1)

image = sl.Mat()
depth = sl.Mat()
point_cloud = sl.Mat()
bridge = CvBridge()

print("Camera Node Running...")

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
import numpy as np

# Inside your loop where you grab data from the ZED camera
while not rospy.is_shutdown():
    if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
        # Retrieve the image, depth, and point cloud
        zed.retrieve_image(image, sl.VIEW.LEFT)
        zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ)

        # Convert to ROS messages
        img_msg = bridge.cv2_to_imgmsg(image.get_data(), encoding="bgr8")

        # Convert point cloud to ROS PointCloud2 message
        point_cloud_data = point_cloud.get_data()  # sl.Mat to numpy
        points = point_cloud_data.reshape(-1, 3)  # Flatten to get x, y, z

        # Create PointCloud2 message
        header = std_msgs.msg.Header( )
        header.stamp = rospy.Time.now()
        header.frame_id = "zed_camera"  # Change this to your camera frame

        # Create the PointCloud2 message with only x, y, z
        point_cloud_msg = pc2.create_cloud_xyz32(header, points)

        depth_msg = Float32MultiArray(data=depth.get_data().flatten())

        # Publish the data
        image_pub.publish(img_msg)
        point_cloud_pub.publish(point_cloud_msg)
        depth_pub.publish(depth_msg)

    rospy.sleep(0.1)
