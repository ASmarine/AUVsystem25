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

#left_image_pub = rospy.Publisher('/zed/left_image_rect', Image, queue_size=10)
left_image_pub = rospy.Publisher('/zed/zed_node/left/image_rect_color', Image, queue_size=10)

#left_hires_pub = rospy.Publisher('/zed/left_image_hires', Image, queue_size=10)

#right_image_pub = rospy.Publisher('/zed/right_image_rect', Image, queue_size=10)
right_image_pub = rospy.Publisher('/zed/zed_node/right/image_rect_color', Image, queue_size=10)
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

left_image = sl.Mat()
right_image = sl.Mat()
#hires_image = sl.Mat()
point_cloud = sl.Mat()
depth = sl.Mat()
bridge = CvBridge()

print("Camera Node Running...")

# Inside your loop where you grab data from the ZED camera
while not rospy.is_shutdown():
    if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:

        # Retrieve the image, depth, and point cloud
        zed.retrieve_image(left_image, sl.VIEW.LEFT)
        #zed.retrieve_image(left_image, sl.VIEW.LEFT,resolution=sl.Resolution(640,320))
        zed.retrieve_image(right_image, sl.VIEW.RIGHT)
        #zed.retrieve_image(right_image, sl.VIEW.RIGHT,resolution=sl.Resolution(640,320))
        zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ)

        
        # Convert to ROS messages
        left_img_msg = bridge.cv2_to_imgmsg(left_image.get_data(), encoding="passthrough")
        right_img_msg = bridge.cv2_to_imgmsg(right_image.get_data(), encoding="passthrough")
        #hires_img_msg = bridge.cv2_to_imgmsg(hires_image.get_data(), encoding="passthrough")
        # Convert point cloud to ROS PointCloud2 message
        pc_data = point_cloud.get_data()  # sl.Mat to numpy
        pc_data = pc_data[:, :, :3]
        pc_data = pc_data.astype(np.float32)
        pc_data_bytes = pc_data.tobytes()
        
        
        # Create a PointCloud2 message
        pc2_msg = PointCloud2()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "zed_frame"
        pc2_msg.header = header  #added
        #note / you might need to specify the dimention to be 640*360
        #visualize the pointcliud output of this node to make sure it's right
        pc2_msg.height = pc_data.shape[0] 
        pc2_msg.width  = pc_data.shape[1] 
        pc2_msg.fields = fields
        pc2_msg.is_bigendian = False
        pc2_msg.point_step = 12  # 3 * 4 bytes for x, y, z (float32)
        pc2_msg.row_step = pc2_msg.point_step * pc_data.shape[0]   
        pc2_msg.data = pc_data_bytes
        pc2_msg.is_dense = True

        
        
        # Publish the data
        point_cloud_pub.publish(pc2_msg) #publish the point cloud first
        
        left_image_pub.publish(left_img_msg)
        right_image_pub.publish(right_img_msg)
        #left_hires_pub.publish(hires_img_msg)
        
        
        calibration_params_rect = zed.get_camera_information().camera_configuration.calibration_parameters_raw
        fx=calibration_params_rect.left_cam.fx
        fy=calibration_params_rect.left_cam.fy
        cx=calibration_params_rect.left_cam.cx
        cy=calibration_params_rect.left_cam.cy
        print(fx,fy,cx,cy)
        print("Res",zed.get_camera_information().camera_configuration.resolution.width,"x",zed.get_camera_information().camera_configuration.resolution.height)


    rate.sleep()

