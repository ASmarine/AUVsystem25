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


# Initialize the node
rospy.init_node('camera_node')


# Create the publishers
left_image_pub = rospy.Publisher('/zed/zed_node/left/image_rect_color', Image, queue_size=10)
right_image_pub = rospy.Publisher('/zed/zed_node/right/image_rect_color', Image, queue_size=10)
point_cloud_pub = rospy.Publisher('/zed/PC', PointCloud2, queue_size=5)


# Initialize the ZED camera
zed = sl.Camera()
rate = rospy.Rate(15)  # 15 Hz
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.VGA
init_params.depth_mode = sl.DEPTH_MODE.ULTRA
init_params.coordinate_units = sl.UNIT.MILLIMETER
init_params.camera_fps = 15
init_params.camera_disable_self_calib = True
init_params.optional_settings_path="/usr/local/zed/settings/SN16133.conf"

# Open the camera and check for errors
runtime_params = sl.RuntimeParameters()
err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    print('Camera initialization failed')
    exit(-1)





# Create the image and point cloud objects
left_image = sl.Mat()
right_image = sl.Mat()
point_cloud = sl.Mat()
bridge = CvBridge()


# Create the PointCloud2 message fields
fields = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1)
]

print("Camera Node Running...")
while not rospy.is_shutdown():
    if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:

        # Retrieve the right image, left image, and point cloud
        zed.retrieve_image(left_image, sl.VIEW.LEFT)
        zed.retrieve_image(right_image, sl.VIEW.RIGHT)
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ)

        
        # Convert to ROS messages
        left_img = bridge.cv2_to_imgmsg(left_image.get_data(), encoding="passthrough")
        right_img = bridge.cv2_to_imgmsg(right_image.get_data(), encoding="passthrough")
        pc_data = point_cloud.get_data()
        pc_data = pc_data[:, :, :3]
        pc_data = pc_data.astype(np.float32)
        pc_data_bytes = pc_data.tobytes()
        
        
        # Create a PointCloud2 and image message
        pc2_msg = PointCloud2()
        left_img_msg = Image()  #added
        right_img_msg = Image() #added
        
        #Create the header
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "zed_frame"+str(header.seq) #edited
        
        pc2_msg.header = header  
        left_img_msg.header = header    #added
        right_img_msg.header = header   #added
        
        #Set the height, width, fields, and other metadata
        #hight
        pc2_msg.height= pc_data.shape[0] 
        left_img_msg.height = left_image.get_height() #added 
        right_img_msg.height =right_image.get_height() #added
        #width
        pc2_msg.width= pc_data.shape[1]  
        left_img_msg.width=  left_image.get_width()  #added
        right_img_msg.width = right_image.get_width()  #added
        #fields
        left_img_msg.step = left_image.get_step() #added
        right_img_msg.step = right_image.get_step() #added
        pc2_msg.fields = fields
        pc2_msg.is_bigendian = left_img_msg.is_bigendian = right_img_msg.is_bigendian = False   #added
        pc2_msg.point_step = 12  # 3 * 4 bytes for x, y, z (float32)
        pc2_msg.row_step = pc2_msg.point_step * pc_data.shape[0]   
        pc2_msg.is_dense = True
        
        #Set the data
        pc2_msg.data = pc_data_bytes
        left_img_msg.data = left_img
        right_img_msg.data = right_img
        
        
        #Publish the data
        point_cloud_pub.publish(pc2_msg) #publish the point cloud first
        left_image_pub.publish(left_img_msg)
        right_image_pub.publish(right_img_msg)


    rate.sleep()

