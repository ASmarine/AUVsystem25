#!/usr/bin/env python3
import sensor_msgs.point_cloud2 as pc2
import pyzed.sl as sl
import rospy
import std_msgs
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse
from pub_sub.srv import PointCloudValue, PointCloudValueResponse
import cv2
from cv_bridge import CvBridge

rospy.init_node('camera_node')

image_pub = rospy.Publisher('/zed/image_raw', Image, queue_size=10)
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
point_cloud = sl.Mat()
bridge = CvBridge()

print("Camera Node Running...")

# Service callback for retrieving point cloud value
def handle_pointcloud_request(req):
    global point_cloud
    zed_u_mid_left = req.u
    zed_v_mid_left = req.v
    zed_P = point_cloud.get_value(zed_u_mid_left, zed_v_mid_left)
    response = PointCloudValueResponse()
    response.x = zed_P[1][0]
    response.y = zed_P[1][1]
    response.z = zed_P[1][2]
    return response

# Advertise the point cloud service
rospy.Service('/get_pointcloud_value', PointCloudValue, handle_pointcloud_request)

# Inside your loop where you grab data from the ZED camera
while not rospy.is_shutdown():
    if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
        # Retrieve the image and point cloud
        zed.retrieve_image(image, sl.VIEW.LEFT)
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ)

        # Convert to ROS messages
        img_msg = bridge.cv2_to_imgmsg(image.get_data(), encoding="passthrough")

        # Publish the image
        image_pub.publish(img_msg)

    rospy.sleep(0.05)


#done for now
#done by Amr