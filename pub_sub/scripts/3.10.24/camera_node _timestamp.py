#!/usr/bin/env python3
import sensor_msgs.point_cloud2 as pc2
import pyzed.sl as sl
import rospy
import std_msgs
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from collections import deque
from custom_srvs.srv import PointCloudQuery, PointCloudQueryResponse

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

# Buffer to hold point cloud data with their timestamps
pointcloud_buffer = deque(maxlen=5)

def handle_pointcloud_query(req):
    global pointcloud_buffer
    zed_u_mid_left, zed_v_mid_left, target_timestamp = req.u, req.v, req.timestamp
    
    # Find matching point cloud data by looping over the buffer
    for data in pointcloud_buffer:
        if data['timestamp'] == target_timestamp:
            zed_p = data['pointcloud'].get_value(zed_u_mid_left, zed_v_mid_left)
            
            response = PointCloudQueryResponse()
            response.x = zed_p[1][0]
            response.y = zed_p[1][1]
            response.z = zed_p[1][2]
            response.timestamp = data['timestamp']
            return response

    # If no match found, return an error response
    rospy.logwarn("No matching point cloud data found for the requested timestamp.")
    return PointCloudQueryResponse(x=float('nan'), y=float('nan'), z=float('nan'), timestamp=rospy.Time(0))

# Register the service
pointcloud_service = rospy.Service('pointcloud_query', PointCloudQuery, handle_pointcloud_query)

print("Camera Node Running...")

while not rospy.is_shutdown():
    if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
        # Retrieve the image and point cloud
        zed.retrieve_image(image, sl.VIEW.LEFT)
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ)

        # Get the timestamp from the ZED camera
        current_timestamp = zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)

        # Save point cloud and timestamp in buffer
        pointcloud_buffer.append({'timestamp': current_timestamp, 'pointcloud': point_cloud})

        # Convert to ROS message and publish
        img_msg = bridge.cv2_to_imgmsg(image.get_data(), encoding="rgba8")
        img_msg.header.stamp = rospy.Time.from_sec(current_timestamp.get_seconds())
        image_pub.publish(img_msg)

    rospy.sleep(0.1)
