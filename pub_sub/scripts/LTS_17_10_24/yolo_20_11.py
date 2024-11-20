#!/usr/bin/env python3
import joblib
import rospy
import torch
from sensor_msgs.msg import Image, PointCloud2
from pub_sub.msg import Multi_instance, instance
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2
import sensor_msgs.point_cloud2 as pc2
import time
from collections import deque

# Initialize ROS node
rospy.init_node('detection_node')
rate = rospy.Rate(15)  # 15 Hz

# Initialize YOLO model
torch.cuda.set_device(0)
device = torch.device('cuda')
model_weights_path = "/home/jetsonx/catkin_ws/src/pub_sub/scripts/best.pt"
model = YOLO(model_weights_path)
model.to(device=device)

# Load SVM model
correction_svm_model = joblib.load('/home/jetsonx/catkin_ws/src/pub_sub/scripts/depth_correction_model.pkl')
print("Detection Node Running...")

# Initialize point cloud and image deques
pc = deque(maxlen=15)
ros_images = deque(maxlen=15)

# Callback for point cloud data
def pointcloud_callback(msg):
    print("pc callback")
    global pc, points_array
    width = msg.width
    height = msg.height 
    points_array = np.frombuffer(msg.data, dtype=np.float32).reshape((height, width, 3))[:, :, :3]
    pc.append(points_array)

# Callback for image data
def image_callback(img_msg):
    print("image callback")
    global ros_images
    ros_images.append(img_msg)  # Append the ROS Image message to the deque

# Get instance name based on instance number
def getInstanceName(instance_number):
    labels = ['gate', 'path marker', 'badge', 'gun', 'box', 'hand', 'barrel', 'note', 'phone', 'bottle', 'gman', 'bootlegger', 'axe', 'dollar', 'beer']
    return labels[instance_number]

# YOLO model function
def yolo_model():
    print("Running YOLO detection...")
    try:
        global ros_images, pc
        
        if len(ros_images) == 0 or len(pc) == 0:
            rospy.logwarn("Image or point cloud data not available.")
            return

        # Get the ROS Image message from the deque
        ros_image_msg = ros_images[0]

        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(ros_image_msg, "passthrough")
        cvt_img = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2RGB)

        # Run YOLO detection
        result = model.predict(source=cvt_img, show=False, conf=0.30)

        perc23_msg = Multi_instance()
        perc23_msg.header.stamp = ros_image_msg.header.stamp  # Use the original ROS message timestamp

        num_of_instances = result[0].boxes.data.size()[0]

        if num_of_instances == 0:
            det_pub.publish(perc23_msg)
            return

        zed_P = pc[0]

        for i in range(num_of_instances):
            zed_x_top_left = int(result[0].boxes.data[i][0].item())
            zed_y_top_left = int(result[0].boxes.data[i][1].item())
            zed_x_bottom_right = int(result[0].boxes.data[i][2].item())
            zed_y_bottom_right = int(result[0].boxes.data[i][3].item())
            zed_u_mid_left = int((zed_x_top_left + zed_x_bottom_right) / 2)
            zed_v_mid_left = int((zed_y_top_left + zed_y_bottom_right) / 2)

            instance_type = getInstanceName(int(result[0].boxes.data[i][5].item()))
            confidence_level = result[0].boxes.data[i][4].item()

            # Extract X, Y, Z coordinates from the point cloud based on the pixel position
            X = zed_P[zed_v_mid_left, zed_u_mid_left, 0]
            Y = zed_P[zed_v_mid_left, zed_u_mid_left, 1]
            Z = zed_P[zed_v_mid_left, zed_u_mid_left, 2]

            # Check for NaN values
            """
            if np.isnan(X) or np.isnan(Y) or np.isnan(Z):
                rospy.logwarn("NaN values detected in point cloud data")
                continue
            """
            perc23_msg.data.append(instance())
            perc23_msg.data[i].ID = instance_type
            perc23_msg.data[i].u = zed_u_mid_left
            perc23_msg.data[i].v = zed_v_mid_left

            # Correct Z using the SVM model
            corrected_z = correction_svm_model.predict(np.array([X / 1000, Y / 1000, Z / 1000]).reshape(1, -1))
            perc23_msg.data[i].x = X  # / 1000
            perc23_msg.data[i].y = Y  # / 1000
            perc23_msg.data[i].z = Z
            perc23_msg.data[i].confidence = corrected_z[0]
            '''
            with open('/home/jetsonx/catkin_ws/src/Perception2_Tum_Data_3.tum', 'a') as file:
                file.write(f"original {X} {Y} {Z}\n")
                file.write(f"corrected {X} {Y} {corrected_z[0]}\n")
            '''
    except Exception as e:
        rospy.logerr(f"Error in YOLO model: {e}")

    det_pub.publish(perc23_msg)

# Initialize CvBridge for image conversion
bridge = CvBridge()
det_pub = rospy.Publisher('/landmarks', Multi_instance, queue_size=100)

# Subscribe to the image and point cloud topics
rospy.Subscriber('/zed/zed_node/left/image_rect_color', Image, image_callback)
rospy.Subscriber('/zed/PC', PointCloud2, pointcloud_callback)

# Run YOLO model at approximately 15 Hz using a timer
timer = rospy.Timer(rospy.Duration(1.0/14), lambda event: yolo_model())

rospy.spin()
