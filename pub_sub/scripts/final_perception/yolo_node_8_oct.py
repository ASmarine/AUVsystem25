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

rospy.init_node('detection_node')
rate = rospy.Rate(15)  # 15 Hz

# Initialize YOLO model
device = torch.device('cpu')  # Set to 'cpu' for now, you can switch to GPU later
model_weights_path = "/home/hussein/AUV_ws/src/pub_sub/best.pt"
model = YOLO(model_weights_path)
model.to(device=device)

# Load SVM model
correction_svm_model = joblib.load('/home/hussein/AUV_ws/src/pub_sub/depth_correction_model.pkl')
print("Detection Node Running...")

# Initialize point cloud and image lists
pc = []
image = []

# Callback for point cloud data
def pointcloud_callback(msg):  #4.3e-5ms
    #t1=time.time()
    print("pc callback")
    global pc, points_array
    width = msg.width
    height = msg.height 
    points_array = np.frombuffer(msg.data, dtype=np.float32).reshape((height, width, 3))[:, :, :3]   #when using wrapper change the number of chanles to 4
    pc.append(points_array)
    #t2=time.time()
    #print("pc callback time " + str(t2-t1))

# Callback for image data
def image_callback(img_msg):  #8.8e-5ms
    #t1=time.time()
    print("image callback")
    global image
    cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    image.append(cv_image)
    #t2=time.time()
    #print("image callback time " + str(t2-t1))

# Get instance name based on instance number
def getInstanceName(instance_number):
    labels = ['gate', 'path marker', 'badge', 'gun', 'box', 'hand', 'barrel', 'note', 'phone', 'bottle', 'gman', 'bootlegger', 'axe', 'dollar', 'beer']
    return labels[instance_number]

# YOLO model function
def yolo_model(): #it takes 360ms
    t1=time.time()
    print("Running YOLO detection...")
    try: #because of a problem in data[i].ID
        global image, pc
        if len(image) == 0 or len(pc) == 0:
            rospy.logwarn("Image or point cloud data not available.")
            return

        

        # Convert ROS message to OpenCV image
        cvt_img = cv2.cvtColor(image[0], cv2.COLOR_RGBA2RGB)
        result = model.predict(source=cvt_img, show=False, conf=0.55)  # Set show=False to avoid rendering the image

        perc23_msg = Multi_instance()  # Initialize message
        num_of_instances = result[0].boxes.data.size()[0]

        if num_of_instances == 0:
            det_pub.publish(perc23_msg)  # Publish an empty message
            image.pop(0)
            pc.pop(0)
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
            if np.isnan(X) or np.isnan(Y) or np.isnan(Z):
                rospy.logwarn("NaN values detected in point cloud data")
                continue

            perc23_msg.data.append(instance())
            perc23_msg.data[i].ID = instance_type
            perc23_msg.data[i].u = zed_u_mid_left
            perc23_msg.data[i].v = zed_v_mid_left

            # Correct Z using the SVM model
            corrected_z = correction_svm_model.predict(np.array([X / 1000, Y / 1000, Z / 1000]).reshape(1, -1))
            perc23_msg.data[i].x = X / 1000
            perc23_msg.data[i].y = Y / 1000
            perc23_msg.data[i].z = corrected_z[0]
            perc23_msg.data[i].confidence = confidence_level
    except Exception as e:
        rospy.logerr(f"Error in YOLO model: {e}")
        
    det_pub.publish(perc23_msg)
    image.pop(0)
    pc.pop(0)
    t2=time.time()
    print(perc23_msg)
    print("yolo time " + str(1000*(t2-t1)))
    

# Initialize CvBridge for image conversion
bridge = CvBridge()
det_pub = rospy.Publisher('/landmarks', Multi_instance, queue_size=100)

# Subscribe to the image and point cloud topics
rospy.Subscriber('/zed/image_raw', Image, image_callback)
rospy.Subscriber('/zed/PC', PointCloud2, pointcloud_callback)

# Run YOLO model at 15 Hz using a timer
timer = rospy.Timer(rospy.Duration(1.0 /7 ), lambda event: yolo_model())
rospy.spin()
