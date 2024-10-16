#!/usr/bin/env python3

import rospy
import torch
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from pub_sub.msg import Multi_instance, instance
from ultralytics import YOLO
import joblib
import numpy as np
import sensor_msgs.point_cloud2 as pc2

rospy.init_node('detection_node')

# Initialize YOLO model
torch.cuda.set_device(0)
device = torch.device('cuda')
model_weights_path = "/home/jetsonx/catkin_ws/src/pub_sub/scripts/best.pt"
model = YOLO(model_weights_path)
model.to(device=device)

# Subscriber setup
bridge = CvBridge()
det_pub = rospy.Publisher('/landmarks', Multi_instance, queue_size=10)

perc23_msg = Multi_instance()  






# Function to get instance name based on the instance number
def getInstanceName(instance_number):
    labels = ['gate', 'path marker', 'badge', 'gun', 'box', 'hand', 'barrel', 'note', 'phone', 'bottle', 'gman', 'bootlegger', 'axe', 'dollar', 'beer']
    return labels[instance_number]





def image_callback(img_msg):
    

    cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    result = model.predict(source=cv_image, show=False, conf=0.55)

    # perc23_msg = Multi_instance()
    num_of_instances = result[0].boxes.data.size()[0]

    for i in range(num_of_instances):
      
        zed_x_top_left = int(result[0].boxes.data[i][0].item())
        zed_y_top_left = int(result[0].boxes.data[i][1].item())
        zed_x_bottom_right = int(result[0].boxes.data[i][2].item())
        zed_y_bottom_right = int(result[0].boxes.data[i][3].item())
        zed_u_mid_left = int((zed_x_top_left + zed_x_bottom_right) / 2.0)
        zed_v_mid_left = int((zed_y_top_left + zed_y_bottom_right) / 2.0) 
        zed_P = pcloud.get_value(zed_u_mid_left, zed_v_mid_left)

        instance_type = getInstanceName(int(result[0].boxes.data[i][5].item()))
        confidence_level = result[0].boxes.data[i][4].item()

        X_depth = zed_P[1][0]
        Y_depth = zed_P[1][1]
        Z_depth = zed_P[1][2]
        perc23_msg.data.append(instance())        
        if np.isnan(X_depth) or np.isnan(Y_depth) or np.isnan(Z_depth):
            X_depth = 50
            Y_depth = 50 
            Z_depth = 50 

        # This line appends a new instance of an object to the data list within the perc23_msg message object
        perc23_msg.data[i].ID = instance_type     
        # assigns the value of instance_type to the ID attribute of the i-th element in the data list. 
        # It sets the ID of the instance to the value representing the type or class of the detected object.
        
        # If you want to display center pixels of bounding boxes, uncomment the following two lines
        perc23_msg.data[i].u = zed_u_mid_left  # X-coordinate of the center point of the bounding box
        perc23_msg.data[i].v = zed_v_mid_left  # Y-coordinate of the center point of the bounding box
        
        correction_svm_model = joblib.load('/home/jetsonx/catkin_ws/src/pub_sub/scripts/depth_correction_model.pkl')
        corrected_z = correction_svm_model.predict(np.array([X_depth / 1000, Y_depth / 1000, Z_depth / 1000]).reshape(1, -1))

        perc23_msg.data[i].x = X_depth / 1000  # X-depth of the instance
        perc23_msg.data[i].y = Y_depth / 1000  # Y-depth of the instance
        perc23_msg.data[i].z = corrected_z      # Z-depth of the instance
        perc23_msg.data[i].confidence = Z_depth / 1000  # Confidence level

    det_pub.publish(perc23_msg)





def pointcloud2_to_sl_mat(point_cloud_msg):
    

    # Extract points from the ROS PointCloud2 message
    points = pc2.read_points(point_cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
    points_array = np.array(list(points), dtype=np.float32)  # Shape: (N, 3)

    # Create a sl.Mat object
    pcloud = sl.Mat()
    pcloud.init(720, 1280, sl.MAT_TYPE.F32_C1)  # Adjust height and width as needed

    # Fill sl.Mat with point cloud data
    pcloud.set_data(points_array)

    return pcloud





# ROS subscriber setup
def pointcloud_callback(point_cloud_msg):
    


    global pcloud
    pcloud = pointcloud2_to_sl_mat(point_cloud_msg)
    # Now you can use pcloud for your application

# Subscribe to the image and point cloud topics
rospy.Subscriber('/zed/image_raw', Image, image_callback)
rospy.Subscriber('/zed/point_cloud', PointCloud2, pointcloud_callback) 

print("Detection Node Running...")
rospy.spin()
