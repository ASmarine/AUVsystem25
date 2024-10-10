#!/usr/bin/env python3
import joblib
import rospy
import torch
from sensor_msgs.msg import Image
from pub_sub.msg import Multi_instance, instance
from pub_sub.srv import PointCloudValue
from cv_bridge import CvBridge
from ultralytics import YOLO
import time
import numpy as np

rospy.init_node('detection_node')

# Initialize YOLO model
torch.cuda.set_device(0)
device = torch.device('cuda')
model_weights_path = "/home/jetsonx/catkin_ws/src/pub_sub/scripts/best.pt"
model = YOLO(model_weights_path)
model.to(device=device)

# Load SVM model once
correction_svm_model = joblib.load('/home/jetsonx/catkin_ws/src/pub_sub/scripts/depth_correction_model.pkl')

# Subscriber setup
bridge = CvBridge()
det_pub = rospy.Publisher('/landmarks', Multi_instance, queue_size=10)

print("Detection Node Running...")

# Function to get instance name based on the instance number
def getInstanceName(instance_number):
    labels = ['gate', 'path marker', 'badge', 'gun', 'box', 'hand', 'barrel', 'note', 'phone', 'bottle', 'gman', 'bootlegger', 'axe', 'dollar', 'beer']
    return labels[instance_number]

def image_callback(img_msg):

    #Convert ros msg to cvikage

    start_fetching=time.time()


    cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")

    finsh_fetching=time.time()


    result = model.predict(source=cv_image, show=False, conf=0.55)

    finsh_predict=time.time()


    print("fetching_time"+(-start_fetching+finsh_fetching)+"     ")
    print("bridge_time"+(finsh_predict-finsh_fetching)+"\n")

    perc23_msg = Multi_instance()  # Initialize message
    num_of_instances = result[0].boxes.data.size()[0]

    if num_of_instances == 0:
        det_pub.publish(perc23_msg)  # Publish an empty message
        return

    # Service proxy for getting point cloud values
    rospy.wait_for_service('/get_pointcloud_value')
    try:
        pointcloud_service = rospy.ServiceProxy('/get_pointcloud_value', PointCloudValue)

        for i in range(num_of_instances):
            zed_x_top_left = int(result[0].boxes.data[i][0].item())
            zed_y_top_left = int(result[0].boxes.data[i][1].item())
            zed_x_bottom_right = int(result[0].boxes.data[i][2].item())
            zed_y_bottom_right = int(result[0].boxes.data[i][3].item())
            zed_u_mid_left = int((zed_x_top_left + zed_x_bottom_right) / 2.0)
            zed_v_mid_left = int((zed_y_top_left + zed_y_bottom_right) / 2.0)

            # Get point cloud value from the service
            point_response = pointcloud_service(zed_u_mid_left, zed_v_mid_left)

            instance_type = getInstanceName(int(result[0].boxes.data[i][5].item()))
            confidence_level = result[0].boxes.data[i][4].item()

            X_depth = point_response.x
            Y_depth = point_response.y
            Z_depth = point_response.z

            if np.isnan(X_depth) or np.isnan(Y_depth) or np.isnan(Z_depth):
                continue  # Skip this instance if depth values are NaN

            perc23_msg.data.append(instance())
            perc23_msg.data[i].ID = instance_type
            perc23_msg.data[i].u = zed_u_mid_left
            perc23_msg.data[i].v = zed_v_mid_left

            corrected_z = correction_svm_model.predict(np.array([X_depth / 1000, Y_depth / 1000, Z_depth / 1000]).reshape(1, -1))
            perc23_msg.data[i].x = X_depth / 1000
            perc23_msg.data[i].y = Y_depth / 1000
            perc23_msg.data[i].z = corrected_z[0]  # Access the first element
            perc23_msg.data[i].confidence = confidence_level 

        det_pub.publish(perc23_msg)

    except rospy.ServiceException as e:
        rospy.logwarn(f"Service call failed: {e}")

# Subscribe to the image topic
rospy.Subscriber('/zed/image_raw', Image, image_callback)

rospy.spin()
