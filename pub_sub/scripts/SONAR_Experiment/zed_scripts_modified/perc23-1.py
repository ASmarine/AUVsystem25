#!/usr/bin/env python3

#######################################################################################################################
#                     NEW      VERSION
#######################################################################################################################

from ultralytics import YOLO
import cv2
from datetime import datetime
import csv
import rospy
from pub_sub.msg import instance
from pub_sub.msg import Multi_instance

#Node initialization
rospy.init_node('perc23')    # Ros Node                                               
pub = rospy.Publisher('/landmarks', Multi_instance, queue_size = 10) # Publisher 
# Custom message
perc23_msg = Multi_instance()  

##############################################
# Set Camera Resolution
##############################################
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_file_name = f"{timestamp}.csv"
    
    # Header names for the CSV
header = ['u_left_uw', 'v_left_uw','u_right_uw','v_right_uw', 'X_depth', 'Y_depth', 'Z_depth']


##############################################
# Set Camera Resolution
##############################################

#################################################
# Stereo Calibration and rectification
#################################################

# Camera parameters to undistort and rectify images

cv_file = cv2.FileStorage()  # The FileStorage class is used for reading and writing XML/YAML files.
cv_file.open('/media/philo/Philo/AS Marine/AAPOOL/Visualize Disparity/Single Pixel/stereoMapbate.xml', cv2.FileStorage_READ) # open the XML file. It prepares the file for reading the camera parameters.

#These lines retrieves the matrix data stored in the XML file under the nodes names.
# It uses the getNode() method to access the specific node and the mat() method to retrieve the matrix data.
stereoMapL_x = cv_file.getNode('stereoMapL_x').mat()
stereoMapL_y = cv_file.getNode('stereoMapL_y').mat()
stereoMapR_x = cv_file.getNode('stereoMapR_x').mat()
stereoMapR_y = cv_file.getNode('stereoMapR_y').mat()

# This line retrieves the matrix data stored in the XML file under the node name "q". 
# The matrix Q likely represents the disparity-to-depth mapping matrix used for rectification and 3D reconstruction.

Q = cv_file.getNode('q').mat() #--> Calibration parameters matric (contains fucking parameters!!)

# Q = [1 	0	 0	 -cx
#      0 	1	 0	 -cy
#      0 	0	 0	  f
#      0 	0 	1/B        0] 

#################################################
# Summary:
#################################################
# In summary, the code reads camera parameter data from the "stereoMapbate.xml" file,
# including stere        os.makedirs(left_rectified_path)
# and the disparity-to-depth mapping matrix (Q). 
# These parameters are commonly used for image undistortion, rectification, and depth estimation in stereo vision applications.

####################################################################################################################################################################################


#############################################
# Model initialization
#############################################

model_weights_path = "/home/philo/AS_marine_WS/src/pub_sub/scripts/best.pt"    # This line assigns the file path to the variable.
model = YOLO(model_weights_path)
model2 = YOLO(model_weights_path)                                                            # passing the weights file to YOLO Model.

# Define the Labels by taking the instance Number and return the Instance itself:
def getInstanceName(instance_number):
    
    labels = ['gate','path marker','badge','gun','box','hand','barrel','note','phone','bottle','gman','bootlegger','axe','dollar','beer']
    return labels[instance_number] 



while not rospy.is_shutdown():
    succes, full_img = cap.read()
    img_left = full_img[:,0:int(full_img.shape[1]/2)] # This line extracts the left half of the full_img by slicing the image array.
    img_right = full_img[:, int(full_img.shape[1]/2):] # This line extracts the right half of the full_img by slicing the image array.

  # Note that: 
    # The purpose of splitting the image into left and right parts could be related to stereo vision or multi-camera setups. 
    # Each resulting image (img_left and img_right) likely represents the view from a different camera or perspective,
    # which can be used for further processing such as stereo matching, depth estimation, or any other application that benefits from having separate views of the scene.

    # Undistort and rectify images
    imgR = cv2.remap(img_right, stereoMapR_x, stereoMapR_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0) # This line applies remapping to the img_right image using the cv2.remap() function.
    imgL = cv2.remap(img_left, stereoMapL_x, stereoMapL_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0) #This line applies remapping to the img_left image using the cv2.remap() function.
    

    # Crop Image to Classify
    # The code snippet involves cropping an image (imgL) to a specific region of interest (ROI) for classification.

    start_crop_v = 24  # The starting y-coordinate of the cropping region
    len_crop_v = 667   # the length of the cropping region in the y-direction 

    start_crop_u = 54  # The starting x-coordinate of the cropping region
    len_crop_u = 1186  # the length of the cropping region in the x-direction 

    # This line performs the actual cropping of the imgL image. 
    # It uses array slicing to extract the specific region defined by the starting coordinates and lengths specified earlier.
    # The result_lefting cropped image is assigned to the variable cropped_left_img

    cropped_left_img = imgL[start_crop_v : start_crop_v + len_crop_v, start_crop_u : start_crop_u + len_crop_u] # The purpose of this cropping is to isolate a specific portion of the image that is relevant for classification.
    cropped_right_img = imgR[start_crop_v : start_crop_v + len_crop_v, start_crop_u : start_crop_u + len_crop_u]
    result_left = model.predict(source=cropped_left_img, show = True)
    cv2.waitKey(2000)
    result_right = model2.predict(source=cropped_right_img, show = True)
    cv2.waitKey(2000)
    cv2.destroyAllWindows()

    num_of_instances_left = result_left[0].boxes.data.size()[0]
    num_of_instances_right = result_right[0].boxes.data.size()[0]
    calculations_flag = 0
    if((num_of_instances_left == num_of_instances_right) and (num_of_instances_left == 1) and ((result_left[0].boxes.data[0][5].item()) == 11)):
        calculations_flag = 1
    #################################################
        # Summary:
        #################################################
        # this code snippet involves predicting objects using a model, computing a disparity map for depth estimation, and obtaining the number of instances detected in the prediction result_lefts.

        ####################################################################################################################################################################################
        for i in range(num_of_instances_left):
            
            # The following 4 lines of code snippet involves calculating the coordinates of the top-left and bottom-right corners of a bounding box within the cropped image region.
            
            u_top_left_left = (int)(result_left[0].boxes.data[i][0].item()) + start_crop_u    # calculates the x-coordinate of the top-left corner of the bounding box.
            v_top_left_left = (int)(result_left[0].boxes.data[i][1].item()) + start_crop_v    # calculates the y-coordinate of the top-left corner of the bounding box.

            u_bottom_right_left = (int)(result_left[0].boxes.data[i][2].item()) + start_crop_u   # calculates the x-coordinate of the bottom-right corner of the bounding box
            v_bottom_right_left = (int)(result_left[0].boxes.data[i][3].item()) + start_crop_v   # calculates the y-coordinate of the bottom-right corner of the bounding box.  

            # retrieves the confidence level associated with a specific bounding box from the result_left object. 
            confidence_level_left = result_left[0].boxes.data[i][4].item()  # This confidence level can be used to assess the reliability or strength of the object detection or classification result_lefts associated with the specific bounding box.

            instance_type_left = getInstanceName((int)(result_left[0].boxes.data[i][5].item())) #The purpose of this line is to retrieve the specific label or name associated with the detected object within the bounding box. It allows for identifying and referring to the detected instances using their corresponding names or labels.

            # calculates the coordinates of the center point of a bounding box based on the coordinates of its top-left and bottom-right corners. 
            u_left = (int)((u_top_left_left + u_bottom_right_left)/2.0)  # calculates the x-coordinate of the center point of the bounding box. 
            v_left = (int)((v_top_left_left + v_bottom_right_left)/2.0)  # calculates the y-coordinate of the center point of the bounding box.
            
            # appending data to a message object perc23_msg. It assigns values to different attributes of the appended data.     

        for i in range(num_of_instances_right):
            
            # The following 4 lines of code snippet involves calculating the coordinates of the top-left and bottom-right corners of a bounding box within the cropped image region.
            
            u_top_left_right = (int)(result_right[0].boxes.data[i][0].item()) + start_crop_u    # calculates the x-coordinate of the top-left corner of the bounding box.
            v_top_left_right = (int)(result_right[0].boxes.data[i][1].item()) + start_crop_v    # calculates the y-coordinate of the top-left corner of the bounding box.

            u_bottom_right_right = (int)(result_right[0].boxes.data[i][2].item()) + start_crop_u   # calculates the x-coordinate of the bottom-right corner of the bounding box
            v_bottom_right_right = (int)(result_right[0].boxes.data[i][3].item()) + start_crop_v   # calculates the y-coordinate of the bottom-right corner of the bounding box.  

            # retrieves the confidence level associated with a specific bounding box from the result_left object. 
            confidence_level_right = result_right[0].boxes.data[i][4].item()  # This confidence level can be used to assess the reliability or strength of the object detection or classification result_lefts associated with the specific bounding box.

            instance_type_right = getInstanceName((int)(result_right[0].boxes.data[i][5].item())) #The purpose of this line is to retrieve the specific label or name associated with the detected object within the bounding box. It allows for identifying and referring to the detected instances using their corresponding names or labels.

            # calculates the coordinates of the center point of a bounding box based on the coordinates of its top-left and bottom-right corners. 
            u_right = (int)((u_top_left_right + u_bottom_right_right)/2.0)  # calculates the x-coordinate of the center point of the bounding box. 
            v_right = (int)((v_top_left_right + v_bottom_right_right)/2.0)  # calculates the y-coordinate of the center point of the bounding box.
            
            # appending data to a message object perc23_msg. It assigns values to different attributes of the appended data.     


    if(calculations_flag == 1):
        calculations_flag = 0
        disparity = u_left - u_right
        cx = -Q[0][3]
        cy = -Q[1][3]
        focal = 800
        B = 120
        Z_depth = B*focal/disparity
        X_depth = (u_left - 644.275) * 120/disparity
        Y_depth = (v_left - 332.0365) * 120/disparity
        data = [[u_left, v_left,u_right,v_right, X_depth, Y_depth, Z_depth]]

        with open('csv_file_name', mode='a', newline='') as file:
            writer = csv.writer(file)
        
            if file.tell() == 0:
                writer.writerow(header)

        
            # Write the data rows
            writer.writerows(data)
    
#######################################################################################################################
#                      OLD       VERSION
#######################################################################################################################

# # libraries Include
# import rospy
# from pub_sub.msg import instance
# from pub_sub.msg import Multi_instance
# from ultralytics import YOLO
# import cv2
# import time
# import numpy as np
# import os
# #Node initialization
# rospy.init_node('perc23')    # Ros Node                                               
# pub = rospy.Publisher('/landmarks', Multi_instance, queue_size = 10) # Publisher 
# # Custom message
# perc23_msg = Multi_instance()  
# ##############################################
# # Set Camera Resolution
# ##############################################
# # cap = cv2.VideoCapture(0)
# # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
# # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# #################################################
# # Stereo Calibration and rectification
# #################################################

# # Camera parameters to undistort and rectify images

# cv_file = cv2.FileStorage()  # The FileStorage class is used for reading and writing XML/YAML files.
# cv_file.open('/home/philo/AS_marine_WS/src/pub_sub/scripts/stereoMapbate.xml', cv2.FileStorage_READ) # open the XML file. It prepares the file for reading the camera parameters.


# left_folder_path = "/home/philo/Downloads/data/left3"

# right_folder_path = "/home/philo/Downloads/data/right3"


# left_image_paths = [os.path.join(left_folder_path, f) for f in os.listdir(left_folder_path) if f.endswith('.jpg')]

# right_image_paths = [os.path.join(right_folder_path, g) for g in os.listdir(right_folder_path) if g.endswith('.jpg')]

# #These lines retrieves the matrix data stored in the XML file under the nodes names.
# # It uses the getNode() method to access the specific node and the mat() method to retrieve the matrix data.
# stereoMapL_x = cv_file.getNode('stereoMapL_x').mat()
# stereoMapL_y = cv_file.getNode('stereoMapL_y').mat()
# stereoMapR_x = cv_file.getNode('stereoMapR_x').mat()
# stereoMapR_y = cv_file.getNode('stereoMapR_y').mat()

# # This line retrieves the matrix data stored in the XML file under the node name "q". 
# # The matrix Q likely represents the disparity-to-depth mapping matrix used for rectification and 3D reconstruction.

# Q = cv_file.getNode('q').mat() #--> Calibration parameters matric (contains fucking parameters!!)

# # Q = [1 	0	 0	 -cx
# #      0 	1	 0	 -cy
# #      0 	0	 0	  f
# #      0 	0 	1/B        0] 

# #################################################
# # Summary:
# #################################################
# # In summary, the code reads camera parameter data from the "stereoMapbate.xml" file,
# # including stereo map data for left and right images (stereoMapL_x, stereoMapL_y, stereoMapR_x, stereoMapR_y) 
# # and the disparity-to-depth mapping matrix (Q). 
# # These parameters are commonly used for image undistortion, rectification, and depth estimation in stereo vision applications.

# ####################################################################################################################################################################################


# #############################################
# # Model initialization
# #############################################

# model_weights_path = "/home/philo/AS_marine_WS/src/pub_sub/scripts/best.pt"    # This line assigns the file path to the variable.
# model = YOLO(model_weights_path)                                                            # passing the weights file to YOLO Model.

# # Define the Labels by taking the instance Number and return the Instance itself:
# def getInstanceName(instance_number):
    
#     labels = ['gate','path marker','badge','gun','box','hand','barrel','note','phone','bottle','gman','bootlegger','axe','dollar','beer']
#     return labels[instance_number] 

# # left_index = 0
# # right_index = 0
# rakam = 1
# while not rospy.is_shutdown() :
#     #This line captures a frame from a video capture source (cap) using the read() method. 
#     # The return value success indicates whether the frame was successfully read, while full_img represents the captured image frame.
#     # succes, full_img = cap.read()
#     # img_left = cv2.imread(left_image_paths[left_index]) # This line extracts the left half of the full_img by slicing the image array.
#     # img_right = cv2.imread(right_image_paths[right_index]) # This line extracts the right half of the full_img by slicing the image array.
#     img_left = cv2.imread('/home/philo/Downloads/data/left3/'+str(rakam)+'.jpg')
#     img_right = cv2.imread('/home/philo/Downloads/data/right3/'+str(rakam)+'.jpg')

#     # left_index += 1
#     # right_index +=1
#     # if left_index== len(left_image_paths):
#     #    left_index = 0
#     #    right_index = 0
#   # Note that: 
#     # The purpose of splitting the image into left and right parts could be related to stereo vision or multi-camera setups. 
#     # Each resulting image (img_left and img_right) likely represents the view from a different camera or perspective,
#     # which can be used for further processing such as stereo matching, depth estimation, or any other application that benefits from having separate views of the scene.

#     # Undistort and rectify images
#     # imgR = cv2.remap(img_right, stereoMapR_x, stereoMapR_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0) # This line applies remapping to the img_right image using the cv2.remap() function.
#     # imgL = cv2.remap(img_left, stereoMapL_x, stereoMapL_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0) #This line applies remapping to the img_left image using the cv2.remap() function.
#     imgL = img_left
#     imgR = img_right

#     imgL_RGB = cv2.cvtColor(imgL, cv2.COLOR_BGR2RGB)     # Convert Image from BGR To Gray scale 
#     imgR_RGB = cv2.cvtColor(imgR, cv2.COLOR_BGR2RGB)     # Convert Image from BGR To Gray scale 
#     # Crop Image to Classify
#     # The code snippet involves cropping an image (imgL) to a specific region of interest (ROI) for classification.
    
#     start_crop_v = 24  # The starting y-coordinate of the cropping region
#     len_crop_v = 667   # the length of the cropping region in the y-direction 

#     start_crop_u = 54  # The starting x-coordinate of the cropping region
#     len_crop_u = 1186  # the length of the cropping region in the x-direction 

#     # This line performs the actual cropping of the imgL image. 
#     # It uses array slicing to extract the specific region defined by the starting coordinates and lengths specified earlier.
#     # The resulting cropped image is assigned to the variable cropped_left_img

#     cropped_left_img = imgL[start_crop_v : start_crop_v + len_crop_v, start_crop_u : start_crop_u + len_crop_u] # The purpose of this cropping is to isolate a specific portion of the image that is relevant for classification.
    
    
#     imgLgray = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)     # Convert Image from BGR To Gray scale 
#     imgRgray = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)     # Convert Image from BGR To Gray scale 

#     block_size = 5
#     min_disp = 1
#     max_disp = 144
#     num_disp = max_disp - min_disp # Needs to be divisible by 16
    
#     # Create Block matching object. 
#     # The code is using the OpenCV library in Python to create a StereoSGBM (Semi-Global Block Matching) object for stereo matching in computer vision. 
#     # this code  sets up the StereoSGBM object with various parameters to perform stereo matching, 
#     # which involves finding corresponding points between left and right images to estimate depth information.
    
#     stereo = cv2.StereoSGBM_create(minDisparity= min_disp,      # It represents the minimum possible disparity value to consider during stereo matching.
#                                    numDisparities = num_disp,   # It indicates the number of disparity values to search for. The actual range of disparities considered is calculated as numDisparities * 16.
#                                    blockSize = block_size,      # It specifies the size of the block/window used for matching. It should be an odd number.
#                                    uniquenessRatio = 3,         # It defines the uniqueness ratio threshold, which helps in filtering out ambiguous disparity matches.
#                                   # speckleWindowSize and speckleRange: These parameters are used for speckle filtering. A speckle is a small region of incorrect disparity values.
#                                   # speckleWindowSize defines the maximum size of a speckle in pixels, and speckleRange specifies the maximum disparity variation within a speckle.
#                                    speckleWindowSize = 5,       
#                                    speckleRange = 2,
#                                    disp12MaxDiff = 312,           # It represents the maximum allowed difference in disparity between the left and right views.
                                   
#                                    # P1 and P2: These parameters are used to control the smoothness of the disparity map.
#                                    # They influence the penalty on disparity changes and gradient-based disparity regularization. 
#                                    # The values are calculated based on the block size, number of image channels (typically 3 for RGB images), and other factors.
#                                    P1 = 8 * 3 * block_size**2,#8*img_channels*block_size**2,
#                                    P2 = 32 * 3 * block_size**2) #32*img_channels*block_size**2)
    
#     #This line invokes the predict method of the model object. 
#     # It takes the cropped_left_img as the input source for prediction. 
#     # The show=True parameter suggests that the prediction results should be displayed or visualized. 
#     # The variable result likely holds the output of the prediction, which may include information about detected objects, their bounding boxes, confidence scores, and other relevant details.
#     result = model.predict(source=cropped_left_img, show = True,conf = 0.66)
#     disp_img = cv2.imread('/home/philo/Downloads/data/disp/'+str(rakam)+'.jpg')

#     # Show the image
#     cv2.namedWindow('disparity', cv2.WINDOW_NORMAL)

# # Resize the window
#     cv2.resizeWindow('disparity', 640, 480)

#     cv2.imshow("disparity",disp_img)
#     # This line performs stereo matching using the stereo object and the left (imgLgray) and right (imgRgray) grayscale images.
#     # Stereo matching aims to estimate the disparity or depth map of the scene by comparing corresponding pixels in the left and right images. 
#     # The variable disparity_map stores the resulting disparity map, which provides depth information for each pixel in the image.
#     disparity_map = stereo.compute(imgLgray, imgRgray)
#     #  This line calculates the number of instances or objects detected in the result.
#     # It accesses the boxes attribute of the first element (result[0]), which likely contains information about the detected bounding boxes. 
#     # The size() method returns the size of the tensor, and [0] retrieves the size value at index 0, representing the number of instances.

#     disparity_map = np.float32(np.divide(disparity_map, 16.0))

#     # norm_image = cv2.normalize(disparity_map, None, alpha = 0, beta = 1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
#     # cv2.imwrite('/home/philo/Downloads/data/disp/'+str(rakam)+'.jpg', disparity_map)

#     num_of_instances = result[0].boxes.data.size()[0]
    
# #################################################
# # Summary:
# #################################################
# # this code snippet involves predicting objects using a model, computing a disparity map for depth estimation, and obtaining the number of instances detected in the prediction results.

# ####################################################################################################################################################################################
#     ts = time.time()
#     for i in range(num_of_instances):
      
#       # The following 4 lines of code snippet involves calculating the coordinates of the top-left and bottom-right corners of a bounding box within the cropped image region.
      
#       u_top_left = (int)(result[0].boxes.data[i][0].item()) + start_crop_u    # calculates the x-coordinate of the top-left corner of the bounding box.
#       v_top_left = (int)(result[0].boxes.data[i][1].item()) + start_crop_v    # calculates the y-coordinate of the top-left corner of the bounding box.

#       u_bottom_right = (int)(result[0].boxes.data[i][2].item()) + start_crop_u   # calculates the x-coordinate of the bottom-right corner of the bounding box
#       v_bottom_right = (int)(result[0].boxes.data[i][3].item()) + start_crop_v   # calculates the y-coordinate of the bottom-right corner of the bounding box.  

#       # retrieves the confidence level associated with a specific bounding box from the result object. 
#       confidence_level = result[0].boxes.data[i][4].item()  # This confidence level can be used to assess the reliability or strength of the object detection or classification results associated with the specific bounding box.

#       instance_type = getInstanceName((int)(result[0].boxes.data[i][5].item())) #The purpose of this line is to retrieve the specific label or name associated with the detected object within the bounding box. It allows for identifying and referring to the detected instances using their corresponding names or labels.
    
#       # calculates the coordinates of the center point of a bounding box based on the coordinates of its top-left and bottom-right corners. 
#       u = (int)((u_top_left + u_bottom_right)/2.0)  # calculates the x-coordinate of the center point of the bounding box. 
#       v = (int)((v_top_left + v_bottom_right)/2.0)  # calculates the y-coordinate of the center point of the bounding box.
      
#       # appending data to a message object perc23_msg. It assigns values to different attributes of the appended data.     
#       cx = -Q[0][3]
#       cy = -Q[1][3]
#       focal = Q[2][3]
#       B = 1/Q[3][2]
      
#       #calculates the depth values (Z_depth, X_depth, and Y_depth) in a 3D coordinate system based on the provided parameters and the disparity map.
      
#       Z_depth = B*focal/disparity_map[v][u]  
#       X_depth = (u - cx) * Z_depth / focal
#       Y_depth = (v - cy) * Z_depth / focal
      
#       # print(X_depth,Y_depth,Z_depth, instance_type)      
      
#       perc23_msg.data.append(instance())        # This line appends a new instance of an object to the data list within the perc23_msg message object
#       perc23_msg.data[i].ID = instance_type     # assigns the value of instance_type to the ID attribute of the i-th element in the data list. It sets the ID of the instance to the value representing the type or class of the detected object.
      
#       #if u want to display center pixels of bounding boxes ---> UNCOMMENT the following two lines
      
#       perc23_msg.data[i].u = u                  # It sets the x-coordinate of the center point of the bounding box for the instance.
#       perc23_msg.data[i].v = v                  # It sets the y-coordinate of the center point of the bounding box for the instance.
      
#       perc23_msg.data[i].x = X_depth                   # It sets the X-depth of the instance.
#       perc23_msg.data[i].y = Y_depth                   # It sets the Y-depth of the instance.
#       perc23_msg.data[i].z = Z_depth                   # It sets the Z-depth of the instance.
      
#       perc23_msg.data[i].confidence = confidence_level *100   #  It represents the confidence level of the detected object, scaled by a factor of 100.

      


#       # image_left_path = 'C:/Users/mohab/Downloads/left/(LEFT-'+str(ts)+'--'+str(u)+'-'+str(v)+'--'+str(X_depth)+'-'+str(Y_depth)+'-'+str(Z_depth)+'--'+instance_type+').jpg'
#       # image_right_path = 'C:/Users/mohab/Downloads/right/(RIGHT-'+str(ts)+'--'+str(u)+'-'+str(v)+'--'+str(X_depth)+'-'+str(Y_depth)+'-'+str(Z_depth)+'--'+instance_type+').jpg'
#       # cv2.imwrite(image_left_path,imgL_RGB)
#       # cv2.imwrite(image_right_path,imgR_RGB)
#       # print('left image is written to '+image_left_path)
#       # print('right image is written to '+image_right_path)

#       #################################################
#       # Summary:
#       #################################################
#       # By appending the data to the perc23_msg.data list and assigning values to its attributes, the code organizes information about the detected instances, including their IDs, coordinates, and confidence levels, within the perc23_msg message object.
#       ####################################################################################################################################################################################

#     pub.publish(perc23_msg)
#     perc23_msg.data.clear()
#     rakam +=1
#     if rakam ==19:
#        rakam = 1
#     # rospy.sleep(1)
