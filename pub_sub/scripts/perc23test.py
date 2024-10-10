#!/usr/bin/env python3

import rospy
from pub_sub.msg import instance
from pub_sub.msg import Multi_instance
from ultralytics import YOLO
import cv2

#Node initialization
rospy.init_node('detect23')
pub = rospy.Publisher('/landmarks', Multi_instance, queue_size = 10) 
part = instance()
msg = Multi_instance()


##############################################
# Set Camera Resolution
##############################################
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)


#################################################
# Stereo Calibration and rectification
#################################################
# Camera parameters to undistort and rectify images
cv_file = cv2.FileStorage()
cv_file.open('/home/jetsonx/catkin_ws/src/pub_sub/scripts/stereoMapbate.xml', cv2.FileStorage_READ)

stereoMapL_x = cv_file.getNode('stereoMapL_x').mat()
stereoMapL_y = cv_file.getNode('stereoMapL_y').mat()
stereoMapR_x = cv_file.getNode('stereoMapR_x').mat()
stereoMapR_y = cv_file.getNode('stereoMapR_y').mat()

Q = cv_file.getNode('q').mat()




#############################################



#Model initialization

model_weights_path = "/home/jetsonx/trials/pub_sub/scripts/best.pt"
model = YOLO(model_weights_path)

def getInstanceName(instance_number):
    
    labels = ['gate','path marker','badge','gun','box','hand','barrel','note','phone','bottle','gman','bootlegger','axe','dollar','beer']
    return labels[instance_number] 


while not rospy.is_shutdown():
    
    succes, full_img = cap.read()
    img_left = full_img[:,0:int(full_img.shape[1]/2)]
    img_right = full_img[:, int(full_img.shape[1]/2):]


    # Undistort and rectify images
    imgR = cv2.remap(img_right, stereoMapR_x, stereoMapR_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
    imgL = cv2.remap(img_left, stereoMapL_x, stereoMapL_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
    
    # Crop Image to Classify

    start_crop_y = 24
    len_crop_y = 667

    start_crop_x = 54
    len_crop_x = 1186

    cropped_left_img = imgL[start_crop_y:start_crop_y+len_crop_y, start_crop_x:start_crop_x+len_crop_x]
    
    imgLgray = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    imgRgray = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

    block_size = 1
    min_disp = 16
    max_disp = 288
    num_disp = max_disp - min_disp # Needs to be divisible by 16
    # Create Block matching object. 
    stereo = cv2.StereoSGBM_create(minDisparity= min_disp,
                                   numDisparities = num_disp,
                                   blockSize = block_size,
                                   uniquenessRatio = 2,
                                   speckleWindowSize = 0,
                                   speckleRange = 1,
                                   disp12MaxDiff = 272,
                                   P1 = 8 * 3 * block_size**2,#8*img_channels*block_size**2,
                                   P2 = 32 * 3 * block_size**2) #32*img_channels*block_size**2)
    
    
    result = model.predict(source=cropped_left_img, show = True)
    disparity_map = stereo.compute(imgLgray, imgRgray)
    disparity_map = np.float32(np.divide(disparity_map, 16.0))
    num_of_instances = result[0].boxes.data.size()[0]
    
        
    for i in range(num_of_instances):
    
      x_top_left = (int)(result[0].boxes.data[i][0].item()) + start_crop_x
      y_top_left = (int)(result[0].boxes.data[i][1].item()) + start_crop_y

      x_bottom_right = (int)(result[0].boxes.data[i][2].item()) + start_crop_x
      y_bottom_right = (int)(result[0].boxes.data[i][3].item()) + start_crop_y

      confidence_level = result[0].boxes.data[i][4].item()

      instance_type = getInstanceName((int)(result[0].boxes.data[i][5].item()))
    
      x = (int)((x_top_left + x_bottom_right)/2.0)
      y = (int)((y_top_left + y_bottom_right)/2.0)
      
      msg.data.append(instance())
      msg.data[i].ID = instance_type
      msg.data[i].x = x
      msg.data[i].y = y
      msg.data[i].confidence = confidence_level *100

      cx = -Q[0][3]
      cy = -Q[1][3]
      focal = Q[2][3]
      B = 1.0/Q[3][2]
      
      Z_depth = B*focal/disparity_map[y][x]  
      X_depth = (x - cx) * Z_depth / focal
      Y_depth = (y - cy) * Z_depth / focal
      print(X_depth,Y_depth,Z_depth, instance_type)
    pub.publish(msg)
    msg.data.clear()
    rospy.sleep(5)
