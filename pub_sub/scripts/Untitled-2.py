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


#Model initialization

model_weights_path = "/media/philo/Philo/AS Marine/best.pt"
model = YOLO(model_weights_path)

def getInstanceName(instance_number):
    
    labels = ['gate','path marker','badge','gun','box','hand','barrel','note','phone','bottle','gman','bootlegger','axe','dollar','beer']
    return labels[instance_number] 

def

while not rospy.is_shutdown():


    msgs = []
    my_img = cv2.imread('/media/philo/Philo/AS Marine/frame0-00-15.15.png')
    result = model.predict(source=my_img)
    num_of_instances = result[0].boxes.boxes.size()[0]
    
        
    for i in range(num_of_instances):
    
      gates = []
      x_top_left = (int)(result[0].boxes.boxes[i][0].item())
      y_top_left = (int)(result[0].boxes.boxes[i][1].item())

      x_bottom_right = (int)(result[0].boxes.boxes[i][2].item())
      y_bottom_right = (int)(result[0].boxes.boxes[i][3].item())

      confidence_level = result[0].boxes.boxes[i][4].item()

      instance_type = getInstanceName((int)(result[0].boxes.boxes[i][5].item()))
      if(instance_type =='gate'):
        gates.append((x_top_left,y_top_left,x_bottom_right,y_bottom_right))

    
      x = (int)((x_top_left + x_bottom_right)/2.0)
      y = (int)((y_top_left + y_bottom_right)/2.0)
      
      msg.data.append(instance())
      msg.data[i].ID = instance_type
      msg.data[i].x = x
      msg.data[i].y = y
      msg.data[i].confidence = confidence_level *100
    pub.publish(msg)
    msg.data.clear()
    rospy.sleep(1)

    
    
    
    
    
    
    
    
    
    
