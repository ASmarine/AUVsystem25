#!/usr/bin/env python3

import rospy
from pub_sub.msg import instance
from pub_sub.msg import Multi_instance

#Node initialization
rospy.init_node('detect23')
pub = rospy.Publisher('/landmarks', Multi_instance, queue_size = 10) 
part = instance()
msg = Multi_instance()


#Model initialization

#model_weights_path = "/media/philo/Philo/AS Marine/local train/wehights/weights/best.pt"
#model = YOLO(model_weights_path)

def getInstanceName(instance_number):
    
    labels = ['gate','path marker','badge','gun','box','hand','barrel','note','phone','bottle','gman','bootlegger','axe','dollar','beer']
    return labels[instance_number] 


while not rospy.is_shutdown():


    #my_img = cv2.imread('/media/philo/Philo/AS Marine/frame0-00-08.29.jpg')
    #result = model.predict(source=my_img, show = True )
    #num_of_instances = result[0].boxes.data.size()[0]
    
        
    #for i in range(num_of_instances):
    for i in range(10):
    
      x_top_left = 10*i
      y_top_left = 20*i

      x_bottom_right = 30*i
      y_bottom_right = 40*i

      confidence_level = (i+1)*0.1

      instance_type = getInstanceName(i)
    
      u = (int)((x_top_left + x_bottom_right)/2.0)
      v = (int)((y_top_left + y_bottom_right)/2.0)
      
      msg.data.append(instance())
      msg.data[i].ID = instance_type
      msg.data[i].u = u
      msg.data[i].v = v
      
      msg.data[i].x = u * 100
      msg.data[i].y = u * 1000
      msg.data[i].z = u * 10000	
      
      msg.data[i].confidence = confidence_level *100
      print(msg.data[i])
    pub.publish(msg)
    msg.data.clear()
    rospy.sleep(10)
