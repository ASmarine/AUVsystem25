#!/usr/bin/env python3
# Camera footage publisher
# Author:
# - Samer A. Ahmed
# - https://github.com/
 
import rospy 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2 
  
def publish_message():
 
  # Publishers
  pub_left = rospy.Publisher('/camera/left/image_raw', Image, queue_size=10)
  pub_right = rospy.Publisher('/camera/right/image_raw', Image, queue_size=10)
     
  # Node initialization
  rospy.init_node('cam_pub', anonymous=True)
     
  # Go through the loop 10 times per second
  rate = rospy.Rate(10) # 10hz
     
  # Create VideoCapture objects
  stereo = cv2.VideoCapture(0)
  if stereo.isOpened() == 0:
  	print("cam not available")
  else:
  	print('Cam Available')

  print(stereo.type())
  

  	   
  # Bridge to convert between ROS and OpenCV images
  br = CvBridge()
  
  # Connection warning status
  warn_stereo = True
 
  while not rospy.is_shutdown():
     
      # Capture frame-by-frame
      # This method returns the return status and frame data
      ret, frame = stereo.read()
         
      if ret == True:     
        # Publish the captured image
        pub_left.publish(br.cv2_to_imgmsg(frame[:,0:int(frame.shape[1]/2)]))
        pub_right.publish(br.cv2_to_imgmsg(frame[:,int(frame.shape[1]/2)+1:]))
      else:
        # Display a warning message, once
        if warn_stereo == True:
          rospy.loginfo('stereo camera is not connected!!')
          warn_stereo = False
             
      # Sleep just enough to maintain the desired rate
      rate.sleep()
         
if __name__ == '__main__':
  try:
    publish_message()
  except rospy.ROSInterruptException:
    pass
