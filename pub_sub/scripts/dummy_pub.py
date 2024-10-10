#!/usr/bin/env python3

import rospy

from std_msgs.msg import Int16


#Node initialization


pub = rospy.Publisher('/distance0', Int16, queue_size = 5) 
rospy.init_node('dummy_pub')
rate = rospy.Rate(10)
print("dummy initialization")

#Loop

while not rospy.is_shutdown():
    pub.publish(10)
    print("published - value")
    rate.sleep()
    
