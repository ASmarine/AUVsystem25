#!/usr/bin/env python3

import rospy

from std_msgs.msg import Int16


#Node initialization

rospy.init_node('pub')
pub = rospy.Publisher('/ay_7aga', Int16, queue_size = 1) 

#Loop

while not rospy.is_shutdown():
    pub.publish(1)
    rospy.sleep(1)
