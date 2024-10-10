#!/usr/bin/env python3

import rospy

from pub_sub.msg import Multi_instance


def cam_callback(cam_msg):
    print(cam_msg.data)    


#Node initialization

rospy.init_node('sub')
rospy.Subscriber('/landmarks', Multi_instance, cam_callback)

#Loop

rospy.spin()
