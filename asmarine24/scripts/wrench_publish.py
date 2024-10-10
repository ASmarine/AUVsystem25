#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import WrenchStamped

def publish_vector():
    rospy.init_node('wrench_publisher', anonymous=True)
    pub = rospy.Publisher('/Wrench', WrenchStamped, queue_size=1)
    rate = rospy.Rate(1)  # 10 Hz

    while not rospy.is_shutdown():
        vector_msg = WrenchStamped()
        vector_msg.wrench.force.z = 0
        vector_msg.wrench.force.x = 0
        vector_msg.wrench.torque.y = 0
        pub.publish(vector_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_vector()
    except rospy.ROSInterruptException:
        pass

