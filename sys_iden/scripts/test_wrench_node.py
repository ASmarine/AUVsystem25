#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray

def publish_vector():
    rospy.init_node('wrench_publisher', anonymous=True)
    pub = rospy.Publisher('/thruster_controller/wrench_vector', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(1)  # 10 Hz

    while not rospy.is_shutdown():
        vector_msg = Float32MultiArray()
        vector_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Taw column vector

        pub.publish(vector_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_vector()
    except rospy.ROSInterruptException:
        pass
