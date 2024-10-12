#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import WrenchStamped

def talker():
    # Initialize the ROS node
    rospy.init_node('wrench_publisher', anonymous=True)

    # Create the publisher for the 'wrench' topic
    pub = rospy.Publisher('/Wrench', WrenchStamped, queue_size=10)
    
    rospy.loginfo("Publishing the zero Wrench message.")
    
    zero_wrench_msg = WrenchStamped()
    zero_wrench_msg.header.stamp = rospy.Time.now()
    zero_wrench_msg.wrench.force.x = 0.0
    zero_wrench_msg.wrench.force.y = 0.0
    zero_wrench_msg.wrench.force.z = 0.0
    zero_wrench_msg.wrench.torque.x = 0.0
    zero_wrench_msg.wrench.torque.y = 0.0
    zero_wrench_msg.wrench.torque.z = 0.0

    zero_wrench_msg.header.stamp = rospy.Time.now()
    pub.publish(zero_wrench_msg)
    
    # Parse command line arguments
    if len(sys.argv) < 7:
        rospy.logerr("Usage: rosrun <pkg_name> <node_file_name> <force_x> <force_y> <force_z> <torque_x> <torque_y> <torque_z> <time>")
        return

    # Retrieve the parameters from the command line
    force_x = float(sys.argv[1])
    force_y = float(sys.argv[2])
    force_z = float(sys.argv[3])
    torque_x = float(sys.argv[4])
    torque_y = float(sys.argv[5])
    torque_z = float(sys.argv[6])
    
    # Check if time is provided, default to 1 second if not
    if len(sys.argv) == 8:
        delay_time = float(sys.argv[7])
    else:
        delay_time = 0.5  # Default to 1 second if no time is provided

    rospy.loginfo("Publishing the Wrench message with forces: [%.2f, %.2f, %.2f] and torques: [%.2f, %.2f, %.2f]" %
                  (force_x, force_y, force_z, torque_x, torque_y, torque_z))
   
    # Create a Wrench message with the specified forces and torques
    wrench_msg = WrenchStamped()
    wrench_msg.wrench.force.x = force_x
    wrench_msg.wrench.force.y = force_y
    wrench_msg.wrench.force.z = force_z
    wrench_msg.wrench.torque.x = torque_x
    wrench_msg.wrench.torque.y = torque_y
    wrench_msg.wrench.torque.z = torque_z

    rospy.loginfo("Publishing the zero Wrench message.")

    for i in range(int(delay_time/0.1)):
        wrench_msg.header.stamp = rospy.Time.now()
        pub.publish(wrench_msg)
        rospy.sleep(0.1)
        # Sleep for a 10 ms

    # Publish a zero Wrench message
    rospy.loginfo("Publishing the zero Wrench message.")
    for i in range(int(10*delay_time/0.1)):
        zero_wrench_msg.header.stamp = rospy.Time.now()
        # Create a Wrench message with the specified forces and torques
        pub.publish(zero_wrench_msg)
        rospy.sleep(0.1)
        # Sleep for a 10 ms

    rospy.loginfo("Publishing complete. Node will now exit.")

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
