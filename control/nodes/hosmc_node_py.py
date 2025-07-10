#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import FluidPressure, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from control.srv import SetDesired, SetDesiredResponse
import tf

from control.hosmc_controller import HOSMCController

class HOSMCNode:
    def __init__(self):
        rospy.init_node("hosmc_node")
        self.controller = HOSMCController()

        self.eta_real = np.zeros(4)
        self.eta_dot = np.zeros(4)
        self.eta_desired = np.zeros(4)

        self.eta_set = False
        self.depth_received = False
        self.imu_received = False
        self.state_received = False

        rospy.Subscriber("/pressure", FluidPressure, self.depth_callback)
        rospy.Subscriber("/imu", Imu, self.imu_callback)
        rospy.Subscriber("/odom", Odometry, self.state_callback)

        self.control_pub = rospy.Publisher("/HOSMC/Wrench", WrenchStamped, queue_size=10)
        self.set_service = rospy.Service("set_desired", SetDesired, self.set_desired_callback)

        self.control_timer = rospy.Timer(rospy.Duration(0.04), self.control_loop)

    def depth_callback(self, msg):
        self.eta_real[0] = (msg.fluid_pressure * 100 - 101325.0) / (9.8 * 997.0)
        self.depth_received = True

    def imu_callback(self, msg):
        q = msg.orientation
        quaternion = (q.x, q.y, q.z, q.w)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)

        self.eta_real[1] = roll
        self.eta_real[2] = pitch
        self.eta_real[3] = yaw

        self.imu_received = True

    def state_callback(self, msg):
        pos = msg.pose.pose.position
        self.eta_real[0] = pos.z

        q = msg.pose.pose.orientation
        quaternion = (q.x, q.y, q.z, q.w)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)

        self.eta_real[1] = roll
        self.eta_real[2] = pitch
        self.eta_real[3] = yaw

        self.eta_dot[0] = msg.twist.twist.linear.z
        self.eta_dot[1] = msg.twist.twist.angular.x
        self.eta_dot[2] = msg.twist.twist.angular.y
        self.eta_dot[3] = msg.twist.twist.angular.z

        self.state_received = True

    def set_desired_callback(self, req):
        rospy.loginfo("Received z_desired: [%f]", req.z)
        self.eta_desired[0] = req.z
        self.eta_desired[1] = 0.0
        self.eta_desired[2] = 0.0
        self.eta_desired[3] = req.yaw

        self.eta_set = True
        return SetDesiredResponse(success=True)

    def control_loop(self, event):
        if self.eta_set and ((self.depth_received and self.imu_received) or self.state_received):
            f_ver = self.controller.stabilize(self.eta_real, self.eta_desired)

            msg = WrenchStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "base_link"
            msg.wrench.force.x = 0.0
            msg.wrench.force.y = 0.0
            msg.wrench.force.z = f_ver[0]
            msg.wrench.torque.x = f_ver[1]
            msg.wrench.torque.y = f_ver[2]
            msg.wrench.torque.z = f_ver[3]

            self.control_pub.publish(msg)
        else:
            
            rospy.logwarn_throttle(5, "Waiting for eta_desired and sensor data...")

if __name__ == "__main__":
    node = HOSMCNode()
    rospy.spin()
