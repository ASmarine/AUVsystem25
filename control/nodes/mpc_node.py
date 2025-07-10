#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from control.srv import SetDesired, SetDesiredResponse
import tf
import casadi

from control.MPC import ControlMPC

class MPCNode:
    def __init__(self):
        rospy.init_node('mpc_controller_node')

        # Parameters
        self.dT = 0.1

        # MPC object
        dummy_sens = {
            'orbX': 0.0,
            'orbY': 0.0,
            'ps': 0.0,
            'imu_euler': [0.0, 0.0, 0.0]
        }

        self.controller = ControlMPC(sens=dummy_sens, dT=self.dT)

        self.desired = [0, 0, 0, 0, 0, 0]

        # Subscribers
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.path_sub = rospy.Subscriber("/mpc_path", Path, self.path_callback)


        # Publisher
        self.cmd_pub = rospy.Publisher("/cmd_wrench", WrenchStamped, queue_size=10)

        # Service
        self.set_desired_srv = rospy.Service("/set_desired", SetDesired, self.set_desired_callback)

        # Timer for control loop
        self.timer = rospy.Timer(rospy.Duration(self.dT), self.control_loop)

        # State
        self.current_odom = None

        # Flags
        self.state_received = False
        self.path = False
        self.eta_set = False

    def odom_callback(self, msg):
        self.current_odom = msg
        self.state_received = True

    def path_callback(self, msg):
        if not msg.poses:
            rospy.logwarn("Received empty path.")
            return

        path_list = []
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z
            path_list.append([x, y, z])

        path_np = np.array(path_list)
        self.controller.set_path(path_np)
        rospy.loginfo("Received path with %d points.", path_np.shape[0])
        self.path = True

    def set_desired_callback(self, req):
        self.desired = [req.x, req.y, req.yaw, 0, 0, 0]
        rospy.loginfo("Updated desired point: %s", self.desired)
        self.eta_set = True
        return SetDesiredResponse(success=True)

    def control_loop(self, event):
        if (self.eta_set or self.path) and self.state_received:
            pos = self.current_odom.pose.pose.position
            vel = self.current_odom.twist.twist

            q = self.current_odom.pose.pose.orientation
            quaternion = (q.x, q.y, q.z, q.w)
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)

            current_pose = {
                    'orbX': pos.x,
                    'orbY': pos.y,
                    'ps': pos.z,
                    'imu_euler': [roll, pitch, yaw]
                }

            u = self.controller.actuate(current_pose, self.desired).flatten()

            msg = WrenchStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "base_link"  # or whatever frame your robot uses

            msg.wrench.force.x = u[0]
            msg.wrench.force.y = u[1]
            msg.wrench.torque.z = u[2]

            self.cmd_pub.publish(msg)
        else:
            rospy.logwarn("Waiting for state or path data.")


if __name__ == "__main__":
    try:
        MPCNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
