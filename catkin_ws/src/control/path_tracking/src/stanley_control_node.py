#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import sys
from CubicSpline import cubic_spline_planner

# ROS
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Quaternion, Point
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler


class StanleyControlNode(object):
    def __init__(self):
        rospy.on_shutdown(self.shutdown_cb)

        print(rospy.get_name() + ' is ready.')
        self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pub_path_flat = rospy.Publisher('flat_path', Path, queue_size=1)
        self.sub_path = rospy.Subscriber("walkable_path", Path, self.path_cb, queue_size=1)


    def path_cb(self, msg):
        path_x_raw = []
        path_y_raw = []
        for i in range(len(msg.poses)-1, 0, -1):
            path_x_raw.append(msg.poses[i].pose.position.x)
            path_y_raw.append(msg.poses[i].pose.position.y)
            # print(path_x_raw[-1], path_y_raw[-1])
        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        path_x_raw, path_y_raw, ds=0.2)

        flat_path_msg = Path()
        for i in range(len(cx)):
            tmp_pose = PoseStamped()
            tmp_pose.pose.position = Point(cx[i], cy[i], 0)
            q = quaternion_from_euler(0, 0, cyaw[i])
            tmp_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
            flat_path_msg.poses.append(tmp_pose)

        flat_path_msg.header.frame_id = msg.header.frame_id
        flat_path_msg.header.stamp = rospy.Time.now()
        self.pub_path_flat.publish(flat_path_msg)



    def shutdown_cb(self):
        rospy.loginfo("Shutdown " + rospy.get_name())
        

if __name__ == '__main__':
    rospy.init_node('stanley_control_node', anonymous=False)
    node = StanleyControlNode()
    rospy.spin()
        