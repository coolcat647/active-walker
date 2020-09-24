#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import math
import time
import copy
import numpy as np

import rospy
from geometry_msgs.msg import WrenchStamped, Vector3, Twist

MIN_ENABLE_FORCE = 5.0
MIN_ENABLE_TORQUE = 2.0

MAX_LINEAR_VELOCITY = 0.6
MAX_ANGULAR_VELOCITY = 1.4

MASS = 30.0
MOMENT_OF_INERTIA = 2.61 / 2.0 
DAMPING_XY = 40.0
DAMPING_THETA = 8.0

class Force2CmdNode(object):
    def __init__(self):
        self.last_time = None
        self.last_linear_velocity = 0
        self.last_angular_velocity = 0

        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub_force = rospy.Subscriber("/force_filtered", WrenchStamped, self.force_cb, queue_size=1)
        rospy.loginfo(rospy.get_name() + ' is ready.')
        

    def force_cb(self, msg):
        if self.last_time is None:
            self.last_time = rospy.Time().now()
            return

        force_y = msg.wrench.force.y if np.abs(msg.wrench.force.y) > MIN_ENABLE_FORCE else 0.0
        torque_z = msg.wrench.torque.z if np.abs(msg.wrench.torque.z) > MIN_ENABLE_TORQUE else 0.0

        now_time = rospy.Time().now()
        dt = (now_time - self.last_time).to_sec()
        
        # Calculate target velocity by human force
        linear_velocity = (force_y / MASS * dt + self.last_linear_velocity) / (1 + DAMPING_XY / MASS * dt)
        angular_velocity = (torque_z / MOMENT_OF_INERTIA * dt + self.last_angular_velocity) / (1 + DAMPING_THETA / MOMENT_OF_INERTIA * dt)
        
        cmd_msg = Twist()
        cmd_msg.linear.x = np.clip(linear_velocity, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY)
        cmd_msg.angular.z = np.clip(angular_velocity, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY)
        rospy.loginfo("v, omega = ({:.2f},{:.2f})".format(cmd_msg.linear.x, cmd_msg.angular.z))
        self.pub_vel.publish(cmd_msg)
        
        self.last_time = now_time
        self.last_linear_velocity = linear_velocity
        self.last_angular_velocity = angular_velocity


if __name__ == '__main__':
    rospy.init_node('force2cmd_node', anonymous=False)
    node = Force2CmdNode()
    rospy.spin()