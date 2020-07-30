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

MAX_FORCE = 50.0
MAX_TORQUE = 50.0

MAX_LINEAR_VELOCITY = 0.5
MIN_LINEAR_VELOCITY = 0.0
MAX_ANGULAR_VELOCITY = 0.5

class Force2CmdNode(object):
    def __init__(self):
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub_force = rospy.Subscriber("/force_filtered", WrenchStamped, self.force_cb, queue_size=1)
        print(rospy.get_name() + ' is ready.')
        

    def force_cb(self, msg):
        force_y = min(msg.wrench.force.y, MAX_FORCE)
        torque_z = min(msg.wrench.torque.z, MAX_TORQUE)

        linear_vel = max(force_y / MAX_FORCE * MAX_LINEAR_VELOCITY, MIN_LINEAR_VELOCITY)
        angular_vel = torque_z / MAX_TORQUE * MAX_ANGULAR_VELOCITY

        cmd_msg = Twist()
        cmd_msg.linear.x = linear_vel
        cmd_msg.angular.z = angular_vel
        self.pub_vel.publish(cmd_msg)
        

if __name__ == '__main__':
    rospy.init_node('force2cmd_node', anonymous=False)
    node = Force2CmdNode()
    rospy.spin()
