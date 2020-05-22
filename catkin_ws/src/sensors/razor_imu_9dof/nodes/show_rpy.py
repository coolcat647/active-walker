#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import math


def callback(msg):
    q = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    )
    (roll, pitch, yaw) = euler_from_quaternion(q)
    # print('yaw= {}, pitch={}, roll={}'.format(yaw, pitch, roll))
    print('yaw= {}, pitch={}, roll={}'.format(yaw * 180.0 / math.pi, pitch * 180.0 / math.pi, roll * 180.0 / math.pi)) 

if __name__ == '__main__':
    rospy.init_node('show_rpy_node', anonymous=True)
    rospy.Subscriber("/imu", Imu, callback)
    rospy.spin()
