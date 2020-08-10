#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import math
import time
import copy
import numpy as np
from filterpy.kalman import KalmanFilter

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import WrenchStamped, Vector3

FORCE_DATA_DIMENSION = 6

class ForceFilteringNode(object):
    def __init__(self):
        # Kalman filter
        self.kf = KalmanFilter(dim_x=FORCE_DATA_DIMENSION, dim_z=FORCE_DATA_DIMENSION)       
        self.kf.F = np.array([[1,0,0,0,0,0],      # state transition matrix
                              [0,1,0,0,0,0],
                              [0,0,1,0,0,0],
                              [0,0,0,1,0,0],  
                              [0,0,0,0,1,0],  
                              [0,0,0,0,0,1]])

        self.kf.H = np.array([[1,0,0,0,0,0],      # measurement function,
                              [0,1,0,0,0,0],
                              [0,0,1,0,0,0],
                              [0,0,0,1,0,0],
                              [0,0,0,0,1,0],
                              [0,0,0,0,0,1]])

        self.kf.R[0:,0:] *= 10.   # measurement uncertainty
        self.kf.P *= 10.

        # self.kf.Q[-1,-1] *= 0.01    # process uncertainty
        self.kf.Q[:, :] *= 0.01
        # self.kf.x[:3] = det2D.reshape((3, 1)) # x, y, r

        # Calculate the offset by force averaging
        force_data_array = np.zeros(FORCE_DATA_DIMENSION)
        for i in range(1, 250):
            try:
                msg = rospy.wait_for_message('/force', WrenchStamped, timeout=0.1)    
            except rospy.ROSException as e:
                rospy.logerr("Timeout while waiting for force data!")
                exit(-1)

            tmp_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                                    msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z], dtype=np.float32)
            if force_data_array.shape == 0:
                force_data_array = tmp_force
            else:
                force_data_array = np.vstack((force_data_array, tmp_force))
                # print(force_data_array.shape)

        # Force offset & initial value
        self.force_offset = np.average(force_data_array, axis=0)
        self.kf.x[:] = (force_data_array[-1, :] - self.force_offset).reshape((FORCE_DATA_DIMENSION, 1))

        self.pub_force = rospy.Publisher('/force_filtered', WrenchStamped, queue_size=1)
        self.sub_force = rospy.Subscriber("/force", WrenchStamped, self.force_cb, queue_size=1)
        print(rospy.get_name() + ' is ready.')
        

    def force_cb(self, msg):
        force_raw = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                                msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z], dtype=np.float32)
        force_tmp = force_raw - self.force_offset

        # Kalman filter predict
        self.kf.predict()
        # Kalman filter update
        self.kf.update(force_tmp)
        force_filtered = self.kf.x.reshape((FORCE_DATA_DIMENSION, ))

        output_msg = WrenchStamped()
        output_msg.header.frame_id = msg.header.frame_id
        output_msg.wrench.force = Vector3(x=force_filtered[0], y=force_filtered[1], z=force_filtered[2])
        output_msg.wrench.torque = Vector3(x=force_filtered[3], y=force_filtered[4], z=force_filtered[5])
        output_msg.header.stamp = msg.header.stamp #rospy.Time.now()
        self.pub_force.publish(output_msg)
        

if __name__ == '__main__':
    rospy.init_node('force_extraction_node', anonymous=False)
    node = ForceFilteringNode()
    rospy.spin()
