#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import threading
import numpy as np
import matplotlib.pyplot as plt
import sys
from CubicSpline import cubic_spline_planner

# ROS
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Quaternion, Point, Pose2D
from nav_msgs.msg import Path, Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion


k = 0.5  # control gain
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time difference
L = 2.9  # [m] Wheel base of vehicle
max_steer = np.radians(30.0)  # [rad] max steering angle


class StanleyControlNode(object):
    def __init__(self):
        rospy.on_shutdown(self.shutdown_cb)

        self.flat_path = []
        self.robot_pose = Pose2D()
        self.robot_twist = Twist()
        self.pose_update_mutex = threading.Lock()

        # ROS publisher & subscriber
        self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # self.pub_path_flat = rospy.Publisher('flat_path', Path, queue_size=1)
        self.sub_path = rospy.Subscriber("walkable_path", Path, self.path_cb, queue_size=1)
        self.sub_odom = rospy.Subscriber("wheel_odom", Odometry, self.odom_cb, queue_size=1)
        print(rospy.get_name() + ' is ready.')
        

    def odom_cb(self, msg):
        ### Critical section start ###
        self.pose_update_mutex.acquire()
        self.robot_pose.x = msg.pose.pose.position.x
        self.robot_pose.y = msg.pose.pose.positino.y
        euler_angle = euler_from_quaternion([msg.piose.pose.orientation.x, 
                                            msg.pose.pose.orientation.y, 
                                            msg.pose.pose.orientation.z, 
                                            msg.pose.pose.orientation.w])
        self.robot_pose.yaw = euler_angle[2]
        self.robot_twist = msg.twist.twist
        self.pose_update_mutex.release()
        ### Critical section end ###


    def path_cb(self, msg):
        path_x_raw = []
        path_y_raw = []
        for i in range(len(msg.poses)-1, 0, -1):
            path_x_raw.append(msg.poses[i].pose.position.x)
            path_y_raw.append(msg.poses[i].pose.position.y)
        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        path_x_raw, path_y_raw, ds=0.2)

        self.flat_path = []
        for i in range(len(cx)):
            self.flat_path.append(Pose2D(x=cx[i], y=cy[i], yaw=cyaw[i]))

        # # Visualiztion
        # flat_path_msg = Path()
        # for i in range(len(cx)):
        #     tmp_pose = PoseStamped()
        #     tmp_pose.pose.position = Point(cx[i], cy[i], 0)
        #     q = quaternion_from_euler(0, 0, cyaw[i])
        #     tmp_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        #     flat_path_msg.poses.append(tmp_pose)
        # flat_path_msg.header.frame_id = msg.header.frame_id
        # flat_path_msg.header.stamp = rospy.Time.now()
        # self.pub_path_flat.publish(flat_path_msg)


    def pid_control(target, current):
        """
        Proportional control for the speed.

        :param target: (float)
        :param current: (float)
        :return: (float)
        """
        return Kp * (target - current)


    def stanley_control(robot_pose, robot_twist, target_path, last_target_idx):
        """
        Stanley steering control.

        :param robot_pose: (Pose2D)
        :param target_path: ([Pose2D])
        :param last_target_idx: (int)
        :return: (float, int)
        """
        current_target_idx, error_front_axle = calc_target_index(robot_pose, target_path)

        if last_target_idx >= current_target_idx:
            current_target_idx = last_target_idx

        # theta_e corrects the heading error
        theta_e = normalize_angle(target_path[current_target_idx].yaw - robot_pose.yaw)
        # theta_d corrects the cross track error
        theta_d = np.arctan2(k * error_front_axle, robot_twist.linear.x * np.cos(robot_twist.angular.z)) ########################## TODO
        # Steering control
        delta = theta_e + theta_d

        return delta, current_target_idx


    def normalize_angle(angle):
        """
        Normalize an angle to [-pi, pi].

        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle


    def calc_target_index(robot_pose, target_path):
        """
        Compute index in the trajectory list of the target.

        :param robot_pose: (State object)
        :param cx: [float]
        :param cy: [float]
        :return: (int, float)
        """
        # Calc front axle position
        fx = robot_pose.x + L * np.cos(robot_pose.yaw)
        fy = robot_pose.y + L * np.sin(robot_pose.yaw)

        # Search nearest point index
        dx = []
        dy = []
        for tmp_pose in target_path:
            dx.append(fx - tmp_pose.x)
            dy.append(fy - tmp_pose.y)
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(robot_pose.yaw + np.pi / 2),
                          -np.sin(robot_pose.yaw + np.pi / 2)]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle


    def shutdown_cb(self):
        rospy.loginfo("Shutdown " + rospy.get_name())
        

if __name__ == '__main__':
    rospy.init_node('stanley_control_node', anonymous=False)
    node = StanleyControlNode()
    
    target_speed = 0.4

    rate = rospy.Rate(5.0) # 5hz
    while not rospy.is_shutdown():
        # node.flat_path
        while node.pose_update_mutex.locked():
            pass

        accel_linear = pid_control(target_speed, node.robot_twist.linear.x)

        rate.sleep()
        