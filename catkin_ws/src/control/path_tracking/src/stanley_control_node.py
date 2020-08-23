#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import threading
import numpy as np
import matplotlib.pyplot as plt
import sys
import time
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
CMD_RATE = 5.0

class StanleyControlNode(object):
    def __init__(self):
        rospy.on_shutdown(self.shutdown_cb)

        # Path related
        self.flat_path = []
        self.is_new_path = False
        self.path_update_mutex = threading.Lock()

        # Odom related
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
        self.robot_pose.y = msg.pose.pose.position.y
        euler_angle = euler_from_quaternion([msg.pose.pose.orientation.x, 
                                            msg.pose.pose.orientation.y, 
                                            msg.pose.pose.orientation.z, 
                                            msg.pose.pose.orientation.w])
        self.robot_pose.theta = euler_angle[2]
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

        ### Critical section start ###
        self.path_update_mutex.acquire()
        self.flat_path = []
        for i in range(len(cx)):
            self.flat_path.append(Pose2D(x=cx[i], y=cy[i], theta=cyaw[i]))
        self.is_new_path = True
        self.path_update_mutex.release()
        ### Critical section end ###

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


    def pid_control(self, target, current):
        """
        Proportional control for the speed.

        :param target: (float)
        :param current: (float)
        :return: (float)
        """
        return Kp * (target - current)


    def stanley_control(self, robot_pose, robot_twist, target_path, last_target_idx):
        """
        Stanley steering control.

        :param robot_pose: (Pose2D)
        :param target_path: ([Pose2D])
        :param last_target_idx: (int)
        :return: (float, int)
        """
        current_target_idx, error_front_axle = self.calc_target_index(robot_pose, target_path)

        if last_target_idx >= current_target_idx:
            current_target_idx = last_target_idx

        # theta_e corrects the heading error
        theta_e = self.normalize_angle(target_path[current_target_idx].theta - robot_pose.theta)
        # theta_d corrects the cross track error
        # theta_d = np.arctan2(k * error_front_axle, robot_twist.linear.x * np.cos(robot_twist.angular.z)) ########################## TODO: Check
        theta_d = np.arctan2(k * error_front_axle, robot_twist.linear.x) ########################## TODO: Check
        

        # Steering control
        delta = theta_e + theta_d

        return delta, current_target_idx


    def normalize_angle(self, angle):
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


    def calc_target_index(self, robot_pose, target_path):
        """
        Compute index in the trajectory list of the target.

        :param robot_pose: (State object)
        :param cx: [float]
        :param cy: [float]
        :return: (int, float)
        """
        # Calc front axle position
        fx = robot_pose.x + L * np.cos(robot_pose.theta)
        fy = robot_pose.y + L * np.sin(robot_pose.theta)

        # Search nearest point index
        dx = []
        dy = []
        for tmp_pose in target_path:
            dx.append(fx - tmp_pose.x)
            dy.append(fy - tmp_pose.y)
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)
        # print("target_idx:", target_idx)

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(robot_pose.theta + np.pi / 2),
                          -np.sin(robot_pose.theta + np.pi / 2)]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle


    def shutdown_cb(self):
        if self.path_update_mutex.locked():
            self.path_update_mutex.release()
        if self.pose_update_mutex.locked():
            self.pose_update_mutex.release()
        rospy.loginfo("Shutdown " + rospy.get_name())
        

if __name__ == '__main__':
    rospy.init_node('stanley_control_node', anonymous=False)
    node = StanleyControlNode()
    
    target_speed = 0.4
    is_first_run = True
    last_run_time = None

    # Wait for the first flat path comming
    while len(node.flat_path) == 0 and not rospy.is_shutdown():
        rospy.sleep(0.1)
    while node.pose_update_mutex.locked() and not rospy.is_shutdown():
        rospy.sleep(0.1)
    target_idx, _  = node.calc_target_index(node.robot_pose, node.flat_path)

    rate = rospy.Rate(CMD_RATE)
    while not rospy.is_shutdown():
        if is_first_run:
            is_first_run = False
            last_run_time = rospy.Time.now()
            continue
        current_run_time = rospy.Time.now()

        # It promise that you always get the newest robot odometry before sending the control command
        while node.pose_update_mutex.locked() or node.path_update_mutex.locked():
            pass

        if node.is_new_path == True:
            node.is_new_path = False
            target_idx, _  = node.calc_target_index(node.robot_pose, node.flat_path)

        accel_linear = node.pid_control(target_speed, node.robot_twist.linear.x)
        delta_omega, target_idx = node.stanley_control(node.robot_pose, node.robot_twist, node.flat_path, target_idx)
        # state.update(accel_linear, di)

        dt = (current_run_time - last_run_time).to_sec()
        cmd_msg = Twist()
        cmd_msg.linear.x = node.robot_twist.linear.x + accel_linear * dt
        cmd_msg.angular.z = delta_omega * dt
        node.pub_cmd.publish(cmd_msg)

        # time += dt
        last_run_time = current_run_time
        rate.sleep()
        