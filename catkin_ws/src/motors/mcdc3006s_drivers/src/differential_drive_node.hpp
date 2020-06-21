#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <iomanip>
#include "serial/serial.h"

// For ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>

using namespace std;

// Need to consider
//   estop

class DiffDriveNode {
public:
    DiffDriveNode();
    static void sigint_cb(int sig);

    // RS232 Command functions
    void send_motor_cmd(serial::Serial* ser, const char* cmd);
    int ask_motor_feedback(serial::Serial* ser, const char* cmd);

    serial::Serial* ser_l_;
    serial::Serial* ser_r_;

private:
    void emergency_stop(void);
    void motors_init(int baudrate);
    // ROS timer callback
    void timer_cb(const ros::TimerEvent& event);
    // ROS subscriber callback
    void cmd_cb(const geometry_msgs::Twist& msg);
    // ROS service callback
    bool rst_odom_cb(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp);

    // ROS related
    ros::NodeHandle nh_;                            // Private ros node handler
    ros::Timer timer_;
    ros::Publisher pub_odom_;  
    ros::Subscriber sub_cmd_;
    ros::ServiceServer srv_rst_odom_;                     
    geometry_msgs::Twist cmd_msg_;                  // velocity msg;
    tf::TransformBroadcaster  odom_broadcaster_;

    // For odometry feedback
    bool is_first_odom_ = true;
    volatile int pulsel_, pulser_;
    volatile int last_pulsel_, last_pulser_;
    volatile double robot_x_, robot_y_, robot_theta_;
    volatile double desire_robot_x_, desire_robot_y_, desire_robot_theta_;

    // Motor configure related
    string ser_name_l_;
    string ser_name_r_;
    double wheel_radius_;
    double wheel_dis_;
    double gear_ratio_;




    // Timer related
    double timer_interval_;                         // ROS Timer interval
    double wtd_interval_;                           // Watchdog timer interval
    ros::Time last_cmd_time_ = ros::Time();
};