#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

// For ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include "serial/serial.h"

using namespace std;

static const string COLOR_RED = "\e[0;31m";
static const string COLOR_GREEN = "\e[0;32m";
static const string COLOR_YELLOW = "\e[0;33m"; 
static const string COLOR_NC = "\e[0m";

// Need to consider
//   estop

namespace geometry_msgs {
    bool operator== (const Twist &cmd1, const Twist &cmd2) {
        double epsilon = 1e-6;
        return std::fabs(cmd1.linear.x - cmd2.linear.x) < epsilon && 
                std::fabs(cmd1.angular.z - cmd2.angular.z) < epsilon;
    }
    bool operator!= (const Twist &cmd1, const Twist &cmd2) {
        return !(cmd1 == cmd2);
    }
}

class DiffDriveNode {
public:
    DiffDriveNode();
    static void sigint_cb(int sig);
    void send_motor_cmd(serial::Serial* ser, const char* cmd);
    int ask_motor_feedback(serial::Serial* ser, const char* cmd);

    serial::Serial* ser_l_;
    serial::Serial* ser_r_;

private:
    void motors_init(int baudrate);
    void timer_cb(const ros::TimerEvent& event);
    void cmd_cb(const geometry_msgs::Twist& msg);

    // ROS related
    ros::NodeHandle nh_;                            // Private ros node handler
    ros::Timer timer_;
    ros::Publisher pub_odom_;  
    ros::Subscriber sub_cmd_;                     
    geometry_msgs::Twist cmd_msg_;                  // velocity msg;
    tf::TransformBroadcaster  odom_broadcaster_;
    volatile int real_rpml_, real_rpmr_;
    volatile int last_real_rpml_, last_real_rpmr_;
    bool is_first_odom = true;
    double robot_x_, robot_y_, robot_theta_;


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


//    __   __        __  ___  __        __  ___  __   __  
//   /  ` /  \ |\ | /__`  |  |__) |  | /  `  |  /  \ |__) 
//   \__, \__/ | \| .__/  |  |  \ \__/ \__,  |  \__/ |  \ 
//  
DiffDriveNode::DiffDriveNode(){
    // ROS publisher & subscriber
    pub_odom_ = nh_.advertise<nav_msgs::Odometry>("wheel_odom", 50);
    sub_cmd_ = nh_.subscribe("/cmd_vel", 1, &DiffDriveNode::cmd_cb, this);  

    // driverConf_t config;
    // ros::param::param<int>("~maxPos", (int&)config.maxPos, 100000);
    // ros::param::param<int>("~minPos", (int&)tmp, -100000); config.minPos = (long)tmp;
    // ros::param::param<int>("~maxVel", (int&)config.maxVel, 1000);
    // ros::param::param<int>("~maxAcc", (int&)config.maxAcc, 200);
    // ros::param::param<int>("~maxDec", (int&)config.maxDec, 200);

    int baudrate;
    ros::param::param<int>("~baudRate", baudrate, 115200);
    ros::param::param<std::string>("~serial_lDevice", ser_name_l_, "/dev/walker_motor_left");
    ros::param::param<std::string>("~serial_rDevice", ser_name_r_, "/dev/walker_motor_right");
    ros::param::param<double>("~wheelWidth", wheel_radius_, 0.105);
    ros::param::param<double>("~wheelDistance", wheel_dis_, 0.59);
    ros::param::param<double>("~gearRatio", gear_ratio_, 14.0);

    // Timer parameters
    ros::param::param<double>("~timerInterval", timer_interval_, 0.1);
    ros::param::param<double>("~wtdInterval", wtd_interval_, 0.5);  // Watchdog for robot safety

    motors_init(baudrate);

    signal(SIGINT, sigint_cb);

    // Timer setup
    timer_ = nh_.createTimer(ros::Duration(timer_interval_), &DiffDriveNode::timer_cb, this);
}

void DiffDriveNode::send_motor_cmd(serial::Serial* ser, const char* cmd){
    string cmd_plus_end = string(cmd) + "\n\r"; 
    ser->write(cmd_plus_end);
    usleep(100);
    ser->readline();
}

int DiffDriveNode::ask_motor_feedback(serial::Serial* ser, const char* cmd){
    string cmd_plus_end = string(cmd) + "\n\r"; 
    ser->write(cmd_plus_end);
    usleep(100);
    return atoi(ser->readline().c_str());
}

void DiffDriveNode::timer_cb(const ros::TimerEvent& event) {
    // Watchdog for robot safety
    if(ros::Time::now() - last_cmd_time_ >= ros::Duration(wtd_interval_)){
        cmd_msg_ = geometry_msgs::Twist();      // zero v, zero omega
        last_cmd_time_ = ros::Time::now();
        // cout << "Stop robot automatically by watchdog counter." << endl;
    }

    double     v = cmd_msg_.linear.x;
    double omega = cmd_msg_.angular.z;
    double desire_rpml = (2 * v - omega * wheel_dis_) / (2 * wheel_radius_) * 60 * gear_ratio_;
    double desire_rpmr = (2 * v + omega * wheel_dis_) / (2 * wheel_radius_) * 60 * gear_ratio_;

    // Send cmd to motor & get motor pulse
    char cmd[15];
    sprintf(cmd, "V%d", (int)desire_rpml);
    send_motor_cmd(ser_l_, cmd);
    sprintf(cmd, "V%d", -(int)desire_rpmr);     // notice the "minus" note due to right motor setup
    send_motor_cmd(ser_r_, cmd);
    real_rpml_ = ask_motor_feedback(ser_l_, "POS");
    real_rpmr_ = ask_motor_feedback(ser_r_, "POS");

    cout << "rmp_l: " << desire_rpml << ", rmp_r: " << desire_rpmr << endl;
    cout << "real_l: " << real_rpml_ << ", real_r: " << real_rpmr_ << endl;

    // Wheel odometry process
    if(is_first_odom) {
        is_first_odom = false;
        robot_x_ = robot_y_ = robot_theta_ = 0.0;
    }
    int delta_rpml = real_rpml_ - last_real_rpml_;
    int delta_rpmr = real_rpmr_ - last_real_rpmr_;
    double vl = (double)delta_rpml / 41400.0 * 2 * M_PI / timer_interval_;
    double vr = -(double)delta_rpmr / 41400.0 * 2 * M_PI / timer_interval_;     // notice the "minus"
    double real_v = wheel_radius_ / 2 * (vr + vl);
    double real_omega = wheel_radius_ / wheel_dis_ * (vr - vl);
    
    robot_x_ += real_v * cos(robot_theta_) * timer_interval_;
    robot_y_ += real_v * sin(robot_theta_) * timer_interval_;
    robot_theta_ += real_omega * timer_interval_;

    ros::Time current_time = ros::Time::now();
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_theta_);
    geometry_msgs::TransformStamped odom_trans;
    
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = robot_x_;
    odom_trans.transform.translation.y = robot_y_;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster_.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = robot_x_;
    odom.pose.pose.position.y = robot_y_;
    odom.pose.pose.orientation = odom_quat;
    pub_odom_.publish(odom);

    last_real_rpml_ = real_rpml_;
    last_real_rpmr_ = real_rpmr_;
} 

void DiffDriveNode::cmd_cb(const geometry_msgs::Twist& msg) {
    cmd_msg_ = msg;
    last_cmd_time_ = ros::Time::now();
}

void DiffDriveNode::motors_init(int baudrate) {
    // Motors port init
    try {
        ser_l_ = new serial::Serial(ser_name_l_, baudrate, serial::Timeout::simpleTimeout(1000));
    } catch(exception & e){
        cout << COLOR_RED << "Can not open serial port: " << ser_name_l_ << ", please check the port is available." << COLOR_NC << endl;
    }
    try {
        ser_r_ = new serial::Serial(ser_name_r_, baudrate, serial::Timeout::simpleTimeout(1000));
    } catch(exception & e){
        cout << COLOR_RED << "Can not open serial port: " << ser_name_r_ << ", please check the port is available." << COLOR_NC << endl;
        exit(-1);
    }

    // Set Acceleration and deceleration maximum
    send_motor_cmd(ser_l_, "AC10");
    send_motor_cmd(ser_l_, "DEC100");
    send_motor_cmd(ser_r_, "AC10");
    send_motor_cmd(ser_r_, "DEC100");

    cout << COLOR_GREEN << "Motor drivers are ready." << COLOR_NC << endl;
}

void DiffDriveNode::sigint_cb(int sig) {
    cout << "\nNode name: " << ros::this_node::getName() << " is shutdown." << endl;
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}


//            
//   |\/|  /\  | |\ | 
//   |  | /~~\ | | \| 
//  
int main (int argc, char** argv) {
    ros::init(argc, argv, "differential_drive_node"); 
    DiffDriveNode node;
    ros::spin();

    // Safety concern
    node.ser_l_->write("V0\r\n");
    node.ser_r_->write("V0\r\n");
    return 0;
}
