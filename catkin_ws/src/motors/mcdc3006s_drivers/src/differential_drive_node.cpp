#include <stdio.h>
#include <stdlib.h>

// For ROS
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Header.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include "mcdc3006s_drivers/mcdc3006s.h"

#define TRUE 1
#define FALSE 0

typedef int boolean;

using namespace std;

static const string FRAME_ID = "base_link";
static const string COLOR_RED = "\e[0;31m";
static const string COLOR_GREEN = "\e[0;32m";
static const string COLOR_YELLOW = "\e[0;33m"; 
static const string COLOR_NC = "\e[0m";

// Parameter:
//   motor_controlling rate
//   motor_radius
//   motor_distance
//   serial_buad_rate

// Need to consider
//   estop
// JS Stick Letters
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
    void check_ret_val(int ret_val, char* serialDevice, char* sem);
    void timer_cb(const ros::TimerEvent& event);
    void cmd_cb(const geometry_msgs::Twist& msg); 

    Mcdc3006s motor_l;
    Mcdc3006s motor_r;

private:
    void motor_init(driverConf_t* config, int baudrate);
    
    const char *l_sem_ = "/tmp/tmpSemaphore_l";
    const char *r_sem_ = "/tmp/tmpSemaphore_r";

    ros::NodeHandle nh_;                            // Private node handler
    ros::Timer timer_;
    ros::Publisher pub_odom_;  
    ros::Subscriber sub_cmd_;                     
    geometry_msgs::Twist cmd_msg_;                  // velocity msg;
    geometry_msgs::Twist old_cmd_msg_;

    string serial_l_;
    string serial_r_;
    driverSensor_t odo_l_;
    driverSensor_t odo_r_;

    double wheel_radius;
    double wheel_dis_;
    double gear_ratio_=14.0;
    
    double timer_interval_;                         // ROS Timer interval
    double wt_interval_;                            // Watchdog timer interval
    ros::Time last_cmd_time = ros::Time();
};


//    __   __        __  ___  __        __  ___  __   __  
//   /  ` /  \ |\ | /__`  |  |__) |  | /  `  |  /  \ |__) 
//   \__, \__/ | \| .__/  |  |  \ \__/ \__,  |  \__/ |  \ 
//  
DiffDriveNode::DiffDriveNode(){
    // ROS publisher
    // pub_sound_ = nh_.advertise<robotx_msgs::HydrophoneData>("hydrophone_data", 10);
    sub_cmd_ = nh_.subscribe("/cmd_vel", 1, &DiffDriveNode::cmd_cb, this);  

    // Motor parameters
    driverConf_t config;
    int tmp, baudrate;
    ros::param::param<int>("~maxPos", (int&)config.maxPos, 100000);
    ros::param::param<int>("~minPos", (int&)tmp, -100000); config.minPos = (long)tmp;
    ros::param::param<int>("~maxVel", (int&)config.maxVel, 1000);
    ros::param::param<int>("~maxAcc", (int&)config.maxAcc, 200);
    ros::param::param<int>("~maxDec", (int&)config.maxDec, 200);
    ros::param::param<int>("~baudRate", baudrate, 115200);
    ros::param::param<std::string>("~serial_lDevice", serial_l_, "/dev/walker_motor_left");
    ros::param::param<std::string>("~serial_rDevice", serial_r_, "/dev/walker_motor_right");
    ros::param::param<double>("~wheelWidth", wheel_radius, 0.105);
    ros::param::param<double>("~wheelDistance", wheel_dis_, 0.59);

    // ROS related parameters
    ros::param::param<double>("~timerInterval", timer_interval_, 0.1);
    ros::param::param<double>("~wtInterval", wt_interval_, 0.5);

    // Motor configuration
    motor_init(&config, baudrate);
    
    // Timer setup
    timer_ = nh_.createTimer(ros::Duration(timer_interval_), &DiffDriveNode::timer_cb, this);
}

void DiffDriveNode::timer_cb(const ros::TimerEvent& event) {
    // Watchdog for robot safety
    if(ros::Time::now() - last_cmd_time >= ros::Duration(wt_interval_)){
        cmd_msg_ = geometry_msgs::Twist();
        last_cmd_time = ros::Time::now();
        // cout << "Stop robot automatically by watchdog counter." << endl;
    }

    double     v = cmd_msg_.linear.x;
    double omega = cmd_msg_.angular.z;
    double rpm_l = (2 * v - omega * wheel_dis_) / (2 * wheel_radius) * 60 * gear_ratio_;
    double rpm_r = (2 * v + omega * wheel_dis_) / (2 * wheel_radius) * 60 * gear_ratio_;
    cout << "rmp_l: " << rpm_l << ", rmp_r: " << rpm_r << endl;

    if (motor_l.move_vel(rpm_l) != ERR_NOERR)
        fprintf(stderr, "There has been an error.\n\r");
    if (motor_r.move_vel(-rpm_r) != ERR_NOERR)              // notice robot setup
        fprintf(stderr, "There has been an error.\n\r");
} 

void DiffDriveNode::cmd_cb(const geometry_msgs::Twist& msg) {
    cmd_msg_ = msg;
    last_cmd_time = ros::Time::now();
}

void DiffDriveNode::motor_init(driverConf_t* config, int baudrate){  
    int ret_val;
    char serialDevice[128];
    strcpy(serialDevice, serial_l_.c_str());
    ret_val = motor_l.init(baudrate, serialDevice, (char*)l_sem_);
    check_ret_val(ret_val, serialDevice, (char*)r_sem_);

    strcpy(serialDevice, serial_r_.c_str());
    ret_val = motor_r.init(baudrate, serialDevice, (char*)r_sem_);
    check_ret_val(ret_val, serialDevice, (char*)r_sem_);
    
    motor_l.set_config(*config);
    motor_r.set_config(*config);

    cout << COLOR_GREEN << "Motor drivers are ready." << COLOR_NC << endl;
}


void DiffDriveNode::check_ret_val(int ret_val, char* serialDevice, char *sem){
    switch(ret_val) {
        case -1:
            fprintf(stderr, "Could not establish connections with serial port at %s\n\r", serialDevice);
            exit(-1);
        case -2:
            fprintf(stderr, "Could not create semaphore file at %s\n\r", sem);
            exit(-1);
        case -3:
            fprintf(stderr, "Could not set zero odometry\n\r");
            exit(-1);
        case -4:
            fprintf(stderr, "Semaphore related error\n\r");
            exit(-1);
        default:
            break;
    }
}

void sigint_cb(int sig)
{
  cout << "\nfucking close program !!" << endl;

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
    // node.run();
    signal(SIGINT, sigint_cb);
    ros::spin();
    return 0;
}
