#include <stdio.h>
#include <stdlib.h>

// For ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include "mcdc3006s_drivers/mcdc3006s.h"

#define TRUE 1
#define FALSE 0

#define RIGHT_MOTOR_SERIAL_DEVICE   "/dev/ttyUSB0"
#define LEFT_MOTOR_SERIAL_DEVICE   "/dev/ttyUSB1"

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


class DiffDriveNode {
public:
    DiffDriveNode();
    void check_ret_val(int ret_val, char* serialDevice, char* sem);

    Mcdc3006s l_motor;
    Mcdc3006s r_motor;

private:
    ros::NodeHandle nh_;                            // Private node handler
    ros::Publisher pub_odom_;                       
    // robotx_msgs::HydrophoneData sound_msg_;      // Hydrophone data message

    string l_serial_;
    string r_serial_;

    driverSensor_t l_odo_;
    driverSensor_t r_odo_;

    const char *l_sem_ = "/tmp/tmpSemaphore_l";
    const char *r_sem_ = "/tmp/tmpSemaphore_r";
};


//    __   __        __  ___  __        __  ___  __   __  
//   /  ` /  \ |\ | /__`  |  |__) |  | /  `  |  /  \ |__) 
//   \__, \__/ | \| .__/  |  |  \ \__/ \__,  |  \__/ |  \ 
//  
DiffDriveNode::DiffDriveNode(){
    // ROS publisher
    // pub_sound_ = nh_.advertise<robotx_msgs::HydrophoneData>("hydrophone_data", 10);
    
    driverConf_t config;
    int baudrate = 115200;
    int tmp;
    ros::param::get("~maxPos", tmp); config.maxPos = tmp;
    ros::param::get("~minPos", tmp); config.minPos = tmp;
    ros::param::get("~maxVel", tmp); config.maxVel = tmp;
    ros::param::get("~maxAcc", tmp); config.maxAcc = tmp;
    ros::param::get("~maxDec", tmp); config.maxDec = tmp;
    ros::param::get("~baudRate", tmp); baudrate = tmp;
    if(!ros::param::get("~l_serialDevice", l_serial_)){
        cout << COLOR_YELLOW << "No left_serial_device param has been passed, use " << LEFT_MOTOR_SERIAL_DEVICE << COLOR_NC << endl;
        l_serial_ = LEFT_MOTOR_SERIAL_DEVICE;
    }
    if(!ros::param::get("~r_serialDevice", r_serial_)){
        cout << COLOR_YELLOW << "No right_serial_device param has been passed, use " << RIGHT_MOTOR_SERIAL_DEVICE<< COLOR_NC << endl;
        r_serial_ = RIGHT_MOTOR_SERIAL_DEVICE;
    }
    
    int ret_val;
    char serialDevice[128];
    strcpy(serialDevice, l_serial_.c_str());
    ret_val = l_motor.init(baudrate, serialDevice, (char*)l_sem_);
    check_ret_val(ret_val, serialDevice, (char*)r_sem_);

    strcpy(serialDevice, r_serial_.c_str());
    ret_val = r_motor.init(baudrate, serialDevice, (char*)r_sem_);
    check_ret_val(ret_val, serialDevice, (char*)r_sem_);
    
    l_motor.set_config(config);
    r_motor.set_config(config);

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


//            
//   |\/|  /\  | |\ | 
//   |  | /~~\ | | \| 
//  
int main (int argc, char** argv) {
    ros::init(argc, argv, "differential_drive_node");   
    DiffDriveNode node;
    // node.run();
    return 0;
}
