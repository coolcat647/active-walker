
#include "differential_drive_node.hpp"

using namespace std;

static const string COLOR_RED = "\e[0;31m";
static const string COLOR_GREEN = "\e[0;32m";
static const string COLOR_YELLOW = "\e[0;33m"; 
static const string COLOR_NC = "\e[0m";

// Custom operators
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

//    __   __        __  ___  __        __  ___  __   __  
//   /  ` /  \ |\ | /__`  |  |__) |  | /  `  |  /  \ |__) 
//   \__, \__/ | \| .__/  |  |  \ \__/ \__,  |  \__/ |  \ 
//  
DiffDriveNode::DiffDriveNode(){
    // ROS publisher & subscriber
    pub_odom_ = nh_.advertise<nav_msgs::Odometry>("wheel_odom", 50);
    sub_cmd_ = nh_.subscribe("/cmd_vel", 1, &DiffDriveNode::cmd_cb, this);
    srv_rst_odom_ = nh_.advertiseService("reset_odom", 
                                                   &DiffDriveNode::rst_odom_cb,
                                                   this);    

    int baudrate;
    ros::param::param<int>("~baud", baudrate, 115200);
    ros::param::param<std::string>("~portL", ser_name_l_, "/dev/walker_motor_left");
    ros::param::param<std::string>("~portR", ser_name_r_, "/dev/walker_motor_right");
    ros::param::param<double>("~whlRadius", wheel_radius_, 0.0625); // 0.105
    ros::param::param<double>("~whlDistance", wheel_dis_, 0.6);     // 0.59
    ros::param::param<double>("~gearRatio", gear_ratio_, 14.0); // 13.69863

    // Timer parameters
    ros::param::param<double>("~timerInterval", timer_interval_, 0.1);
    ros::param::param<double>("~wtdInterval", wtd_interval_, 0.5);  // Watchdog for robot safety

    motors_init(baudrate);

    signal(SIGINT, sigint_cb);
    cout <<setprecision(3) << setiosflags(ios::fixed);
    
    // Timer setup
    timer_ = nh_.createTimer(ros::Duration(timer_interval_), &DiffDriveNode::timer_cb, this);
}

bool DiffDriveNode::rst_odom_cb(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp) {
    robot_x_ = 0;
    robot_y_ = 0;
    robot_theta_ = 0;
    is_first_odom_ = true;

    string message = "Reset wheel odometry.";
    cout << COLOR_GREEN << message << COLOR_NC << endl;
    resp.success = true; // boring, but valid response info
    resp.message = message;
    return true;
}

void DiffDriveNode::send_motor_cmd(serial::Serial* ser, const char* cmd){
    string cmd_plus_end = string(cmd) + "\n"; 
    ser->write(cmd_plus_end);
    usleep(100);
    ser->readline();
}

int DiffDriveNode::ask_motor_feedback(serial::Serial* ser, const char* cmd){
    string cmd_plus_end = string(cmd) + "\n"; 
    ser->write(cmd_plus_end);
    usleep(100);
    return atoi(ser->readline().c_str());
}

double err_rpm_r = 0, err_rpm_l = 0;

void DiffDriveNode::timer_cb(const ros::TimerEvent& event) {
    // Watchdog for robot safety
    if(ros::Time::now() - last_cmd_time_ >= ros::Duration(wtd_interval_)){
        cmd_msg_ = geometry_msgs::Twist();      // zero v, zero omega
        last_cmd_time_ = ros::Time::now();
        // cout << "Stop robot automatically by watchdog counter." << endl;
    }
    double     v = cmd_msg_.linear.x;
    double omega = cmd_msg_.angular.z;
    double desire_rpml = (2 * v - omega * wheel_dis_) / (2 * wheel_radius_) / M_PI / 2 * 60 * gear_ratio_;
    double desire_rpmr = (2 * v + omega * wheel_dis_) / (2 * wheel_radius_) / M_PI / 2 * 60 * gear_ratio_;

    // Send cmd to motor & get motor pulse
    char cmd[15];
    sprintf(cmd, "V%d", (int)desire_rpml);
    send_motor_cmd(ser_l_, cmd);
    sprintf(cmd, "V%d", -(int)desire_rpmr);     // notice the "minus" note due to right motor setup
    send_motor_cmd(ser_r_, cmd);

    pulsel_ = ask_motor_feedback(ser_l_, "POS");
    pulser_ = ask_motor_feedback(ser_r_, "POS");
    

    // Wheel odometry process
    if(is_first_odom_) {
        is_first_odom_ = false;
        robot_x_ = robot_y_ = robot_theta_ = 0.0;
        last_pulsel_ = pulsel_;
        last_pulser_ = pulser_;

        desire_robot_x_ = desire_robot_y_ = desire_robot_theta_ = 0.0;
    }
    // double vl = (double)(pulsel_ - last_pulsel_) / 41400.0 * 2 * M_PI / timer_interval_;
    // double vr = -(double)(pulser_ - last_pulser_) / 41400.0 * 2 * M_PI / timer_interval_;     // notice the "minus"
    double vl = (double)(pulsel_ - last_pulsel_) / 3000 / gear_ratio_ * 2 * M_PI / timer_interval_;
    double vr = -(double)(pulser_ - last_pulser_) / 3000 / gear_ratio_ * 2 * M_PI / timer_interval_;     // notice the "minus"
    
    bool display_for_motor = false;
    if(display_for_motor){
        cout << "motor -> desire (rpm_l, rpm_r): " << desire_rpml << ", " << desire_rpmr \
                << "\nmotor ->   real (rpm_l, rpm_r): " <<  vl * gear_ratio_ * 60 / 2 / M_PI \
                << ", " << vr * gear_ratio_ * 60 / 2 / M_PI << endl;
    }else{
        cout << "wheel -> desire (rpm_l, rpm_r): " << desire_rpml / gear_ratio_ \
                << ", " << desire_rpmr / gear_ratio_ \
                << "\nwheel ->   real (rpm_l, rpm_r): " <<  vl * 60 / 2 / M_PI \
                << ", " << vr * 60 / 2 / M_PI << endl;
    }
    // double tmp_r = abs(desire_rpml - vl * gear_ratio_ * 60 / 2 / M_PI);
    // double tmp_l = abs(desire_rpmr - vr * gear_ratio_ * 60 / 2 / M_PI);
    // if(tmp_l > err_rpm_l && tmp_l < 10) {
    //     err_rpm_l = tmp_l;
    // }
    // if(tmp_r > err_rpm_r && tmp_r < 10) {
    //     err_rpm_r = tmp_r;
    // }
    // cout << "max err rpm_l, rpmr = " << err_rpm_l << ", " <<  err_rpm_r << endl;

    double real_v = wheel_radius_ / 2 * (vr + vl);
    double real_omega = wheel_radius_ / wheel_dis_ * (vr - vl);
    
    // cout << "real (v, w) = " << real_v << ", " << real_omega << endl;

    robot_x_ += real_v * cos(robot_theta_) * timer_interval_;
    robot_y_ += real_v * sin(robot_theta_) * timer_interval_;
    robot_theta_ += real_omega * timer_interval_;

    desire_robot_x_ += v * cos(desire_robot_theta_) * timer_interval_;
    desire_robot_y_ += v * sin(desire_robot_theta_) * timer_interval_;
    desire_robot_theta_ += omega * timer_interval_;

    // cout << "err_odom_x: " << desire_robot_x_ - robot_x_ << "err_odom_y: " << desire_robot_y_ - robot_y_ << endl;


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

    last_pulsel_ = pulsel_;
    last_pulser_ = pulser_;
} 

void DiffDriveNode::cmd_cb(const geometry_msgs::Twist& msg) {
    cmd_msg_ = msg;
    last_cmd_time_ = ros::Time::now();
}

void DiffDriveNode::motors_init(int baudrate) {
    // Motors port init
    try {
        ser_l_ = new serial::Serial(ser_name_l_, baudrate, serial::Timeout::simpleTimeout(timer_interval_*1000));
    } catch(exception & e){
        cout << COLOR_RED << "Can not open serial port: " << ser_name_l_ << ", please check the port is available." << COLOR_NC << endl;
    }
    try {
        ser_r_ = new serial::Serial(ser_name_r_, baudrate, serial::Timeout::simpleTimeout(timer_interval_*1000));
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
