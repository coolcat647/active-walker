#include <chrono>
#include <signal.h>

#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "Astar.hpp"

using namespace std;

ros::Publisher pub_map;
ros::Publisher pub_path;
double solver_timeout_ms;

void map_cb(const nav_msgs::OccupancyGrid::ConstPtr &map_msg_ptr) {
    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    int map_width = map_msg_ptr->info.width;
    int map_height = map_msg_ptr->info.height;

    nav_msgs::Path result_path;
    result_path.header.frame_id = map_msg_ptr->header.frame_id;
    Astar::Solver solver;
    int origin_idx = map_width * map_height / 2 - map_width/2; 
    bool flag_success = solver.solve_ros(map_msg_ptr, &result_path, origin_idx, origin_idx + 50, solver_timeout_ms);
    if(flag_success){
        result_path.header.stamp = ros::Time::now();
        pub_path.publish(result_path);
    }
    else{
        ROS_WARN("No solution for path finding in timeout: %.1f ms", solver_timeout_ms);
        result_path.header.stamp = ros::Time::now();
        pub_path.publish(result_path);
    }

    // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
}

void sigint_cb(int sig) {
    cout << "\nNode name: " << ros::this_node::getName() << " is shutdown." << endl;
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_finding_node");
    signal(SIGINT, sigint_cb);

    ros::NodeHandle nh;
    ros::Subscriber sub_scan = nh.subscribe("local_map", 5, map_cb);
    pub_path = nh.advertise<nav_msgs::Path>("path_test", 1);

    ros::param::param<double>("~solver_timeout_ms", solver_timeout_ms, 40.0);
    
    ros::spin();
    return 0;
}
