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
nav_msgs::OccupancyGrid* a;

void map_cb(const nav_msgs::OccupancyGrid &map_msg) {
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    a = new nav_msgs::OccupancyGrid(map_msg);
    int map_width = map_msg.info.width;
    int map_height = map_msg.info.height;

    nav_msgs::Path result_path;
    result_path.header.frame_id = map_msg.header.frame_id;
    Astar::Solver solver;
    int origin_idx = map_width * map_height / 2 - map_width/2; 
    bool flag_success = solver.solve_ros(a, &result_path, origin_idx, origin_idx + 15, 10.0);
    pub_path.publish(result_path);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
}

void sigint_cb(int sig) {
    cout << "\nNode name: " << ros::this_node::getName() << " is shutdown." << endl;
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_finding_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_scan = nh.subscribe("local_map", 5, map_cb);
    pub_path = nh.advertise<nav_msgs::Path>("path_test", 1);
    signal(SIGINT, sigint_cb);
    ros::spin();
    return 0;
}
