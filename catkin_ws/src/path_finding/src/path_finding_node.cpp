#include <chrono>
#include <signal.h>
#include <math.h>

// ROS
#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

// Custom library
#include "Astar.hpp"

using namespace std;


class AstarPathfindingNode {
public:
    AstarPathfindingNode(ros::NodeHandle nh, ros::NodeHandle pnh);
    static void sigint_cb(int sig);
    void localmap_cb(const nav_msgs::OccupancyGrid::ConstPtr &map_msg_ptr);
    int get_cost(vector<int8_t> vec, int map_width, int map_height, int target_idx);

    // ROS related
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber sub_localmap_;
    ros::Publisher pub_walkable_path_;
    ros::Publisher pub_marker_array_;

    double solver_timeout_ms_; 
    visualization_msgs::Marker mkr_subgoal_candidate_;
};


AstarPathfindingNode::AstarPathfindingNode(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh) {
    // Signal handler
    signal(SIGINT, sigint_cb);

    // ROS parameters
    ros::param::param<double>("~solver_timeout_ms", solver_timeout_ms_, 40.0);

    // ROS publishers & subscribers
    sub_localmap_ = nh.subscribe("local_map", 5, &AstarPathfindingNode::localmap_cb, this);
    pub_walkable_path_ = nh.advertise<nav_msgs::Path>("workable_path", 1);
    pub_marker_array_ = nh.advertise<visualization_msgs::MarkerArray>("path_vis", 1);

    // Marker init
    mkr_subgoal_candidate_.header.stamp = ros::Time();
    mkr_subgoal_candidate_.ns = "subgoal_candidate";
    mkr_subgoal_candidate_.type = visualization_msgs::Marker::LINE_LIST;
    mkr_subgoal_candidate_.action = visualization_msgs::Marker::ADD;
    mkr_subgoal_candidate_.pose.orientation.w = 1.0;
    mkr_subgoal_candidate_.scale.x = 0.05;
    mkr_subgoal_candidate_.color.a = 0.2; // Don't forget to set the alpha!
    mkr_subgoal_candidate_.color.r = 1.0;
    mkr_subgoal_candidate_.color.g = 1.0;
    mkr_subgoal_candidate_.color.b = 1.0;
    mkr_subgoal_candidate_.lifetime = ros::Duration(2.0);
}


int AstarPathfindingNode::get_cost(vector<int8_t> vec, int map_width, int map_height, int target_idx) {
    int kernel_size = 3;
    int bound = kernel_size / 2;
    int cost = 0;
    for(int y = -bound; y <= bound; y++) {
        for (int x = -bound; x <= bound; x++) {
            int op_idx = target_idx + x + map_width * y;
            if(op_idx < 0 || op_idx > map_width * map_height) continue;           // upper and bottom bound
            else if(abs((op_idx % map_width) - (target_idx % map_width)) > bound) continue;  // left and right bound
            else 
                cost += vec[op_idx];
        }
    }
    return cost;
}


void AstarPathfindingNode::localmap_cb(const nav_msgs::OccupancyGrid::ConstPtr &map_msg_ptr) {
    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    double map_resolution = map_msg_ptr->info.resolution;
    double map_origin_x = map_msg_ptr->info.origin.position.x;
    double map_origin_y = map_msg_ptr->info.origin.position.y;
    int map_width = map_msg_ptr->info.width;
    int map_height = map_msg_ptr->info.height;
    int map_limit = map_width * map_height;

    nav_msgs::Path result_path;
    result_path.header.frame_id = map_msg_ptr->header.frame_id;
    Astar::Solver solver;

    // coordinate to map grid
    double origin_x, origin_y;
    origin_x = 0.0;
    origin_y = 0.0;
    int map_x = std::round((origin_x - map_origin_x) / map_resolution);
    int map_y = std::round((origin_y - map_origin_y) / map_resolution);
    int idx = map_y * map_width + map_x;

    bool flag_success = solver.solve_ros(map_msg_ptr, &result_path, idx, idx + 50, solver_timeout_ms_);
    if(flag_success){
        result_path.header.stamp = ros::Time::now();
        pub_walkable_path_.publish(result_path);
    }
    else{
        ROS_WARN("No solution for path finding in timeout: %.1f ms", solver_timeout_ms_);
        result_path.header.stamp = ros::Time::now();
        pub_walkable_path_.publish(result_path);
    }

    // Marker reset
    visualization_msgs::MarkerArray mrk_array_;
    mkr_subgoal_candidate_.header.frame_id = map_msg_ptr->header.frame_id;
    mkr_subgoal_candidate_.points.clear();  

    double prefer_subgoal_distance = 8.0;
    double distance_resolution = map_resolution * 2;
    for(int i = 0; i <= 18; i++) {
        double theta_from_yaxis = M_PI / 18 * i;
        int max_distance_idx = std::round(prefer_subgoal_distance / distance_resolution);
        double tmp_dis;
        for(int j = 1; j < max_distance_idx; j++) {
            tmp_dis = distance_resolution * j;
            int map_x = std::round((tmp_dis * std::sin(theta_from_yaxis) - map_origin_x) / map_resolution);
            int map_y = std::round((tmp_dis * std::cos(theta_from_yaxis) - map_origin_y) / map_resolution);
            int idx = map_y * map_width + map_x;
            // if(get_cost((map_msg_ptr->data), map_width, map_height, idx) >= 50) {
            if((0 < idx) && (idx < map_limit)){
                if(map_msg_ptr->data[idx] >= 50) {
                    tmp_dis -= distance_resolution;
                    break;
                }
            }
        }

        mkr_subgoal_candidate_.id = i;
        
        double tmp_x = tmp_dis * std::sin(theta_from_yaxis);
        double tmp_y = tmp_dis * std::cos(theta_from_yaxis);
        
        geometry_msgs::Point pt;
        mkr_subgoal_candidate_.points.push_back(pt);
        pt.x = tmp_x;
        pt.y = tmp_y;
        mkr_subgoal_candidate_.points.push_back(pt);
    }

    
    mrk_array_.markers.push_back(mkr_subgoal_candidate_);



    pub_marker_array_.publish(mrk_array_);

    // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
}


void AstarPathfindingNode::sigint_cb(int sig) {
    cout << "\nNode name: " << ros::this_node::getName() << " is shutdown." << endl;
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "astar_path_finding_node");
    ros::NodeHandle nh, pnh("~");
    AstarPathfindingNode node(nh, pnh);    
    ros::spin();
    return 0;
}
