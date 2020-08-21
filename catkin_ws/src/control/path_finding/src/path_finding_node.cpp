#include <chrono>
#include <signal.h>
#include <math.h>
#include <algorithm>

// ROS
#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

// TF
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"

// Custom library
#include "Astar.hpp"

using namespace std;


template<class ForwardIterator>
inline size_t argmin(ForwardIterator first, ForwardIterator last) {
    return std::distance(first, std::min_element(first, last));
}

template<class ForwardIterator>
inline size_t argmax(ForwardIterator first, ForwardIterator last) {
    return std::distance(first, std::max_element(first, last));
}


class AstarPathfindingNode {
public:
    AstarPathfindingNode(ros::NodeHandle nh, ros::NodeHandle pnh);
    static void sigint_cb(int sig);
    void localmap_cb(const nav_msgs::OccupancyGrid::ConstPtr &map_msg_ptr);
    int get_cost(vector<int8_t> vec, int map_width, int map_height, int target_idx);
    void timer_cb(const ros::TimerEvent&);

    // ROS related
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber sub_localmap_;
    ros::Publisher pub_walkable_path_;
    ros::Publisher pub_marker_array_;
    ros::Timer timer_;
    nav_msgs::OccupancyGrid::ConstPtr localmap_ptr_;
    string path_frame_id_;

    // TF related
    tf::TransformListener tflistener_;

    double subgoal_timer_interval_;
    double solver_timeout_ms_; 
    visualization_msgs::Marker mkr_subgoal_candidate_;
    visualization_msgs::Marker mrk_subgoal_;
    bool flag_busy_;
};


AstarPathfindingNode::AstarPathfindingNode(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh) {
    // Signal handler
    signal(SIGINT, sigint_cb);

    // ROS parameters
    ros::param::param<double>("~solver_timeout_ms", solver_timeout_ms_, 40.0);
    ros::param::param<double>("~subgoal_timer_interval", subgoal_timer_interval_, 2.0);
    // Fixed parameters
    ros::param::param<string>("~path_frame_id", path_frame_id_, "odom");

    // ROS publishers & subscribers
    sub_localmap_ = nh_.subscribe("local_map", 5, &AstarPathfindingNode::localmap_cb, this);
    pub_walkable_path_ = nh_.advertise<nav_msgs::Path>("walkable_path", 1);
    pub_marker_array_ = nh_.advertise<visualization_msgs::MarkerArray>("path_vis", 1);

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
    mkr_subgoal_candidate_.lifetime = ros::Duration(subgoal_timer_interval_);

    mrk_subgoal_.header.frame_id = path_frame_id_;
    mrk_subgoal_.ns = "subgoal";
    mrk_subgoal_.type = visualization_msgs::Marker::SPHERE;
    mrk_subgoal_.action = visualization_msgs::Marker::ADD;
    mrk_subgoal_.pose.orientation.w = 1.0;
    mrk_subgoal_.scale.x = 0.4;
    mrk_subgoal_.scale.y = 0.4;
    mrk_subgoal_.scale.z = 0.4;
    mrk_subgoal_.color.a = 0.8;
    mrk_subgoal_.color.g = 1.0;
    mrk_subgoal_.lifetime = ros::Duration(subgoal_timer_interval_);
    mrk_subgoal_.id = 0;

    // Timer related
    flag_busy_ = false;
    timer_ = nh_.createTimer(ros::Duration(subgoal_timer_interval_), &AstarPathfindingNode::timer_cb, this);

    cout << ros::this_node::getName() << " is ready." << endl;
}


int AstarPathfindingNode::get_cost(vector<int8_t> vec, int map_width, int map_height, int target_idx) {
    int kernel_size = 5;
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
    if(!flag_busy_){
        localmap_ptr_ = map_msg_ptr;
    }
}

void AstarPathfindingNode::timer_cb(const ros::TimerEvent&){
    // Lock
    flag_busy_ = true;

    if(!localmap_ptr_){
        ROS_WARN("Empty local map, skip");
    } else{
        // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        double map_resolution = localmap_ptr_->info.resolution;
        double map_origin_x = localmap_ptr_->info.origin.position.x;
        double map_origin_y = localmap_ptr_->info.origin.position.y;
        int map_width = localmap_ptr_->info.width;
        int map_height = localmap_ptr_->info.height;
        int map_limit = map_width * map_height;
    
        // Marker reset
        visualization_msgs::MarkerArray mrk_array_;
        mkr_subgoal_candidate_.header.frame_id = localmap_ptr_->header.frame_id;
        mkr_subgoal_candidate_.points.clear();  
    
        std::vector<double> candidate_score_list;
        double prefer_subgoal_distance = 8.0;
        double distance_resolution = map_resolution * 4;

        // List all walkable space
        for(int i = 18; i >= 0; i--) {
            double theta_from_yaxis = M_PI / 18 * i;
            int max_distance_idx = std::round(prefer_subgoal_distance / distance_resolution);
            double tmp_dis;
            for(int j = 2; j <= max_distance_idx; j++) {
                tmp_dis = distance_resolution * j;
                int map_x = std::round((tmp_dis * std::sin(theta_from_yaxis) - map_origin_x) / map_resolution);
                int map_y = std::round((tmp_dis * std::cos(theta_from_yaxis) - map_origin_y) / map_resolution);
                int idx = map_y * map_width + map_x;
                if(get_cost((localmap_ptr_->data), map_width, map_height, idx) >= 50) {
                // if((0 < idx) && (idx < map_limit)){
                    if(localmap_ptr_->data[idx] >= 40) {
                        tmp_dis -= distance_resolution*2;
                        break;
                    }
                }
            }
            // Calculate score
            double score = tmp_dis / prefer_subgoal_distance * std::sin(theta_from_yaxis);
            candidate_score_list.push_back(score);

            // Visualization
            geometry_msgs::Point pt;
            mkr_subgoal_candidate_.points.push_back(pt);    // Origin point
            mkr_subgoal_candidate_.id = i;
            pt.x = tmp_dis * std::sin(theta_from_yaxis);
            pt.y = tmp_dis * std::cos(theta_from_yaxis);
            mkr_subgoal_candidate_.points.push_back(pt);
        }
        mrk_array_.markers.push_back(mkr_subgoal_candidate_);

        // Get transformation from base to odom
        tf::StampedTransform tf_base2odom;
        try{
            tflistener_.waitForTransform(path_frame_id_, "/base_link",
                                  ros::Time(0), ros::Duration(subgoal_timer_interval_));
            tflistener_.lookupTransform(path_frame_id_, "/base_link",
                               ros::Time(0), tf_base2odom);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        tf::Vector3 trans_base2odom = tf_base2odom.getOrigin();
        tf::Matrix3x3 rot_base2odom = tf_base2odom.getBasis();
        // cout << rot_base2odom[0][0] << " " << rot_base2odom[0][1] << " " << rot_base2odom[0][2] << endl;
        // cout << rot_base2odom[1][0] << " " << rot_base2odom[1][1] << " " << rot_base2odom[1][2] << endl;
        // cout << rot_base2odom[2][0] << " " << rot_base2odom[2][1] << " " << rot_base2odom[2][2] << "\n" << endl;

        // Find the farthest walkable space
        int index = argmax(candidate_score_list.begin(), candidate_score_list.end());
        // Convert goal point from baselink coordinate to odom coordinate
        tf::Vector3 subgoal_vec(mkr_subgoal_candidate_.points[index * 2 + 1].x, 
                                mkr_subgoal_candidate_.points[index * 2 + 1].y, 
                                mkr_subgoal_candidate_.points[index * 2 + 1].z);
        subgoal_vec = rot_base2odom * subgoal_vec + trans_base2odom;
        mrk_subgoal_.pose.position.x = subgoal_vec.getX();
        mrk_subgoal_.pose.position.y = subgoal_vec.getY();
        mrk_subgoal_.header.stamp = ros::Time();
        mrk_array_.markers.push_back(mrk_subgoal_);
        
        // Publish marker array
        pub_marker_array_.publish(mrk_array_);

        // Path finding
        nav_msgs::Path result_path;
        result_path.header.frame_id = path_frame_id_;
        Astar::Solver solver;
        // coordinate to map grid
        int origin_idx = std::round(-map_origin_y / map_resolution) * map_width + std::round(-map_origin_x / map_resolution);
        int map_x = std::round((mkr_subgoal_candidate_.points[index * 2 + 1].x - map_origin_x) / map_resolution);
        int map_y = std::round((mkr_subgoal_candidate_.points[index * 2 + 1].y - map_origin_y) / map_resolution);
        int target_idx = map_y * map_width + map_x;
    
        bool flag_success = solver.solve_ros(localmap_ptr_, &result_path, origin_idx, target_idx, solver_timeout_ms_);
        if(flag_success){
            for(std::vector<geometry_msgs::PoseStamped>::iterator it = result_path.poses.begin() ; it != result_path.poses.end(); ++it) {
                // tf::Quaternion q;
                tf::Matrix3x3 mat_raw(tf::Quaternion(it->pose.orientation.x, it->pose.orientation.y, it->pose.orientation.z, it->pose.orientation.w));
                tf::Vector3 vec_raw(it->pose.position.x, it->pose.position.y, it->pose.position.z);
                // tf::quaternionMsgToTF
                tf::Matrix3x3 mat_transformed = rot_base2odom * mat_raw;
                tf::Vector3 vec_transformed = rot_base2odom * vec_raw + trans_base2odom;
                it->pose.position.x = vec_transformed.getX();
                it->pose.position.y = vec_transformed.getY();
                it->pose.position.z = vec_transformed.getZ();
                tf::Quaternion q_transformed;
                mat_transformed.getRotation(q_transformed);
                // tf::quaternionTFToMsg(q_transformed, it->pose.orientation);
            }

            result_path.header.stamp = ros::Time();
            pub_walkable_path_.publish(result_path);
        }
        else{
            ROS_WARN("No solution for path finding in timeout: %.1f ms", solver_timeout_ms_);
            result_path.header.stamp = ros::Time();
            pub_walkable_path_.publish(result_path);
        }
    
        // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    }

    flag_busy_ = false;
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
