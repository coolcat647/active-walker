#include <chrono>
#include <cmath>
#include <math.h>
#include <signal.h>

#include "ros/ros.h"
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

// TF
#include <tf/transform_listener.h>

// PCL
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h> // ros2pcl
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGBPtr;


class Scan2LocalmapNode {
public:
    Scan2LocalmapNode(ros::NodeHandle nh, ros::NodeHandle pnh);
    static void sigint_cb(int sig);
    void butterworth_filter(vector<int8_t> &vec, int map_width, int map_height, int target_idx, int peak_value);
    void asymmetric_gaussian_filter(vector<int8_t> &vec, double map_resolution, int map_width, int map_height, int target_idx, double target_yaw, double target_speed, int peak_value);
    void butterworth_filter_generate(double filter_radius, int filter_order, double map_resolution, int peak_value);
    void scan_cb(const sensor_msgs::LaserScan &laser_msg);
    void scan_cb_deprecated(const sensor_msgs::LaserScan &laser_msg);

    // ROS related
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber sub_scan_;
    ros::Publisher pub_map_;
    ros::Publisher pub_footprint_;
    string localmap_frameid_;                               // Localmap frame_id
    nav_msgs::OccupancyGrid::Ptr localmap_ptr_;             // Localmap msg
    geometry_msgs::PolygonStamped::Ptr footprint_ptr_;      // Robot footprint
    laser_geometry::LaserProjection projector_;             // Projector of laserscan

    // TF listener
    tf::TransformListener* tflistener_ptr_;
    tf::StampedTransform tf_base2laser_;    

    // Inflation filter kernel
    vector<vector<int8_t> > inflation_kernel_;

    // PCL Cropbox filter
    pcl::CropBox<pcl::PointXYZ> box_filter_; 
    pcl::VoxelGrid<pcl::PointXYZ> vg_filter_;
};


Scan2LocalmapNode::Scan2LocalmapNode(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh) {
    // Signal handler
    signal(SIGINT, Scan2LocalmapNode::sigint_cb);

    // ROS parameters
    double inflation_radius;
    double map_resolution;
    double localmap_range_x, localmap_range_y;
    ros::param::param<double>("~inflation_radius", inflation_radius, 0.2);
    ros::param::param<double>("~map_resolution", map_resolution, 0.1);
    ros::param::param<double>("~localmap_range_x", localmap_range_x, 10.0);     // map_width --> x axis
    ros::param::param<double>("~localmap_range_y", localmap_range_y, 10.0);     // map_height --> y_axis
    ros::param::param<string>("~localmap_frameid", localmap_frameid_, "base_link");
    
    // ROS publishers & subscribers
    sub_scan_ = nh_.subscribe("scan", 1, &Scan2LocalmapNode::scan_cb, this);
    pub_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("local_map", 1);
    pub_footprint_ = nh_.advertise<geometry_msgs::PolygonStamped>("footprint", 1);

    // Prepare the transformation matrix from laser to base
    tflistener_ptr_ = new tf::TransformListener();
    ROS_INFO("Wait for TF from laserscan to %s in 10 seconds...", localmap_frameid_.c_str());
    try{
        tflistener_ptr_->waitForTransform(localmap_frameid_, "laser_link",
                                    ros::Time(), ros::Duration(10.0));
        tflistener_ptr_->lookupTransform(localmap_frameid_, "laser_link",
                                    ros::Time(), tf_base2laser_);
        ROS_INFO("Done.");
    }
    catch (tf::TransformException ex){
        ROS_ERROR("\nCannot get TF from laserscan to %s: %s. Aborting...", localmap_frameid_.c_str(), ex.what());
        exit(-1);
    }
    
    // Initialize localmap meta information
    localmap_ptr_ = nav_msgs::OccupancyGrid::Ptr(new nav_msgs::OccupancyGrid());
    localmap_ptr_->info.width = localmap_range_x * 2 / map_resolution;      // map_width --> x axis
    localmap_ptr_->info.height = localmap_range_y * 2 / map_resolution;     // map_height --> y_axis
    localmap_ptr_->info.resolution = map_resolution;
    localmap_ptr_->info.origin.position.x = -localmap_ptr_->info.resolution * localmap_ptr_->info.width / 2;
    localmap_ptr_->info.origin.position.y = -localmap_ptr_->info.resolution * localmap_ptr_->info.height / 2;
    localmap_ptr_->info.origin.orientation.w = 1.0;
    localmap_ptr_->data.resize(localmap_ptr_->info.width * localmap_ptr_->info.height);
    localmap_ptr_->header.frame_id = localmap_frameid_;
    ROS_INFO("Default range of localmap:+-%.1fx%.1f m, size:%dx%d", 
                localmap_range_x, localmap_range_y, localmap_ptr_->info.width, localmap_ptr_->info.height);
    
    // Footprint generator
    footprint_ptr_ = geometry_msgs::PolygonStamped::Ptr(new geometry_msgs::PolygonStamped());
    footprint_ptr_->header.frame_id = localmap_frameid_;
    geometry_msgs::Point32 pt;
    pt.x = -0.1, pt.y = 0.3314, pt.z = 0.0;         // end 1
    footprint_ptr_->polygon.points.push_back(pt);
    pt.x = 0.3, pt.y = 0.3314, pt.z = 0.0;
    footprint_ptr_->polygon.points.push_back(pt);
    pt.x = 0.4414, pt.y = 0.19, pt.z = 0.0;         // front 1
    footprint_ptr_->polygon.points.push_back(pt);
    pt.x = 0.5914, pt.y = 0.19, pt.z = 0.0;         // front 2
    footprint_ptr_->polygon.points.push_back(pt);
    pt.x = 0.5914, pt.y = -0.19, pt.z = 0.0;        // front 3
    footprint_ptr_->polygon.points.push_back(pt);
    pt.x = 0.4414, pt.y = -0.19, pt.z = 0.0;        // front 4
    footprint_ptr_->polygon.points.push_back(pt);
    pt.x = 0.3, pt.y = -0.3314, pt.z = 0.0;
    footprint_ptr_->polygon.points.push_back(pt);
    pt.x = -0.1, pt.y = -0.3314, pt.z = 0.0;        // end 2
    footprint_ptr_->polygon.points.push_back(pt);
    pt.x = -0.1, pt.y = -0.2014, pt.z = 0.0;        // end 3
    footprint_ptr_->polygon.points.push_back(pt);
    pt.x = 0.25, pt.y = -0.2014, pt.z = 0.0;
    footprint_ptr_->polygon.points.push_back(pt);
    pt.x = 0.3561, pt.y = -0.0953, pt.z = 0.0;
    footprint_ptr_->polygon.points.push_back(pt);
    pt.x = 0.3561, pt.y = 0.0953, pt.z = 0.0;
    footprint_ptr_->polygon.points.push_back(pt);
    pt.x = 0.25, pt.y = 0.2014, pt.z = 0.0;
    footprint_ptr_->polygon.points.push_back(pt);
    pt.x = -0.1, pt.y = 0.2014, pt.z = 0.0;         // end 4
    footprint_ptr_->polygon.points.push_back(pt);

    // Cropbox filter init
    box_filter_.setMax(Eigen::Vector4f(0.1, 0.30, 5.0, 1.0));
    box_filter_.setMin(Eigen::Vector4f(-1.0, -0.30, -5.0, 1.0));
    box_filter_.setKeepOrganized(false);
    box_filter_.setNegative(true);

    // VoxelGrid filter init
    double voxel_grid_size = 0.2;
    vg_filter_.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);

    // Filter kernel generator
    // butterworth_filter_generate(inflation_radius, 6, map_resolution, 100);  
}


void Scan2LocalmapNode::asymmetric_gaussian_filter(vector<int8_t> &vec, double map_resolution, int map_width, int map_height, int target_idx, double target_yaw, double target_speed, int peak_value) {
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // Asymmetric Gaussian Filter kernel
    vector<vector<int8_t> > agf_kernel;
    double kernel_range = 2.0 * 2;
    for(double y = -kernel_range / 2 ; y <= kernel_range / 2 * 1.00000001; y += map_resolution){
        vector<int8_t> tmp_row;
        for(double x = -kernel_range / 2; x <= kernel_range / 2 * 1.00000001; x += map_resolution){
            double sigma_head = std::max(target_speed * 2, 0.5);
            double sigma_side = sigma_head * 2 / 3;
            double sigma_rear = sigma_head / 2;

            double alpha = std::atan2(-y, -x) - target_yaw - M_PI * 0.5;
            double alpha_prime = std::atan2(std::sin(alpha), std::cos(alpha));
            double sigma_front = (alpha_prime > 0)? sigma_head : sigma_rear;
            double sin_p2 = std::pow(std::sin(target_yaw), 2);
            double cos_p2 = std::pow(std::cos(target_yaw), 2);
            double sigma_side_p2 = std::pow(sigma_side, 2);
            double sigma_front_p2 = std::pow(sigma_front, 2);
            double g_a = cos_p2 / (2 * sigma_front_p2) + sin_p2 / (2 * sigma_side_p2);
            double g_b = std::pow(std::sin(2 * target_yaw), 2) / (4 * sigma_front_p2) - std::pow(std::sin(2 * target_yaw), 2) / (4 * sigma_side_p2);
            double g_c = sin_p2 / (2 * sigma_front_p2) + cos_p2 / (2 * sigma_side_p2);
            double z = 1.0 / std::exp(g_a * std::pow(x, 2) + 2 * g_b * x * y + g_c * std::pow(y, 2)) * peak_value;

            // double r = sqrt(x * x + y * y);
            // double z = (1 / sqrt(1 + pow(r / filter_radius, (2 * filter_order)))) * peak_value;
            tmp_row.push_back(z);
        }
        agf_kernel.push_back(tmp_row);
    }

    // for(int i = 0; i < agf_kernel.size(); i++){
    //     for(int j = 0; j < agf_kernel[i].size(); j++){
    //         if(i == agf_kernel.size() / 2 && j == agf_kernel[0].size() / 2)
    //             agf_kernel[i][j] = peak_value;
    //         printf("%3d,", agf_kernel[i][j]);
    //     }
    //     cout << endl;
    // }
    agf_kernel[agf_kernel.size() / 2][agf_kernel[0].size() / 2] = peak_value;

    cout << "kernel size: (" << agf_kernel.size() << ", " << agf_kernel[0].size() << ")" << endl;
    if(agf_kernel.size() % 2 == 0){
        ROS_ERROR("Even kernel size! Please assign the new filter radius so that it can generate odd kernel size");
        exit(-1);
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;

    exit(-1);

    int kernel_size = agf_kernel.size();
    int bound = agf_kernel.size() / 2;

    for(int y = -bound; y <= bound; y++) {
        for (int x = -bound; x <= bound; x++) {
            int op_idx = target_idx + x + map_width * y;
            // if(vec[op_idx] < 0) continue;                                 // do not apply filter out of laser range
            if(agf_kernel[y + bound][x + bound] == 0 || vec[op_idx] >= agf_kernel[y + bound][x + bound]) continue;
            else if(op_idx < 0 || op_idx > map_width * map_height) continue;           // upper and bottom bound
            else if(abs((op_idx % map_width) - (target_idx % map_width)) > bound) continue;  // left and right bound
            else 
                vec[op_idx] = agf_kernel[y + bound][x + bound];
        }
    }
}


void Scan2LocalmapNode::butterworth_filter(vector<int8_t> &vec, int map_width, int map_height, int target_idx, int peak_value) {
    int kernel_size = inflation_kernel_.size();
    int bound = inflation_kernel_.size() / 2;

    for(int y = -bound; y <= bound; y++) {
        for (int x = -bound; x <= bound; x++) {
            int op_idx = target_idx + x + map_width * y;
            // if(vec[op_idx] < 0) continue;                                 // do not apply filter out of laser range
            if(inflation_kernel_[y + bound][x + bound] == 0 || vec[op_idx] >= inflation_kernel_[y + bound][x + bound]) continue;
            else if(op_idx < 0 || op_idx > map_width * map_height) continue;           // upper and bottom bound
            else if(abs((op_idx % map_width) - (target_idx % map_width)) > bound) continue;  // left and right bound
            else 
                vec[op_idx] = inflation_kernel_[y + bound][x + bound];
        }
    }
}


void Scan2LocalmapNode::butterworth_filter_generate(double filter_radius, int filter_order, double map_resolution, int peak_value) {
    double kernel_range = filter_radius * 4;
    cout << "Filter kernel: " << endl;
    for(double y = -kernel_range / 2 ; y <= kernel_range / 2 * 1.00000001; y += map_resolution){
        vector<int8_t> tmp_row;
        for(double x = -kernel_range / 2; x <= kernel_range / 2 * 1.00000001; x += map_resolution){
            double r = sqrt(x * x + y * y);
            double z = (1 / sqrt(1 + pow(r / filter_radius, (2 * filter_order)))) * peak_value;
            tmp_row.push_back(z);
        }
        inflation_kernel_.push_back(tmp_row);
    }

    for(int i = 0; i < inflation_kernel_.size(); i++){
        for(int j = 0; j < inflation_kernel_[i].size(); j++){
            if(i == inflation_kernel_.size() / 2 && j == inflation_kernel_[0].size() / 2)
                inflation_kernel_[i][j] = peak_value;
            printf("%3d,", inflation_kernel_[i][j]);
        }
        cout << endl;
    }

    cout << "Inflation kernel size: (" << inflation_kernel_.size() << ", " << inflation_kernel_[0].size() << ")" << endl;
    if(inflation_kernel_.size() % 2 == 0){
        ROS_ERROR("Even kernel size! Please assign the new filter radius so that it can generate odd kernel size");
        exit(-1);
    }
}


void Scan2LocalmapNode::scan_cb(const sensor_msgs::LaserScan &laser_msg) {
    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // Convert laserscan to pointcloud:  laserscan --> ROS PointCloud2 --> PCL PointCloudXYZ
    sensor_msgs::PointCloud2 cloud_msg;
    PointCloudXYZPtr cloud_raw(new PointCloudXYZ);
    PointCloudXYZPtr cloud_transformed(new PointCloudXYZ);
    projector_.projectLaser(laser_msg, cloud_msg);
    pcl::fromROSMsg(cloud_msg, *cloud_raw);
    pcl_ros::transformPointCloud(*cloud_raw, *cloud_transformed, tf_base2laser_);

    // Apply cropbox filter
    // box_filter_.setInputCloud(cloud_transformed);
    // box_filter_.filter(*cloud_transformed);

    // Apply voxel grid filter
    vg_filter_.setInputCloud(cloud_transformed);
    vg_filter_.filter(*cloud_transformed);

    // Localmap init
    std::fill(localmap_ptr_->data.begin(), localmap_ptr_->data.end(), -1);

    double resolution = localmap_ptr_->info.resolution;
    double map_origin_x = localmap_ptr_->info.origin.position.x;
    double map_origin_y = localmap_ptr_->info.origin.position.y;
    int map_width = localmap_ptr_->info.width;
    int map_height = localmap_ptr_->info.height;
    int map_limit = map_width * map_height;

    for(int i = 0; i < localmap_ptr_->data.size(); i++) {
        double grid_real_x = (i % map_width) * resolution + map_origin_x;
        double grid_real_y = (i / map_width) * resolution + map_origin_y;
        // printf("real xy (%.2f, %.2f)\n", grid_real_x, grid_real_y);
        double grid_real_anlge = std::atan2(grid_real_y, grid_real_x);
        double grid_real_distance = std::hypot(grid_real_x, grid_real_y);

        bool is_behind_obstacle = false;
        std::vector<double> distance_list;
        for(int j = 0; j < cloud_transformed->points.size(); j++){
            double obstacle_angle = std::atan2(cloud_transformed->points[j].y, cloud_transformed->points[j].x);
            double obstacle_distance = std::hypot(cloud_transformed->points[j].x, cloud_transformed->points[j].y);
            if(std::abs(obstacle_angle - grid_real_anlge) < (M_PI / 36) && obstacle_distance < grid_real_distance) {
                is_behind_obstacle = true;
                break;
            }
            distance_list.push_back(std::hypot(cloud_transformed->points[j].x - grid_real_x, cloud_transformed->points[j].y - grid_real_y));
        }
        if(is_behind_obstacle) continue;
        double closest_distance = *min_element(distance_list.begin(), distance_list.end());
        localmap_ptr_->data[i] = (closest_distance <= 1.2)? 80 - (50.0 / (1 + std::exp(-2.0 * closest_distance)) - 25.0) / 55 * 85: 
                                                            80 - (105.0 / (1 + std::exp(-0.5 * (closest_distance + 0.4))) - 51.5) / 55 * 85;
    }

    // Publish localmap
    ros::Time now = ros::Time(0);
    localmap_ptr_->header.stamp = now;
    pub_map_.publish(*localmap_ptr_);

    // Publish footprint
    footprint_ptr_->header.stamp = now;
    pub_footprint_.publish(*footprint_ptr_);

    // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
}


void Scan2LocalmapNode::sigint_cb(int sig) {
    cout << "\nNode name: " << ros::this_node::getName() << " is shutdown." << endl;
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "scan2comfortmap_node");
    ros::NodeHandle nh, pnh("~");
    Scan2LocalmapNode node(nh, pnh);    
    ros::spin();
    return 0;
}
