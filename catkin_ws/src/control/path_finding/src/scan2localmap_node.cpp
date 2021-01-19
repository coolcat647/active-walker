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
#include <walker_msgs/Trk3DArray.h>
#include <walker_msgs/Trk3D.h>

// TF
#include <tf/transform_listener.h>

// PCL
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h> // ros2pcl
#include <pcl/filters/crop_box.h>

// using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGBPtr;


class Scan2LocalmapNode {
public:
    Scan2LocalmapNode(ros::NodeHandle nh, ros::NodeHandle pnh);
    static void sigint_cb(int sig);
    void apply_butterworth_filter(std::vector<int8_t> &vec, int map_width, int map_height, int target_idx, int peak_value);
    void apply_asymmetric_gaussian_filter(std::vector<int8_t> &vec, double map_resolution, int map_width, int map_height, int target_idx, double target_yaw, double target_speed, int peak_value);
    void butterworth_filter_generate(double filter_radius, int filter_order, double map_resolution, int peak_value);
    void scan_cb(const sensor_msgs::LaserScan &laser_msg);
    void trk3d_cb(const walker_msgs::Trk3DArray::ConstPtr &msg_ptr);
    void scan_cb_deprecated(const sensor_msgs::LaserScan &laser_msg);

    // ROS related
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber sub_scan_;
    ros::Publisher pub_map_;
    ros::Publisher pub_footprint_;
    std::string localmap_frameid_;                               // Localmap frame_id
    nav_msgs::OccupancyGrid::Ptr localmap_ptr_;             // Localmap msg
    geometry_msgs::PolygonStamped::Ptr footprint_ptr_;      // Robot footprint
    laser_geometry::LaserProjection projector_;             // Projector of laserscan

    // TF listener
    tf::TransformListener* tflistener_ptr_;
    tf::StampedTransform tf_laser2base_;    

    // Inflation filter kernel
    std::vector<std::vector<int8_t> > inflation_kernel_;

    // PCL Cropbox filter
    pcl::CropBox<pcl::PointXYZ> box_filter_; 

    // Flag AGF use or not
    bool flag_agf_using_;
};


Scan2LocalmapNode::Scan2LocalmapNode(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh) {
    // Signal handler
    signal(SIGINT, Scan2LocalmapNode::sigint_cb);

    // ROS parameters
    double inflation_radius;
    double map_resolution;
    double localmap_range_x, localmap_range_y;
    std::string scan_src_frameid;
    ros::param::param<double>("~inflation_radius", inflation_radius, 0.2);
    ros::param::param<double>("~map_resolution", map_resolution, 0.1);
    ros::param::param<double>("~localmap_range_x", localmap_range_x, 10.0);     // map_width --> x axis
    ros::param::param<double>("~localmap_range_y", localmap_range_y, 10.0);     // map_height --> y_axis
    ros::param::param<std::string>("~localmap_frameid", localmap_frameid_, "base_link");
    ros::param::param<std::string>("~scan_src_frameid", scan_src_frameid, "laser_link");
    ros::param::param<bool>("~use_agf", flag_agf_using_, false);

    // ROS publishers & subscribers
    if(flag_agf_using_)
        sub_scan_ = nh_.subscribe("trk3d_result", 1, &Scan2LocalmapNode::trk3d_cb, this);
    else
        sub_scan_ = nh_.subscribe("scan", 1, &Scan2LocalmapNode::scan_cb, this);

    pub_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("local_map", 1);
    pub_footprint_ = nh_.advertise<geometry_msgs::PolygonStamped>("footprint", 1);

    // Prepare the transformation matrix from laser to base
    tflistener_ptr_ = new tf::TransformListener();
    ROS_INFO("Wait for TF from laser_link to %s in 10 seconds...", localmap_frameid_.c_str());
    try{
        tflistener_ptr_->waitForTransform(localmap_frameid_, scan_src_frameid,
                                    ros::Time(), ros::Duration(10.0));
        tflistener_ptr_->lookupTransform(localmap_frameid_, scan_src_frameid,
                                    ros::Time(), tf_laser2base_);
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
    box_filter_.setMax(Eigen::Vector4f(0.4, 0.50, 5.0, 1.0));
    box_filter_.setMin(Eigen::Vector4f(-1.5, -0.50, -5.0, 1.0));
    box_filter_.setKeepOrganized(false);
    box_filter_.setNegative(true);

    // Filter kernel generator
    butterworth_filter_generate(inflation_radius, 6, map_resolution, 100);

    ROS_INFO_STREAM(ros::this_node::getName() + " is ready.");
}


void Scan2LocalmapNode::apply_asymmetric_gaussian_filter(std::vector<int8_t> &vec, double map_resolution, int map_width, int map_height, int target_idx, double target_yaw, double target_speed, int peak_value) {
    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // Asymmetric Gaussian Filter kernel
    std::vector<std::vector<int8_t> > agf_kernel;
    // double kernel_range = 2.4 * 2;
    double kernel_range = 5.0 * 2;
    double max_kernel_range = kernel_range / 2 * 1.0000001;
    for(double y = -kernel_range / 2 ; y <= max_kernel_range; y += map_resolution){
        std::vector<int8_t> tmp_row;
        for(double x = -kernel_range / 2; x <= max_kernel_range; x += map_resolution){
            double sigma_head = std::max(target_speed, 0.5);
            double sigma_side = sigma_head * 2 / 5;
            double sigma_rear = sigma_head / 2;

            double alpha = std::atan2(-y, -x) - target_yaw - M_PI * 0.5;
            double alpha_prime = std::atan2(std::sin(alpha), std::cos(alpha));
            double sigma_front = (alpha_prime > 0)? sigma_head : sigma_rear;
            double sin_p2 = std::pow(std::sin(target_yaw), 2);
            double cos_p2 = std::pow(std::cos(target_yaw), 2);
            double sigma_side_p2 = std::pow(sigma_side, 2);
            double sigma_front_p2 = std::pow(sigma_front, 2);
            double g_a = cos_p2 / (2 * sigma_front_p2) + sin_p2 / (2 * sigma_side_p2);
            double g_b = std::sin(2 * target_yaw) / (4 * sigma_front_p2) - std::sin(2 * target_yaw) / (4 * sigma_side_p2);
            double g_c = sin_p2 / (2 * sigma_front_p2) + cos_p2 / (2 * sigma_side_p2);
            double z = 1.0 / std::exp(g_a * std::pow(x, 2) + 2 * g_b * x * y + g_c * std::pow(y, 2)) * peak_value;
            tmp_row.push_back(z);
        }
        agf_kernel.push_back(tmp_row);
    }

    // std::cout << "target_yaw: " << target_yaw << std::endl;
    // for(int i = 0; i < agf_kernel.size(); i++){
    //     for(int j = 0; j < agf_kernel[i].size(); j++){
    //         if(i == agf_kernel.size() / 2 && j == agf_kernel[0].size() / 2)
    //             agf_kernel[i][j] = peak_value;
    //         printf("%3d,", agf_kernel[i][j]);
    //     }
    //     std::cout << std::endl;
    // }
    agf_kernel[agf_kernel.size() / 2][agf_kernel[0].size() / 2] = peak_value;

    // std::cout << "kernel size: (" << agf_kernel.size() << ", " << agf_kernel[0].size() << ")" << std::endl;
    // if(agf_kernel.size() % 2 == 0){
    //     ROS_ERROR("Even kernel size! Please assign the new filter radius so that it can generate odd kernel size");
    //     exit(-1);
    // }
    
    int kernel_size = agf_kernel.size();
    int bound = agf_kernel.size() / 2;

    int min_bound = -bound;
    int max_bound = (bound + kernel_size % 2);
    int max_map_idx = map_width * map_height - 1;

    for(int y = min_bound; y < max_bound; y++) {
        for (int x = min_bound; x < max_bound; x++) {
            int op_idx = target_idx + x + map_width * y;
            int8_t op_kernel_val = agf_kernel[y + bound][x + bound];

            // if(vec[op_idx] < 0) continue;                                 // do not apply filter out of laser range
            if(op_kernel_val == 0 || vec[op_idx] >= op_kernel_val) continue;
            else if(op_idx < 0 || op_idx > max_map_idx) continue;           // upper and bottom bound
            else if(abs((op_idx % map_width) - (target_idx % map_width)) >= bound) continue;  // left and right bound
            else{
                int tmp_val = (int)op_kernel_val + vec[op_idx];
                vec[op_idx] = (tmp_val > peak_value)? peak_value : tmp_val;
            }
        }
    }

    // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
}


void Scan2LocalmapNode::trk3d_cb(const walker_msgs::Trk3DArray::ConstPtr &msg_ptr) {

    // Get the transformation from base to 
    tf::StampedTransform tf_trk2base;
    try{
        tflistener_ptr_->waitForTransform(localmap_frameid_, msg_ptr->header.frame_id,
                                    ros::Time(), ros::Duration(0.1));
        tflistener_ptr_->lookupTransform(localmap_frameid_, msg_ptr->header.frame_id,
                                    ros::Time(), tf_trk2base);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("\nCannot get TF from odom to %s: %s. Aborting...", localmap_frameid_.c_str(), ex.what());
        exit(-1);
    }

    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    sensor_msgs::LaserScan laser_msg = msg_ptr->scan;

    // Convert laserscan to pointcloud:  laserscan --> ROS PointCloud2 --> PCL PointCloudXYZ
    sensor_msgs::PointCloud2 cloud_msg;
    PointCloudXYZPtr cloud_raw(new PointCloudXYZ);
    PointCloudXYZPtr cloud_transformed(new PointCloudXYZ);
    projector_.projectLaser(laser_msg, cloud_msg);
    pcl::fromROSMsg(cloud_msg, *cloud_raw);
    pcl_ros::transformPointCloud(*cloud_raw, *cloud_transformed, tf_laser2base_);

    // Apply cropbox filter
    box_filter_.setInputCloud(cloud_transformed);
    box_filter_.filter(*cloud_transformed);

    // Localmap init
    std::fill(localmap_ptr_->data.begin(), localmap_ptr_->data.end(), 0);

    double resolution = localmap_ptr_->info.resolution;
    double map_origin_x = localmap_ptr_->info.origin.position.x;
    double map_origin_y = localmap_ptr_->info.origin.position.y;
    int map_width = localmap_ptr_->info.width;
    int map_height = localmap_ptr_->info.height;
    int map_limit = map_width * map_height - 1;

    // Static obstacle inflation
    // for(int i = 0; i < cloud_transformed->points.size(); i++) {
    //     double laser_x = cloud_transformed->points[i].x;
    //     double laser_y = cloud_transformed->points[i].y;
    //     if(fabs(laser_x) > map_width * resolution / 2)
    //         continue;
    //     else if(fabs(laser_y) > map_height * resolution / 2)
    //         continue;

    //     int map_x = std::floor((laser_x - map_origin_x) / resolution);
    //     int map_y = std::floor((laser_y - map_origin_y) / resolution);
    //     int idx = map_y * map_width + map_x;
        
    //     if((0 < idx) && (idx < map_limit)){
    //         if(localmap_ptr_->data[idx] == 100)
    //             continue;
    //         apply_butterworth_filter(localmap_ptr_->data, map_width, map_height, idx, 100);
    //     }
    // }

    // Proxemics generation
    for(int i = 0; i < msg_ptr->trks_list.size(); i++) {
        // Convert object pose from laser coordinate to base coordinate
        tf::Vector3 pt_laser(msg_ptr->trks_list[i].x, msg_ptr->trks_list[i].y, 0);
        // tf::Vector3 pt_base = tf_laser2base_.getBasis() * pt_laser + tf_laser2base_.getOrigin();
        tf::Vector3 pt_base = tf_trk2base.getBasis() * pt_laser + tf_trk2base.getOrigin();
        tf::Quaternion q;
        q.setRPY(0, 0, msg_ptr->trks_list[i].yaw);
        double yaw, pitch, roll;
        tf::Matrix3x3 mat(q);
        // mat = tf_laser2base_.getBasis() * mat;
        mat = tf_trk2base.getBasis() * mat;
        mat.getEulerYPR(yaw, pitch, roll);

        double speed = std::hypot(msg_ptr->trks_list[i].vx, msg_ptr->trks_list[i].vy);

        // Calculate object position in local map
        int map_x = std::floor((pt_base.getX() - map_origin_x) / resolution);
        int map_y = std::floor((pt_base.getY() - map_origin_y) / resolution);
        int idx = map_y * map_width + map_x;

        // if((0 < idx) && (idx < map_limit) && (speed > 0.1)){
        if((0 < idx) && (idx < map_limit)){
            apply_asymmetric_gaussian_filter(localmap_ptr_->data, resolution, map_width, map_height, idx, yaw, speed * 1.2, 100);
        }
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


void Scan2LocalmapNode::apply_butterworth_filter(std::vector<int8_t> &vec, int map_width, int map_height, int target_idx, int peak_value) {
    int kernel_size = inflation_kernel_.size();
    int bound = inflation_kernel_.size() / 2;

    int min_bound = -bound;
    int max_bound = (bound + kernel_size % 2);
    int max_map_idx = map_width * map_height - 1;

    // ROS_WARN("kernel_size: %d, bound: %d", kernel_size, bound);

    for(int y = min_bound; y < max_bound; y++) {
        for (int x = min_bound; x < max_bound; x++) {
            int op_idx = target_idx + x + map_width * y;
            int8_t op_kernel_val = inflation_kernel_[y + bound][x + bound];

            // ROS_WARN("idx: %d, %d", y + bound, x + bound);
            // if(vec[op_idx] < 0) continue;                                 // do not apply filter out of laser range
            if(op_kernel_val == 0 || vec[op_idx] >= op_kernel_val) continue;
            else if(op_idx < 0 || op_idx > max_map_idx) continue;           // upper and bottom bound
            else if(abs((op_idx % map_width) - (target_idx % map_width)) >= bound) continue;  // left and right bound
            else{
                int tmp_val = op_kernel_val + vec[op_idx];
                vec[op_idx] = (tmp_val > peak_value)? peak_value : tmp_val;
            }
        }
    }
    // exit(-1);
}


void Scan2LocalmapNode::butterworth_filter_generate(double filter_radius, int filter_order, double map_resolution, int peak_value) {
    double kernel_range = filter_radius * 4;
    // std::cout << "Filter kernel: " << std::endl;
    for(double y = -kernel_range / 2 ; y <= kernel_range / 2 * 1.00000001; y += map_resolution){
        std::vector<int8_t> tmp_row;
        for(double x = -kernel_range / 2; x <= kernel_range / 2 * 1.00000001; x += map_resolution){
            double r = sqrt(x * x + y * y);
            double z = (1 / sqrt(1 + pow(r / filter_radius, (2 * filter_order)))) * peak_value;
            tmp_row.push_back(z);
        }
        inflation_kernel_.push_back(tmp_row);
    }

    for(int i = 0; i < inflation_kernel_.size(); i++){
        for(int j = 0; j < inflation_kernel_[i].size(); j++){
            // if(i == inflation_kernel_.size() / 2 && j == inflation_kernel_[0].size() / 2)
                // inflation_kernel_[i][j] = peak_value;
            printf("%3d,", inflation_kernel_[i][j]);
        }
        std::cout << std::endl;
    }

    ROS_INFO_STREAM("Inflation kernel size: (" << inflation_kernel_.size() << ", " << inflation_kernel_[0].size() << ")");
    // if(inflation_kernel_.size() % 2 == 0){
    //     ROS_ERROR("Even kernel size! Please assign the new filter radius so that it can generate odd kernel size");
    //     exit(-1);
    // }
}


void Scan2LocalmapNode::scan_cb(const sensor_msgs::LaserScan &laser_msg) {
    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // Convert laserscan to pointcloud:  laserscan --> ROS PointCloud2 --> PCL PointCloudXYZ
    sensor_msgs::PointCloud2 cloud_msg;
    PointCloudXYZPtr cloud_raw(new PointCloudXYZ);
    PointCloudXYZPtr cloud_transformed(new PointCloudXYZ);
    projector_.projectLaser(laser_msg, cloud_msg);
    pcl::fromROSMsg(cloud_msg, *cloud_raw);
    pcl_ros::transformPointCloud(*cloud_raw, *cloud_transformed, tf_laser2base_);

    // Apply cropbox filter
    box_filter_.setInputCloud(cloud_transformed);
    box_filter_.filter(*cloud_transformed);

    // Localmap init
    std::fill(localmap_ptr_->data.begin(), localmap_ptr_->data.end(), 0);

    double resolution = localmap_ptr_->info.resolution;
    double map_origin_x = localmap_ptr_->info.origin.position.x;
    double map_origin_y = localmap_ptr_->info.origin.position.y;
    int map_width = localmap_ptr_->info.width;
    int map_height = localmap_ptr_->info.height;
    int map_limit = map_width * map_height - 1;

    for(int i = 0; i < cloud_transformed->points.size(); i++) {
        double laser_x = cloud_transformed->points[i].x;
        double laser_y = cloud_transformed->points[i].y;
        if(fabs(laser_x) > map_width * resolution / 2)
            continue;
        else if(fabs(laser_y) > map_height * resolution / 2)
            continue;

        // Add wall(non-walkable) space
        int map_x = std::floor((laser_x - map_origin_x) / resolution);
        int map_y = std::floor((laser_y - map_origin_y) / resolution);
        int idx = map_y * map_width + map_x;
        
        if((0 < idx) && (idx < map_limit)){
            if(localmap_ptr_->data[idx] == 100)
                continue;
            apply_butterworth_filter(localmap_ptr_->data, map_width, map_height, idx, 100);
        }
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
    ROS_INFO_STREAM("Node name: " << ros::this_node::getName() << " is shutdown.");
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "laserscan_mapping_node");
    ros::NodeHandle nh, pnh("~");
    Scan2LocalmapNode node(nh, pnh);    
    ros::spin();
    return 0;
}
