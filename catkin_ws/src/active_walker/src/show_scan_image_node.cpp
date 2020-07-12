#include <iostream>
#include <time.h>
#include <math.h>
#include <string.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <laser_geometry/laser_geometry.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// Custom srv
#include <walker_msgs/Detection2DTrigger.h>

// Message filter
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

// Eigen
#include <Eigen/Dense>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// TF
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h> // Centroid
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h> // ros2pcl
#include <pcl/filters/radius_outlier_removal.h> // RemoveOutlier
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;
using namespace cv;

typedef message_filters::sync_policies::ApproximateTime<cv_bridge::CvImage, sensor_msgs::LaserScan> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> MySynchronizer;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGBPtr;

// Just for color words display
static const string COLOR_RED = "\e[0;31m";
static const string COLOR_GREEN = "\e[0;32m";
static const string COLOR_YELLOW = "\e[0;33m"; 
static const string COLOR_NC = "\e[0m";

static const int kNumOfInterestClass = 1;
static const string kInterestClassNames[kNumOfInterestClass] = {"person"};

template <typename T, typename A>
int arg_max(std::vector<T, A> const& vec) {
    return static_cast<int>(std::distance(vec.begin(), max_element(vec.begin(), vec.end())));
}

template <typename T, typename A>
int arg_min(std::vector<T, A> const& vec) {
    return static_cast<int>(std::distance(vec.begin(), min_element(vec.begin(), vec.end())));
}

class ObjInfo {
public:
    ObjInfo(){
        cloud = PointCloudXYZPtr(new PointCloudXYZ);
    }
    walker_msgs::BBox2D box;
    PointCloudXYZPtr cloud;
    geometry_msgs::Point location;
    geometry_msgs::Point dimension;
};


class ScanImageCombineNode {
public:
    ScanImageCombineNode(ros::NodeHandle nh, ros::NodeHandle pnh);
    void img_scan_cb(const cv_bridge::CvImage::ConstPtr &cv_ptr, const sensor_msgs::LaserScan::ConstPtr &laser_msg_ptr);
    void separate_outlier_points(PointCloudXYZPtr cloud_in, PointCloudXYZPtr cloud_out);
    bool is_interest_class(string class_name);


    // Transformation
    tf::Matrix3x3 rot_cam2laser_;
    tf::Vector3 tras_cam2laser_;
    cv::Mat K_;
    cv::Mat D_;

    // ROS related
    ros::NodeHandle nh_, pnh_;
    tf::TransformListener tf_listener_;
    laser_geometry::LaserProjection projector_;
    ros::Publisher pub_combined_image_;
    ros::Publisher pub_marker_array_;
    ros::Publisher pub_colored_pc_;
    ros::ServiceClient yolov4_detect_;  // ROS Service client
    // Message filters
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
    message_filters::Subscriber<cv_bridge::CvImage> image_sub_;
    boost::shared_ptr<MySynchronizer> sync_;

    // Object list
    std::vector<ObjInfo> obj_list;
};


ScanImageCombineNode::ScanImageCombineNode(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh) {
    // ROS parameters
    string scan_topic;
    string img_topic;
    string caminfo_topic;
    string yolo_srv_name = "yolov4_node/yolo_detect";
    ros::param::param<string>("~scan_topic", scan_topic, "scan");
    ros::param::param<string>("~img_topic", img_topic, "usb_cam/image_raw"); 
    ros::param::param<string>("~caminfo_topic", caminfo_topic, "/usb_cam/camera_info"); 

    // ROS publisher & subscriber & message filter
    pub_combined_image_ = nh_.advertise<sensor_msgs::Image>("debug_reprojection", 1);
    pub_marker_array_ = nh.advertise<visualization_msgs::MarkerArray>("obj_marker", 1);
    pub_colored_pc_ = nh.advertise<sensor_msgs::PointCloud2>("colored_pc", 1);
    scan_sub_.subscribe(nh_, scan_topic, 1);
    image_sub_.subscribe(nh_, img_topic, 1);
    sync_.reset(new MySynchronizer(MySyncPolicy(10), image_sub_, scan_sub_));
    sync_->registerCallback(boost::bind(&ScanImageCombineNode::img_scan_cb, this, _1, _2));

    // ROS service client
    ROS_INFO_STREAM("Wait for yolo detection service in 5 seconds...");
    if(!ros::service::waitForService(yolo_srv_name, ros::Duration(5.0))) {
        ROS_ERROR("Cannot get the detection service: %s. Aborting...", yolo_srv_name.c_str());
        exit(-1);
    }
    yolov4_detect_ = nh_.serviceClient<walker_msgs::Detection2DTrigger>(yolo_srv_name);

    // Prepare extrinsic matrix
    tf::StampedTransform stamped_transform;
    try{
        tf_listener_.waitForTransform("camera_link", "laser_link",
                                    ros::Time(0), ros::Duration(5.0));
        tf_listener_.lookupTransform("camera_link", "laser_link", 
                                    ros::Time(0), stamped_transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("Cannot get TF from camera to laserscan: %s. Aborting...", ex.what());
        exit(-1);
    }
    rot_cam2laser_ = tf::Matrix3x3(stamped_transform.getRotation());
    tras_cam2laser_ = stamped_transform.getOrigin();
    // cout << "tras_cam2laser:\n" << tras_cam2laser_[0] << ", " << tras_cam2laser_[1] << ", " << tras_cam2laser_[2] << endl;

    // Prepare intrinsic matrix
    boost::shared_ptr<sensor_msgs::CameraInfo const> caminfo_ptr;
    double fx, fy, cx, cy;
    double k1, k2, p1, p2;
    ROS_INFO_STREAM("Wait for camera_info message in 5 seconds");
    caminfo_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(caminfo_topic, ros::Duration(5.0));
    if(caminfo_ptr != NULL){       
        fx = caminfo_ptr->P[0];
        fy = caminfo_ptr->P[5];
        cx = caminfo_ptr->P[2];
        cy = caminfo_ptr->P[6];

        k1 = caminfo_ptr->D[0];
        k2 = caminfo_ptr->D[1];
        p1 = caminfo_ptr->D[2];
        p2 = caminfo_ptr->D[3];
    }else {
        ROS_INFO_STREAM("No camera_info received, use default values");
        fx = 518.34283;
        fy = 522.27271;
        cx = 305.42936;
        cy = 244.1336;

        k1 = 0.052152;
        k2 = -0.122459;
        p1 = -0.003933;
        p2 = -0.005683;
    }
    K_ = (Mat_<double>(3, 3) << fx, 0., cx, 0., fy, cy, 0., 0., 1.);
    D_ = (Mat_<double>(5, 1) << k1, k2, p1, p2, 0.0);
    // cout << "K:\n" << K_ << endl;
    // cout << "D:\n" << D_ << endl;

    cout << COLOR_GREEN << ros::this_node::getName() << " is ready." << COLOR_NC << endl;
}


void ScanImageCombineNode::separate_outlier_points(PointCloudXYZPtr cloud_in, PointCloudXYZPtr cloud_out) {
    // Copy cloud_in to pc_copied
    PointCloudXYZPtr pc_copied(new PointCloudXYZ);
    pcl::copyPointCloud(*cloud_in, *pc_copied);

    // Euclidean Cluster Extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_in);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euler_extractor;
    euler_extractor.setClusterTolerance(0.2);
    euler_extractor.setMinClusterSize(3);
    euler_extractor.setMaxClusterSize(100);  // need to check the max pointcloud size of each object
    euler_extractor.setSearchMethod(tree);
    euler_extractor.setInputCloud(cloud_in);
    euler_extractor.extract(cluster_indices);

    // Find the cloud cluster which is closest to ego
    int idx_closest_cloud = 0;
    std::vector<float> candidates;
    for(int i = 0; i < cluster_indices.size(); i++) {
        Eigen::Vector3f centroid;
        centroid << 0, 0, 0;
        for(int j = 0; j < cluster_indices[i].indices.size(); j++) {
            centroid[0] += cloud_in->points[cluster_indices[i].indices[j]].x;
            centroid[1] += cloud_in->points[cluster_indices[i].indices[j]].y;
            centroid[2] += cloud_in->points[cluster_indices[i].indices[j]].z;
        }
        centroid /= cluster_indices[i].indices.size();
        candidates.push_back(centroid.norm());
    }
    idx_closest_cloud = arg_min(candidates);

    // Remove outlier
    pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices(cluster_indices[idx_closest_cloud]));
    PointCloudXYZPtr cloud_extracted(new PointCloudXYZ);
    pcl::ExtractIndices<pcl::PointXYZ> extractor;
    extractor.setInputCloud(cloud_in);
    extractor.setIndices(inliers_ptr);
    extractor.setNegative(false);
    extractor.filter(*cloud_extracted);

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud_extracted);
    outrem.setRadiusSearch(0.2);
    outrem.setMinNeighborsInRadius(5);
    outrem.filter(*(cloud_out));

    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;   
    // sor.setInputCloud(cloud_in);
    // sor.setMeanK(50);
    // sor.setStddevMulThresh(0.25);
    // sor.filter(*cloud_out);                    
}


bool ScanImageCombineNode::is_interest_class(string class_name){
    for(int i = 0; i < kNumOfInterestClass; i++) {
        if(strcmp(class_name.c_str(), kInterestClassNames[i].c_str()) == 0)
            return true;
    }
    return false;
}


void ScanImageCombineNode::img_scan_cb(const cv_bridge::CvImage::ConstPtr &cv_ptr, const sensor_msgs::LaserScan::ConstPtr &laser_msg_ptr){
    // Object list init
    obj_list.clear();
    visualization_msgs::MarkerArray marker_array;

    // 2D bounding box detection service
    walker_msgs::Detection2DTrigger srv;
    srv.request.image = *(cv_ptr->toImageMsg());
    if(!yolov4_detect_.call(srv)){
        ROS_ERROR("Failed to call service");
        return;
    }
    
    // Collect all interest classes to obj_list
    std::vector<walker_msgs::BBox2D> boxes = srv.response.result.boxes;
    for(int i = 0; i < boxes.size(); i++) {
        if(is_interest_class(boxes[i].class_name)) {
            ObjInfo obj_info;
            obj_info.box = boxes[i];
            obj_list.push_back(obj_info);
        }
    }

    // Reconstruct undistorted cvimage from detection result image
    cv::Mat cvimage;
    cv_bridge::CvImagePtr detected_cv_ptr = cv_bridge::toCvCopy(srv.response.result.result_image);
    cv::undistort(detected_cv_ptr->image, cvimage, K_, D_);
    
    // Convert laserscan to pointcloud:  laserscan --> ROS PointCloud2 --> PCL PointCloudXYZ
    sensor_msgs::PointCloud2 cloud_msg;
    PointCloudXYZPtr cloud_raw(new PointCloudXYZ);
    projector_.projectLaser(*laser_msg_ptr, cloud_msg);
    pcl::fromROSMsg(cloud_msg, *cloud_raw);

    // Color pointcloud to visaulize detected points
    PointCloudXYZRGBPtr cloud_colored(new PointCloudXYZRGB);

    // Convert laserscan points to pixel points
    std::vector<cv::Point2d> pts_uv;
    for (int i = 0; i < cloud_raw->points.size(); ++i) {
        // Transform to camera frame
        tf::Vector3 pt_l(cloud_raw->points[i].x, cloud_raw->points[i].y, cloud_raw->points[i].z); 
        tf::Vector3 pt_c = rot_cam2laser_ * pt_l + tras_cam2laser_;
        if(pt_c.getZ() <= 0.0) // points behind ego
            continue;

        // Normalization: z --> 1
        pt_c.setX(pt_c.getX() / pt_c.getZ());
        pt_c.setY(pt_c.getY() / pt_c.getZ());
        pt_c.setZ(1.0);

        // Trasform to pixel frame
        cv::Mat pt_cam = (cv::Mat_<double>(3, 1) << pt_c.getX(), pt_c.getY(), pt_c.getZ());
        cv::Mat uv = K_ * pt_cam;
        cv::Point2d pt_uv(uv.at<double>(0, 0), uv.at<double>(1, 0));
        pts_uv.push_back(pt_uv);

        // Connect relationship between valid laserscan points to interest classes
        for(int j = 0; j < obj_list.size(); j++) {
            float diff_x = fabs((float)(pt_uv.x - obj_list[j].box.center.x));
            float diff_y = fabs((float)(pt_uv.y - obj_list[j].box.center.y));
            if(diff_x < obj_list[j].box.size_x / 2 && diff_y < obj_list[j].box.size_y / 2) {
                 obj_list[j].cloud->points.push_back(cloud_raw->points[i]);
            }
            // Note that the pointcloud would be registered repeatly, so need to filter it later.
        }        
    }

    // Remove outlier for each object cloud
    for(int i = 0; i < obj_list.size(); i++) {
        if(obj_list[i].cloud->points.size() > 1){

            // Merge raw detected points with color to visualization
            if(pub_colored_pc_.getNumSubscribers() > 0) {
                PointCloudXYZRGBPtr tmp_cloud(new PointCloudXYZRGB);
                pcl::copyPointCloud(*(obj_list[i].cloud), *tmp_cloud);
                for(auto& point: *tmp_cloud) {
                    point.r = 255;
                    point.g = 0;
                    point.b = 0;
                }
                *cloud_colored += *tmp_cloud;
            }

            // Clustering and outlier removing
            separate_outlier_points(obj_list[i].cloud, obj_list[i].cloud);
            if(obj_list[i].cloud->points.size() < 1)
                continue;

            // Find the centroid of each object
            pcl::PointXYZ min_point, max_point;
            Eigen::Vector3f center;
            pcl::getMinMax3D(*(obj_list[i].cloud), min_point, max_point);
            center = (min_point.getVector3fMap() + max_point.getVector3fMap()) / 2.0;
            obj_list[i].location.x = center[0];
            obj_list[i].location.y = center[1];

            visualization_msgs::Marker marker;
            marker.header.frame_id = laser_msg_ptr->header.frame_id;
            marker.header.stamp = ros::Time();
            marker.ns = "detection_result";
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.lifetime = ros::Duration(0.2);
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = obj_list[i].location.x;
            marker.pose.position.y = obj_list[i].location.y;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.4;
            marker.scale.y = 0.4;
            marker.scale.z = 0.4;
            marker.color.a = 0.4;
            marker.color.g = 1.0;

            marker_array.markers.push_back(marker);
        }
        // TODO: deal with the objects which has no matched points
    }

    // Publish visualization topics
    if(pub_marker_array_.getNumSubscribers() > 0)
        pub_marker_array_.publish(marker_array);
    if(pub_colored_pc_.getNumSubscribers() > 0) {
        sensor_msgs::PointCloud2 colored_cloud_msg;
        pcl::toROSMsg(*cloud_colored, colored_cloud_msg);
        colored_cloud_msg.header.frame_id = "laser_link";
        pub_colored_pc_.publish(colored_cloud_msg);
    }
    if(pub_combined_image_.getNumSubscribers() > 0){
        // Draw points in images
        for (int j = 0; j < pts_uv.size(); ++j)
            cv::circle(cvimage, pts_uv[j], 1, Scalar(0, 255, 0), 1);
        cv_bridge::CvImage result_image(cv_ptr->header, "rgb8", cvimage);
        pub_combined_image_.publish(result_image.toImageMsg());
    }

    if(obj_list.size() > 0){
        cout << "Prediction result:" << endl;
        for(int i = 0; i < obj_list.size(); i++) {
            cout << obj_list[i].box.class_name << ", cloud size: " << obj_list[i].cloud->points.size() << endl; 
        }
        cout << "\n===================" << endl;
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "show_scan_image_node");
    ros::NodeHandle nh, pnh("~");
    ScanImageCombineNode node(nh, pnh);
    ros::spin();
    return 0;
}
