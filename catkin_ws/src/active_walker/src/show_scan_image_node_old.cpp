#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <laser_geometry/laser_geometry.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <Eigen/Dense>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <time.h>

using namespace std;
using namespace cv;

laser_geometry::LaserProjection projector_;
image_transport::Publisher pub_result_image;
int image_count = 0;

cv::Mat K;
cv::Mat D;
cv::Mat Rcl;
cv::Mat tcl;


void callback(const cv_bridge::CvImage::ConstPtr &img_msg,const sensor_msgs::LaserScan::ConstPtr &laser_msg)
{
    /**
     * Test the sync results!
     */
    // ROS_INFO_STREAM("image timestamp: " << img_msg->header.stamp.toNSec() << " ns");
    // ROS_INFO_STREAM("scan  timestamp: " << laser_msg->header.stamp.toNSec() << " ns");
    // double diff = img_msg->header.stamp.toSec() - laser_msg->header.stamp.toSec();
    // if(diff < 0)
    //     diff = -1.0 * diff;
    // ROS_INFO_STREAM("           diff: " << diff << " s" );
    cv::Mat inImage;
    undistort(img_msg->image, inImage, K, D);
    double scale = 480 / img_msg->image.rows;
    cv::Size dsize = Size(img_msg->image.cols * scale, img_msg->image.rows * scale);
    cv::resize(inImage, inImage, dsize);
    // cvtColor(inImage, inImage, COLOR_GRAY2BGR);

    sensor_msgs::PointCloud cloud;
    projector_.projectLaser(*laser_msg, cloud);
    std::vector<Point2d> pts_uv;

    for (int i = 0; i < cloud.points.size(); ++i)
    {

        /// Reprojection
        cv::Mat point_l(3,1,CV_64FC1);
        point_l.at<double>(0,0) = cloud.points[i].x;
        point_l.at<double>(1,0) = cloud.points[i].y;
        point_l.at<double>(2,0) = cloud.points[i].z;
        cv::Mat point_c = Rcl * point_l + tcl;


        if(point_c.at<double>(2,0) <= 0.)
            continue;
        point_c.at<double>(0,0) /= point_c.at<double>(2,0);
        point_c.at<double>(1,0) /= point_c.at<double>(2,0);
        point_c.at<double>(2,0) = 1.0;

        cv::Mat uv = K * point_c;
        Point2d pt_uv(uv.at<double>(0,0), uv.at<double>(1,0));
        pts_uv.push_back(pt_uv);

    }
    ///Draw points in images
    for (int j = 0; j < pts_uv.size(); ++j) {
        cv::circle(inImage, pts_uv[j], 1, Scalar(0,255,0), 1);
    }
    cv_bridge::CvImage debug_image(img_msg->header,"rgb8", inImage);
    if(pub_result_image.getNumSubscribers() > 0)
        pub_result_image.publish(debug_image.toImageMsg());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "show_scan_image_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    /// ==========================================
    std::string scan_topic_name = "scan";
    std::string img_topic_name = "usb_cam/image_raw";
    // std::string img_topic_name = "/detection_result";

    std::cout <<"\t"<<scan_topic_name << std::endl;
    std::cout <<"\t"<<img_topic_name << std::endl;

    /// ==========================================

    cv::Mat Rlc;
    cv::Mat tlc;

    Eigen::Quaterniond q_cl(0.5072711, -0.4931572, 0.5025891, -0.4968664); // w, x, y, z
    Eigen::Matrix3d tmp = q_cl.toRotationMatrix();
    Rlc = (Mat_<double>(3,3) << tmp(0,0), tmp(0,1), tmp(0,2),
                                tmp(1,0), tmp(1,1), tmp(1,2),
                                tmp(2,0), tmp(2,1), tmp(2,2));
    tlc = (Mat_<double>(3, 1) << 5.03168100114148e-02, 9.5209630006627572e-02, -7.872196022398195e-02);

    Rcl = Rlc.t();
    tcl = -Rcl * tlc;
    

    ROS_INFO_STREAM("\n" << Rcl);
    ROS_INFO_STREAM("\n" << tcl);

    boost::shared_ptr<sensor_msgs::CameraInfo const> shared_caminfo;
    double fx, fy, cx, cy;
    double k1, k2, p1, p2;
    ROS_INFO_STREAM("Wait for camera_info message in 3 seconds");
    shared_caminfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/usb_cam/camera_info", ros::Duration(3));
    if(shared_caminfo != NULL){       
        /// Corner detect
        fx = shared_caminfo->P[0];
        fy = shared_caminfo->P[5];
        cx = shared_caminfo->P[2];
        cy = shared_caminfo->P[6];

        k1 = shared_caminfo->D[0];
        k2 = shared_caminfo->D[1];
        p1 = shared_caminfo->D[2];
        p2 = shared_caminfo->D[3];
    }else {
        ROS_INFO_STREAM("No camera_info received, use default values");
        /// Corner detect
        fx = 518.34283; // 506.745880;
        fy = 522.27271; // 511.669586;
        cx = 305.42936; // 308.913380;
        cy = 244.1336; // 245.596730;

        k1 = 0.052152; // 0.039701;
        k2 = -0.122459;// -0.106165;
        p1 = -0.003933;// -0.002774;
        p2 = -0.005683;// -0.002310;
    }

    K = (Mat_<double>(3, 3) << fx, 0., cx, 0., fy, cy, 0., 0., 1.);
    D = (Mat_<double>(5,1) << k1, k2, p1, p2, 0.0);
    cout << "K:\n" << K << endl;
    cout << "D:\n" << D << endl;

    image_transport::ImageTransport it(nh);

    pub_result_image = it.advertise("debug_reprojection", 1);
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, scan_topic_name, 1);
    message_filters::Subscriber<cv_bridge::CvImage> image_sub(nh, img_topic_name, 1);

    typedef message_filters::sync_policies::ApproximateTime<cv_bridge::CvImage, sensor_msgs::LaserScan> sync_policy;
    message_filters::Synchronizer<sync_policy> sync(sync_policy(10), image_sub, scan_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    std::cout << "start spin.." << std::endl;
    ros::spin();

    return 0;
}
