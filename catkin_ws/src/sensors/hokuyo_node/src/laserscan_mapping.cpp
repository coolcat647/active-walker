#include <chrono>
#include <signal.h>

#include "ros/ros.h"
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

nav_msgs::OccupancyGrid* laser_map;
ros::Publisher pub_map;

int8_t kgaussian_kernel3[] = {
    37, 61, 37, 
    61, 100,61, 
    37, 61, 37
};
int8_t kgaussian_kernel5[] = {
    2,  8,  14, 8,  2,  
    8,  37, 61, 37, 8,  
    14, 61, 100,61, 14, 
    8,  37, 61, 37, 8,  
    2,  8,  14, 8,  2
};
int8_t kgaussian_kernel7[] = {
    0,  0,  1,  1,  1,  0,  0,  
    0,  2,  8,  14, 8,  2,  0,  
    1,  8,  37, 61, 37, 8,  1,  
    1,  14, 61, 100,61, 14, 1,  
    1,  8,  37, 61, 37, 8,  1,  
    0,  2,  8,  14, 8,  2,  0,  
    0,  0,  1,  1,  1,  0,  0
};
int8_t kgaussian_kernel9[] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  
    0,  0,  0,  1,  1,  1,  0,  0,  0,  
    0,  0,  2,  8,  14, 8,  2,  0,  0,  
    0,  1,  8,  37, 61, 37, 8,  1,  0,  
    0,  1,  14, 61, 100,61, 14, 1,  0,  
    0,  1,  8,  37, 61, 37, 8,  1,  0,  
    0,  0,  2,  8,  14, 8,  2,  0,  0,  
    0,  0,  0,  1,  1,  1,  0,  0,  0,  
    0,  0,  0,  0,  0,  0,  0,  0,  0
};


void gauss_filter(vector<int8_t> &vec, int width, int height, int idx, int kernel_size, int peak_value) { 
    int bound = kernel_size/2;

    int8_t* kernel_table;
    switch(kernel_size){
        case 3:
            kernel_table = kgaussian_kernel3; break;
        case 5:
            kernel_table = kgaussian_kernel5; break;
        case 7:
            kernel_table = kgaussian_kernel7; break;
        case 9:
            kernel_table = kgaussian_kernel9; break;
    }

    // // Script for table generator
    // cout <<setprecision(0) << setiosflags(ios::fixed);
    // cout << "double kgaussian_kernel" << kernel_size << "[] = {\n\t";
    // for(int i = 0; i < kernel_size; ++i){
    //     for (int j = 0; j < kernel_size; ++j) {
    //         // kernel[i][j] = kernel[i][j] / sum * peak_value;      // Norm by sum
    //         if((j == kernel_size -1) && (i == kernel_size -1)) cout << kernel[i][j];
    //         else cout << kernel[i][j] << ",\t";
    //     }
    //     if(i < kernel_size -1) cout << "\n\t";
    //     else cout << endl;
    // }
    // cout << "};" << endl;

    // Apply filter to the input      
    for(int y = -bound; y <= bound; y++) 
        for (int x = -bound; x <= bound; x++) {
            int op_idx = idx + x + width*y;
            if(vec[op_idx] < 0) continue;                                 // do not apply filter out of laser range
            if(op_idx < 0 || op_idx > width * height) continue;           // upper and bottom bound
            else if(abs(op_idx % width - idx % width) > bound) continue;  // left and right bound
            int result = vec[op_idx] + kernel_table[(y + bound)*kernel_size + x + bound];
            vec[op_idx] = (result > peak_value)? peak_value: result;
        }
}

void scan_cb(const sensor_msgs::LaserScan &laser_msg) {
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    std::fill(laser_map->data.begin(), laser_map->data.end(), -1);

    geometry_msgs::PointStamped point_lidar, point_robot;
    double angle_min = laser_msg.angle_min;
    double angle_max = laser_msg.angle_max;
    double angle_increment = laser_msg.angle_increment;
    double angle_laser;
    double resolution = laser_map->info.resolution;
    double map_origin_x = laser_map->info.origin.position.x;
    double map_origin_y = laser_map->info.origin.position.y;
    int map_width = laser_map->info.width;
    int map_height = laser_map->info.height;
    int map_limit = map_width * map_height;

    for(int i = 0; angle_min + angle_increment * i <= angle_max; ++i) {
        angle_laser = angle_min + angle_increment * i;
        if (laser_msg.ranges[i] < laser_msg.range_min || laser_msg.ranges[i] > laser_msg.range_max)
            continue;

        // Add walkable space
        for (double range_current = 0.0; range_current < laser_msg.ranges[i]; range_current += resolution) {
            int x = ((range_current * cos(angle_laser) - map_origin_x) / resolution);
            int y = ((range_current * sin(angle_laser) - map_origin_y) / resolution);
            int idx = y * map_width + x;
            if((0 < idx) && (idx < map_limit)){
                if(laser_map->data[idx] < 0)
                    laser_map->data[idx] = 0;
            }
        }

        // Add wall(non-walkable) space
        int x = ((laser_msg.ranges[i] * cos(angle_laser) - map_origin_x) / resolution);
        int y = ((laser_msg.ranges[i] * sin(angle_laser) - map_origin_y) / resolution);
        int idx = y * map_width + x;
        
        if((0 < idx) && (idx < map_limit)){
            if(laser_map->data[idx] == 100)
                continue;
            gauss_filter(laser_map->data, map_width, map_height, idx, 7, 100);
        }
    }
    pub_map.publish(*laser_map);

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
    ros::init(argc, argv, "laserscan_mapping_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_scan = nh.subscribe("/scan", 5, scan_cb);
    pub_map = nh.advertise<nav_msgs::OccupancyGrid>("local_map", 1);

    laser_map = new nav_msgs::OccupancyGrid();
    laser_map->header.frame_id = "laser";
    laser_map->info.resolution = 0.1;
    laser_map->info.width = 60;
    laser_map->info.height = 60;
    laser_map->info.origin.position.x = -laser_map->info.resolution * laser_map->info.width / 2;
    laser_map->info.origin.position.y = -laser_map->info.resolution * laser_map->info.height / 2;
    laser_map->info.origin.orientation.w = 1.0;
    laser_map->data.resize(laser_map->info.width * laser_map->info.height);
    

    signal(SIGINT, sigint_cb);
    ros::spin();
    return 0;
}
