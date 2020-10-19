#include <iostream>
using namespace std;
#include <nav_msgs/GetMap.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <tf/tf.h>
using namespace cv;

void mapCallback(const nav_msgs::OccupancyGridConstPtr &map) {
    geometry_msgs::Quaternion orientation = map->info.origin.orientation;
    double yaw, pitch, roll;
    tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y,
                                     orientation.z, orientation.w));
    mat.getEulerYPR(yaw, pitch, roll);
    double map_theta = yaw;

    ROS_INFO("Received a %d X %d map @ %.3f m/pix_%f_%f_%f", map->info.width,
             map->info.height, map->info.resolution,
             map->info.origin.position.x, map->info.origin.position.y,
             map_theta);

    Mat img_out =
        Mat::zeros(cv::Size(map->info.width, map->info.height), CV_8UC1);

    for(unsigned int y = 0; y < map->info.height; y++) {
        for(unsigned int x = 0; x < map->info.width; x++) {
            unsigned int i = x + (map->info.height - y - 1) * map->info.width;
            int intensity = 205;
            if(map->data[i] >= 0 && map->data[i] <= 100)
                intensity = round((float)(100.0 - map->data[i]) * 2.55);
            img_out.at<unsigned char>(y, x) = intensity;
        }
    }

    imwrite("map.png", img_out);

    ros::shutdown();
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "map_save");
    ros::NodeHandle n;
    ros::Subscriber map_sub_ = n.subscribe("map", 2, mapCallback);
    ros::Rate r(10.0);
    ros::spin();
    return 0;
}