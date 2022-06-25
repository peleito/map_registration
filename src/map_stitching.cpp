#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <typeinfo>
#include <iostream>
#include <cstddef>

#include "geometry_msgs/PoseStamped"
#include "sensor_msgs/PointCloud2"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

ros::Subscriber map_sub;
ros::Subscriber scan_sub;
ros::Subscriber pose_sub;
ros::Publisher map_pub;

void mapCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    // Convert to PCD or VTK for libpointmatcher
}

void scanCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    // Convert to PCD or VTK for libpointmatcher
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Save for inital guess for ICP
}

void icp()
{
    // ENSURE SYNCHRONIZATION!!!
    // Take in map (PCD), scan (PCD), and pose ([x,y,z,rx,ry,rz,rw])
    // Filter scan using libpointmatcher
    // Using pose as inital guess, match scan to map
    // Combine point clouds
    // Convert combined to PC2 for ROS
    // Publish combined as map
}

int main(int argc, char **argv)
{
    std::string map_topic;
    std::string scan_topic;
    std::string pose_topic;

    ros::init(argc, argv, "map_stitching");

    ros::NodeHandle nh("~");
    
    ROS_INFO("Running");
    nh.getParam("map_topic", map_topic);
    nh.getParam("scan_topic", scan_topic);
    nh.getParam("pose_topic", pose_topic);
    ROS_INFO("map_topic : %s", map_topic.c_str());
    ROS_INFO("scan_topic : %s", scan_topic.c_str());
    ROS_INFO("pose_topic : %s", pose_topic.c_str());

    map_sub = nh.subscribe(map_topic, 1000, mapCallback);
    scan_sub = nh.subscribe(scan_topic, 1000, scanCallback);
    pose_sub = nh.subscribe(pose_topic, 1000, poseCallback);

    map_pub = nh.advertise<sensor_msgs::PointCloud2>(map_topic, 1000);

    ros::spin();
    return 0;
}