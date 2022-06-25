#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <typeinfo>
#include <iostream>
#include <cstddef>

#include "geometry_msgs/PoseStamped"
#include "nav_msgs/Odometry"

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/ColorOcTree.h>

ros::Subscriber pose_sub;
ros::Publisher estimate_pub;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Save as octomath::Pose6D
}

void globalCallback(const octomap_msgs::Octomap::ConstPtr &msg)
{
    // Save as octomap::OccupancyOcTreeBase
}

void scanCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    // Save as octomap::Pointcloud
}

void octomap_merge()
{
    // Create octomap::ScanNode from scan and pose
    // Add new scan to map using insertPointCloud and ScanNode on OccupancyOcTreeBase
    // Publish octomap
}

int main(int argc, char **argv)
{
    std::string pose_topic;
    std::string local_topic;
    std::string global_topic;

    ros::init(argc, argv, "octomap_creation");

    ros::NodeHandle nh("~");
    
    ROS_INFO("Running");
    nh.getParam("pose_topic", pose_topic);
    nh.getParam("local_topic", local_topic);
    nh.getParam("global_topic", global_topic);
    ROS_INFO("pose_topic : %s", pose_topic.c_str());
    ROS_INFO("local_topic : %s", local_topic.c_str());
    ROS_INFO("global_topic : %s", global_topic.c_str());

    pose_sub = nh.subscribe(pose_topic, 1000, poseCallback);
    global_sub = nh.subscribe(global_topic, 1000, globalCallback);
    scan_sub = nh.subscribe(scan_topic, 1000, scanCallback);

    local_pub = nh.advertise<octomap_msgs::Octomap>(local_topic, 1000);
    global_pub = nh.advertise<octomap_msgs::Octomap>(global_topic, 1000);

    ros::spin();
    return 0;
}