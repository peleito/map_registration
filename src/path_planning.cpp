#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <typeinfo>
#include <iostream>
#include <cstddef>

#include "geometry_msgs/PoseArray"
#include "nav_msgs/Odometry"

ros::Subscriber pose_sub;
ros::Publisher estimate_pub;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Save current location for planning new path
}

void octCallback(const octomap_msgs::Octomap::ConstPtr &msg)
{
    // Determine regions of low entropy
    // Determine regions with high keypoints (features)
}

void path_planning()
{
    // Determine new path based on entroypy and keypoints of map and robot region
}

int main(int argc, char **argv)
{
    std::string pose_topic;
    std::string out_topic;

    ros::init(argc, argv, "path_planning");

    ros::NodeHandle nh("~");
    
    ROS_INFO("Running");
    nh.getParam("pose_topic", pose_topic);
    nh.getParam("output_topic", out_topic);
    ROS_INFO("pose_topic : %s", pose_topic.c_str());
    ROS_INFO("output_topic : %s", out_topic.c_str());

    pose_sub = nh.subscribe(pose_topic, 1000, poseCallback);
    
    estimate_pub = nh.advertise<geometry_msgs::PoseArray>("/planned_path", 1000);

    ros::spin();
    return 0;
}