#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <typeinfo>
#include <iostream>
#include <cstddef>

#include "geometry_msgs/PoseStamped"
#include "nav_msgs/Odometry"

ros::Subscriber pose_sub;
ros::Publisher estimate_pub;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Wheel +  manipulator odometry??
}

int main(int argc, char **argv)
{
    std::string pose_topic;
    std::string out_topic;

    ros::init(argc, argv, "pose_estimation");

    ros::NodeHandle nh("~");
    
    ROS_INFO("Running");
    nh.getParam("pose_topic", pose_topic);
    nh.getParam("output_topic", out_topic);
    ROS_INFO("pose_topic : %s", pose_topic.c_str());
    ROS_INFO("output_topic : %s", out_topic.c_str());

    pose_sub = nh.subscribe(pose_topic, 1000, poseCallback);
    
    estimate_pub = nh.advertise<nav_msgs::Odometry>(out_topic, 1000);

    ros::spin();
    return 0;
}