#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "regulator");
    ros::NodeHandle nh("~");

    double rate;
    nh.getParam("rate",rate);

    ros::Rate loop_rate(rate);

    while (ros::ok())
    {
        nh.setParam("/register",true);
        // ROS_INFO("spin");
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}