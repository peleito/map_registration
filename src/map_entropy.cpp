#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <typeinfo>
#include <iostream>
#include <cstddef>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/ColorOcTree.h>

ros::Subscriber octomap_sub;
ros::Publisher entropy_pub;

void octCallback(const octomap_msgs::Octomap::ConstPtr &msg)
{
    double entropy = 0;
    octomap::AbstractOcTree* deserial;
    deserial = octomap_msgs::fullMsgToMap(*msg);

    octomap::ColorOcTree* octomap;
    octomap = (octomap::ColorOcTree*) deserial;

    // get dimensions of octree
    double minX, minY, minZ, maxX, maxY, maxZ;
    octomap->getMetricMin(minX, minY, minZ);
    octomap->getMetricMax(maxX, maxY, maxZ);
    double resolution = octomap->getResolution();
    size_t octomap_size = octomap->getNumLeafNodes();

    double volume = abs(maxX-minX)*abs(maxY-minY)*abs(maxZ-minZ);
    double region_size = volume/pow(resolution,3);
    long long int unexplored_size = static_cast<long long int>(region_size)-static_cast<long long int>(octomap_size);

    // traverse all leafs in the tree:
    unsigned int treeDepth = octomap->getTreeDepth();
    double maxHeight = maxZ;
    double minHeight = minZ;
    int stepSize = 1 << (octomap->getTreeDepth() - treeDepth); // for pruning of occluded voxels
    for (octomap::ColorOcTree::leaf_iterator it = octomap->begin(treeDepth), end = octomap->end(); it != end; ++it)
    {
        if(it.getZ() <= maxHeight && it.getZ() >= minHeight)
        {
            float occupied_probability;
            float unoccupied_probability;
            occupied_probability = it->getOccupancy();
            unoccupied_probability = 1-occupied_probability;
            ROS_INFO("Cell probability : %f", occupied_probability);
            entropy = entropy-(occupied_probability*log2(occupied_probability)+unoccupied_probability*log2(unoccupied_probability));
        }
    }
    double unexplored_probability = 0.5;
    entropy = entropy-(unexplored_size)*(2*(unexplored_probability*log2(unexplored_probability)));
    ROS_INFO("Entropy : %f", entropy);
    
    std_msgs::Int64 entropy_msg;
    entropy_msg.data = entropy;
    entropy_pub.publish(entropy_msg);

    delete octomap;
}

int main(int argc, char **argv)
{
    std::string oct_topic;
    std::string out_topic;

    ros::init(argc, argv, "map_entropy");

    ros::NodeHandle nh("~");
    
    ROS_INFO("Running");
    nh.getParam("octomap_topic", oct_topic);
    nh.getParam("output_topic", out_topic);
    ROS_INFO("octomap_topic : %s", oct_topic.c_str());
    ROS_INFO("output_topic : %s", out_topic.c_str());

    octomap_sub = nh.subscribe(oct_topic, 1000, octCallback);
    
    entropy_pub = nh.advertise<std_msgs::Int64>(out_topic, 1000);

    ros::spin();
    return 0;
}