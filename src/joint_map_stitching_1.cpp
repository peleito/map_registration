#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <typeinfo>
#include <iostream>
#include <cstddef>

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pointmatcher/PointMatcher.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/ros_logger.h"

#include <cassert>
#include <fstream>
#include <algorithm>
#include <Eigen/Dense>

ros::Publisher map_pub;

sensor_msgs::PointCloud2 old_map;
sensor_msgs::PointCloud2 scan;
PointMatcher<float>::TransformationParameters initialT;

PointMatcher<float>::DataPointsFilters inputFilter;
PointMatcher<float>::ICP icp;

Eigen::Matrix<float,-1,-1> TI(4,4);
Eigen::Matrix<float,-1,-1> T(4,4);


void mapCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    // Convert to PCD or VTK for libpointmatcher
    old_map = *msg;
}

void scanCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    // Convert to PCD or VTK for libpointmatcher
    scan = *msg;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Save for inital guess for ICP

    geometry_msgs::PoseStamped pose = *msg;

    float x,y,z,rx,ry,rz,rw;
    x = pose.pose.position.x;
    y = pose.pose.position.y;
    z = pose.pose.position.z;
    rx = pose.pose.orientation.x;
    ry = pose.pose.orientation.y;
    rz = pose.pose.orientation.z;
    rw = pose.pose.orientation.w;

    Eigen::Vector3f position(x,y,z);
    Eigen::Quaternionf orientation(rw,rx,ry,rz);

    Eigen::Matrix<float,-1,-1> T(4,4);
    T.setIdentity();
    T.block(0, 3, 3, 1) = static_cast<Eigen::Matrix<float,3,1>>(position);
    T.block(0, 0, 3, 3) = static_cast<Eigen::Quaternion<float>>(orientation).toRotationMatrix();
    initialT = T*TI;
    
    // delete pose;
}

void icp_algorithm()
{
    // ENSURE SYNCHRONIZATION!!!
    // Take in map (PCD), scan (PCD), and pose ([x,y,z,rx,ry,rz,rw])
    // Filter scan using libpointmatcher
    // Using pose as inital guess, match scan to map
    // Combine point clouds
    // Convert combined to PC2 for ROS
    // Publish combined as map

    try
    {
        // New Sensor Data (PointCloud2) Converted to DataPoints
        // sensor_msgs::PointCloud2 scene_Cloud_libpointmatcher;
        ROS_INFO("1");
        PointMatcher<float>::DataPoints scene = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(old_map, false);

        // Old Map Data (PointCloud2) Converted to DataPoints
        // sensor_msgs::PointCloud2 obj_Cloud_libpointmatcher;
        ROS_INFO("2");
        PointMatcher<float>::DataPoints object = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(scan, false);
        ROS_INFO("3");
        inputFilter.apply(object);

        // Find the transformation from running the ICP algorithm
        ROS_INFO("4");
        PointMatcher<float>::TransformationParameters finalT = icp(object, scene, initialT);

        // Print and Apply Transformation Matrix
        ROS_INFO("5");
        std::cout << "Transformation Matrix = \n" << finalT << std::endl;
        // std::cout << "Transformation Matrix = \n" << initialT << std::endl;

        PointMatcher<float>::DataPoints transformed_object(object);
        ROS_INFO("6");
        // icp.transformations.apply(transformed_object, finalT);
        icp.transformations.apply(transformed_object, finalT);

        ROS_INFO("7");
        scene.concatenate(transformed_object);

        // Save DataPoints as PointCloud2
        ROS_INFO("8");
        sensor_msgs::PointCloud2 new_map = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(scene, "map", ros::Time::now());

        // Publish new_map on map topic
        ROS_INFO("9");
        map_pub.publish(new_map);
    }
    catch (const PointMatcher<float>::ConvergenceError e)
    {
        ROS_INFO("Convergence FAILED");
    }
    catch (...)
    {
        ROS_INFO("ICP FAILED");
    }
    
}

void setInputFilters(std::string input_config)
{
    ROS_INFO("Input Filters");
    // Set up input scan filter
    try 
    {
		std::ifstream in(input_config);
		if (!in.is_open()) 
        {
			throw std::runtime_error("config file filters opening failed");
		}
		inputFilter = PointMatcher<float>::DataPointsFilters(in);
		in.close();
	} 
    catch (const std::exception &e) 
    {
		std::cout << e.what() << std::endl;
	}
}

void setICPParameters(std::string icp_config)
{
    ROS_INFO("ICP");
    // Check if icp yaml configuration file is valid before loading
    if (icp_config.empty())
	{
		// See the implementation of setDefault() to create a custom ICP algorithm
		icp.setDefault();
	}
	else
	{
		// load YAML config
		std::ifstream ifs(icp_config.c_str());
		if (!ifs.good())
		{
			std::cerr << "Cannot open config file " << icp_config;
		}
		icp.loadFromYaml(ifs);
	}
}

void setInitialMap(std::string prior_map)
{
    ROS_INFO("PCD");
    // Read PCD file from step 0 and save as pcl
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (prior_map, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }

    // Convert pcl to pc2 
    sensor_msgs::PointCloud2 map;
    pcl::toROSMsg(*cloud,map);

    // Publish step 0 map on the map topic
    map_pub.publish(map);

    // delete &cloud;
    // delete &map;
}

void setInitialTransform()
{
    T.setIdentity();

    float x,y,z,rx,ry,rz,rw;
    x = 0.0415;
    y = 0;
    z = 0.06135;
    rx = 0.5;
    ry = 0.5;
    rz = 0.5;
    rw = 0.5;

    Eigen::Vector3f positionI(x,y,z);
    Eigen::Quaternionf orientationI(rw,rx,ry,rz);

    TI.setIdentity();
    TI.block(0, 3, 3, 1) = static_cast<Eigen::Matrix<float,3,1>>(positionI);
    TI.block(0, 0, 3, 3) = static_cast<Eigen::Quaternion<float>>(orientationI).toRotationMatrix();
}

void initialize(std::string input_config, std::string icp_config, std::string prior_map)
{
    setInputFilters(input_config);

    setICPParameters(icp_config);

    setInitialMap(prior_map);

    setInitialTransform();
}

int main(int argc, char **argv)
{
    std::string input_config;
    std::string icp_config;
    std::string prior_map;
    std::string map_topic;
    std::string scan_topic;
    std::string pose_topic;

    ros::Subscriber map_sub;
    ros::Subscriber scan_sub;
    ros::Subscriber pose_sub;

    ros::init(argc, argv, "joint_map_stitching");

    ros::NodeHandle nh("~");

    ROS_INFO("Running");
    nh.getParam("map_topic", map_topic);
    nh.getParam("scan_topic", scan_topic);
    nh.getParam("pose_topic", pose_topic);
    nh.getParam("icp_config", icp_config);
    nh.getParam("prior_map", prior_map);
    nh.getParam("input_config", input_config);

    ROS_INFO("map_topic : %s", map_topic.c_str());
    ROS_INFO("scan_topic : %s", scan_topic.c_str());
    ROS_INFO("pose_topic : %s", pose_topic.c_str());
    ROS_INFO("icp_config : %s", icp_config.c_str());
    ROS_INFO("prior_map : %s", prior_map.c_str());
    ROS_INFO("input_config : %s", input_config.c_str());

    map_sub = nh.subscribe(map_topic, 1000, mapCallback);
    scan_sub = nh.subscribe(scan_topic, 1000, scanCallback);
    pose_sub = nh.subscribe(pose_topic, 1000, poseCallback);

    map_pub = nh.advertise<sensor_msgs::PointCloud2>(map_topic, 1000);

    ROS_INFO("Begin Initialize");
    initialize(input_config,icp_config,prior_map);
    ROS_INFO("Finish Initialize");
    // ros::spin();

    ros::Rate loop_rate(2);
    while (ros::ok())
    {        
        ROS_INFO("Cycle");
        ros::spinOnce();
        
        icp_algorithm();
        
        loop_rate.sleep();
    }
    return 0;
}