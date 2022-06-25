#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <typeinfo>
#include <iostream>
#include <cstddef>
#include <csignal>

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/point_cloud.h"
#include "pcl/point_cloud.h"

#include "pointmatcher/PointMatcher.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/ros_logger.h"

#include <cassert>
#include <fstream>
#include <algorithm>
#include <Eigen/Dense>

#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "boost/filesystem.hpp"

#include <cassert>
#include <fstream>
#include <iostream>
#include <algorithm>

ros::Publisher map_pub;
ros::Publisher test_1;
ros::Publisher test_2;

sensor_msgs::PointCloud2 lower;
sensor_msgs::PointCloud2 upper;

PointMatcher<float>::TransformationParameters initialT;
PointMatcher<float>::DataPointsFilters inputFilter;
PointMatcher<float>::ICP icp;

PointMatcher<float>::DataPoints scene;
PointMatcher<float>::DataPoints matcher;
PointMatcher<float>::DataPoints object;
PointMatcher<float>::DataPoints og;
PointMatcher<float>::TransformationParameters finalT;
sensor_msgs::PointCloud2 new_map;

float x = -2.7;
float y = 2.9;
float z = -1;
float roll = 0;
float pitch = 0;
float yaw  = -117.5;

PointMatcher<float>::DataPoints test;
sensor_msgs::PointCloud2 input;
sensor_msgs::PointCloud2 reference;

void icp_algorithm()
{
    ROS_INFO("Begin Method");

    try
    {
        lower.header.frame_id = "map";
        upper.header.frame_id = "map";
        test_1.publish(lower);
        test_2.publish(upper);

        scene = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(lower, false);
        matcher = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(lower, false);

        object = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(upper, false);
        og = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(upper, false);

        inputFilter.apply(object);

        ROS_INFO("Starting Clouds");
        test = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(upper, false);
        icp.transformations.apply(test, initialT);
        input = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(matcher, "map", ros::Time::now());
        reference = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(test, "map", ros::Time::now());
        test_1.publish(input);
        test_2.publish(reference);
        ROS_INFO("End Starting Clouds");

        ROS_INFO("Start ICP");
        ros::Time begin = ros::Time::now();
        ROS_INFO("1");
        finalT = icp(object, matcher, initialT);
        ROS_INFO("2");
        ros::Time after = ros::Time::now();
        ROS_INFO("End ICP : %lf", (after-begin).toSec());
        std::cout << "Transformation Matrix = \n" << finalT << std::endl;


        icp.transformations.apply(og, finalT);
        scene.concatenate(og);

        new_map = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(scene, "map", ros::Time::now());
        map_pub.publish(new_map);

        // std::string outputBaseFile("test");

        // matcher.save(outputBaseFile + "_ouster.vtk");
        // test.save(outputBaseFile + "_iphone.vtk");
        // scene.save(outputBaseFile + "_combined.vtk");

        // if(true) 
        // {
        //     std::ofstream transfoFile;
        //     std::string initFileName = outputBaseFile + "_init_transfo.txt";
        //     std::string icpFileName = outputBaseFile + "_icp_transfo.txt";
        //     std::string completeFileName = outputBaseFile + "_complete_transfo.txt";

        //     transfoFile.open(initFileName.c_str());
        //     if(transfoFile.is_open()) 
        //     {
        //         transfoFile << initialT << std::endl;
        //         transfoFile.close();
        //     } 
        //     else 
        //     {
        //         std::cerr << "Unable to write the initial transformation file\n" << std::endl;
        //     }

        //     transfoFile.open(icpFileName.c_str());
        //     if(transfoFile.is_open()) 
        //     {
        //         transfoFile << finalT << std::endl;
        //         transfoFile.close();
        //     } 
        //     else 
        //     {
        //         std::cerr << "Unable to write the ICP transformation file\n" << std::endl;
        //     }

        //     transfoFile.open(completeFileName.c_str());
        //     if(transfoFile.is_open()) 
        //     {
        //         transfoFile << finalT*initialT << std::endl;
        //         transfoFile.close();
        //     } 
        //     else
        //     {
        //         std::cerr << "Unable to write the complete transformation file\n" << std::endl;
        //     }
        // }
        // else 
        // {
        //     if(true)
        //     {
        //         std::cout << "ICP transformation:" << std::endl << finalT << std::endl;
        //     }
        // }

    }
    catch (const PointMatcher<float>::ConvergenceError e)
    {
        ROS_INFO("Convergence FAILED");
    }
    // catch (...)
    // {
    //     ROS_INFO("ICP FAILED");
    // }

    ROS_INFO("DONE");
}

void setTransform()
{
    float rx, ry, rz, rw;
    tf2::Quaternion quat;
    quat.setRPY(roll*3.14/180.0,pitch*3.14/180.0,yaw*3.14/180.0);

    rx = quat.getX();
    ry = quat.getY();
    rz = quat.getZ();
    rw = quat.getW();

    Eigen::Vector3f position(x,y,z);
    Eigen::Quaternionf orientation(rw,rx,ry,rz);

    Eigen::Matrix<float,-1,-1> T(4,4);
    T.setIdentity();
    T.block(0, 3, 3, 1) = static_cast<Eigen::Matrix<float,3,1>>(position);
    T.block(0, 0, 3, 3) = static_cast<Eigen::Quaternion<float>>(orientation).toRotationMatrix();
    initialT = T;
}

void setInputFilters(std::string input_config)
{
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
    if (icp_config.empty())
	{
		icp.setDefault();
	}
	else
	{
		std::ifstream ifs(icp_config.c_str());
		if (!ifs.good())
		{
			std::cerr << "Cannot open config file " << icp_config;
		}
		icp.loadFromYaml(ifs);
	}
}

void setInitialMap_lower(std::string prior_map)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (prior_map, *cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }

    sensor_msgs::PointCloud2 map;
    pcl::toROSMsg(*cloud,map);

    lower = map;
}

void setInitialMap_upper(std::string prior_map)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (prior_map, *cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }

    sensor_msgs::PointCloud2 map;
    pcl::toROSMsg(*cloud,map);

    upper = map;
}

void initialize(std::string input_config, std::string icp_config, std::string map_topic_lower, std::string map_topic_upper)
{

    ROS_INFO("1");
    setInputFilters(input_config);
    ROS_INFO("2");
    setICPParameters(icp_config);
    ROS_INFO("3");
    setInitialMap_lower(map_topic_lower);
    ROS_INFO("4");
    setInitialMap_upper(map_topic_upper);
    ROS_INFO("5");
    setTransform();
    ROS_INFO("6");
}

int main(int argc, char **argv)
{

    std::string input_config;
    std::string icp_config;
    std::string lower;
    std::string upper;

    ros::init(argc, argv, "upper_lower_stitching");

    ros::NodeHandle nh("~");

    test_1 = nh.advertise<sensor_msgs::PointCloud2>("/test1", 1000);
    test_2 = nh.advertise<sensor_msgs::PointCloud2>("/test2", 1000);

    nh.getParam("icp_config", icp_config);
    nh.getParam("lower", lower);
    nh.getParam("upper", upper);
    nh.getParam("input_config", input_config);
    ROS_INFO("prior_map : %s", lower.c_str());
    ROS_INFO("input_config : %s", upper.c_str());

    map_pub = nh.advertise<sensor_msgs::PointCloud2>("/combined", 1000);

    ROS_INFO("Begin Initialize");
    initialize(input_config,icp_config,lower,upper);
    ROS_INFO("Finish Initialize");

    icp_algorithm();

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        try
        {
            test_1.publish(input);
            test_2.publish(reference);
            map_pub.publish(new_map);
        }
        catch(...)
        {
            ROS_INFO("Error");
        }
        
        loop_rate.sleep();
        
    }
    return 0;
}