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

// #include "cvsba/cvsba.h"
// #include <vector>

ros::Publisher map_pub;
ros::Publisher map_pub_2;

ros::Publisher test_1;
ros::Publisher test_2;

// bool first;

sensor_msgs::PointCloud2 old_map;
sensor_msgs::PointCloud2 scan;
sensor_msgs::PointCloud2 single;

PointMatcher<float>::TransformationParameters initialT;
std::shared_ptr<PointMatcher<float>::DataPointsFilter> boundingBox;
float xmin,xmax,ymin,ymax,zmin,zmax;

PointMatcher<float>::DataPointsFilters inputFilter;
PointMatcher<float>::ICP icp;

Eigen::Matrix<float,-1,-1> TI(4,4);
Eigen::Matrix<float,-1,-1> T(4,4);

PointMatcher<float>::DataPoints scene; // = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(old_map, false);
PointMatcher<float>::DataPoints matcher;
PointMatcher<float>::DataPoints object; // = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(scan, false);
PointMatcher<float>::DataPoints test; // = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(scan, false);
PointMatcher<float>::DataPoints og; // = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(scan, false);
sensor_msgs::PointCloud2 input; // = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(matcher, "map", ros::Time::now());
sensor_msgs::PointCloud2 reference; // = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(test, "map", ros::Time::now());
PointMatcher<float>::TransformationParameters finalT; // = icp(object, matcher, initialT);
PointMatcher<float>::DataPoints transformed_object; //(og);
PointMatcher<float>::DataPoints current_map;
sensor_msgs::PointCloud2 new_map; // = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(scene, "map", ros::Time::now());
sensor_msgs::PointCloud2 single_map; // = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(current_map, "map", ros::Time::now());

// PointMatcher<float>::DataPoints current_map;

void map_saver();

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

void singleCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    // Convert to PCD or VTK for libpointmatcher
    single = *msg;

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

    // Compute bounds for field of view of l515 to only activate part of the lidar map

    float bound_x_min = -2;
    float bound_x_max = 2;
    float bound_y_min = -1;
    float bound_y_max = 1;
    float bound_z_min = 0.25;
    float bound_z_max = 3;

    Eigen::Vector4f pre_point1(bound_x_min,bound_y_min,bound_z_min,1);
    Eigen::Vector4f pre_point2(bound_x_min,bound_y_min,bound_z_max,1);
    Eigen::Vector4f pre_point3(bound_x_min,bound_y_max,bound_z_max,1);
    Eigen::Vector4f pre_point4(bound_x_min,bound_y_max,bound_z_min,1);
    Eigen::Vector4f pre_point5(bound_x_max,bound_y_min,bound_z_min,1);
    Eigen::Vector4f pre_point6(bound_x_max,bound_y_min,bound_z_max,1);
    Eigen::Vector4f pre_point7(bound_x_max,bound_y_max,bound_z_max,1);    
    Eigen::Vector4f pre_point8(bound_x_max,bound_y_max,bound_z_min,1);

    Eigen::Vector4f post_point1;
    Eigen::Vector4f post_point2;
    Eigen::Vector4f post_point3;
    Eigen::Vector4f post_point4;
    Eigen::Vector4f post_point5;
    Eigen::Vector4f post_point6;
    Eigen::Vector4f post_point7;
    Eigen::Vector4f post_point8;

    post_point1 = initialT*pre_point1;
    post_point2 = initialT*pre_point2;
    post_point3 = initialT*pre_point3;
    post_point4 = initialT*pre_point4;
    post_point5 = initialT*pre_point5;
    post_point6 = initialT*pre_point6;
    post_point7 = initialT*pre_point7;
    post_point8 = initialT*pre_point8;

    float x_points[] = {post_point1(0,0),post_point2(0,0),post_point3(0,0),post_point4(0,0),post_point5(0,0),post_point6(0,0),post_point7(0,0),post_point8(0,0)};

    float y_points[] = {post_point1(1,0),post_point2(1,0),post_point3(1,0),post_point4(1,0),post_point5(1,0),post_point6(1,0),post_point7(1,0),post_point8(1,0)};

    float z_points[] = {post_point1(2,0),post_point2(2,0),post_point3(2,0),post_point4(2,0),post_point5(2,0),post_point6(2,0),post_point7(2,0),post_point8(2,0)};

    xmin = *std::min_element(std::begin(x_points),std::end(x_points));
    xmax = *std::max_element(std::begin(x_points),std::end(x_points));
    ymin = *std::min_element(std::begin(y_points),std::end(y_points));
    ymax = *std::max_element(std::begin(y_points),std::end(y_points));
    zmin = *std::min_element(std::begin(z_points),std::end(z_points));
    zmax = *std::max_element(std::begin(z_points),std::end(z_points));

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
        // ROS_INFO("1");
        scene = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(old_map, false);

        boundingBox = PointMatcher<float>::get().DataPointsFilterRegistrar.create(
            "BoundingBoxDataPointsFilter",
            {
                {"xMin", PointMatcherSupport::toParam(xmin)},
                {"xMax", PointMatcherSupport::toParam(xmax)},
                {"yMin", PointMatcherSupport::toParam(ymin)},
                {"yMax", PointMatcherSupport::toParam(ymax)},
                {"zMin", PointMatcherSupport::toParam(zmin)},
                {"zMax", PointMatcherSupport::toParam(zmax)},
                {"removeInside",PointMatcherSupport::toParam(0)}

            }
        );

        matcher = boundingBox->filter(scene);

        // Old Map Data (PointCloud2) Converted to DataPoints
        // ROS_INFO("2");
        object = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(scan, false);
        test = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(scan, false);
        og = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(scan, false);
        // ROS_INFO("3");
        inputFilter.apply(test);
        inputFilter.apply(object);

        icp.transformations.apply(test, initialT);

        // ROS_INFO("3");
        input = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(matcher, "map", ros::Time::now());
        // ROS_INFO("4");
        reference = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(test, "map", ros::Time::now());
        test_1.publish(input);
        test_2.publish(reference);


        // Find the transformation from running the ICP algorithm
        // ROS_INFO("4");
        ROS_INFO("Start ICP");
        ros::Time begin = ros::Time::now();
        finalT = icp(object, matcher, initialT);
        ros::Time after = ros::Time::now();
        ROS_INFO("End ICP : %lf", (after-begin).toSec());

        // Print and Apply Transformation Matrix
        // ROS_INFO("5");
        std::cout << "Transformation Matrix = \n" << finalT << std::endl;

        transformed_object = og;
        // ROS_INFO("6");
        icp.transformations.apply(transformed_object, finalT);

        // ROS_INFO("7");
        scene.concatenate(transformed_object);
        
        try
        {
            // ROS_INFO("try");
            current_map = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(single, false);
            current_map.concatenate(transformed_object);
        }
        catch(...)
        {
            // ROS_INFO("catch");
            current_map = transformed_object;
        }
        
        // Save DataPoints as PointCloud2
        // ROS_INFO("8");
        new_map = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(scene, "map", ros::Time::now());
        single_map = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(current_map, "map", ros::Time::now());

        // Publish new_map on map topic
        // ROS_INFO("9");
        map_pub.publish(new_map);
        map_pub_2.publish(single_map);

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
    // ROS_INFO("Input Filters");
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
    // ROS_INFO("ICP");
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
    // ROS_INFO("PCD");
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
}

void setInitialTransform()
{
    T.setIdentity();

    float x,y,z,rx,ry,rz,rw;
    x = 0.05;
    y = 0;
    z = 0.07831;
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
    ROS_INFO("1");
    setInputFilters(input_config);
    ROS_INFO("2");
    setICPParameters(icp_config);
    ROS_INFO("3");
    setInitialMap(prior_map);
    ROS_INFO("4");
    setInitialTransform();
    ROS_INFO("5");
}

void map_saver()
{
    ROS_INFO("Saving");

    // ROS_INFO("1");
    // ROS_INFO("2");
    pcl::PointCloud<pcl::PointXYZRGB> camera;
    // ROS_INFO("3");
    pcl::fromROSMsg(single,camera);
    // ROS_INFO("4");
    pcl::io::savePCDFileASCII("/home/trex/map_ws/camera.pcd",camera);
    ROS_INFO("5");

    // ROS_INFO("1");
    // ROS_INFO("2");
    pcl::PointCloud<pcl::PointXYZRGB> combined;
    // ROS_INFO("3");
    pcl::fromROSMsg(old_map,combined);
    ROS_INFO("4");
    pcl::io::savePCDFileASCII("/home/trex/map_ws/combined.pcd",combined);
    ROS_INFO("5");

}

void signal_handler(int signal)
{
    map_saver();

    ROS_INFO("Quitting");
    exit(signal);
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
    ros::Subscriber map_sub_2;
    ros::Subscriber scan_sub;
    ros::Subscriber pose_sub;

    ros::init(argc, argv, "joint_map_stitching",ros::init_options::NoSigintHandler);
    std::signal(SIGINT, signal_handler);

    ros::NodeHandle nh("~");

    // ROS_INFO("Running");
    nh.getParam("map_topic", map_topic);
    nh.getParam("scan_topic", scan_topic);
    nh.getParam("pose_topic", pose_topic);
    nh.getParam("icp_config", icp_config);
    nh.getParam("prior_map", prior_map);
    nh.getParam("input_config", input_config);

    // ROS_INFO("map_topic : %s", map_topic.c_str());
    // ROS_INFO("scan_topic : %s", scan_topic.c_str());
    // ROS_INFO("pose_topic : %s", pose_topic.c_str());
    // ROS_INFO("icp_config : %s", icp_config.c_str());
    // ROS_INFO("prior_map : %s", prior_map.c_str());
    // ROS_INFO("input_config : %s", input_config.c_str());

    std::string map_topic_2 = map_topic+"/l515";

    map_sub = nh.subscribe(map_topic, 1000, mapCallback);
    map_sub_2 = nh.subscribe(map_topic_2, 1000, singleCallback);
    scan_sub = nh.subscribe(scan_topic, 1000, scanCallback);
    pose_sub = nh.subscribe(pose_topic, 1000, poseCallback);

    map_pub = nh.advertise<sensor_msgs::PointCloud2>(map_topic, 1000);
    map_pub_2 = nh.advertise<sensor_msgs::PointCloud2>(map_topic_2, 1000);
    test_1 = nh.advertise<sensor_msgs::PointCloud2>("/test1", 1000);
    test_2 = nh.advertise<sensor_msgs::PointCloud2>("/test2", 1000);


    ROS_INFO("Begin Initialize");
    initialize(input_config,icp_config,prior_map);
    ROS_INFO("Finish Initialize");
    // ros::spin();

    nh.setParam("/register",false);
    bool accept = false;

    ros::Rate loop_rate(1);
    while (ros::ok())
    {        
        // ROS_INFO("Cycle");
        ros::spinOnce();
        
        
        nh.getParam("/register",accept);
        std::cout << accept;
        if (accept)
        {
            ROS_INFO("Accepting");
            icp_algorithm();
            nh.setParam("/register",false);
        }
        else
        {
            ROS_INFO("Not ready to accept");
        }
        
        loop_rate.sleep();
    }
    return 0;
}