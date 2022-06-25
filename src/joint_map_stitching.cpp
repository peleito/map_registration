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
std::shared_ptr<PointMatcher<float>::DataPointsFilter> fov;
std::shared_ptr<PointMatcher<float>::DataPointsFilter> downsample;

float pose_x, pose_y, pose_z, pose_qx, pose_qy, pose_qz, pose_qw;
float max_distance = 7;
float max_horizontal = 1.50; // 0.75 is too small
float max_vertical = 1.0;
float weight_prob = 1.0;
float sample_method = 0;

PointMatcher<float>::DataPointsFilters inputFilter;
PointMatcher<float>::ICP icp;

Eigen::Matrix<float,-1,-1> TI(4,4);
Eigen::Matrix<float,-1,-1> T(4,4);

PointMatcher<float>::DataPoints scene; // = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(old_map, false);
PointMatcher<float>::DataPoints start_map;
bool first = true;
PointMatcher<float>::DataPoints matcher;
PointMatcher<float>::DataPoints object; // = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(scan, false);
PointMatcher<float>::DataPoints test; // = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(scan, false);
PointMatcher<float>::DataPoints test2; // = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(scan, false);
PointMatcher<float>::DataPoints og; // = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(scan, false);
sensor_msgs::PointCloud2 input; // = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(matcher, "map", ros::Time::now());
sensor_msgs::PointCloud2 reference; // = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(test, "map", ros::Time::now());
PointMatcher<float>::TransformationParameters finalT; // = icp(object, matcher, initialT);
PointMatcher<float>::DataPoints transformed_object; //(og);
PointMatcher<float>::DataPoints current_map;
sensor_msgs::PointCloud2 new_map; // = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(scene, "map", ros::Time::now());
sensor_msgs::PointCloud2 single_map; // = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(current_map, "map", ros::Time::now());

bool processing = false;

float seq = 0;
float shift1 = 4280; // 4280->4360->4450
float shift2 = 4450;
float zLow = -0.08;
float zHigh = 0.35;
float slope = (zHigh-zLow)/(shift2-shift1);
bool path = true;
// PointMatcher<float>::DataPoints current_map;

void map_saver();

void mapCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    // Convert to PCD or VTK for libpointmatcher
    if (!processing)
    {
        old_map = *msg;
    }
    

}

void scanCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    // Convert to PCD or VTK for libpointmatcher
    if (!processing)
    {
        scan = *msg;
        seq = scan.header.seq;
    }
    
    
}

void singleCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    // Convert to PCD or VTK for libpointmatcher
    if (!processing)
    {
        single = *msg;
    }
    

}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Save for inital guess for ICP
    if (!processing)
    {
        geometry_msgs::PoseStamped pose = *msg;

        float x,y,z,rx,ry,rz,rw;
        x = pose.pose.position.x;
        y = pose.pose.position.y;
        z = pose.pose.position.z;
        rx = pose.pose.orientation.x;
        ry = pose.pose.orientation.y;
        rz = pose.pose.orientation.z;
        rw = pose.pose.orientation.w;

        if (path)
        {
            if (seq<shift1)
            {
                z = zLow;
            }
            else if (seq<shift2)
            {
                z = zLow+(seq-shift1)*slope;
            }
            else
            {
                z = zHigh;
            }
            rx = 0;
            ry = 0;
            float mag = sqrt((rz*rz+rw*rw));
            rz = rz/mag;
            rw = rw/mag;
        }

        Eigen::Vector3f position(x,y,z);
        Eigen::Quaternionf orientation(rw,rx,ry,rz);

        Eigen::Matrix<float,-1,-1> T(4,4);
        T.setIdentity();
        T.block(0, 3, 3, 1) = static_cast<Eigen::Matrix<float,3,1>>(position);
        T.block(0, 0, 3, 3) = static_cast<Eigen::Quaternion<float>>(orientation).toRotationMatrix();
        initialT = T*TI;

        pose_x = x;
        pose_y = y;
        pose_z = z;
        pose_qx = rx;
        pose_qy = ry;
        pose_qz = rz;
        pose_qw = rw;
    }

    

}

void icp_algorithm()
{
    ROS_INFO("Begin Method");
    ros::Time begin2 = ros::Time::now();
    // ENSURE SYNCHRONIZATION!!!
    // Take in map (PCD), scan (PCD), and pose ([x,y,z,rx,ry,rz,rw])
    // Filter scan using libpointmatcher
    // Using pose as inital guess, match scan to map
    // Combine point clouds
    // Convert combined to PC2 for ROS
    // Publish combined as map
    processing = true;

    

    try
    {
        
        // New Sensor Data (PointCloud2) Converted to DataPoints
        // ROS_INFO("1");
        scene = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(old_map, false);

        ///////////////////////////////////////////////////////
        tf::Quaternion q(pose_qx,pose_qy,pose_qz,pose_qw);
        // tf::Matrix3x3 m(q);
        // double roll, pitch, yaw;
        // m.getRPY(roll, pitch, yaw);
        
        // ROS_INFO("pose_x  : %f", pose_x);
        // ROS_INFO("pose_y  : %f", pose_y);
        // ROS_INFO("pose_z  : %f", pose_z);
        // // ROS_INFO("pose_qx : %f", pose_qx);
        // // ROS_INFO("pose_qy : %f", pose_qy);
        // // ROS_INFO("pose_qz : %f", pose_qz);
        // // ROS_INFO("pose_qw : %f", pose_qw); 

        // ROS_INFO("pose_qy : %f", roll*180/3.14);
        // ROS_INFO("pose_qz : %f", pitch*180/3.14);
        // ROS_INFO("pose_qw : %f", yaw*180/3.14);

        // static tf::TransformBroadcaster br;
        // tf::Transform transform;
        // transform.setOrigin( tf::Vector3(pose_x, pose_y, pose_z) );
        // transform.setRotation(q);
        // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "camera_center"));

        // ROS_INFO("2");
        fov = PointMatcher<float>::get().DataPointsFilterRegistrar.create(
            "FOVDataPointsFilter",
            {
                {"x", PointMatcherSupport::toParam(pose_x)},
                {"y", PointMatcherSupport::toParam(pose_y)},
                {"z", PointMatcherSupport::toParam(pose_z)},
                {"qx", PointMatcherSupport::toParam(pose_qx)},
                {"qy", PointMatcherSupport::toParam(pose_qy)},
                {"qz", PointMatcherSupport::toParam(pose_qz)},
                {"qw", PointMatcherSupport::toParam(pose_qw)},
                {"distance", PointMatcherSupport::toParam(max_distance)},
                {"horizontal", PointMatcherSupport::toParam(max_horizontal)},
                {"vertical", PointMatcherSupport::toParam(max_vertical)},
                {"removeInside",PointMatcherSupport::toParam(0)}

            }
        );

        // ROS_INFO("3");
        matcher = fov->filter(scene);
        // matcher = scene;

        // if (first)
        // {
        //     start_map = scene;
        //     first = false;
        // }
        // matcher = start_map;
        // matcher = scene;

        // Old Map Data (PointCloud2) Converted to DataPoints
        // ROS_INFO("4");
        object = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(scan, false);
        // test = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(scan, false);
        og = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(scan, false);
        ROS_INFO("5");
        // inputFilter.apply(test);
        inputFilter.apply(object);

        //
        // ROS_INFO("6");
        // test2 = matcher;
        // inputFilter.apply(test2);
        //

        // ROS_INFO("7");
        // icp.transformations.apply(test, initialT);

        // ROS_INFO("8");
        // input = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(test2, "map", ros::Time::now());
        // ROS_INFO("4");
        // reference = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(test, "map", ros::Time::now());
        // test_1.publish(input);
        // test_2.publish(reference);


        // Find the transformation from running the ICP algorithm
        // ROS_INFO("4");
        ROS_INFO("Start ICP");
        ros::Time begin = ros::Time::now();
        finalT = icp(object, matcher, initialT);
        ros::Time after = ros::Time::now();
        ROS_INFO("End ICP : %lf", (after-begin).toSec());

        // Print and Apply Transformation Matrix
        // ROS_INFO("5");
        // std::cout << "Transformation Matrix = \n" << finalT << std::endl;

        transformed_object = og;
        // ROS_INFO("6");
        // icp.transformations.apply(transformed_object, initialT);
        icp.transformations.apply(transformed_object, finalT);

        // Eigen::Matrix<float,-1,-1> T(4,4);
        // T.setIdentity();
        // T.block(0, 3, 3, 1) = finalT.block(0, 3, 3, 1);
        // T.block(0, 0, 3, 3) = initialT.block(0, 0, 3, 3);
        // icp.transformations.apply(transformed_object, T);
        
        

        // ADD BUNDLE ADJUSTMENT HERE
        // PERFORM ON PREVIOUS n FRAMES

        // ROS_INFO("7");
        ///////////////////////////////////////////////////////////////////////////
        /////////////////////// GLOBAL OPTIMIZATION ///////////////////////////////
        ///////////////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////////////
        /////////////////////// WEIGHT FUNCTION ///////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////
        // downsample = PointMatcher<float>::get().DataPointsFilterRegistrar.create(
        //     "RandomSamplingDataPointsFilter",
        //     {
        //         {"prob", PointMatcherSupport::toParam(weight_prob)},
        //         {"randomSamplingMethod", PointMatcherSupport::toParam(sample_method)}

        //     }
        // );
        // scene.concatenate(downsample->filter(transformed_object));
        scene.concatenate(transformed_object);
        ///////////////////////////////////////////////////////////////////////////
        
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

    processing = false;

    ros::Time after2 = ros::Time::now();
    ROS_INFO("End Method : %lf", (after2-begin2).toSec());
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
    z = 0.07831-0.15;
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
        
        
        
        nh.getParam("/register",accept);
        // std::cout << accept;
        // icp_algorithm();
        if (accept)
        {   
            ros::spinOnce();
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