#ifndef EXAMPLE_ROS_CLASS_H_
#define EXAMPLE_ROS_CLASS_H_

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <math.h>


class UAVClass
{
public:
    UAVClass(ros::NodeHandle* nodehandle); 
private:
    nav_msgs::Odometry odomdata1;
	nav_msgs::Odometry odomdata2;
	nav_msgs::Odometry odomdata3;
	ros::Subscriber gps_subscriber1;
	ros::Subscriber gps_subscriber2;
	ros::Subscriber gps_subscriber3;
    //float_t[4] goal1;
	//std_msgs::Float64MultiArray goal1, goal2, goal3;
    //goal1.data.resize(4);
    //goal2.data.resize(4);
    //goal3.data.resize(4);
    boost::array<double,4> goal1 = {0.0, 0.0, 0.0, 0.0};
	boost::array<double,4> goal2 = {0.0, 0.0, 0.0, 0.0};
	boost::array<double,4> goal3 = {0.0, 0.0, 0.0, 0.0};
    
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    
    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services


    
    void subscriberCallback1(const nav_msgs::Odometry& message_holder); //prototype for callback of example subscriber
    void subscriberCallback2(const nav_msgs::Odometry& message_holder);
    void subscriberCallback3(const nav_msgs::Odometry& message_holder);
    
    void servicestarter();
    void calculator();
}; 

#endif

