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
    void servicestarter();
private:
    nav_msgs::Odometry odomdata1;
	nav_msgs::Odometry odomdata2;
	nav_msgs::Odometry odomdata3;
	ros::Subscriber gps_subscriber1;
	ros::Subscriber gps_subscriber2;
	ros::Subscriber gps_subscriber3;
    boost::array<double,4> goal1 = {0.0, 0.0, 0.0, 0.0};
	boost::array<double,4> goal2 = {0.0, 0.0, 0.0, 0.0};
	boost::array<double,4> goal3 = {0.0, 0.0, 0.0, 0.0};
    
    
    ros::NodeHandle nh_; 
    
    
    void initializeSubscribers(); 


    
    void subscriberCallback1(const nav_msgs::Odometry& message_holder); 
    void subscriberCallback2(const nav_msgs::Odometry& message_holder);
    void subscriberCallback3(const nav_msgs::Odometry& message_holder);
    
    
    void calculator();
}; 



UAVClass::UAVClass(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ 
    ROS_INFO("in class constructor of UAVClass");
    initializeSubscribers(); 

    //servicestarter();
    
}


void UAVClass::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    gps_subscriber1 = nh_.subscribe("/uav1/odometry/odom_gps", 100, &UAVClass::subscriberCallback1,this);
    gps_subscriber2 = nh_.subscribe("/uav2/odometry/odom_gps", 100, &UAVClass::subscriberCallback2,this);
    gps_subscriber3 = nh_.subscribe("/uav3/odometry/odom_gps", 100, &UAVClass::subscriberCallback3,this);  
    // add more subscribers here, as needed
}




void UAVClass::subscriberCallback1(const nav_msgs::Odometry& message_holder) {
    

    odomdata1 = message_holder; 
}



void UAVClass::subscriberCallback2(const nav_msgs::Odometry& message_holder) {
   

    odomdata2 = message_holder; 
    
}



void UAVClass::subscriberCallback3(const nav_msgs::Odometry& message_holder) {


    odomdata3 = message_holder; 
    
    
}



void UAVClass::calculator() {

    float centroidx, centroidy, centroidz;

    centroidx = (odomdata1.pose.pose.position.x + odomdata2.pose.pose.position.x + odomdata3.pose.pose.position.x)/3;
    centroidy = (odomdata1.pose.pose.position.y + odomdata2.pose.pose.position.y + odomdata3.pose.pose.position.y)/3;
    centroidz = (odomdata1.pose.pose.position.z + odomdata2.pose.pose.position.z + odomdata3.pose.pose.position.z)/3;
    goal2[0] = centroidx;
    goal2[1] = centroidy;
    goal2[2] = centroidz;

    goal1[0] = centroidx + 5.0;
    goal1[1] = centroidy;
    goal1[2] = centroidz;

    goal3[0] = centroidx - 5.0;
    goal3[1] = centroidy;
    goal3[2] = centroidz;

}

void UAVClass::servicestarter() {

    
    calculator();
    ros::ServiceClient client1 = nh_.serviceClient<mrs_msgs::Vec4>("/uav1/control_manager/goto");
    ros::ServiceClient client2 = nh_.serviceClient<mrs_msgs::Vec4>("/uav2/control_manager/goto");
    ros::ServiceClient client3 = nh_.serviceClient<mrs_msgs::Vec4>("/uav3/control_manager/goto");
    
    mrs_msgs::Vec4 srv1, srv2, srv3;
     
    srv1.request.goal = goal1;
    srv2.request.goal = goal2;
    srv3.request.goal = goal3;

    client1.call(srv1);
    client2.call(srv2);
    client3.call(srv3);

    
    
    
}




int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "FormationController"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type UAVClass");
    UAVClass UAVClass(&nh);  //instantiate an UAVClass object and pass in pointer to nodehandle for constructor to use

    ros::Rate r(10);
    while (ros::ok()){
    ros::service::waitForService("/uav1/control_manager/goto",10);
    ros::service::waitForService("/uav2/control_manager/goto",10);
    ros::service::waitForService("/uav3/control_manager/goto",10);    
    UAVClass.servicestarter();
    
    
    ros::spinOnce();
    r.sleep();
    }

    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
} 

