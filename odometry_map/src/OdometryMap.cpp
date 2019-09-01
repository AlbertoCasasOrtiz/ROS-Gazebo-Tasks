#include "OdometryMap.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include <ros/timer.h>
#include <math.h>
#include <ros/ros.h>
#include <ros/timer.h>
#include <std_msgs/Float64.h>
#include <boost/bind.hpp>

static const std::string OPENCV_WINDOW = "Image window";

OdometryMap::OdometryMap(int argc, char **argv) {
	Map map;
	OdometryMap::heading = Map::Dir::UP;
	OdometryMap::flag_odom = 0;

    ros::init(argc, argv, "OdometryMap");

    ros::NodeHandle nh;

    OdometryMap::cmdVelPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	OdometryMap::odomSubscriber = nh.subscribe("/odom", 1, &OdometryMap::odomCallback, this);

	OdometryMap::timer = nh.createTimer(ros::Duration(5.0), &OdometryMap::timerCallback, this);

	ros::spin();
}

void OdometryMap::timerCallback(const ros::TimerEvent& e)
{
	// Stop moving.
	ROS_INFO("Stopping...");
	geometry_msgs::Twist cmd;
	cmd.linear.x = 0.0;
	cmd.angular.z = 0.0;
	OdometryMap::cmdVelPublisher.publish(cmd);

	// Send flag to start moving again.
	OdometryMap::flag_odom = 1;
}


void OdometryMap::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	if(flag_odom == 1){
		// Get odom info.
		ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);

		// Stop getting info.
		OdometryMap::flag_odom = 0;

		// Start moving to next point.
		ROS_INFO("Running...");
		geometry_msgs::Twist cmd;
		cmd.linear.x = 0.2;
		cmd.angular.z = 0.0;
		OdometryMap::cmdVelPublisher.publish(cmd);
	}
}

OdometryMap::~OdometryMap() {
	// Close the display window
	cv::destroyWindow(OPENCV_WINDOW);
}

