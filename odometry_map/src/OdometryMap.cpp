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
	OdometryMap::flag_timer_forward = 0;
	OdometryMap::flag_range = 1;
	OdometryMap::flag_timer_turn = 1;

    ros::init(argc, argv, "OdometryMap");

    ros::NodeHandle nh;

    OdometryMap::cmdVelPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	OdometryMap::odomSubscriber = nh.subscribe("/odom", 1, &OdometryMap::odomCallback, this);
	OdometryMap::rangeSubscriber = nh.subscribe("/sensor/ir_front", 1, &OdometryMap::rangeCallback, this);
	OdometryMap::timerForward = nh.createTimer(ros::Duration(5.0), &OdometryMap::timerForwardCallback, this);
	OdometryMap::timerTurn = nh.createTimer(ros::Duration(10.0), &OdometryMap::timerForwardCallback, this);

	ros::spin();
}

void OdometryMap::timerForwardCallback(const ros::TimerEvent& e){
	if(OdometryMap::flag_range == 1){
		// Stop moving.
		ROS_INFO("Stopping...");
		geometry_msgs::Twist cmd;
		cmd.linear.x = 0.0;
		cmd.angular.z = 0.0;

		OdometryMap::cmdVelPublisher.publish(cmd);

		// Send flag to start moving again.
		OdometryMap::flag_timer_forward = 1;
	}
}


void OdometryMap::timerTurnCallback(const ros::TimerEvent& e){
	if(OdometryMap::flag_range == 0){
		// Stop moving.
		ROS_INFO("Turning...");
		geometry_msgs::Twist cmd;
		cmd.linear.x = 0.0;
		cmd.angular.z = 0.3;
		OdometryMap::cmdVelPublisher.publish(cmd);

		// Send flag to start moving again.
		OdometryMap::flag_timer_turn = 1;
	}
}


void OdometryMap::rangeCallback(const sensor_msgs::Range::ConstPtr& msg){
	if(msg->range < 0.5){
		// Nobody moves.
		ROS_INFO("Stopping...");
		geometry_msgs::Twist cmd;
		cmd.linear.x = 0.0;
		cmd.angular.z = 0.2;
		OdometryMap::cmdVelPublisher.publish(cmd);
		// Advise turn.
		OdometryMap::flag_range = 0;
		if(OdometryMap::flag_timer_turn == 1){
			// Turn ended, everybody moves.
			OdometryMap::flag_range = 1;
		}
	}
}


void OdometryMap::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	ROS_INFO("Range flag: [%i]", OdometryMap::flag_range);
	ROS_INFO("Odon flag: [%i]", OdometryMap::flag_timer_forward);
	// If range sensor allows...
	if(OdometryMap::flag_range == 1){
		// If timer allows, store data and run.
		if(flag_timer_forward == 1){
			// Get odom info.
			// Stop getting info.
			OdometryMap::flag_timer_forward = 0;

			// Start moving to next point.
			ROS_INFO("Running...");
			geometry_msgs::Twist cmd;
			cmd.linear.x = 0.2;
			cmd.angular.z = 0.0;
			OdometryMap::cmdVelPublisher.publish(cmd);
		}
	}
	if(OdometryMap::flag_timer_turn == 1){
		float gyro = msg->pose.pose.orientation.z;

		// Start moving to next point.
		ROS_INFO("Running...");
		while((msg->pose.pose.orientation.z - gyro) < M_PI/2){
			ROS_INFO("Position-> z: [%f], g: [%f]", msg->pose.pose.orientation.z,gyro);

			geometry_msgs::Twist cmd;
			cmd.linear.x = 0.0;
			cmd.angular.z = 0.2;
			OdometryMap::cmdVelPublisher.publish(cmd);
		}
		OdometryMap::flag_timer_turn == 0;
	}
}

OdometryMap::~OdometryMap() {
	// Close the display window
	cv::destroyWindow(OPENCV_WINDOW);
}

