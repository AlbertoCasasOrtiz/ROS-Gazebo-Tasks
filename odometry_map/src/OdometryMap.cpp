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
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

static const std::string OPENCV_WINDOW = "Image window";

OdometryMap::OdometryMap(int argc, char **argv) {
	Map map;

	OdometryMap::heading = Map::Dir::UP;
	OdometryMap::flag_forward = 1;
	OdometryMap::flag_store_memory = 1;
	OdometryMap::flag_turn = 0;

	OdometryMap::currentX, OdometryMap::currentY = 0;

	OdometryMap::posX, OdometryMap::posY, OdometryMap::turnZ= 0;


    ros::init(argc, argv, "OdometryMap");

    ros::NodeHandle nh;

    OdometryMap::cmdVelPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	OdometryMap::odomSubscriber = nh.subscribe("/odom", 1, &OdometryMap::odomCallback, this);
	OdometryMap::rangeSubscriber = nh.subscribe("/sensor/ir_front", 1, &OdometryMap::rangeCallback, this);
	//OdometryMap::timerForward = nh.createTimer(ros::Duration(5.0), &OdometryMap::timerForwardCallback, this);
	//OdometryMap::timerTurn = nh.createTimer(ros::Duration(10.0), &OdometryMap::timerForwardCallback, this);

	ros::spin();
}


void OdometryMap::rangeCallback(const sensor_msgs::Range::ConstPtr& msg){

	if(msg->range < 0.5 && OdometryMap::flag_turn == 0){
		OdometryMap::flag_forward = 0;
		OdometryMap::flag_turn = 1;
		flag_store_memory = 1;
	}
}


void OdometryMap::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	// If allowed to go forward....
	if(OdometryMap::flag_forward == 1){
		// Store memory once.
		if(OdometryMap::flag_store_memory == 1){
			ROS_INFO("Forward...");
			posX = msg->pose.pose.position.x;
			posY = msg->pose.pose.position.y;
			OdometryMap::flag_store_memory = 0;
		}
		// Move robot forward.
		OdometryMap::moveRobotForward();
		// If advanced one unit...
		if(abs(posX - msg->pose.pose.position.x) >1.0 || abs(posY - msg->pose.pose.position.y) > 1.0){
			// Stop robot and update current point.
			OdometryMap::stopRobot();
			OdometryMap::updateCurrentPoint();

			// Show stored point.
			ROS_INFO("Stored point ([%f], [%f])", OdometryMap::currentX, OdometryMap::currentY);

			// Add current point.
			OdometryMap::map.addPoint(OdometryMap::currentX, OdometryMap::currentY);

			// Notify to update current points.
			OdometryMap::flag_store_memory = 1;

			// Print map to file.
			OdometryMap::map.printMap();
		}
	}

	// If allowed to turn...
	if(OdometryMap::flag_turn == 1){
		// Store memory once.
		if(OdometryMap::flag_store_memory == 1){
			ROS_INFO("Turning...");
			OdometryMap::flag_store_memory = 0;
			// TODO depends of left or right.
			OdometryMap::changeHeading(true);
			ROS_INFO("Heading: [%s]", Map::DirToString(OdometryMap::heading).c_str());
		}

		// TODO Turn.
		OdometryMap::moveRobotLeft();
	
		// Calculate heading angle in degrees.
		geometry_msgs::Pose2D pose2d;
		pose2d.x = msg->pose.pose.position.x;
		pose2d.y = msg->pose.pose.position.y;
		tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		yaw = ((yaw + M_PI)*180/M_PI);

		ROS_INFO("Gyro [%i]: [%i]", (int)yaw, (int)(turnZ));

		// If target angle reached...
		if((int)yaw == (int)OdometryMap::turnZ){
			// Stop robot.
			OdometryMap::stopRobot();
			// Notify stopped turning.
			OdometryMap::flag_turn = 0;
			// Notify go forward.
			OdometryMap::flag_forward = 1;
			// Notify store memory.
			OdometryMap::flag_store_memory = 1;
		}
	}
}

void OdometryMap::moveRobotForward(){
	geometry_msgs::Twist cmd;
	cmd.linear.x = 0.2;
	cmd.angular.z = 0.0;
	OdometryMap::cmdVelPublisher.publish(cmd);
}

void OdometryMap::stopRobot(){
	ROS_INFO("Stoping...");
	geometry_msgs::Twist cmd;
	cmd.linear.x = 0.0;
	cmd.angular.z = 0.0;
	OdometryMap::cmdVelPublisher.publish(cmd);
}

void OdometryMap::moveRobotLeft(){
	//ROS_INFO("Turning Left...");
	geometry_msgs::Twist cmd;
	cmd.linear.x = 0.0;
	cmd.angular.z = 0.2;
	OdometryMap::cmdVelPublisher.publish(cmd);
}

void OdometryMap::moveRobotRight(){
	//ROS_INFO("Turning Right...");
	geometry_msgs::Twist cmd;
	cmd.linear.x = 0.0;
	cmd.angular.z = -0.2;
	OdometryMap::cmdVelPublisher.publish(cmd);
}

void OdometryMap::changeHeading(bool left){
	if(left){
		switch(OdometryMap::heading){
			case Map::Dir::UP:
				OdometryMap::turnZ = 270;
				OdometryMap::heading = Map::Dir::LEFT;
				break;
			case Map::Dir::LEFT:
				OdometryMap::turnZ = 0;
				OdometryMap::heading = Map::Dir::DOWN;
				break;
			case Map::Dir::DOWN:
				OdometryMap::turnZ = 90;
				OdometryMap::heading = Map::Dir::RIGHT;
				break;
			case Map::Dir::RIGHT:
				OdometryMap::turnZ = 180;
				OdometryMap::heading = Map::Dir::UP;
		}
	} else {
		switch(OdometryMap::heading){
			case Map::Dir::UP:
				OdometryMap::turnZ = 90;
				OdometryMap::heading = Map::Dir::RIGHT;
				break;
			case Map::Dir::LEFT:
				OdometryMap::turnZ = 180;
				OdometryMap::heading = Map::Dir::UP;
				break;
			case Map::Dir::DOWN:
				OdometryMap::turnZ = 270;
				OdometryMap::heading = Map::Dir::LEFT;
				break;
			case Map::Dir::RIGHT:
				OdometryMap::turnZ = 0;
				OdometryMap::heading = Map::Dir::DOWN;
		}

	}
}

void OdometryMap::updateCurrentPoint(){
	switch(OdometryMap::heading){
		case Map::Dir::UP:
			OdometryMap::currentX++;
			break;
		case Map::Dir::LEFT:
			OdometryMap::currentY--;
			break;
		case Map::Dir::DOWN:
			OdometryMap::currentX--;
			break;
		case Map::Dir::RIGHT:
			OdometryMap::currentY++;
	}
}

OdometryMap::~OdometryMap() {
	// Close the display window
	cv::destroyWindow(OPENCV_WINDOW);
}



