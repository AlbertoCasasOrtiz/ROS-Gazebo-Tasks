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
	OdometryMap::flag_chosen_direction = 0;
	OdometryMap::current.x = 0;
	OdometryMap::current.y = 0;


	// Show stored point.
	ROS_INFO("Stored point ([%i], [%i])", OdometryMap::current.x, OdometryMap::current.y);
	// Add current point.
	OdometryMap::map.addPoint(current);

	OdometryMap::posX, OdometryMap::posY, OdometryMap::turnZ= 0;

    ros::init(argc, argv, "OdometryMap");

    ros::NodeHandle nh;

    OdometryMap::cmdVelPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	OdometryMap::odomSubscriber = nh.subscribe("/odom", 1, &OdometryMap::odomCallback, this);
	OdometryMap::rangeSubscriber = nh.subscribe("/sensor/ir_front", 1, &OdometryMap::rangeCallback, this);

	ros::spin();
}


void OdometryMap::rangeCallback(const sensor_msgs::Range::ConstPtr& msg){

	if(msg->range < 0.6 && OdometryMap::flag_turn == 0){
		OdometryMap::flag_forward = 0;
		OdometryMap::flag_turn = 1;
		OdometryMap::flag_store_memory = 1;
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
		//ROS_INFO("Position ([%f], [%f])", msg->pose.pose.position.x, msg->pose.pose.position.y);
		// If advanced one unit...
		if(fabs(posX - msg->pose.pose.position.x) > 2.0 || fabs(posY - msg->pose.pose.position.y) > 2.0){
			OdometryMap::savePoint(msg);
		}
	}
	/*
	// If stopped by range sensor, save current point if necessary.
	if(OdometryMap::flag_turn == 1){
		//ROS_INFO("Difference x: [%f] y: [%f]", fabs(fabs(posX) - fabs(msg->pose.pose.position.x)), fabs(fabs(posY) - fabs(msg->pose.pose.position.y)));
		if(fabs(fabs(posX) - fabs(msg->pose.pose.position.x)) > 1.5 || fabs(fabs(posY) - fabs(msg->pose.pose.position.y)) > 1.5){
			OdometryMap::savePoint(msg);
		}
	}
*/

	// If allowed to turn...
	if(OdometryMap::flag_turn == 1){
		// Store memory once.
		if(OdometryMap::flag_store_memory == 1){
			ROS_INFO("Turning...");
			OdometryMap::flag_store_memory = 0;
			OdometryMap::flag_chosen_direction = 0;
		}

		// Choose direction.
		OdometryMap::chooseDirection();

		// Calculate heading angle in degrees.
		int yaw = OdometryMap::getYaw(msg);

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
			// Before turn, save position.
			posX = msg->pose.pose.position.x;
			posY = msg->pose.pose.position.y;
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
	ROS_INFO("Stopping...");
	geometry_msgs::Twist cmd;
	cmd.linear.x = 0.0;
	cmd.angular.z = 0.0;
	OdometryMap::cmdVelPublisher.publish(cmd);
}

void OdometryMap::moveRobotLeft(){
	geometry_msgs::Twist cmd;
	cmd.linear.x = 0.0;
	cmd.angular.z = 0.3;
	OdometryMap::cmdVelPublisher.publish(cmd);
}

void OdometryMap::moveRobotRight(){
	geometry_msgs::Twist cmd;
	cmd.linear.x = 0.0;
	cmd.angular.z = -0.3;
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
				break;
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
				break;
		}

	}
}

void OdometryMap::updateCurrentPoint(){
	switch(OdometryMap::heading){
		case Map::Dir::UP:
			OdometryMap::current.x++;
			break;
		case Map::Dir::LEFT:
			OdometryMap::current.y--;
			break;
		case Map::Dir::DOWN:
			OdometryMap::current.y--;
			break;
		case Map::Dir::RIGHT:
			OdometryMap::current.x++;
			break;
	}
}

int OdometryMap::getYaw(const nav_msgs::Odometry::ConstPtr& msg){
	geometry_msgs::Pose2D pose2d;
	pose2d.x = msg->pose.pose.position.x;
	pose2d.y = msg->pose.pose.position.y;
	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	return (int)((yaw + M_PI)*180/M_PI);
}

void OdometryMap::chooseDirection(){

	// If already visited left, else right.
	if(OdometryMap::map.numAppearances(OdometryMap::current) == 1 && OdometryMap::flag_chosen_direction == 0){
		OdometryMap::moveRobotLeft();
		ROS_INFO("CHOSEN LEFT");
		OdometryMap::changeHeading(true);
		OdometryMap::flag_chosen_direction = 1;
		ROS_INFO("Heading: [%s]", Map::DirToString(OdometryMap::heading).c_str());
	}else if(OdometryMap::map.numAppearances(OdometryMap::current) > 1 && OdometryMap::flag_chosen_direction == 0){
		OdometryMap::moveRobotRight();
		ROS_INFO("CHOSEN RIGHT");
		OdometryMap::changeHeading(false);
		OdometryMap::flag_chosen_direction = 1;
		ROS_INFO("Heading: [%s]", Map::DirToString(OdometryMap::heading).c_str());
	}

}

void OdometryMap::savePoint(const nav_msgs::Odometry::ConstPtr& msg){

	// Stop robot and update current point.
	OdometryMap::stopRobot();
	OdometryMap::updateCurrentPoint();

	// Show stored point.
	ROS_INFO("Stored point ([%i], [%i])", OdometryMap::current.x, OdometryMap::current.y);
	// Add current point.
	OdometryMap::map.addPoint(OdometryMap::current);


	// Print map to file.
	OdometryMap::map.printMap();
	posX = msg->pose.pose.position.x;
	posY = msg->pose.pose.position.y;
}

OdometryMap::~OdometryMap() {
	// Close the display window
	cv::destroyWindow(OPENCV_WINDOW);
}


