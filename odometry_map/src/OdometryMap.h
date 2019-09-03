
#ifndef ODOMETRY_MAP_SRC_ODOMETRYMAP_H
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <iomanip>
	#include "Map.h"
#define ODOMETRY_MAP_SRC_ODOMETRYMAP_H_

class OdometryMap {
private:
	ros::Publisher cmdVelPublisher;
	ros::Publisher odomPublisher;
	ros::Subscriber odomSubscriber;
	ros::Subscriber rangeSubscriber;

	ros::Timer timerForward;
	ros::Timer timerTurn;

	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void rangeCallback(const sensor_msgs::Range::ConstPtr& msg);

	void moveRobotForward();
	void moveRobotLeft();
	void moveRobotRight();
	void stopRobot();
	void changeHeading(bool left);
	void updateCurrentPoint();
	int getYaw(const nav_msgs::Odometry::ConstPtr& msg);
	void savePoint(const nav_msgs::Odometry::ConstPtr& msg);
	void chooseDirection();

	Map::Dir heading;
	void exploreMap();

	float error_turn;

	int flag_forward;
	int flag_turn;
	int flag_store_memory;
	int flag_chosen_direction;

	float posX, posY;
	float turnZ;

	Map::point current;
public:
	Map map;
	OdometryMap(int argc, char **argv);
	virtual ~OdometryMap();
};

#endif /* ODOMETRY_MAP_SRC_ODOMETRYMAP_H_ */
