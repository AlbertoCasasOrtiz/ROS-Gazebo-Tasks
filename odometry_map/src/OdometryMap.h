
#ifndef ODOMETRY_MAP_SRC_ODOMETRYMAP_H
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <iostream>
#include <opencv2/opencv.hpp>

	#include "Map.h"
#define ODOMETRY_MAP_SRC_ODOMETRYMAP_H_

class OdometryMap {
private:
	ros::Publisher cmdVelPublisher;
	ros::Subscriber odomSubscriber;
	ros::Subscriber rangeSubscriber;
	ros::Timer timerForward;
	ros::Timer timerTurn;
	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void rangeCallback(const sensor_msgs::Range::ConstPtr& msg);
	void timerForwardCallback(const ros::TimerEvent& e);
	void timerTurnCallback(const ros::TimerEvent& e);

	Map::Dir heading;
	void exploreMap();

	int flag_timer_forward;
	int flag_timer_turn;
	int flag_range;

public:
	OdometryMap(int argc, char **argv);
	virtual ~OdometryMap();
};

#endif /* ODOMETRY_MAP_SRC_ODOMETRYMAP_H_ */
