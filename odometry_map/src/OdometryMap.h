
#ifndef ODOMETRY_MAP_SRC_ODOMETRYMAP_H
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <opencv2/opencv.hpp>

	#include "Map.h"
#define ODOMETRY_MAP_SRC_ODOMETRYMAP_H_

class OdometryMap {
private:
	ros::Publisher cmdVelPublisher;
	ros::Subscriber odomSubscriber;
	ros::Timer timer;
	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void timerCallback(const ros::TimerEvent& e);

	Map::Dir heading;
	void exploreMap();

	int flag_odom;

public:
	OdometryMap(int argc, char **argv);
	virtual ~OdometryMap();
};

#endif /* ODOMETRY_MAP_SRC_ODOMETRYMAP_H_ */
