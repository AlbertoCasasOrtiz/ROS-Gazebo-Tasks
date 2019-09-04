
#ifndef ODOMETRY_MAP_SRC_ODOMETRYMAP_H
#define ODOMETRY_MAP_SRC_ODOMETRYMAP_H_
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <iomanip>
#include "Map.h"

class OdometryMap {
private:
    //Publishers and subscribers.
    /// Publisher for movement of the robot.
	ros::Publisher cmdVelPublisher;
    /// Subscriber for odometry.
	ros::Subscriber odomSubscriber;
	/// Subscriber for IR sensor.
	ros::Subscriber rangeSubscriber;

	//Callback functions.
	/// Callback function for odometry.
	/// \param msg Message from odometer.
	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
	/// Callback function for IR sensor.
	/// \param msg Message from IR sensor.
	void rangeCallback(const sensor_msgs::Range::ConstPtr& msg);

	//Robot instructions.
	/// Go forward.
	void goForward();
	/// Turn left.
	void turnLeft();
	/// Turn right.
	void turnRight();
	/// Stop robot.
	void stopRobot();

	//Class functions.
	/// Change heading of the robot.
	/// \param left If left, true. False otherwise.
	void changeHeading(bool left);
	/// Update current point when forward.
	void updateCurrentPoint();
	/// Save point in map.
	/// \param msg Message from odometry.
	void savePoint(const nav_msgs::Odometry::ConstPtr& msg);
	/// Chose direction to turn.
	void chooseDirection();

	//Helpers
    /// Get yaw angle of the robot in degrees.
    /// \param msg Message from odometry.
    /// \return Yaw angle in degrees.
    int getYaw(const nav_msgs::Odometry::ConstPtr& msg);

    //Flags
	/// Flag for going forward.
	int flag_forward;
	/// Flag for turn.
	int flag_turn;
	/// Flag for store memory.
	int flag_store_memory;
	/// Flag for choose a direction.
	int flag_chosen_direction;

	//Memory of movement.
	/// Memory from last position and angle before start moving.
	float posX, posY;
	float turnZ;

	//Robot orientation and position.
    /// Direction where the robot is heading.
    Map::Dir heading;
    /// Current position of the robot.
	Map::point current;
public:
    /// Map that the robot is building.
	Map map;

    //Constructor and destructor
    /// Constructor of OdometryMao
	OdometryMap(int argc, char **argv);
	/// Destructor of OdometryMap
	virtual ~OdometryMap();
};

#endif /* ODOMETRY_MAP_SRC_ODOMETRYMAP_H_ */
