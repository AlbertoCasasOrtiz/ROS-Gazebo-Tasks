//
// Created by alberto on 4/09/19.
//

#ifndef SRC_PILOT_H
#define SRC_PILOT_H

#include <ros/ros.h>
#include <queue>
#include <nav_msgs/Odometry.h>


class Pilot {
public:

    //Enumerates and structs
    /// Set of directions the robot can take.
    enum class Dir{UP, DOWN, LEFT, RIGHT};

    //Publishers and subscribers
    /// Publisher for movement of the robot,
    ros::Publisher cmdVelPublisher;
    /// Subscriber for odometry.
    ros::Subscriber odomSubscriber;

    //Callback functions
    /// Callback function for odometry.
    /// \param msg Message from odometer.
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);


    //Robot instructions.
    /// Go forward.
    void goForward();
    /// Turn left.
    void turnLeft();
    /// Turn right.
    void turnRight();
    /// Stop robot.
    void stop();

    //Class functions.
    /// Change heading of the robot.
    /// \param left If left, true. False otherwise.
    void updateHeading(bool left);

    //Helpers
    /// Get yaw angle of the robot in degrees.
    /// \param msg Message from odometry.
    /// \return Yaw angle in degrees.
    int getYaw(const nav_msgs::Odometry::ConstPtr& msg);

    //Flags
    /// Flag for store memory.
    int flag_init;

    //Memory of movement.
    /// Memory for last command executed.
    int currentCommand;
    /// Memory from last position and angle before start moving.
    float posX, posY;
    float turnZ;

    //Robot orientation and position.
    /// Direction where the robot is heading.
    Dir heading;

    //IO functions
    /// Read commands from a file.
    void readCommands();

    //Constructor and destructor
    /// Constructor of pilot.
    Pilot(int argc, char **argv);
    
    std::queue<std::string> commands;

};


#endif //SRC_PILOT_H
