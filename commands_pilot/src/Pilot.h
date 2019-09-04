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
    enum class Commands {FORWARD, LEFT, RIGHT};
    enum class Dir{UP, DOWN, LEFT, RIGHT};

    Pilot(int argc, char **argv);

    std::queue<std::string> commands;

    ros::Publisher cmdVelPublisher;
    ros::Subscriber odomSubscriber;
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    int getYaw(const nav_msgs::Odometry::ConstPtr& msg);
    void updateHeading(bool left);

    void goForward();
    void turnLeft();
    void turnRight();
    void stop();

    void readCommands();

    float posX, posY;
    float turnZ;
    int currentCommand;

    int flag_init;
    Dir heading;
};


#endif //SRC_PILOT_H
