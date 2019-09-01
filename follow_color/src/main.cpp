#include "Follower.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "follower");
    GoToGoal follower;
    ros::spin();
    return 0;
}
