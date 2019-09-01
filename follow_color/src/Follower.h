#include <ros/ros.h>
#include <image_transport/image_transport.h>

class GoToGoal {
private:
    ros::NodeHandle nh;
    image_transport::ImageTransport imageTransportYellow;
    image_transport::Subscriber imageSubscriberYellow;
	ros::Publisher cmdVelPublisher;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

public:
    GoToGoal();
    virtual ~GoToGoal();
};
