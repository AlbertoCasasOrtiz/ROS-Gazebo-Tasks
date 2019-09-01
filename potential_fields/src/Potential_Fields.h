#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/opencv.hpp>

class PotentialFields {
private:
    ros::NodeHandle nh;
    image_transport::ImageTransport imageTransportYellow;
    image_transport::Subscriber imageSubscriberYellow;
	ros::Publisher cmdVelPublisher;
	float camera_width = 1200;
	float camera_height = 600;
	float resolution = camera_width*camera_height;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    struct vector{
    	float magnitude;
    	float angle;
    };

public:
    PotentialFields();
    virtual ~PotentialFields();
    vector* avoidObstacle(cv::Mat HSVImage, cv::Scalar lowerHSV, cv::Scalar upperHSV);
    vector* goToGoal(cv::Mat HSVImage, cv::Scalar lowerHSV, cv::Scalar upperHSV);
    vector* sumVectors(std::vector<vector*> *array);
    void explore(vector* red, vector* green, vector* yellow, vector* result);
};
