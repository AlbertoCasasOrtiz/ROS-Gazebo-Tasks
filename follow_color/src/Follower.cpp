#include "Follower.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>

static const cv::String OPENCV_WINDOW = "Image window";

Follower::Follower(): imageTransport(nh) {
    imageSubscriber = imageTransport.subscribe("/pioneer3at/camera1/image_raw", 1,
            &Follower::imageCallback, this);

    // Create a display window
    cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_AUTOSIZE);
}

Follower::~Follower() {
    // Close the display window
    cv::destroyWindow(OPENCV_WINDOW);
}
void Follower::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // convert the ROS image message to a CvImage
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Convert input image to HSV
    cv::Mat image = cv_ptr->image;
    cv::Mat hsvImage;
    cv:cvtColor(image, hsvImage, CV_BGR2HSV);

    // Threshold the HSV image, keep only the yellow pixels
	cv::Mat mask;
	cv::Scalar lower_yellow(20, 100, 100);
	cv::Scalar upper_yellow(30, 255, 255);
	cv::inRange(hsvImage, lower_yellow, upper_yellow, mask);

    // Update the GUI window
    cv::imshow(OPENCV_WINDOW, mask);
    cv::waitKey(3);
}

