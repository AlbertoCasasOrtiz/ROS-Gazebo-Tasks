#include "Potential_Fields.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include <math.h>

static const std::string OPENCV_WINDOW = "Image window";

PotentialFields::PotentialFields() : imageTransportYellow(nh) {
	imageSubscriberYellow = imageTransportYellow.subscribe("/pioneer3at/camera1/image_raw", 1,
			&PotentialFields::imageCallback, this);
	cmdVelPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

	// Create a display window
	cv::namedWindow(OPENCV_WINDOW);
}

PotentialFields::~PotentialFields() {
	// Close the display window
	cv::destroyWindow(OPENCV_WINDOW);
}

void PotentialFields::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
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
	cv::Scalar lower_yellow(20, 100, 100);
	cv::Scalar upper_yellow(30, 255, 255);

	// Threshold the HSV image, keep only the red pixels
	cv::Scalar lower_red(0, 100, 100);
	cv::Scalar upper_red(10, 255, 255);

	// Threshold the HSV image, keep only the green pixels
	cv::Scalar lower_green(40, 100, 100);
	cv::Scalar upper_green(70, 255, 255);

	// Calculate potential fields
	PotentialFields::vector *v_yellow = PotentialFields::avoidObstacle(hsvImage, lower_yellow, upper_yellow);
	PotentialFields::vector *v_green = PotentialFields::avoidObstacle(hsvImage, lower_green, upper_green);
	PotentialFields::vector *v_red = PotentialFields::goToGoal(hsvImage, lower_red, upper_red);

	// Combine potential fields
	std::vector<PotentialFields::vector*>* vectors = new std::vector<PotentialFields::vector*>();

	vectors->push_back(v_yellow);
	vectors->push_back(v_green);
	vectors->push_back(v_red);

	PotentialFields::vector *result = this->sumVectors(vectors);

	// If no objects, explore
	this->explore(v_red, v_green, v_yellow, result);

	// Publish to ros
	geometry_msgs::Twist cmd;
	cmd.linear.x = result->magnitude;
	cmd.angular.z = result->angle;
	ROS_INFO_STREAM("Angle: " << cmd.angular.z);
	ROS_INFO_STREAM("Magnitude: " << cmd.linear.x);
	cmdVelPublisher.publish(cmd);

	// Clean pointers
	delete v_yellow, v_red, v_green, result, vectors;

	// Update the GUI window
	cv::imshow(OPENCV_WINDOW, image);
	cv::waitKey(3);
}

PotentialFields::vector* PotentialFields::avoidObstacle(cv::Mat HSVImage, cv::Scalar lowerHSV, cv::Scalar upperHSV){
	PotentialFields::vector *vector = new PotentialFields::vector();

	cv::Mat mask;
	cv::inRange(HSVImage, lowerHSV, upperHSV, mask);

	cv::Moments moments = cv::moments(mask);

	vector->magnitude = 0.0f;
	vector->angle = 0.0f;

	std::vector<cv::Point> black_pixels;
	cv::findNonZero(mask, black_pixels);
	float area = black_pixels.size();

	if(area > 0){
		int cx = int(moments.m10 / moments.m00);

		int err = cx -mask.cols/2;

		if(area > 0){
			vector->magnitude = (PotentialFields::resolution - area)/PotentialFields::resolution * 0.1;
			if(err > 0)
				vector->angle = (PotentialFields::camera_width - err)/PotentialFields::camera_width * (PotentialFields::resolution - area)/PotentialFields::resolution * 0.3;
			else
				vector->angle = -(PotentialFields::camera_width - err)/PotentialFields::camera_width * (PotentialFields::resolution - area)/PotentialFields::resolution * 0.3;
		}
	}

	return vector;
}


PotentialFields::vector* PotentialFields::goToGoal(cv::Mat HSVImage, cv::Scalar lowerHSV, cv::Scalar upperHSV){
	PotentialFields::vector* vector = new PotentialFields::vector();

	cv::Mat mask;
	cv::inRange(HSVImage, lowerHSV, upperHSV, mask);

	cv::Moments moments = cv::moments(mask);

	vector->magnitude = 0.0f;
	vector->angle = 0.0f;

	std::vector<cv::Point> black_pixels;
	cv::findNonZero(mask, black_pixels);
	float area = black_pixels.size();
	ROS_INFO_STREAM("Area: " << area);

	if(area > 0){
		int cx = int(moments.m10 / moments.m00);

		int err = cx -mask.cols/2;

		if(area <= PotentialFields::resolution){
			vector->magnitude = (PotentialFields::resolution - area)/PotentialFields::resolution * 0.1;
			if(err > 0 && (PotentialFields::camera_width - err)/PotentialFields::camera_width > 0.1)
				vector->angle = -(PotentialFields::camera_width - err)/PotentialFields::camera_width * (PotentialFields::resolution - area)/PotentialFields::resolution * 0.2;
			else if ( err < 0 && (PotentialFields::camera_width - err)/PotentialFields::camera_width > 0.1)
				vector->angle = (PotentialFields::camera_width - err)/PotentialFields::camera_width * (PotentialFields::resolution - area)/PotentialFields::resolution * 0.2;
		}
	}

	return vector;
}

PotentialFields::vector* PotentialFields::sumVectors(std::vector<PotentialFields::vector*> *array){
	vector* result = new PotentialFields::vector();

	result->magnitude = 0.0f;
	result->angle = 0.0f;

	for(int i = 0; i < array->size(); i++){
		result->magnitude += array->at(i)->magnitude;
		result->angle += array->at(i)->angle;
	}

	return result;
}

void PotentialFields::explore(PotentialFields::vector* red, PotentialFields::vector* green, PotentialFields::vector* yellow, PotentialFields::vector* result){
	if(red->angle == 0 && red->magnitude == 0){
		result->angle = M_PI/4;
		result->magnitude = 0;
	}
}

