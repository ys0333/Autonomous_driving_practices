#ifndef LANEDETECTORNODE_H
#define LANEDETECTORNODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <signal.h>
#include <memory>
#include "lane_detector/InToOutLaneDetector.h"
#include "lane_detector/ConditionalCompile.h"
// #include "waypoint_maker/State.h"

class LaneDetectorNode
{
public:
	int cur_state_ = 0;
	LaneDetectorNode();

	void left_imageCallback(const sensor_msgs::ImageConstPtr& image);
	void right_imageCallback(const sensor_msgs::ImageConstPtr& image);
	// void GPSCallback(const waypoint_maker::State::ConstPtr &lane_num);
	void merge();
	cv::Mat rect_left, rect_right;
	bool is_left;
	bool is_right;
private:
	void getRosParamForConstValue(int& width, int& height, int& steer_max_angle, int& detect_line_count);
	void getRosParamForUpdate();

	cv::Mat parseRawimg(const sensor_msgs::ImageConstPtr& image);
#if DEBUG
	sensor_msgs::ImagePtr getDetectColorImg();
	sensor_msgs::ImagePtr getDetectFinalBinImg();
	sensor_msgs::ImagePtr getDetectGrayBinImg();
	//sensor_msgs::ImagePtr getDetectHsvHBinImg();
	sensor_msgs::ImagePtr getDetectHsvSBinImg();
	//sensor_msgs::ImagePtr getDetectHsvVBinImg();

	std_msgs::String getPrintlog();
#endif

#if RC_CAR
	std_msgs::String makeControlMsg(int steer);
	void printData(std_msgs::String control_msg);
#elif SCALE_PLATFORM
	ackermann_msgs::AckermannDriveStamped makeControlMsg();
	void printData();
#endif

private:
	ros::NodeHandle nh_;
	ros::Publisher control_pub_;
#if DEBUG
	ros::Publisher true_color_pub_;
	ros::Publisher final_bin_pub_;
	ros::Publisher bin_from_gray_pub_;

	//ros::Publisher bin_from_hsv_h_pub_;
	ros::Publisher bin_from_hsv_s_pub_;
	//ros::Publisher bin_from_hsv_v_pub_;

	ros::Publisher printlog_pub_;
#endif
	ros::Subscriber left_image_sub_;
	ros::Subscriber right_image_sub_;
	ros::Subscriber status_sub_;

	int throttle_ = 0;

	std::unique_ptr<InToOutLaneDetector> lanedetector_ptr_;
};

#endif
