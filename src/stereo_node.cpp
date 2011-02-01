#include <exception>
#include <iostream>
#include <string>

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/SetCameraInfo.h>

#include "CameraFrame.hpp"
#include "CameraFrameComparator.hpp"
#include "TimeConverter.hpp"
#include "Webcam.hpp"


using sensor_msgs::SetCameraInfo;
using sensor_msgs::CameraInfo;
using sensor_msgs::Image;

static CameraInfo g_info_left;
static CameraInfo g_info_right;

bool set_info_left(SetCameraInfo::Request &req, SetCameraInfo::Response &res)
{
	g_info_left = req.camera_info;
	res.success        = true;
	res.status_message = "";
	return true;
}

bool set_info_right(SetCameraInfo::Request &req, SetCameraInfo::Response &res)
{
	g_info_right = req.camera_info;
	res.success        = true;
	res.status_message = "";
	return true;
}

Image FrameToImageMsg(CameraFrame const &src)
{
	Image msg;
	msg.width    = src.GetWidth();
	msg.height   = src.GetHeight();
	msg.encoding = sensor_msgs::image_encodings::BGR8;
	msg.step     = src.GetWidth() * 3;

	// Copy data into the message.
	msg.data.resize(msg.step * msg.height);
	uint8_t const *data = src.GetDataBGR();
	for (size_t i = 0; i < msg.step * msg.height; ++i) {
		msg.data[i] = data[i];
	}
	return msg;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "stereo_node");
	ros::NodeHandle nh;

	ros::Publisher pub_left       = nh.advertise<Image>("left/image_raw",  10);
	ros::Publisher pub_right      = nh.advertise<Image>("right/image_raw", 10);
	ros::Publisher pub_info_left  = nh.advertise<CameraInfo>("left/camera_info",  10);
	ros::Publisher pub_info_right = nh.advertise<CameraInfo>("right/camera_info", 10);
	ros::ServiceServer srv_left   = nh.advertiseService("left/set_camera_info",  set_info_left);
	ros::ServiceServer srv_right  = nh.advertiseService("right/set_camera_info", set_info_right);

	// Paths to each of the camera device files.
	std::string dev_left, dev_right;
	double err_ratio;
	int    nbuf;

	ros::NodeHandle nh_priv("~");
	nh_priv.param("device_left",  dev_left,  std::string("/dev/video0"));
	nh_priv.param("device_right", dev_right, std::string("/dev/video1"));
	nh_priv.param("buffers",      nbuf,      10);
	nh_priv.param("error_ratio",  err_ratio, 0.5);
	// TODO: Enable use of the error_ratio parameter.
	// TODO: Allow configuration of other camera parameters.

	if (nbuf < 1) {
		ROS_ERROR("number of kernel buffers must be non-negative");
		return 1;
	} else if (nbuf == 1) {
		ROS_WARN("must use at least two buffers for synchronization to succeed");
	}

	// Convert between system timestamps and ROS timestamps using a single pair
	// of corresonding times.
	ros::Time init_ros = ros::Time::now();
	timeval   init_sys;
	gettimeofday(&init_sys, NULL);

	TimeConverter time_conv(init_sys, init_ros);
	CameraFrameComparator comp(0.0005);

	CameraFrame frame_left, frame_right;

	Webcam cam_left(dev_left, nbuf);
	Webcam cam_right(dev_right, nbuf);

	cam_left.SetStreaming(true);
	cam_right.SetStreaming(true);

	bool adv_left  = true;
	bool adv_right = true;

	while (ros::ok()) {
		// Resynchronize by only advancing the lagging camera.
		if (adv_left)  cam_left.GetFrame(frame_left);
		if (adv_right) cam_right.GetFrame(frame_right);
		adv_left  = false;
		adv_right = false;

		// TODO: Synchronize frames using timestamps.
		int order = comp.Compare(frame_left, frame_right);
		if (order == 0) {
			// TODO: Choose the (slightly) higher of the two timestamps.
			ros::Time time = time_conv.SysToROS(frame_left.GetTimestamp());

			// Left Camera's Image and CameraInfo
			Image msg_left = FrameToImageMsg(frame_left);
			msg_left.header.stamp      = time;
			msg_left.header.frame_id   = "stereo/left_link";
			pub_left.publish(msg_left);

			g_info_left.header.stamp     = time;
			g_info_left.header.frame_id  = "stereo/left_link";
			pub_info_left.publish(g_info_left);

			// Right Camera's Image and CameraInfo
			Image msg_right = FrameToImageMsg(frame_right);
			msg_right.header.stamp     = time;
			msg_right.header.frame_id  = "stereo/right_link";
			pub_right.publish(msg_right);

			g_info_right.header.stamp    = time;
			g_info_right.header.frame_id = "stereo/right_link";
			pub_info_right.publish(g_info_right);

			adv_left  = true;
			adv_right = true;
			// std::cerr << "ok: synchronized" << std::endl;
		} else if (order < 0) {
			adv_left = true;
			// std::cerr << "war: dropping left frame" << std::endl;
		} else if (order > 0) {
			adv_right = true;
			// std::cerr << "war: dropping right frame" << std::endl;
		}
		ros::spinOnce();
	}
	return 0;
}
