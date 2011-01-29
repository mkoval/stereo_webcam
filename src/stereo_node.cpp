#include <exception>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include "CameraFrame.hpp"
#include "TimeConverter.hpp"
#include "Webcam.hpp"

using sensor_msgs::Image;

static size_t   const BUFFER_NUM = 2;
static uint32_t const TIMEOUT_MS = 100;

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
	if (argc <= 2) {
		std::cerr << "err: incorrect number of arguments\n"
		          << "usage: ./stereo_node <left> <right>"
		          << std::endl;
		return 1;
	}

	ros::init(argc, argv, "stereo_node");
	ros::NodeHandle nh;
	ros::Publisher pub_left  = nh.advertise<Image>("stereo/left", 10);
	ros::Publisher pub_right = nh.advertise<Image>("stereo/right", 10);

	// Convert between system timestamps and ROS timestamps using a single pair
	// of corresonding times.
	ros::Time init_ros = ros::Time::now();
	timeval   init_sys;
	gettimeofday(&init_sys, NULL);
	TimeConverter time_conv(init_sys, init_ros);

	CameraFrame frame_left;
	CameraFrame frame_right;

	try {
		Webcam cam_left(argv[1], BUFFER_NUM);
		Webcam cam_right(argv[2], BUFFER_NUM);

		cam_left.SetStreaming(true);
		cam_right.SetStreaming(true);

		// TODO: Use a multi-threaded program instead of a busy loop.
		while (ros::ok()) {
			cam_left.WaitForFrame(TIMEOUT_MS);
			cam_right.WaitForFrame(TIMEOUT_MS);

			cam_left.GetFrame(frame_left);
			cam_right.GetFrame(frame_right);

			// Wrap the frames in a sensor_msgs::Image message.
			Image msg_left = FrameToImageMsg(frame_left);
			msg_left.header.stamp     = time_conv.SysToROS(frame_left.GetTimestamp());
			msg_left.header.frame_id  = "stereo/left_link";

			Image msg_right = FrameToImageMsg(frame_right);
			msg_left.header.stamp     = time_conv.SysToROS(frame_right.GetTimestamp());
			msg_right.header.frame_id = "stereo/right_link";

			pub_left.publish(msg_left);
			pub_right.publish(msg_right);
			ros::spinOnce();
		}
	} catch (std::exception const &err) {
		std::cerr << "err: " << err.what() << std::endl;
		return 1;
	}
	return 0;
}
