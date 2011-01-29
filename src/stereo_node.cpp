#include <exception>
#include <iostream>
#include <string>

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include "CameraFrame.hpp"
#include "CameraFrameComparator.hpp"
#include "CameraMonitor.hpp"
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

		// Configure the accuracy of the equality test using the frame rate.
		CameraFrameComparator frame_comp(0.5 / cam_left.GetFPS());

		cam_left.SetStreaming(true);
		cam_right.SetStreaming(true);

		// Internally buffer each camera to find frame correspondances.
		CameraMonitor mon_left(cam_left, 5);
		CameraMonitor mon_right(cam_right, 5);
		boost::thread thread_left(boost::ref(mon_left));
		boost::thread thread_right(boost::ref(mon_right));

		CameraFrame frame_left;
		CameraFrame frame_right;

		bool adv_left  = true;
		bool adv_right = true;

		while (ros::ok()) {
			// Wait for new frames from the camera(s).
			// TODO: Make this more efficient with condition variables.
			if (adv_left) {
				mon_left.GetFrame(frame_left);
				adv_left = false;
			}
			if (adv_right) {
				mon_right.GetFrame(frame_right);
				adv_right = false;
			}

			// Synchronized (time_left == time_right)
			int frame_comp_res = frame_comp.Compare(frame_left, frame_right);
			if (!frame_comp_res) {
				// TODO: Choose the (slightly) higher of the two timestamps.
				ros::Time time = time_conv.SysToROS(frame_left.GetTimestamp());

				Image msg_left = FrameToImageMsg(frame_left);
				msg_left.header.stamp     = time;
				msg_left.header.frame_id  = "stereo/left_link";
				pub_left.publish(msg_left);

				Image msg_right = FrameToImageMsg(frame_right);
				msg_right.header.stamp     = time;
				msg_right.header.frame_id  = "stereo/right_link";
				pub_right.publish(msg_right);

				adv_left  = true;
				adv_right = true;
			}
			// Left lagging (time_left < time_right)
			else if (frame_comp_res < 0) {
				adv_left  = true;
			}
			// Right lagging (time_left > time_right)
			else {
				adv_right = true;
			}
		}
	} catch (std::exception const &err) {
		std::cerr << "err: " << err.what() << std::endl;
		return 1;
	}
	return 0;
}
