#include <exception>
#include <iostream>
#include <string>

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include "CameraFrame.hpp"
#include "CameraFrameComparator.hpp"
#include "CameraMonitor.hpp"
#include "TimeConverter.hpp"
#include "Webcam.hpp"

using sensor_msgs::CameraInfo;
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
	ros::init(argc, argv, "stereo_node");
	ros::NodeHandle nh;
	ros::Publisher pub_left  = nh.advertise<Image>("left/image_raw",  10);
	ros::Publisher pub_right = nh.advertise<Image>("right/image_raw", 10);
	ros::Publisher pub_info_left  = nh.advertise<CameraInfo>("left/camera_info", 10);
	ros::Publisher pub_info_right = nh.advertise<CameraInfo>("right/camera_info", 10);

	// Paths to each of the camera device files.
	std::string dev_left, dev_right;
	double err_ratio;
	int    nbuf;

	nh.param("device_left",  dev_left,  std::string("/dev/video0"));
	nh.param("device_right", dev_right, std::string("/dev/video1"));
	nh.param("buffers",      nbuf,      10);
	nh.param("error_ratio",  err_ratio, 0.5);
	// TODO: Allow configuration of other camera parameters.

	// Convert between system timestamps and ROS timestamps using a single pair
	// of corresonding times.
	ros::Time init_ros = ros::Time::now();
	timeval   init_sys;
	gettimeofday(&init_sys, NULL);
	TimeConverter time_conv(init_sys, init_ros);

	CameraInfo  info_left,  info_right;
	CameraFrame frame_left, frame_right;

	Webcam cam_left(dev_left, 1);
	Webcam cam_right(dev_right, 1);

	// Buffer the frames in user-space for resynchronization.
	CameraMonitor mon_left(cam_left,   nbuf);
	CameraMonitor mon_right(cam_right, nbuf);
	boost::thread thread_left(boost::ref(mon_left));
	boost::thread thread_right(boost::ref(mon_right));

	// Set the timestamp fuzzy-match threshold using the cameras' FPS.
	CameraFrameComparator frame_comp(err_ratio / cam_left.GetFPS());

	cam_left.SetStreaming(true);
	cam_right.SetStreaming(true);

	bool adv_left  = true;
	bool adv_right = true;

	while (ros::ok()) {
		// Wait for new frames from the camera(s).
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
			pub_info_left.publish(info_left);

			Image msg_right = FrameToImageMsg(frame_right);
			msg_right.header.stamp     = time;
			msg_right.header.frame_id  = "stereo/right_link";
			pub_right.publish(msg_right);
			pub_info_right.publish(info_right);

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

		// XXX: Unable to call ros::spin() as often as needed.
	}
	return 0;
}
