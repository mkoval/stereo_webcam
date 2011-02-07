#include <exception>
#include <iostream>
#include <string>

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include "CameraFrame.hpp"
#include "CameraFrameComparator.hpp"
#include "TimeConverter.hpp"
#include "Webcam.hpp"

using sensor_msgs::CameraInfo;
using sensor_msgs::Image;
using image_transport::CameraPublisher;
using image_transport::ImageTransport;

static Image FrameToImageMsg(CameraFrame const &src)
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

	ImageTransport it(nh);
	CameraPublisher pub_left  = it.advertiseCamera("left/image", 1);
	CameraPublisher pub_right = it.advertiseCamera("right/image", 1);

	// Paths to each of the camera device files.
	std::string dev_left, dev_right;
	std::string info_left, info_right;
	int res_width, res_height;
	int nbuf, fps;

	ros::NodeHandle nh_priv("~");
	nh_priv.param("device_left",  dev_left,   std::string("/dev/video_left"));
	nh_priv.param("device_right", dev_right,  std::string("/dev/video_right"));
	nh_priv.param("info_left",    info_left,  std::string("file:///tmp/calibration_left.yaml"));
	nh_priv.param("info_right",   info_right, std::string("file:///tmp/calibration_right.yaml"));
	nh_priv.param("width",        res_width,  640);
	nh_priv.param("height",       res_height, 480);
	nh_priv.param("fps",          fps,        30);
	nh_priv.param("buffers",      nbuf,       10);
	// TODO: Enable use of the error_ratio parameter.
	// TODO: Allow configuration of other camera parameters.

	if (nbuf < 2) {
		ROS_ERROR("need two buffers or more buffers for synchronization");
		return 1;
	}

	// Negotiate resolution and framerate.
	Webcam cam_left(dev_left, nbuf);
	Webcam cam_right(dev_right, nbuf);

	try {
		cam_left.SetResolution(res_width, res_height);
		cam_right.SetResolution(res_width, res_height);
	} catch (std::runtime_error const &e) {
		std::cerr << "err: unable to negotiate resolution" << std::endl;
		return 1;
	}

	try {
		cam_left.SetFPS(fps);
		cam_right.SetFPS(fps);
	} catch (std::invalid_argument const &e) {
		std::cerr << "err: framerate is unsupported at this resolution" << std::endl;
		return 1;
	}

	// Convert between system timestamps and ROS timestamps using a single pair
	// of corresonding times.
	ros::Time init_ros = ros::Time::now();
	timeval   init_sys;
	gettimeofday(&init_sys, NULL);

	// TODO: Dynamically select this threshold using the FPS.
	TimeConverter time_conv(init_sys, init_ros);
	CameraFrameComparator comp(0.0005);

	CameraInfoManager caminfo_left(ros::NodeHandle("left"), "left", info_left);
	CameraInfoManager caminfo_right(ros::NodeHandle("right"), "right", info_right);

	CameraFrame frame_left;
	CameraFrame frame_right;

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
			msg_left.header.frame_id   = "stereo/stereo_link";

			CameraInfo info_l = caminfo_left.getCameraInfo();
			info_l.header.stamp    = time;
			info_l.header.frame_id = "stereo/stereo_link";
			pub_left.publish(msg_left, info_l);

			// Right Camera's Image and CameraInfo
			Image msg_right = FrameToImageMsg(frame_right);
			msg_right.header.stamp     = time;
			msg_right.header.frame_id  = "stereo/stereo_link";

			CameraInfo info_r = caminfo_right.getCameraInfo();
			info_r.header.stamp    = time;
			info_r.header.frame_id = "stereo/stereo_link";
			pub_right.publish(msg_right, info_r);

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
