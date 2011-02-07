#include <exception>
#include <string>

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

static std::string const def_dev_left   = "/dev/video_left";
static std::string const def_dev_right  = "/dev/video_right";
static std::string const def_file_left  = "calibration_left.yaml";
static std::string const def_file_right = "calibration_right.yaml";
static double const def_threshold = 0.005;
static int const def_width   = 640;
static int const def_height  = 480;
static int const def_fps     = 30;
static int const def_buffers = 10;

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
	ros::NodeHandle nh_priv("~");
	ros::NodeHandle nh_left("left");
	ros::NodeHandle nh_right("right");

	ImageTransport it(nh);
	CameraPublisher pub_left  = it.advertiseCamera("left/image", 1);
	CameraPublisher pub_right = it.advertiseCamera("right/image", 1);

	// Static parameters that are fixed during runtime.
	std::string dev_left,  dev_right;
	std::string file_left, file_right;
	int width, height, fps, buffers;
	double threshold;

	nh_priv.param("device_left",  dev_left,   def_dev_left);
	nh_priv.param("device_right", dev_right,  def_dev_right);
	nh_priv.param("info_left",    file_left,  def_file_left);
	nh_priv.param("info_right",   file_right, def_file_right);
	nh_priv.param("width",        width,      def_width);
	nh_priv.param("height",       height,     def_height);
	nh_priv.param("fps",          fps,        def_fps);
	nh_priv.param("buffers",      buffers,    def_buffers);
	nh_priv.param("threshold",    threshold,  def_threshold);

	if (buffers < 2) {
		ROS_ERROR("need two buffers or more buffers for synchronization");
		return 1;
	}

	// Negotiate resolution and framerate.
	Webcam cam_left(dev_left, buffers);
	Webcam cam_right(dev_right, buffers);

	try {
		cam_left.SetResolution(width, height);
		cam_right.SetResolution(width, height);
	} catch (std::runtime_error const &e) {
		ROS_ERROR("resolution is not supported");
		return 1;
	}

	try {
		cam_left.SetFPS(fps);
		cam_right.SetFPS(fps);
	} catch (std::invalid_argument const &e) {
		ROS_ERROR("framerate is not supported at this resolution");
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

	CameraFrame frame_left, frame_right;
	CameraInfoManager caminfo_left(nh_left, "left", file_left);
	CameraInfoManager caminfo_right(nh_right, "right", file_right);

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

			// Pair each image with its calibration data.
			CameraInfo info_left  = caminfo_left.getCameraInfo();
			CameraInfo info_right = caminfo_right.getCameraInfo();

			Image msg_left = FrameToImageMsg(frame_left);
			msg_left.header.stamp      = time;
			msg_left.header.frame_id   = "stereo_link";
			info_left.header.stamp     = time;
			info_left.header.frame_id  = "stereo_link";
			pub_left.publish(msg_left, info_left);

			Image msg_right = FrameToImageMsg(frame_right);
			msg_right.header.stamp     = time;
			msg_right.header.frame_id  = "stereo_link";
			info_right.header.stamp    = time;
			info_right.header.frame_id = "stereo_link";
			pub_right.publish(msg_right, info_right);

			adv_left  = true;
			adv_right = true;
		} else if (order < 0) {
			adv_left = true;
			ROS_WARN("desynchronized; dropping left frame");
		} else if (order > 0) {
			adv_right = true;
			ROS_WARN("desynchronized; dropping right frame");
		}
		ros::spinOnce();
	}
	return 0;
}
