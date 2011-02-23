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
static int const def_buffers = 2;
static int const def_cameras = 1;

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

	ros::NodeHandle nh, nh_priv("~");

	// Convert between system timestamps and ROS timestamps using a single pair
	// of corresonding times.
	ros::Time init_ros = ros::Time::now();
	timeval   init_sys;
	gettimeofday(&init_sys, NULL);
	TimeConverter time_conv(init_sys, init_ros);

	// Resolution and FPS are shared across all of the cameras.
	int width, height, fps;
	int buffers, cameras;
	double threshold;
	nh_priv.param("width",     width,     def_width);
	nh_priv.param("height",    height,    def_height);
	nh_priv.param("fps",       fps,       def_fps);
	nh_priv.param("buffers",   buffers,   def_buffers);
	nh_priv.param("cameras",   cameras,   def_cameras);
	nh_priv.param("threshold", threshold, def_threshold);

	// Load camera device names and calibration parameters per camera.
	// XXX: Use a smart pointer type to avoid memory leaks.
	std::vector<CameraPublisher>     pub(cameras);
	std::vector<CameraInfoManager *> caminfo(cameras);
	std::vector<Webcam *>            cam(cameras);

	for (int i = 0; i < cameras; ++i) {
		std::stringstream ss;
		std::string id, path, info;

		ss << i;
		id = ss.str();

		// Path to the camera's device file and calibration parameters.
		// XXX: Find a more elegant way of doing this using lists or dicts.
		nh_priv.getParam("camera" + id, path);
		nh_priv.getParam("calurl" + id, info);

		ros::NodeHandle nh_cam("camera" + id);
		ImageTransport  it(nh_cam);
		pub[i]     = it.advertiseCamera("image", 1);
		cam[i]     = new Webcam(path, buffers);
		caminfo[i] = new CameraInfoManager(nh_cam, "", info);

		// Negotiate the resolution, failing if not supported.
		try {
			cam[i]->SetResolution(width, height);
			cam[i]->SetFPS(fps);
			cam[i]->SetStreaming(true);
		} catch (std::runtime_error const &e) {
			ROS_ERROR("resolution is not supported");
			return 1;
		} catch (std::invalid_argument const &e) {
			ROS_ERROR("fps is not supported at this resolution");
			return 1;
		}
	}

	// TODO: Dynamically select this threshold using the FPS.
	CameraFrameComparator    comp(0.0005);
	std::vector<CameraFrame> frames(cameras);
	std::vector<bool>        advance(cameras, true);

	while (ros::ok()) {
		bool sync = true;

		// Resynchronize frames by advancing lagging cameras.
		for (int i = 0; i < cameras; ++i) {
			if (advance[i]) {
				cam[i]->GetFrame(frames[i]);
			}
			advance[i] = false;
		}

		// Select the most recent frame of all cameras.
		size_t newest = 0;
		for (int i = 1; i < cameras; ++i) {
			int order = comp.Compare(frames[i], frames[newest]);
			if (order > 0)
				newest = i;
		}

		// Advance all cameras that are lagging the most recent frame.
		for (int i = 0; i < cameras; ++i) {
			int order = comp.Compare(frames[i], frames[newest]);
			if (order < 0) {
				advance[i] = true;
				sync       = false;
			}
		}

		// Publish all of the Image and CameraInfo messages using with the same
		// timestamp to be matched with a TimeSynchronizer.
		if (sync) {
			ros::Time time = time_conv.SysToROS(frames[0].GetTimestamp());

			for (int i = 0; i < cameras; ++i) {
				CameraInfo info  = caminfo[i]->getCameraInfo();
				Image      image = FrameToImageMsg(frames[i]);

				// TODO: Allow the user to specify frame_id with rosparams.
				image.header.stamp    = time;
				info.header.stamp     = time;
				image.header.frame_id = "stereo_link";
				info.header.frame_id  = "stereo_link";
				pub[i].publish(image, info);
			}
		} else {
			ROS_WARN("desynchronization detected, dropping frame");
		}

		ros::spinOnce();
	}
	return 0;
}
