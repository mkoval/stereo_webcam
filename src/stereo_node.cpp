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
#include "Webcam.hpp"

using sensor_msgs::CameraInfo;
using sensor_msgs::Image;
using image_transport::CameraPublisher;
using image_transport::ImageTransport;

static double const def_threshold = 0.005;
static int const def_width   = 640;
static int const def_height  = 480;
static int const def_fps     = 30;
static int const def_buffers = 2;
static int const def_cameras = 1;
static std::string def_frame = "camera_link";

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

	// Resolution and FPS are shared across all of the cameras.
	std::string frame_id;
	int width, height, fps;
	int buffers, cameras;
	double threshold;

	nh_priv.param("width",     width,     def_width);
	nh_priv.param("height",    height,    def_height);
	nh_priv.param("fps",       fps,       def_fps);
	nh_priv.param("buffers",   buffers,   def_buffers);
	nh_priv.param("cameras",   cameras,   def_cameras);
	nh_priv.param("threshold", threshold, def_threshold);
	nh_priv.param("frame",     frame_id,  def_frame);

	bool freq_filter;
	bool vflip, hflip;
	bool auto_exposure, auto_gain, auto_white;
	int brightness, sharpness, contrast, exposure, gain;

	nh_priv.param<bool>("auto_exposure", auto_exposure, true);
	nh_priv.param<bool>("auto_gain",     auto_gain,     true);
	nh_priv.param<bool>("auto_white",    auto_white,    true);
	nh_priv.param<bool>("freq_filter",   freq_filter,   false);
	nh_priv.param<bool>("vflip",         vflip,         false);
	nh_priv.param<bool>("hflip",         hflip,         false);
	nh_priv.param<int>("brightness", brightness, 0);
	nh_priv.param<int>("sharpness",  sharpness,  0);
	nh_priv.param<int>("exposure",   exposure,   120);
	nh_priv.param<int>("contrast",   contrast,   32);
	nh_priv.param<int>("gain",       gain,       20);

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
		bool ok = nh_priv.getParam("device" + id, path);
		nh_priv.getParam("params" + id, info);

		if (!ok) {
			ROS_ERROR("missing device path for camera %d", i);
			return 1;
		}

		ros::NodeHandle nh_cam("camera" + id);
		ImageTransport  it(nh_cam);
		pub[i]     = it.advertiseCamera("image", 1);
		caminfo[i] = new CameraInfoManager(nh_cam, "", info);

		try {
			cam[i] = new Webcam(path, buffers);
		} catch (std::invalid_argument const &e) {
			// TODO: Clean up the previously allocated memory.
			ROS_ERROR("unable to open '%s'", path.c_str());
			return 1;
		}

		if (!caminfo[i]->isCalibrated()) {
			ROS_WARN("camera %d is not calibrated", i);
		}

		// Negotiate the resolution, failing if not supported.
		try {
			cam[i]->SetResolution(width, height);
			cam[i]->SetFPS(fps);
			cam[i]->SetStreaming(true);
		} catch (std::runtime_error const &e) {
			// TODO: Clean up the previously allocated memory.
			ROS_ERROR("resolution is not supported");
			return 1;
		} catch (std::invalid_argument const &e) {
			ROS_ERROR("fps is not supported at this resolution");
			return 1;
		}
	}

	for (int i = 0; i < cameras; ++i) {
		cam[i]->SetControl(V4L2_CID_AUTOGAIN,           auto_gain);
		cam[i]->SetControl(V4L2_CID_AUTO_WHITE_BALANCE, auto_white);
		cam[i]->SetControl(V4L2_CID_HFLIP,              hflip);
		cam[i]->SetControl(V4L2_CID_VFLIP,              vflip);
		cam[i]->SetControl(V4L2_CID_BRIGHTNESS, brightness);
		cam[i]->SetControl(V4L2_CID_SHARPNESS,  sharpness);
		cam[i]->SetControl(V4L2_CID_EXPOSURE,   exposure);
		cam[i]->SetControl(V4L2_CID_CONTRAST,   contrast);
		cam[i]->SetControl(V4L2_CID_GAIN,       gain);
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
				ROS_WARN("dropping frame from camera %d", i);
			}
		}

		// Publish all of the Image and CameraInfo messages using with the same
		// timestamp to be matched with a TimeSynchronizer.
		if (sync) {
			ros::Time time = ros::Time::now();

			for (int i = 0; i < cameras; ++i) {
				CameraInfo info  = caminfo[i]->getCameraInfo();
				Image      image = FrameToImageMsg(frames[i]);

				// TODO: Allow the user to specify frame_id with rosparams.
				image.header.stamp    = time;
				info.header.stamp     = time;
				image.header.frame_id = frame_id;
				info.header.frame_id  = frame_id;
				pub[i].publish(image, info);

				// No camera is lagging; advance all frames equally.
				for (int i = 0; i < cameras; ++i) {
					advance[i] = true;
				}
			}
		}
		ros::spinOnce();
	}
	return 0;
}
