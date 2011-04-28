#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "stereo_node.hpp"

namespace stereo_webcam {

WebcamNode::WebcamNode(ros::NodeHandle nh, ros::NodeHandle nh_priv)
{
	this->nh      = nh;
	this->nh_priv = nh_priv;
}

WebcamNode::~WebcamNode(void)
{
	for (int i = 0; i < m_num; ++i) {
		if (m_cams[i].driver) {
			m_cams[i].driver->SetStreaming(false);
		}
	}
}

void WebcamNode::onInit(void)
{
	// Parameters that are fixed at runtime.
	nh_priv.param<int>("cameras", m_num,     0);
	nh_priv.param<int>("width",   m_width,   640);
	nh_priv.param<int>("height",  m_height,  480);
	nh_priv.param<int>("buffers", m_buffers, 10);
	nh_priv.param<double>("fps",       m_fps,       30);
	nh_priv.param<double>("threshold", m_threshold, 0.05);

	m_comparator = boost::make_shared<CameraFrameComparator>(m_threshold);
	m_cams.resize(m_num);

	for (int i = 0; i < m_num; ++i) {
		std::stringstream ss;
		std::string id;

		ss << i;
		id = ss.str();

		// Path to the camera's device file and calibration parameters.
		std::string path_dev, path_cal;
		nh_priv.param<std::string>("device" + id, path_dev,           "");
		nh_priv.param<std::string>("params" + id, path_cal,           "");
		nh_priv.param<std::string>("frame"  + id, m_cams[i].frame_id, "");

		// TODO: Handle exceptions.
		ros::NodeHandle nh_cam(nh, "camera" + id);
		InitializeWebcam(nh_cam, path_dev, path_cal, m_cams[i]);
	}

	// Use dynamic_reconfigure to get all other parameters.
	m_srv_dr.setCallback(boost::bind(&WebcamNode::ReconfigureCallback, this, _1, _2));
}

void WebcamNode::SpinOnce(void)
{
	size_t newest = 0;
	bool   sync   = true;

	// Resynchronize advancing lagging cameras.
	for (int i = 0; i < m_num; ++i) {
		if (m_cams[i].advance) {
			m_cams[i].driver->GetFrame(*m_cams[i].frame);
		}
		m_cams[i].advance = false;
	}

	// Select the most recent frame of all cameras.
	for (int i = 1; i < m_num; ++i) {
		int order = m_comparator->Compare(*m_cams[i].frame, *m_cams[newest].frame);
		if (order > 0) {
			newest = i;
		}
	}

	// Advance all cameras that are sufficiently lagging the newest frame.
	for (int i = 0; i < m_num; ++i) {
		int order = m_comparator->Compare(*m_cams[i].frame, *m_cams[newest].frame);
		if (order < 0) {
			m_cams[i].advance = true;
			sync = false;
			ROS_WARN("dropping frame from camera %d", i);
		}
	}

	// Publish all of the Image and CameraInfo messages using with the same
	// timestamp. This allows them to be matched with TimeSynchronizer
	// without using the approximate policy.
	if (sync) {
		ros::Time time = ros::Time::now();

		for (int i = 0; i < m_num; ++i) {
			CameraInfo info  = m_cams[i].manager->getCameraInfo();
			Image      image = FrameToImageMsg(*m_cams[i].frame);

			image.header.stamp = time;
			info.header.stamp  = time;
			image.header.frame_id = m_cams[i].frame_id;
			info.header.frame_id  = m_cams[i].frame_id;
			m_cams[i].pub.publish(image, info);

			// No camera is lagging; advance all frames equally.
			for (int i = 0; i < m_num; ++i) {
				m_cams[i].advance = true;
			}
		}
	}
}

void WebcamNode::ReconfigureCallback(StereoWebcamConfig &config, int32_t level)
{
	for (int i = 0; i < m_num; ++i) {
		m_cams[i].driver->SetControl(V4L2_CID_AUTOGAIN,           config.auto_gain);
		m_cams[i].driver->SetControl(V4L2_CID_AUTO_WHITE_BALANCE, config.auto_white);
		m_cams[i].driver->SetControl(V4L2_CID_HFLIP,              config.hflip);
		m_cams[i].driver->SetControl(V4L2_CID_VFLIP,              config.vflip);
		m_cams[i].driver->SetControl(V4L2_CID_BRIGHTNESS,         config.brightness);
		m_cams[i].driver->SetControl(V4L2_CID_SHARPNESS,          config.sharpness);
		m_cams[i].driver->SetControl(V4L2_CID_EXPOSURE,           config.exposure);
		m_cams[i].driver->SetControl(V4L2_CID_CONTRAST,           config.contrast);
		m_cams[i].driver->SetControl(V4L2_CID_GAIN,               config.gain);
	}
}

void WebcamNode::InitializeWebcam(ros::NodeHandle ns, std::string path_dev, std::string path_cal,
                                  InternalWebcam &webcam) const
{
	it::ImageTransport it(ns);

	webcam.pub     = it.advertiseCamera("image_raw", 1);
	webcam.manager = boost::make_shared<CameraInfoManager>(ns, "", path_cal);
	webcam.driver  = boost::make_shared<Webcam>(path_dev, m_buffers);
	webcam.frame   = boost::make_shared<CameraFrame>();
	webcam.advance = false;

	webcam.driver->SetResolution(m_width, m_height);
	webcam.driver->SetFPS(m_fps);
	webcam.driver->SetStreaming(true);
}

Image WebcamNode::FrameToImageMsg(CameraFrame const &src) const
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

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "stereo_node");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	// Setup dynamic_reconfigure for runtime settings.
	stereo_webcam::WebcamNode node(nh, nh_priv);
	node.onInit();

	while (ros::ok()) {
		node.SpinOnce();
		ros::spinOnce();
	}
	return 0;
}
