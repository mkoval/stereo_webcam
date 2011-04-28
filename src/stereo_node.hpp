#ifndef STEREO_NODE_HPP_
#define STEREO_NODE_HPP_

#include <string>
#include <vector>

#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <stereo_webcam/StereoWebcamConfig.h>

#include "CameraFrame.hpp"
#include "CameraFrameComparator.hpp"
#include "Webcam.hpp"

namespace stereo_webcam {

namespace dr = dynamic_reconfigure;
namespace it = image_transport;

using sensor_msgs::CameraInfo;
using sensor_msgs::Image;

class WebcamNode {
public:
	WebcamNode(ros::NodeHandle nh, ros::NodeHandle nh_priv);
	~WebcamNode(void);
	void onInit(void);
	void SpinOnce(void);
	void ReconfigureCallback(StereoWebcamConfig &config, int32_t level);

private:
	struct InternalWebcam {
		image_transport::CameraPublisher     pub;
		boost::shared_ptr<CameraInfoManager> manager;
		boost::shared_ptr<Webcam>            driver;
		boost::shared_ptr<CameraFrame>       frame;
		std::string frame_id;
		bool        advance;
	};

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv;

	int m_num;
	int m_width, m_height;
	int m_buffers;
	double m_fps;
	double m_threshold;

	std::vector<InternalWebcam>              m_cams;
	dr::Server<StereoWebcamConfig>           m_srv_dr;
	boost::shared_ptr<CameraFrameComparator> m_comparator;
	boost::shared_ptr<it::ImageTransport>    m_it;

	void InitializeWebcam(std::string base, std::string path_dev, std::string path_cal, InternalWebcam &webcam) const;
	Image FrameToImageMsg(CameraFrame const &src) const;
};

};

#endif
