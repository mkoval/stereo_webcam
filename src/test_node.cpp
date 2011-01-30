#include <exception>
#include <iostream>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <boost/thread.hpp>

#include "CameraFrame.hpp"
#include "CameraFrameComparator.hpp"
#include "CameraMonitor.hpp"
#include "Webcam.hpp"

int main(int argc, char **argv)
{
	if (argc <= 4) {
		std::cerr << "err: incorrect number of arguments\n"
		          << "usage: test_node <dev_left> <dev_right> <prefix> <extension>"
		          << std::endl;
		return 1;
	}

	std::string const dev_left  = argv[1];
	std::string const dev_right = argv[2];
	std::string const prefix    = argv[3];
	std::string const extension = argv[4];
	uint32_t const nbuf      = 10;

	Webcam cam_left(dev_left, 1);
	Webcam cam_right(dev_right, 1);

	CameraFrame frame_left;
	CameraFrame frame_right;

	// Buffer the frames in user-space for resynchronization.
	CameraMonitor mon_left(cam_left,   nbuf);
	CameraMonitor mon_right(cam_right, nbuf);
	boost::thread thread_left(boost::ref(mon_left));
	boost::thread thread_right(boost::ref(mon_right));

	cam_left.SetStreaming(true);
	cam_right.SetStreaming(true);

	uint32_t i = 0;
	for (;;) {
		// Wait for new frames from the camera(s).
		mon_left.GetFrame(frame_left);
		mon_right.GetFrame(frame_right);

		// TODO: Pair frames using the timestamp.
		uint32_t width_left   = frame_left.GetWidth();
		uint32_t width_right  = frame_right.GetWidth();
		uint32_t width        = width_left;

		uint32_t height_left  = frame_left.GetHeight();
		uint32_t height_right = frame_right.GetHeight();
		uint32_t height       = height_left;

		if (width_left != width_right || height_left != height_right) {
			throw std::runtime_error("camera resolutions do not match");
		}

		// Grab the (hopefully) synchronized frames.
		uint8_t *data_left  = frame_left.GetDataBGR();
		uint8_t *data_right = frame_right.GetDataBGR();
		cv::Mat src_left(height, width, CV_8UC3, data_left, 3 * width);
		cv::Mat src_right(height, width, CV_8UC3, data_right, 3 * width);

		// Concatenate both frames side-by-side for display.
		cv::Mat dst(height, 2 * width, CV_8UC3);
		cv::Mat dst_left  = dst(cv::Range(0, height), cv::Range(0, width));
		cv::Mat dst_right = dst(cv::Range(0, height), cv::Range(width, 2 * width));
		src_left.copyTo(dst_left);
		src_right.copyTo(dst_right);

		// Write the concatenated frames to a file.
		std::stringstream ss;
		ss << prefix << i << '.' << extension;
		cv::imwrite(ss.str(), dst);
		++i;
	}
	return 0;
}
