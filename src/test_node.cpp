#include <iostream>
#include <iomanip>
#include <sstream>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <boost/thread.hpp>

#include "CameraFrame.hpp"
#include "CameraFrameComparator.hpp"
#include "CameraMonitor.hpp"
#include "Webcam.hpp"

static cv::Point  const TEXT_OFFSET(20, 480 - 20);
static int        const TEXT_FONT(cv::FONT_HERSHEY_SIMPLEX);
static double     const TEXT_SCALE(1.2);
static cv::Scalar const TEXT_COLOR(0, 255, 0);
static int        const TEXT_WIDTH(2);

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

		// Superimpose the timestamp difference on the images.
		timeval time_left  = frame_left.GetTimestamp();
		timeval time_right = frame_right.GetTimestamp();
		timeval time_diff  = CameraFrameComparator::GetTimeDelta(time_left, time_right);

		std::stringstream ss1;
		ss1 << time_diff.tv_sec << '.'
		    << std::setw(6) << std::setfill('0') << time_diff.tv_usec;
		cv::putText(dst, ss1.str(), TEXT_OFFSET, TEXT_FONT, TEXT_SCALE,
		            TEXT_COLOR, TEXT_WIDTH);

		// Write the concatenated frames to a file.
		std::stringstream ss2;
		ss2 << prefix << i << '.' << extension;
		cv::imwrite(ss2.str(), dst);

		// Quit using the escape key.
		//cv::imshow("Synchronization Test", dst);
		//cv::waitKey(10);
		++i;
	}
	return 0;
}
