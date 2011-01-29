#include <iostream>
#include <errno.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "CameraFrame.hpp"
#include "Webcam.hpp"

int main(int argc, char **argv) {
	// OpenCV sets errno to "no such file or directory."
	errno = 0;

	if (argc <= 3) {
		std::cerr << "err: incorrect number of arguments\n"
		          << "usage: ./stereo_webcam <device 1> <device 2> <prefix>" << std::endl;
		return 1;
	}

	try {
		Webcam cam1(argv[1]);
		Webcam cam2(argv[2]);
		cam1.SetStreaming(true);
		cam2.SetStreaming(true);

		CameraFrame frame1;
		CameraFrame frame2;

		// Stream video.
		cv::Mat frame;
		for (;;) {
			cam1.WaitForFrame(100);
			cam2.WaitForFrame(100);

			cam1.GetFrame(frame1);
			cam2.GetFrame(frame2);

			uint32_t width  = frame1.GetWidth();
			uint32_t height = frame1.GetHeight();

			cv::Mat img1(height, width, CV_8UC3, frame1.GetDataBGR(), 0);
			cv::Mat img2(height, width, CV_8UC3, frame2.GetDataBGR(), 0);
			cv::Mat img(height, 2 * width, CV_8UC3);

			cv::Mat img_left  = img(cv::Range(0, height), cv::Range(0,      width));
			cv::Mat img_right = img(cv::Range(0, height), cv::Range(width,  width * 2));
			img1.copyTo(img_left);
			img2.copyTo(img_right);

#if 0
			std::stringstream ss2;
			ss2 << argv[3] << i << ".png";
			cv::imwrite(ss2.str(), img);
#endif

			cv::imshow("SyncTest", img);
			cv::waitKey(10);
		}

#if 0
		std::cout << "BEGIN:\n"
		          << "\tResolution = " << camera.GetWidth()  << " x "
		                               << camera.GetHeight() <<  "\n"
		          << "\tFPS        = " << camera.GetFPS()    <<  "\n"
		          << std::endl;

		// XXX: SetResolution() test one
		camera.SetFPS(125);
		camera.SetFPS(60);
		camera.SetResolution(320, 240);
		std::cout << "SET Resolution(320, 240) FPS(125):\n"
		          << "\tResolution = " << camera.GetWidth()  << " x "
		                               << camera.GetHeight() << "\n"
		          << "\tFPS        = " << camera.GetFPS()    << "\n"
		          << std::endl;

		// XXX: SetResolution() test two
		camera.SetFPS(60);
		camera.SetResolution(640, 320);
		std::cout << "SET Resolution(640, 320) FPS(60):\n"
		          << "\tResolution = " << camera.GetWidth()  << " x "
		                               << camera.GetHeight() << "\n"
		          << "\tFPS        = " << camera.GetFPS()    << "\n"
		          << std::endl;
#endif
	} catch (char const *str) {
		std::cout << str << std::endl;
	}
	return 0;
}
