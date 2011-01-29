#include <exception>
#include <iostream>

#include "CameraFrame.hpp"
#include "Webcam.hpp"

static size_t   const BUFFER_NUM = 2;
static uint32_t const TIMEOUT_MS = 100;

int main(int argc, char **argv)
{
	if (argc <= 2) {
		std::cerr << "err: incorrect number of arguments\n"
		          << "usage: ./stereo_node <left> <right>"
		          << std::endl;
		return 1;
	}

	CameraFrame frame_left;
	CameraFrame frame_right;

	try {
		Webcam cam_left(argv[1], BUFFER_NUM);
		Webcam cam_right(argv[2], BUFFER_NUM);

		cam_left.SetStreaming(true);
		cam_right.SetStreaming(true);

		for (;;) {
			cam_left.WaitForFrame(TIMEOUT_MS);
			cam_right.WaitForFrame(TIMEOUT_MS);

			cam_left.GetFrame(frame_left);
			cam_right.GetFrame(frame_right);

			// TODO: Publish the image messages.
		}
	} catch (std::exception const &err) {
		std::cerr << "err: " << err.what() << std::endl;
		return 1;
	}
	return 0;
}
