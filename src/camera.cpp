#include <iostream>
#include <string>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>

#include <sys/poll.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>

class EyeCam {
public:
	EyeCam(std::string file);
	void GetFrame(void);
	void WaitForFrame(int to_ms) const;
	void SetResolution(uint32_t width, uint32_t height);

	double GetFPS(void) const;
	void SetFPS(uint32_t fps);

private:
	int m_fd;
	v4l2_streamparm m_param;
	v4l2_format     m_fmt_pix;

	void GetParam(v4l2_streamparm &param);
	void SetParam(v4l2_streamparm const &param);

	void GetFormat(v4l2_format &fmt);
	void SetFormat(v4l2_format const &fmt);
};

EyeCam::EyeCam(std::string file) {
	// Access mode is O_RDWR instead of as specified in the V4L documentation.
	m_fd = open(file.c_str(), O_RDWR);

	// TODO: Replace the error messages with a custom exception class.
	if (m_fd == -1) {
		switch (errno) {
		case EACCES:
			throw "err: insufficient permissions to open device";

		case EBUSY:
			throw "err: camera is already in use";

		case ENXIO:
			throw "err: device does not exist";

		case ENOMEM:
		case EMFILE:
		case ENFILE:
			throw "err: an unknown error has occured";

		default:
			throw "err: error opening device";
		}
	}

	GetParam(m_param);
	GetFormat(m_fmt_pix);

	// TODO: DEBUG
	v4l2_fract *fps = &m_param.parm.capture.timeperframe;
}

void EyeCam::GetFrame(void) {
}

void EyeCam::SetFPS(uint32_t fps) {
	m_param.parm.capture.timeperframe.numerator   = 1;
	m_param.parm.capture.timeperframe.denominator = fps;
	SetParam(m_param);
}

double EyeCam::GetFPS(void) const {
	double numer = m_param.parm.capture.timeperframe.numerator;
	double denom = m_param.parm.capture.timeperframe.denominator;
	return denom / numer;
}

void EyeCam::SetResolution(uint32_t width, uint32_t height) {
	std::cout << "Resolution (before): " << m_fmt_pix.fmt.pix.width  << " x "
	                                     << m_fmt_pix.fmt.pix.height
	                                     << std::endl;

	m_fmt_pix.fmt.pix.width  = width;
	m_fmt_pix.fmt.pix.height = height;
	SetFormat(m_fmt_pix);

	if (m_fmt_pix.fmt.pix.width != width || m_fmt_pix.fmt.pix.height != height) {
		throw "err: unable to set resolution";
	}

	std::cout << "Resolution (after): "  << m_fmt_pix.fmt.pix.width  << " x "
	                                     << m_fmt_pix.fmt.pix.height
	                                     << std::endl;
}

void EyeCam::WaitForFrame(int to_ms) const {
	struct pollfd fds = { m_fd, POLLIN };
	poll(&fds, 1, to_ms);
}

void EyeCam::GetParam(v4l2_streamparm &param) {
	param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	int ret = ioctl(m_fd, VIDIOC_G_PARM, &param);

	if (ret == -1) {
		throw "err: unable to get device parameters";
	}
}

void EyeCam::SetParam(v4l2_streamparm const &param) {
	int ret = ioctl(m_fd, VIDIOC_S_PARM, &param);

	if (ret == -1) {
		throw "err: unable to set device parameters";
	}
}

void EyeCam::GetFormat(v4l2_format &fmt) {
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	int ret = ioctl(m_fd, VIDIOC_G_FMT, &fmt);

	if (ret == -1) {
		throw "err: unable to get video format";
	}
}

void EyeCam::SetFormat(v4l2_format const &fmt) {
	int ret = ioctl(m_fd, VIDIOC_S_FMT, &fmt);

	if (ret == -1) {
		throw "err: unable to set video format";
	}
}

int main(int argc, char **argv) {
	if (argc <= 1) {
		std::cerr << "err: incorrect number of arguments\n"
		          << "usage: ./stereo_webcam <device>" << std::endl;
		return 1;
	}

	try {
		EyeCam camera(argv[1]);

		std::cout << "Frame Rate is " << camera.GetFPS() << " FPS" << std::endl;
		camera.SetFPS(60);
		std::cout << "Frame Rate is " << camera.GetFPS() << " FPS" << std::endl;

		std::cout << ">>> SetResolution(320, 240)" << std::endl;
		camera.SetResolution(320, 240);
		std::cout << std::endl;

		std::cout << ">>> SetResolution(640, 480)" << std::endl;
		camera.SetResolution(320, 240);
	} catch (char const *str) {
		std::cout << str << std::endl;
	}
	return 0;
}
