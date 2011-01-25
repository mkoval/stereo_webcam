#include <iostream>
#include <string>
#include <vector>

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>

#include <sys/poll.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>

class EyeCam {
public:
	EyeCam(std::string file);
	~EyeCam(void);

	void GetFrame(void);
	void WaitForFrame(int to_ms) const;
	void SetResolution(uint32_t width, uint32_t height);

	double GetFPS(void) const;
	void SetFPS(uint32_t fps);

private:
	struct Buffer {
		void  *ptr;
		size_t len;
	};

	int m_fd;
	int m_nbufs;
	v4l2_streamparm     m_param;
	v4l2_format         m_fmt_pix;
	std::vector<Buffer> m_bufs;

	void GetParam(v4l2_streamparm &param);
	void SetParam(v4l2_streamparm const &param);

	void GetFormat(v4l2_format &fmt);
	void SetFormat(v4l2_format const &fmt);

	void SetStreaming(bool streaming);
};

EyeCam::EyeCam(std::string file) {
	int ret;

	// Access mode is O_RDWR instead of as specified in the V4L documentation.
	m_fd    = open(file.c_str(), O_RDWR);
	m_nbufs = 2;

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

	// Request the appropriate number of buffers or mmap IO.
	v4l2_requestbuffers req_bufs;
	req_bufs.count  = m_nbufs;
	req_bufs.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req_bufs.memory = V4L2_MEMORY_MMAP;
	ret = ioctl(m_fd, VIDIOC_REQBUFS, &req_bufs);
	if (ret == -1 || req_bufs.count < m_nbufs) {
		throw "err: unable to allocate memory-mapped buffers";
	}

	//
	m_bufs.resize(req_bufs.count);

	for (int i = 0; i < req_bufs.count; ++i) {
		struct v4l2_buffer buffer;

		memset(&buffer, 0, sizeof(v4l2_buffer));
		buffer.type   = req_bufs.type;
		buffer.memory = req_bufs.memory;
		buffer.index  = i;

		ret = ioctl(m_fd, VIDIOC_QUERYBUF, &buffer);
		if (ret == -1) {
			throw "err: unable to memory-map buffer";
		}

		// Save the length for munmap() later.
		m_bufs[i].ptr = mmap(NULL, buffer.length, PROT_READ | PROT_WRITE,
		                     MAP_SHARED, m_fd, buffer.m.offset);
		m_bufs[i].len = buffer.length;
		if (m_bufs[i].ptr == MAP_FAILED) {
			// XXX: Free memory with munmap() before failing.
			throw "err: memory map failed";
		}

		// Begin by enqueing all of the buffers.
		ret = ioctl(m_fd, VIDIOC_QBUF, &buffer);
		if (ret == -1) {
			throw "err: unable to enqueue memory-mapped buffer";
		}
	}

	// Enable streaming.
	ret = ioctl(m_fd, VIDIOC_STREAMON, &req_bufs.type);
	if (ret == -1) {
		throw "err: unable to enable streaming";
	}


	// Enumerate over image formats (i.e. RGB vs YUV).
	v4l2_fmtdesc it_desc;
	it_desc.index = 0;
	it_desc.type  = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	for (;;) {
		ret = ioctl(m_fd, VIDIOC_ENUM_FMT, &it_desc);

		if (!ret) {
			std::cout << "pix fmt: " << it_desc.description << std::endl;
		} else if (errno == EINVAL) {
			break; // Iteration is complete
		} else {
			throw "err: unable to detect formats";
		}

		++it_desc.index;
	}

	// Enumerate over frame sizes.
	// TODO: For all pixel formats.
	v4l2_frmsizeenum it_size;
	it_size.index        = 0;
	it_size.pixel_format = V4L2_PIX_FMT_YUYV;
	for (;;) {
		ret = ioctl(m_fd, VIDIOC_ENUM_FRAMESIZES, &it_size);

		if (!ret) {
			// TODO: Handle the non-discrete case.
			int width  = it_size.discrete.width;
			int height = it_size.discrete.height;
			std::cout << "frame size: " << width << " x " << height << std::endl;
		} else if (errno = EINVAL) {
			break; // Iteration is complete
		} else {
			throw "err: unable to detect frame sizes";
		}

		++it_size.index;
	}

	// Enumerate over all frame intervals.
	// TODO: For all frame sizes and pixel formats.
	v4l2_frmivalenum it_interval;
	it_interval.index        = 0;
	it_interval.pixel_format = V4L2_PIX_FMT_YUYV;
	it_interval.width        = 640;
	it_interval.height       = 480;

	for (;;) {
		ret = ioctl(m_fd, VIDIOC_ENUM_FRAMEINTERVALS, &it_interval);

		if (!ret) {
			// TODO: Handle the non-discrete case.
			double fps = (double)it_interval.discrete.denominator /
			             it_interval.discrete.numerator;
			std::cout << "fps: " << fps << std::endl;
		} else if (errno = EINVAL) {
			break; // Iteration is complete
		} else {
			throw "err: unable to detect frame intervals";
		}
		++it_interval.index;
	}

	// Fetch default configuration data from the camera.
	GetParam(m_param);
	GetFormat(m_fmt_pix);
}

EyeCam::~EyeCam(void) {
	for (int i = 0; i < m_bufs.size(); ++i) {
		munmap(m_bufs[i].ptr, m_bufs[i].len);
	}
}

void EyeCam::GetFrame(void) {
}

void EyeCam::SetFPS(uint32_t fps) {
	m_param.parm.capture.timeperframe.numerator   = 1;
	m_param.parm.capture.timeperframe.denominator = fps;
	SetParam(m_param);

	uint32_t fps_new = m_param.parm.capture.timeperframe.denominator
	                 / m_param.parm.capture.timeperframe.numerator;
	if (fps != fps_new) {
		throw "err: unable to set FPS";
	}

}

double EyeCam::GetFPS(void) const {
	double numer = m_param.parm.capture.timeperframe.numerator;
	double denom = m_param.parm.capture.timeperframe.denominator;
	return denom / numer;
}

void EyeCam::SetResolution(uint32_t width, uint32_t height) {
	m_fmt_pix.fmt.pix.width  = width;
	m_fmt_pix.fmt.pix.height = height;
	m_fmt_pix.fmt.pix.priv   = !!(width == 640);
	SetFormat(m_fmt_pix);

	if (m_fmt_pix.fmt.pix.width != width || m_fmt_pix.fmt.pix.height != height) {
		throw "err: unable to set resolution";
	}
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

#if 0
		std::cout << "Frame Rate is " << camera.GetFPS() << " FPS" << std::endl;
		camera.SetFPS(10);
		std::cout << "Frame Rate is " << camera.GetFPS() << " FPS" << std::endl;
#endif

		std::cout << ">>> SetResolution(640, 480)" << std::endl;
		camera.SetResolution(640, 480);
		camera.SetFPS(30);
		std::cout << std::endl;

		std::cout << ">>> SetResolution(320, 240)" << std::endl;
		camera.SetResolution(320, 240);
		camera.SetFPS(60);

	} catch (char const *str) {
		std::cout << str << std::endl;
	}
	return 0;
}
