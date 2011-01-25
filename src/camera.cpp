#include <iostream>
#include <list>
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

	void SetStreaming(bool streaming);

	void GetFrame(void);
	void WaitForFrame(int to_ms) const;

	uint32_t GetWidth(void) const;
	uint32_t GetHeight(void) const;
	void SetResolution(uint32_t width, uint32_t height);

	double GetFPS(void) const;
	void SetFPS(uint32_t fps);

private:
	struct Resolution {
		uint32_t width;
		uint32_t height;
	};

	struct Buffer {
		void  *ptr;
		size_t len;
	};

	int m_fd;
	uint32_t m_nbufs;
	v4l2_streamparm     m_param;
	v4l2_format         m_fmt_pix;
	std::vector<Buffer> m_bufs;

	std::list<uint32_t>   GetPixelFormats(void) const;
	std::list<Resolution> GetResolutions(uint32_t pixel_format) const;
	std::list<double>     GetFPSs(uint32_t pixel_format, Resolution res) const;

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

	// Allocate memmap buffers for recieving the frames.
	m_bufs.resize(req_bufs.count);
	for (uint32_t i = 0; i < req_bufs.count; ++i) {
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

	}


	std::list<uint32_t> fmt = GetPixelFormats();
	std::list<uint32_t>::iterator fmt_it;
	for (fmt_it = fmt.begin(); fmt_it != fmt.end(); ++fmt_it) {
		std::list<Resolution> res = GetResolutions(*fmt_it);
		std::list<Resolution>::iterator res_it;

		std::cout << "fmt = " << *fmt_it << std::endl;

		for (res_it = res.begin(); res_it != res.end(); ++res_it) {
			std::list<double> fps = GetFPSs(*fmt_it, *res_it);
			std::list<double>::iterator fps_it;

			std::cout << "\tres = " << res_it->width << " x " << res_it->height << std::endl;

			for (fps_it = fps.begin(); fps_it != fps.end(); ++fps_it) {
				std::cout << "\t\tfps = " << *fps_it << std::endl;
			}
		}
	}

	// Fetch default configuration data from the camera.
	GetParam(m_param);
	GetFormat(m_fmt_pix);
	SetStreaming(false);
}

EyeCam::~EyeCam(void) {
	for (uint32_t i = 0; i < m_bufs.size(); ++i) {
		munmap(m_bufs[i].ptr, m_bufs[i].len);
	}
}

void SetStreaming(bool streaming) {
	int ret;

	// Begin by enqueing all of the buffers since they are dequeued when
	// streaming halts (and by default)
	if (streaming) {
		for (size_t i = 0; i < req_bufs.size(); ++i) {
			ret = ioctl(m_fd, VIDIOC_QBUF, &buffer);
			if (ret == -1) {
				throw "err: unable to enqueue memory-mapped buffer";
			}
		}
	}

	// Disabling streaming automatically clears the queue.
	if (streaming) {
		ret = ioctl(m_fd, VIDIOC_STREAMON, &req_bufs.type);
	} else {
		ret = ioctl(m_fd, VIDIOC_STREAMOFF, &req_bufs.type);
	}

	if (ret == -1) {
		throw "err: unable to enable/disable streaming";
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
		throw "err: invalid frame rate";
	}
}

double EyeCam::GetFPS(void) const {
	double numer = m_param.parm.capture.timeperframe.numerator;
	double denom = m_param.parm.capture.timeperframe.denominator;
	return denom / numer;
}

uint32_t EyeCam::GetWidth(void) const {
	return m_fmt_pix.fmt.pix.width;
}

uint32_t EyeCam::GetHeight(void) const {
	return m_fmt_pix.fmt.pix.height;
}

void EyeCam::SetResolution(uint32_t width, uint32_t height) {
	m_fmt_pix.fmt.pix.width  = width;
	m_fmt_pix.fmt.pix.height = height;

	SetFormat(m_fmt_pix);

	if (m_fmt_pix.fmt.pix.width != width || m_fmt_pix.fmt.pix.height != height) {
		throw "err: invalid resolution";
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
	int ret;

	ret = ioctl(m_fd, VIDIOC_S_FMT, &fmt);
	if (ret == -1) {
		throw "err: unable to set video format";
	}

	ret = ioctl(m_fd, VIDIOC_S_FMT, &fmt);
	if (ret == -1) {
		throw "err: unable to set video format";
	}
}

std::list<uint32_t> EyeCam::GetPixelFormats(void) const {
	std::list<uint32_t> formats;
	v4l2_fmtdesc it;
	int ret;

	it.index = 0;
	it.type  = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	for (;;) {
		ret = ioctl(m_fd, VIDIOC_ENUM_FMT, &it);

		if (!ret) {
			formats.push_back(it.pixelformat);
		} else if (errno == EINVAL) {
			break; // Iteration is complete
		} else {
			throw "err: unable to detect formats";
		}

		++it.index;
	}
	return formats;
}

std::list<EyeCam::Resolution> EyeCam::GetResolutions(uint32_t pixel_format) const {
	std::list<Resolution> resolutions;
	v4l2_frmsizeenum it;
	int ret;

	it.index        = 0;
	it.pixel_format = pixel_format;

	for (;;) {
		ret = ioctl(m_fd, VIDIOC_ENUM_FRAMESIZES, &it);

		if (!ret) {
			// TODO: Handle the non-discrete case.
			Resolution resolution;
			resolution.width  = it.discrete.width;
			resolution.height = it.discrete.height;
			resolutions.push_back(resolution);
		} else if (errno == EINVAL) {
			break; // Iteration is complete
		} else {
			throw "err: unable to detect frame sizes";
		}

		++it.index;
	}
	return resolutions;
}

std::list<double> EyeCam::GetFPSs(uint32_t pixel_format, Resolution res) const {
	std::list<double> fpss;
	v4l2_frmivalenum it;
	int ret;

	it.index        = 0;
	it.pixel_format = pixel_format;
	it.width        = res.width;
	it.height       = res.height;

	for (;;) {
		ret = ioctl(m_fd, VIDIOC_ENUM_FRAMEINTERVALS, &it);

		if (!ret) {
			// TODO: Handle the non-discrete case.
			double fps = (double)it.discrete.denominator /
			             it.discrete.numerator;
			fpss.push_back(fps);
		} else if (errno == EINVAL) {
			break; // Iteration is complete
		} else {
			throw "err: unable to detect frame intervals";
		}
		++it.index;
	}
	return fpss;
}

int main(int argc, char **argv) {
	if (argc <= 1) {
		std::cerr << "err: incorrect number of arguments\n"
		          << "usage: ./stereo_webcam <device>" << std::endl;
		return 1;
	}

	try {
		EyeCam camera(argv[1]);

		std::cout << "BEGIN:\n"
		          << "\tResolution = " << camera.GetWidth()  << " x "
		                               << camera.GetHeight() <<  "\n"
		          << "\tFPS        = " << camera.GetFPS()    << std::endl;

		// XXX: SetFPS() test
		camera.SetFPS(50); // valid: 60, 50, 40, 30, 15 (60 fails...)
		std::cout << "BEGIN:\n"
		          << "\tResolution = " << camera.GetWidth()  << " x "
		                               << camera.GetHeight() <<  "\n"
		          << "\tFPS        = " << camera.GetFPS()    << std::endl;

		// XXX: SetResolution() test
		camera.SetResolution(320, 240);
		std::cout << "SET1:\n"
		          << "\tResolution = " << camera.GetWidth()  << " x "
		                               << camera.GetHeight() <<  "\n"
		          << "\tFPS        = " << camera.GetFPS()    << std::endl;

	} catch (char const *str) {
		std::cout << str << std::endl;
	}
	return 0;
}
