#include <iomanip>
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
#include <sys/time.h>

#include <linux/videodev2.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "CameraFrame.hpp"

static int clamp (double x)
{
	int r = x;      /* round to nearest */

	if (r < 0)         return 0;
	else if (r > 255)  return 255;
	else               return r;
}


#define C_ADJ (255 / 122.0)
#define Y_ADJ(y1) ((255 / 219.0) * (y1 - 16))
static void yuv444_to_rgb(uint8_t Y1, uint8_t Cb, uint8_t Cr, uint8_t *dst)
{
	double r, g, b;         /* temporaries */
	double y1, pb, pr;

	pb = (Cb - 128);
	pr = (Cr - 128);

	r = Y_ADJ(Y1) +              255/112.0 * 0.701 * pr;
	g = Y_ADJ(Y1) - 255/112.0 * 0.886 * 0.114/0.587 * pb - 255/112.0*0.701 * 0.299/0.587 * pr;
	b = Y_ADJ(Y1) + 255/112.0 * 0.886 * pb;

	dst[2] = clamp (r ); /* [ok? one should prob. limit y1,pb,pr] */
	dst[1] = clamp (g );
	dst[0] = clamp (b );

}

/* consume 6 bytes of dst and 4 bytes of src */
static void yuv422_to_rgb(uint8_t const *src, uint8_t *dst)
{
	int8_t y1 = src[0];
	int8_t cb = src[1];
	int8_t y2 = src[2];
	int8_t cr = src[3];


	yuv444_to_rgb(y1, cb, cr, dst);
	yuv444_to_rgb(y2, cb, cr, dst + 3);
#if 0
	double r1 = (298.082 * y1                + 408.583 * cr) / 256 - 222.921;
	double g1 = (298.082 * y1 + 100.291 * cb - 208.120 * cr) / 256 + 135.576;
	double b1 = (298.082 * y1 + 516.412 * cb               ) / 256 - 276.836;

	double r2 = (298.082 * y1                + 408.583 * cr) / 256 - 222.921;
	double g2 = (298.082 * y1 + 100.291 * cb - 208.120 * cr) / 256 + 135.576;
	double b2 = (298.082 * y1 + 516.412 * cb               ) / 256 - 276.836;

	dst[0] = b1;
	dst[1] = g1;
	dst[2] = r1;
	dst[3] = b2;
	dst[4] = g2;
	dst[5] = r2;
#endif
}

class EyeCam {
public:
	EyeCam(std::string file);
	~EyeCam(void);

	void SetStreaming(bool streaming);

	CameraFrame &GetFrame(CameraFrame &);
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
		size_t       len;
		void        *ptr;
		v4l2_buffer *buf;
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
		struct v4l2_buffer *buffer = new v4l2_buffer;

		memset(buffer, 0, sizeof(v4l2_buffer));
		buffer->type   = req_bufs.type;
		buffer->memory = req_bufs.memory;
		buffer->index  = i;

		ret = ioctl(m_fd, VIDIOC_QUERYBUF, buffer);
		if (ret == -1) {
			throw "err: unable to memory-map buffer";
		}

		// Save the length for munmap() later.
		m_bufs[i].ptr = mmap(NULL, buffer->length, PROT_READ | PROT_WRITE,
		                     MAP_SHARED, m_fd, buffer->m.offset);
		m_bufs[i].len = buffer->length;
		m_bufs[i].buf = buffer;

		if (m_bufs[i].ptr == MAP_FAILED) {
			// XXX: Free memory with munmap() before failing.
			throw "err: memory map failed";
		}

	}

#if 0
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
#endif

	// Fetch default configuration data from the camera.
	GetParam(m_param);
	GetFormat(m_fmt_pix);
}

EyeCam::~EyeCam(void) {
	for (uint32_t i = 0; i < m_bufs.size(); ++i) {
		munmap(m_bufs[i].ptr, m_bufs[i].len);
	}
}

void EyeCam::SetStreaming(bool streaming) {
	int ret;

	// Begin by enqueing all of the buffers since they are dequeued when
	// streaming halts (and by default)
	if (streaming) {
		for (size_t i = 0; i < m_bufs.size(); ++i) {
			ret = ioctl(m_fd, VIDIOC_QBUF, m_bufs[i].buf);
			if (ret == -1) {
				throw "err: unable to enqueue memory-mapped buffer";
			}
		}
	}

	// Disabling streaming automatically clears the queue.
	uint32_t type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (streaming) {
		ret = ioctl(m_fd, VIDIOC_STREAMON, &type);
	} else {
		ret = ioctl(m_fd, VIDIOC_STREAMOFF, &type);
	}
	std::cout << "errno = " << strerror(errno) << std::endl;

	if (ret == -1) {
		throw "err: unable to enable/disable streaming";
	}
}

CameraFrame &EyeCam::GetFrame(CameraFrame &frame) {
	int ret;

	// Grab the most recent buffered frame; blocking.
	v4l2_buffer buf;

	buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;

	ret = ioctl(m_fd, VIDIOC_DQBUF, &buf);
	if (ret == -1) {
		throw "err: unable to dequeue buffer";
	}

	// Convert the Y'UV 4:2;2 image to RGB.
	// TODO: Detect the image's format dynamically.
	uint32_t width  = GetWidth();
	uint32_t height = GetHeight();
	uint32_t bpl    = m_fmt_pix.fmt.pix.bytesperline;

	frame.Update(width, height, buf.timestamp);
	uint8_t *dst_data = frame.GetDataBGR();

	for (size_t y = 0; y < height; y += 1)
	for (size_t x = 0; x < width;  x += 2) {
		uint8_t *src = (uint8_t *)(m_bufs[buf.index].ptr) + (y * bpl) + (x * 2);
		uint8_t *dst = (uint8_t *)dst_data + (y * width * 3) + (x * 3);
		yuv422_to_rgb(src, dst);
	}

	// Add the buffer back to the queue for reuse.
	ret = ioctl(m_fd, VIDIOC_QBUF, &buf);
	if (ret == -1) {
		throw "err: unable to enqueue used buffer";
	}
	return frame;
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
	// OpenCV sets errno to "no such file or directory."
	errno = 0;

	if (argc <= 3) {
		std::cerr << "err: incorrect number of arguments\n"
		          << "usage: ./stereo_webcam <device 1> <device 2> <prefix>" << std::endl;
		return 1;
	}

	try {
		EyeCam cam1(argv[1]);
		EyeCam cam2(argv[2]);
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
