#include "Webcam.hpp"

#include <cstring>
#include <exception>
#include <list>
#include <stdexcept>
#include <string>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>

#include <sys/poll.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#include <linux/videodev2.h>

static int clamp(double x)
{
	if (x < 0)
		return 0;
	else if (x > 250)
		return 255;
	else
		return x;
}

#define Y_ADJ(y1) ((255 / 219.0) * (y1 - 16))
static void yuv444_to_rgb(uint8_t Y1, uint8_t Cb, uint8_t Cr, uint8_t *dst)
{
	double pb = Cb - 128;
	double pr = Cr - 128;

	double r = Y_ADJ(Y1) + 255/112.0 * 0.701 * pr;
	double g = Y_ADJ(Y1) - 255/112.0 * 0.886 * 0.114/0.587 * pb
	                     - 255/112.0 * 0.701 * 0.299/0.587 * pr;
	double b = Y_ADJ(Y1) + 255/112.0 * 0.886 * pb;

	dst[0] = clamp(b);
	dst[1] = clamp(g);
	dst[2] = clamp(r);
}

// Consumes 6 bytes of dst and 4 bytes of src.
static void yuv422_to_rgb(uint8_t const *src, uint8_t *dst)
{
	int8_t y1 = src[0];
	int8_t cb = src[1];
	int8_t y2 = src[2];
	int8_t cr = src[3];

	yuv444_to_rgb(y1, cb, cr, dst + 0);
	yuv444_to_rgb(y2, cb, cr, dst + 3);
}

Webcam::Webcam(std::string file, size_t nbufs)
	: m_streaming(false),
	  m_nbufs(nbufs),
	  m_bufs(nbufs)
{
	// Access mode is O_RDWR instead of as specified in the V4L documentation.
	m_fd = open(file.c_str(), O_RDWR);
	if (m_fd == -1) {
		throw std::invalid_argument("unable to open device");
	}

	// Initialize the buffers to an invalid state.
	// TODO: Move this to the Webcam::Buffer constructor.
	for (size_t i = 0; i < nbufs; ++i) {
		m_bufs[i].len = 0;
		m_bufs[i].ptr = NULL;
		m_bufs[i].buf = NULL;
	}

	// Read the default device parameters for parameter negotiation.
	GetParam(m_param);
	GetFormat(m_fmt_pix);
}

Webcam::Webcam(Webcam const &src)
{
	throw std::runtime_error("copying is disabled");
}

Webcam &Webcam::operator=(Webcam const &src)
{
	throw std::runtime_error("assignment is disabled");
}

Webcam::~Webcam(void) {
	for (size_t i = 0; i < m_bufs.size(); ++i) {
		DeallocateBuffer(i);
	}
}

void Webcam::SetStreaming(bool streaming) {
	uint32_t type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	int ret;

	m_streaming = streaming;

	// TODO: Only reallocate buffers if the size has changed.

	// Allocate and enqueue the buffers necessary for memory-mapped IO.
	if (streaming) {
		// Request the appropriate number of buffers or mmap IO.
		v4l2_requestbuffers req_bufs;
		req_bufs.count  = m_nbufs;
		req_bufs.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		req_bufs.memory = V4L2_MEMORY_MMAP;
		ret = ioctl(m_fd, VIDIOC_REQBUFS, &req_bufs);
		if (ret == -1 || req_bufs.count < m_nbufs) {
			throw std::runtime_error("unable to allocate memory-mapped buffer");
		}

		// Allocate memory-mapped buffers in the kernel using V4L.
		size_t i;
		try {
			for (i = 0; i < m_nbufs; ++i) {
				m_bufs[i].ptr = NULL;
				AllocateBuffer(i);
			}
		} catch (std::runtime_error &err) {
			for (size_t j = 0; j < i; ++j) {
				DeallocateBuffer(j);
			}
			throw err;
		}

		// Enqueue the freshly allocated buffers.
		for (size_t i = 0; i < m_bufs.size(); ++i) {
			ret = ioctl(m_fd, VIDIOC_QBUF, m_bufs[i].buf);
			if (ret == -1) {
				throw std::runtime_error("unable to enqueue empty buffer");
			}
		}

		ret = ioctl(m_fd, VIDIOC_STREAMON, &type);
		if (ret == -1) {
			throw std::runtime_error("unable to enable streaming");
		}
	}
	// Dequeue (automatic) and deallocate the buffers.
	else {
		ret = ioctl(m_fd, VIDIOC_STREAMOFF, &type);
		if (ret == -1) {
			throw std::runtime_error("unable to disable streaming");
		}

		// Deallocate the buffers.
		for (size_t i = 0; i < m_bufs.size(); ++i) {
			DeallocateBuffer(i);
		}
	}
}

CameraFrame &Webcam::GetFrame(CameraFrame &frame) {
	int ret;

	// Grab the most recent buffered frame; blocking.
	v4l2_buffer buf;

	buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;

	ret = ioctl(m_fd, VIDIOC_DQBUF, &buf);
	if (ret == -1) {
		throw std::runtime_error("unable to dequeue image frame");
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
		throw std::runtime_error("unable to re-enqueue image frame");
	}
	return frame;
}

void Webcam::SetFPS(uint32_t fps) {
	m_param.parm.capture.timeperframe.numerator   = 1;
	m_param.parm.capture.timeperframe.denominator = fps;
	SetParam(m_param);

	uint32_t fps_new = m_param.parm.capture.timeperframe.denominator
	                 / m_param.parm.capture.timeperframe.numerator;
	if (fps != fps_new) {
		throw std::invalid_argument("unsupported frame rate");
	}
}

double Webcam::GetFPS(void) const {
	// TODO: Verify that denom is not zero.
	double numer = m_param.parm.capture.timeperframe.numerator;
	double denom = m_param.parm.capture.timeperframe.denominator;
	return denom / numer;
}

uint32_t Webcam::GetWidth(void) const {
	return m_fmt_pix.fmt.pix.width;
}

uint32_t Webcam::GetHeight(void) const {
	return m_fmt_pix.fmt.pix.height;
}

void Webcam::SetResolution(uint32_t width, uint32_t height) {
	if (m_streaming) {
		throw std::runtime_error("must disable streaming to change resolution");
	}
	// TODO: Disable this function while streaming is enabled.

	m_fmt_pix.fmt.pix.width  = width;
	m_fmt_pix.fmt.pix.height = height;
	SetFormat(m_fmt_pix);

	if (m_fmt_pix.fmt.pix.width != width || m_fmt_pix.fmt.pix.height != height) {
		throw std::invalid_argument("unsupported resolution");
	}
}

void Webcam::WaitForFrame(int to_ms) const {
	struct pollfd fds = { m_fd, POLLIN };
	poll(&fds, 1, to_ms);
}

void Webcam::GetParam(v4l2_streamparm &param) {
	param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	int ret = ioctl(m_fd, VIDIOC_G_PARM, &param);

	if (ret == -1) {
		throw std::runtime_error("unable to fetch device parameters");
	}
}

void Webcam::SetParam(v4l2_streamparm &param) {
	int ret = ioctl(m_fd, VIDIOC_S_PARM, &param);

	if (ret == -1) {
		throw std::runtime_error("unable to change device parameters");
	}
}

void Webcam::GetFormat(v4l2_format &fmt) {
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	int ret = ioctl(m_fd, VIDIOC_G_FMT, &fmt);

	if (ret == -1) {
		throw std::runtime_error("unable to fetch image format");
	}
}

void Webcam::SetFormat(v4l2_format &fmt) {
	int ret;

	ret = ioctl(m_fd, VIDIOC_S_FMT, &fmt);
	if (ret == -1) {
		throw std::runtime_error("unable to negotiate image format");
	}

	ret = ioctl(m_fd, VIDIOC_S_FMT, &fmt);
	if (ret == -1) {
		throw std::runtime_error("unable to change image format");
	}
}

std::list<uint32_t> Webcam::GetPixelFormats(void) const {
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
			throw std::runtime_error("automatic image format detection failed");
		}

		++it.index;
	}
	return formats;
}

std::list<Webcam::Resolution> Webcam::GetResolutions(uint32_t pixel_format) const {
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
			throw std::runtime_error("automatic frame size detection failed");
		}

		++it.index;
	}
	return resolutions;
}

std::list<double> Webcam::GetFPSs(uint32_t pixel_format, Resolution res) const {
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
			throw std::runtime_error("automatic FPS detection failed");
		}
		++it.index;
	}
	return fpss;
}

void Webcam::AllocateBuffer(size_t index)
{
	int ret;
	if (m_bufs[index].ptr != NULL) return;

	// Request a new buffer from V4L.
	v4l2_buffer *buffer = new v4l2_buffer;
	buffer->type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buffer->memory = V4L2_MEMORY_MMAP;
	buffer->index  = index;

	ret = ioctl(m_fd, VIDIOC_QUERYBUF, buffer);
	if (ret == -1) {
		delete buffer;

		switch (errno) {
		case EINVAL:
			throw std::runtime_error("driver does not support memory-mapped IO");

		default:
			throw std::runtime_error("unable to enqueue memory-mapped buffer");
		}
	}

	// Use the information returned by V4L to memmap the kernel's buffer.
	m_bufs[index].ptr = mmap(NULL, buffer->length, PROT_READ | PROT_WRITE,
	                          MAP_SHARED, m_fd, buffer->m.offset);
	m_bufs[index].len = buffer->length;
	m_bufs[index].buf = buffer;

	if (m_bufs[index].ptr == MAP_FAILED) {
		delete buffer;
		throw std::runtime_error("unable to memory map frame buffer");
	}
}

void Webcam::DeallocateBuffer(size_t index)
{
	if (m_bufs[index].ptr == NULL) return;

	delete m_bufs[index].buf;
	munmap(m_bufs[index].ptr, m_bufs[index].len);
	m_bufs[index].len = 0;
	m_bufs[index].buf = NULL;
	m_bufs[index].ptr = NULL;
}
