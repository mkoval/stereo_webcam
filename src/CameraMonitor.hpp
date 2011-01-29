#ifndef CAMERAMONITOR_HPP_
#define CAMERAMONITOR_HPP_

#include <boost/thread/mutex.hpp>
#include <queue>

#include "CameraFrame.hpp"
#include "Webcam.hpp"

class CameraMonitor {
public:
	CameraMonitor(Webcam &cam, size_t nbuf);
	~CameraMonitor(void);

	void operator()(void);

	bool HasFrame(void) const;
	void GetFrame(CameraFrame &);

private:
	struct CmpFramePtrs {
		bool operator()(CameraFrame const *x, CameraFrame const *y) const;
	};

	CameraMonitor(CameraMonitor const &src);
	CameraMonitor &operator=(CameraMonitor const &src);

	Webcam *const m_cam;
	std::queue<CameraFrame *> m_available;
	std::priority_queue<CameraFrame *, std::vector<CameraFrame *>, CmpFramePtrs> m_frames;
	boost::mutex m_frames_mutex;
};

#endif
