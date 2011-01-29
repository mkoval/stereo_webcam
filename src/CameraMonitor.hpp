#ifndef CAMERAMONITOR_HPP_
#define CAMERAMONITOR_HPP_

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
	static bool CmpFramePtrs(CameraFrame const *x, CameraFrame const *y);

	CameraMonitor(CameraMonitor const &src);
	CameraMonitor &operator=(CameraMonitor const &src);

	Webcam *const m_cam;
	std::queue<CameraFrame *> m_available;
	std::priority_queue<CameraFrame *> m_frames; // TODO: Compare function
};

#endif
