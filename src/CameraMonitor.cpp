#include "CameraMonitor.hpp"

#include <boost/thread/mutex.hpp>
#include <queue>
#include <stdexcept>

#include "CameraFrame.hpp"
#include "Webcam.hpp"

CameraMonitor::CameraMonitor(Webcam &cam, size_t nbuf)
	: m_cam(&cam)
{
	for (size_t i = 0; i < nbuf; ++i) {
		m_available.push(new CameraFrame);
	}
}

CameraMonitor::~CameraMonitor(void)
{
	// Clean up buffers that are currently unused.
	while (!m_available.empty()) {
		delete m_available.front();
		m_available.pop();
	}

	// Remove the current contents of the priority queue.
	while (!m_frames.empty()) {
		delete m_frames.top();
		m_frames.pop();
	}
}

void CameraMonitor::operator()(void)
{
	for (;;) {
		m_cam->WaitForFrame(-1);
		boost::mutex::scoped_lock lock(m_frames_mutex);

		// Force the priority queue to act like a ring buffer by throwing away
		// the oldest element when it is full.

		if (m_available.empty()) {
			m_available.push(m_frames.top());
			m_frames.pop();
		}

		// Add the latest frame to the priority queue.
		CameraFrame *frame = m_available.front();
		m_available.pop();

		m_cam->GetFrame(*frame);
		m_frames.push(frame);

		// Notify consumers that a frame is available.
		lock.unlock();
		m_frames_cond.notify_one();
	}
}

void CameraMonitor::GetFrame(CameraFrame &frame)
{
	boost::mutex::scoped_lock lock(m_frames_mutex);

	// Wait for a frame to become available using a condition variable.
	while (m_frames.empty()) {
		m_frames_cond.wait(lock);
	}

	CameraFrame *frame_new = m_frames.top();
	m_frames.pop();
	m_available.push(frame_new);
	frame = *frame_new;
}

CameraMonitor::CameraMonitor(CameraMonitor const &src)
	: m_cam(NULL)
{
	throw std::runtime_error("copying is not permitted");
}

CameraMonitor &CameraMonitor::operator=(CameraMonitor const &src)
{
	throw std::runtime_error("assignment is not permitted");
}

bool CameraMonitor::CmpFramePtrs::operator()(CameraFrame const *x, CameraFrame const *y) const
{
	// Sort by timestamp in ascending order.
	timeval time_x = x->GetTimestamp();
	timeval time_y = y->GetTimestamp();
	return timercmp(&time_x, &time_y, >);
}
