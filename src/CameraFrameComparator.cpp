#include "CameraFrameComparator.hpp"

#include <iomanip>
#include <iostream>
static void PrintTime(timeval time) {
	std::cout << time.tv_sec << '.'
	          << std::setw(6) << std::setfill('0') << time.tv_usec;
}

CameraFrameComparator::CameraFrameComparator(double err_time)
{
	m_err.tv_sec  = (time_t)err_time;
	m_err.tv_usec = (time_t)((err_time - (time_t)err_time) * 1.0e6);
}

timeval CameraFrameComparator::GetTimeDelta(timeval x, timeval y)
{
	timeval delta;
	if (timercmp(&x, &y, >)) {
		timersub(&x, &y, &delta);
	} else {
		timersub(&y, &x, &delta);
	}
	return delta;
}

int CameraFrameComparator::Compare(CameraFrame const &x, CameraFrame const &y) const
{
	timeval time_x = x.GetTimestamp();
	timeval time_y = y.GetTimestamp();
	timeval diff   = { 0, 0 };

	// Calculate the magnitude of error between the frames' timestamps.
	bool x_bigger = timercmp(&time_x, &time_y, >);
	if (x_bigger) {
		timersub(&time_x, &time_y, &diff);
	} else {
		timersub(&time_y, &time_x, &diff);
	}

	if (timercmp(&diff, &m_err, <))
		return 0;
	else if (x_bigger)
		return +1;
	else
		return -1;

}
