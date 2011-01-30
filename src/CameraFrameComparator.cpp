#include "CameraFrameComparator.hpp"

#include <iostream>
CameraFrameComparator::CameraFrameComparator(double err_time)
{
	m_err.tv_sec  = (time_t)err_time;
	m_err.tv_usec = (time_t)((err_time - (int)err_time) * 1.0e6);

	std::cout << "dSeconds      = " << m_err.tv_sec << std::endl;
	std::cout << "dMicroSeconds = " << m_err.tv_usec << std::endl;
}

int CameraFrameComparator::Compare(CameraFrame const &x, CameraFrame const &y) const
{
	timeval time_x = x.GetTimestamp();
	timeval time_y = y.GetTimestamp();
	timeval diff   = { 0, 0 };

	// Calculate the magnitude of error between the frames' timestamps.
	if (timercmp(&time_x, &time_y, >)) {
		timersub(&time_x, &time_y, &diff);
	} else if (timercmp(&time_x, &time_y, >)) {
		timersub(&time_y, &time_x, &diff);
	}

	if      (timercmp(&diff, &m_err, <))    { std::cout << "=" << std::endl;  return  0; }
	else if (timercmp(&time_x, &time_y, <)) {  std::cout << "<" << std::endl; return -1; }
	else                                    { std::cout << ">" << std::endl;  return +1; }
}
