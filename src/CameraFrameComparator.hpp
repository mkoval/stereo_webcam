#ifndef CAMERAFRAMECOMPARATOR_HPP_
#define CAMERAFRAMECOMPARATOR_HPP_

#include "CameraFrame.hpp"

class CameraFrameComparator {
public:
	CameraFrameComparator(double time_err);
	int Compare(CameraFrame const &x, CameraFrame const &y) const;

	static timeval GetTimeDelta(timeval x, timeval y);

private:
	timeval m_err;
};

#endif
