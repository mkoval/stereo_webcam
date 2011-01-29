#ifndef CAMERAFRAMECOMPARATOR_HPP_
#define CAMERAFRAMECOMPARATOR_HPP_

#include "CameraFrame.hpp"

class CameraFrameComparator {
public:
	CameraFrameComparator(double time_err);
	int Compare(CameraFrame const &x, CameraFrame const &y) const;

private:
	timeval m_err;
};

#endif
