#ifndef TIMECONVERTER_HPP_
#define TIMECONVERTER_HPP_

#include <ros/ros.h>
#include <sys/time.h>

class TimeConverter {
public:
	TimeConverter(timeval time_sys, ros::Time time_ros);
	ros::Time SysToROS(timeval time_sys) const;
private:
	timeval   const m_sys;
	ros::Time const m_ros;
};

#endif
