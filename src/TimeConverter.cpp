#include "TimeConverter.hpp"

TimeConverter::TimeConverter(timeval time_sys, ros::Time time_ros)
	: m_sys(time_sys),
	  m_ros(time_ros)
{}

ros::Time TimeConverter::SysToROS(timeval time_sys) const
{
	timeval delta_sys;
	timersub(&time_sys, &m_sys, &delta_sys);
	return m_ros + ros::Duration(delta_sys.tv_sec, delta_sys.tv_usec * 1000);
}
