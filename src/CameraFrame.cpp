#include "CameraFrame.hpp"

#include <cstring>
#include <stdint.h>
#include <sys/time.h>

CameraFrame::CameraFrame(void)
	: m_width(0),
	  m_height(0),
	  m_length(0),
	  m_data(NULL)
{
	m_time.tv_sec  = 0;
	m_time.tv_usec = 0;
}

CameraFrame::CameraFrame(CameraFrame const &src)
	: m_width(src.m_width),
	  m_height(src.m_height),
	  m_length(m_width * m_height * 3),
	  m_data(new uint8_t[m_length]),
	  m_time(src.m_time)
{
	memcpy(m_data, src.m_data, m_width * m_height);
}

CameraFrame::~CameraFrame(void)
{
	delete[] m_data;
}

CameraFrame &CameraFrame::operator=(CameraFrame const &src)
{
	Resize(src.m_width, src.m_height);
	memcpy(m_data, src.m_data, src.m_width * src.m_height * 3);
	m_time = src.m_time;
	return *this;
}

bool CameraFrame::IsValid(void) const
{
	return m_data != NULL;
}

uint32_t CameraFrame::GetWidth(void) const
{
	return m_width;
}

uint32_t CameraFrame::GetHeight(void) const
{
	return m_height;
}

uint8_t *CameraFrame::GetDataBGR(void)
{
	return m_data;
}

uint8_t const *CameraFrame::GetDataBGR(void) const
{
	return m_data;
}

timeval CameraFrame::GetTimestamp(void) const
{
	return m_time;
}

void CameraFrame::Update(uint32_t width, uint32_t height, timeval time)
{
	Resize(width, height);
	m_time = time;
}

void CameraFrame::Resize(uint32_t width, uint32_t height)
{
	if (width * height * 3 > m_length) {
		delete[] m_data;
		m_length = width * height * 3;
		m_data   = new uint8_t[m_length];
	}

	m_width  = width;
	m_height = height;
}

