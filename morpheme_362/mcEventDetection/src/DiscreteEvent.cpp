#include "DiscreteEvent.h"

void DiscreteEvent::setStart(int start)
{
	m_iStart = start;
}
void DiscreteEvent::setEnd(int end)
{
	m_iDuration = end - m_iStart;
}
void DiscreteEvent::setDuration(int duration)
{
	m_iDuration = duration;
}
void DiscreteEvent::setName(const std::wstring& name)
{
	m_name = name;
}
void DiscreteEvent::setUserData(int userdata)
{
	m_userData = userdata;
}

int DiscreteEvent::getStart(void)
{
	return m_iStart;
}
int DiscreteEvent::getEnd(void)
{
	return m_iDuration - m_iStart;
}
int DiscreteEvent::getDuration(void)
{
	return m_iDuration;
}
std::wstring DiscreteEvent::getName(void)
{
	return m_name;
}
int DiscreteEvent::getUserData(void)
{
	return m_userData;
}

virtual void DiscreteEvent::clear(void)
{
	m_iStart = 0;
	m_iDuration = 0;
	m_iUserData = 0;
	m_name.clear();
}