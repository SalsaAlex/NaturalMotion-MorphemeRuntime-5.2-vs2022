#include "NMutils/NMUtils_NMPoint.h"

namespace NMutils
{
	NMPoint::NMPoint(void) { m_point[0] = m_point[1] = 0; }
	NMPoint::NMPoint(int x, int y) { m_point[0] = x; m_point[1] = y; }
	
	void NMPoint::set(const NMPoint& point) { m_point[0] = point.m_point[0]; m_point[1] = point.m_point[1]; }
	void NMPoint::set(int x, int y) { m_point[0] = x; m_point[1] = y; }
	
	NMPoint& NMPoint::operator = (const NMPoint& point) { m_point[0] = point.m_point[0]; m_point[1] = point.m_point[1]; return *this; }
	
	bool NMPoint::operator == (const NMPoint& point) { return m_point[0] == point.m_point[0] && m_point[1] == point.m_point[1]; }
	bool NMPoint::operator != (const NMPoint& point) { return m_point[0] != point.m_point[0] && m_point[1] != point.m_point[1]; }
} //namepsace NMutils