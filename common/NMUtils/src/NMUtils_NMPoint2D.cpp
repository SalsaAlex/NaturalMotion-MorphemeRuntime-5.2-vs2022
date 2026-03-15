#include "NMutils/NMUtils_NMPoint2D.h"

namespace NMutils
{
	NMPoint2D::NMPoint2D(void)						{ m_point[0] =						m_point[1] = 0.0; }
	NMPoint2D::NMPoint2D(const NMPoint2D& point)	{ m_point[0] = point.m_point[0];	m_point[1] = point.m_point[1]; }
	NMPoint2D::NMPoint2D(float x, float y)			{ m_point[0] = x;					m_point[1] = y; }
	
	void NMPoint2D::set(const NMPoint2D& point)		{ m_point[0] = point.m_point[0];	m_point[1] = point.m_point[1]; }
	void NMPoint2D::set(float x, float y)			{ m_point[0] = x;					m_point[1] = y; }
	
	void NMPoint2D::zero() { m_point[0] = 0; m_point[1] = 0; }
	
	bool NMPoint2D::operator == (const NMPoint2D& point)	{ return m_point[0] == point.m_point[0] && m_point[1] == point.m_point[1]; }
	bool NMPoint2D::operator != (const NMPoint2D& point)	{ return m_point[0] != point.m_point[0] && m_point[1] != point.m_point[1]; }
	
	bool NMPoint2D::operator < (const NMPoint2D& point)		{ return m_point[0] > point.m_point[0] && m_point[1] > point.m_point[1]; }
	bool NMPoint2D::operator > (const NMPoint2D& point)		{ return m_point[0] < point.m_point[0] && m_point[1] < point.m_point[1]; }
	
	bool NMPoint2D::operator <= (const NMPoint2D& point)	{ return m_point[0] <= point.m_point[0] && m_point[1] <= point.m_point[1]; }
	bool NMPoint2D::operator >= (const NMPoint2D& point)	{ return m_point[0] >= point.m_point[0] && m_point[1] >= point.m_point[1]; }
	
	NMPoint2D& NMPoint2D::operator = (const NMPoint2D& point) { m_point[0] = point.m_point[0]; m_point[1] = point.m_point[1]; return *this; }
} //namepsace NMutils