#include "NMutils/NMutils_Time.h"

namespace NMutils
{
	Time::Unit Time::m_userInterfaceUnits = Time::Unit::UNIT_UNKNOWN0;
	double Time::m_userInterfaceFps = 1.0;
			
	void Time::setValue(double val, Unit unit, double fps)
	{
	}
	double Time::getValue(Unit unit, double fps)
	{
		return 0.0;
	}
} //namepsace NMutils