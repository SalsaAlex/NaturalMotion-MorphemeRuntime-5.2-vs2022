
namespace NMutils
{
	class __declspec(dllexport) Time
	{
		enum Unit
		{
			UNIT_CUSTOM = 0,
			UNIT_UNKNOWN0,
			UNIT_UNKNOWN1,
			UNIT_UNKNOWN2,
		};

	public:
		void setValue(double val, Unit unit, double fps);
		double getValue(Unit unit, double fps);


	private:
		double m_time;

		static Unit m_userInterfaceUnits;
		static double m_userInterfaceFps;
	};
} //namepsace NMutils