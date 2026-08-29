
namespace NMutils
{
	class __declspec(dllexport) NMPoint
	{
	public:
		NMPoint(void);
		NMPoint(int x, int y);

		void set(const NMPoint& point);
		void set(int x, int y);

		NMPoint& operator = (const NMPoint& point);

		bool operator == (const NMPoint& point);
		bool operator != (const NMPoint& point);

	private:
		int m_point[2];
	};
} //namepsace NMutils