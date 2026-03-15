
namespace NMutils
{
	class __declspec(dllexport) NMPoint2D
	{
	public:
		NMPoint2D(void);
		NMPoint2D(const NMPoint2D& point);
		NMPoint2D(float x, float y);

		void set(const NMPoint2D& point);
		void set(float x, float y);

		void zero();

		bool operator == (const NMPoint2D& point);
		bool operator != (const NMPoint2D& point);

		bool operator < (const NMPoint2D& point);
		bool operator > (const NMPoint2D& point);

		bool operator <= (const NMPoint2D& point);
		bool operator >= (const NMPoint2D& point);

		NMPoint2D& operator = (const NMPoint2D& point);

	private:
		float m_point[2];
	};
} //namepsace NMutils