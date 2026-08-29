
namespace nmx
{
	
namespace gl
{
	
	//sizeof(DeviceStats) == 208 ? this class keeps memsetting itself with size of 208.
	class DeviceStats
	{
	public:
		DeviceStats(void);
		
		void init(void);
		
		void resetFrame(void);
		void renderText(nmx::gl::Font* font);
		
	private:
		double m_unknown[26]; // ? theres alot of QWORDs soo
	}
	
} // namespace gl
	
} // namespace nmx