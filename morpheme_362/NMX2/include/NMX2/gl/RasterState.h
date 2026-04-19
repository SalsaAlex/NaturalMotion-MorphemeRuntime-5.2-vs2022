
namespace nmx
{
	
namespace gl
{
	
	//sizeof(SamplerStateDesc) == 20
	struct RasterStateDesc
	{
		void setToDefaults(void)
		{
			polyfill = 1;
			unknown2 = 0;
			unknown3 = 1;
			unknown4 = 0.0f;
			unknown5 = true;
			unknown6 = false;
			unknown7 = false;
		}
		
		int polyfill;
		int unknown2;
		int unknown3;
		float unknown4;
		bool unknown5;
		bool unknown6;
		bool unknown7;
	}
	
	//sizeof(RasterState) == 32
	class RasterState
	{
	public:
		RasterState(const nmx::gl::RasterStateDesc& desc, nmx::gl::Device* device);
		~RasterState(void);
		
		void setScissorRect( int x, int y, int width, int height);
		
		void push(void);
		void pop(void);
		
	private:
		int m_unknown1;
		int m_unknown2;
		int m_unknown3;
		int m_unknown4;
		GLuint m_rasterstatelist;
		int m_unknown6;
		nmx::gl::Device* m_device;
	}
	
} // namespace gl
	
} // namespace nmx