
namespace nmx
{
	
namespace gl
{
	
	//sizeof(SamplerStateDesc) == 20
	struct RasterStateDesc
	{
		void setToDefaults(void);
		
		float m_unknown[5];
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
		int m_unknown[8];
	}
	
} // namespace gl
	
} // namespace nmx