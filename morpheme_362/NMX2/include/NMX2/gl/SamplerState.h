
namespace nmx
{
	
namespace gl
{
	
	//sizeof(SamplerStateDesc) == 48
	struct SamplerStateDesc
	{
		void setToDefaults(void);
		
		float m_unknown[12];
	}
	
	//sizeof(SamplerState) == 64
	class SamplerState
	{
	public:
		SamplerState(const nmx::gl::SamplerStateDesc& desc, nmx::gl::Device* device);
		~SamplerState(void);
		
		nmx::gl::Device* getDevice(void);
		
	private:
		int m_unknown[16];
	}
	
} // namespace gl
	
} // namespace nmx