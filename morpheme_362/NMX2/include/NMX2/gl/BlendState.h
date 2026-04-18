
namespace nmx
{
	
namespace gl
{
	
	//sizeof(BlendStateDesc) == 40
	struct BlendStateDesc
	{
		void setToDefaults(void);
		
		float m_unknown[10];
	}
	
	//sizeof(BlendState) == 52
	class BlendState
	{
	public:
		BlendState(const nmx::gl::BlendStateDesc& desc, nmx::gl::Device* device);
		~BlendState(void);
		
		void setBlendFactor(const float* factor) //factor = float[4]
		
		void pop(void);
		void push(void);
		
	private:
		int m_unknown[13];
	}
	
} // namespace gl
	
} // namespace nmx