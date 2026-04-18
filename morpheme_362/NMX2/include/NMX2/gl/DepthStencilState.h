
namespace nmx
{
	
namespace gl
{
	
	//sizeof(DepthStencilStateDesc) == 48
	struct DepthStencilStateDesc
	{
		void setToDefaults(void);
		
		float m_unknown[12];
	}
	
	//sizeof(DepthStencilState) == 60
	class DepthStencilState
	{
	public:
		DepthStencilState(const nmx::gl::DepthStencilStateDesc& desc, nmx::gl::Device* device);
		~DepthStencilState(void);
		
		void pop(void);
		void push(void);
		
	private:
		int m_unknown[15];
	}
	
} // namespace gl
	
} // namespace nmx