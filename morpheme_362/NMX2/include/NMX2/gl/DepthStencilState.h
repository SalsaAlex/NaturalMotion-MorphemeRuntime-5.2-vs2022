
namespace nmx
{
	
namespace gl
{
	
	//sizeof(DepthStencilStateDesc) == 48
	struct DepthStencilStateDesc
	{
		void setToDefaults(void)
		{
			depthtest = true;
			depthfunctype = 1;
			depthmask = true;
			//undone
		}
		
		bool depthtest;
		int depthfunctype;
		bool depthmask;
		bool stenciltest;
		int unknown1;
		int unknown2;
		int unknown3;
		int unknown4;
		int unknown5;
		int unknown6;
		int unknown7;
		int unknown8;
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
		int m_unknown1;
		int m_unknown2;
		int m_unknown3;
		int m_unknown4;
		int m_unknown5;
		int m_unknown6;
		int m_unknown7;
		int m_unknown8;
		int m_unknown9;
		int m_unknown10;
		int m_unknown11;
		GLuint m_depthstencilstatelist;
		int m_unknown13;
		nmx::gl::Device* m_device;
		int m_unknown15;
	}
	
} // namespace gl
	
} // namespace nmx