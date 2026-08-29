
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
		
		void setBlendFactor(const float* factor); //factor = float[4]
		
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
		GLuint m_blendstatelist;
		int m_unknown11;
		nmx::gl::Device* m_device;
		int m_unknown13;
	}
	
} // namespace gl
	
} // namespace nmx