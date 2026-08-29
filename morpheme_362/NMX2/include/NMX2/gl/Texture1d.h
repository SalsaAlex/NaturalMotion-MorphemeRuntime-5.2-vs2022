
namespace nmx
{
	
namespace gl
{
	
	struct Texture1dDesc
	{
	}
	
	//sizeof(Texture1d) == 40
	class Texture1d
	{
	public:
		Texture1d(const nmx::gl::Texture1dDesc& desc, const nmx::Image* image, nmx::gl::Device* device);
		~Texture1d(void);
		
		void setData(const nmx::Image& image);
		
		nmx::gl::Texture1dDesc getDesc(void);
		
	private:
		GLuint m_textureid; //(char *)this + 4 | (DWORD *)this + 1
		nmx::gl::Texture1dDesc m_texturedesc; //(char *)this + 8 | (DWORD *)this + 2 (pointer ?)
		
		int m_internalformat; //(char *)this + 20 | (DWORD *)this + 5
	}
	
} // namespace gl
	
} // namespace nmx