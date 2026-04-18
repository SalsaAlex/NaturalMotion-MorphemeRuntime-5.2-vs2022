
namespace nmx
{
	
namespace gl
{
	
	struct Texture2dDesc
	{
	}
	
	//sizeof(Texture2d) == 44
	class Texture2d
	{
	public:
		Texture2d(const nmx::gl::Texture2dDesc& desc, const nmx::Image* image, nmx::gl::Device* device);
		~Texture2d(void);
		
		void clear(void);
		
		void setData(const nmx::Image& image);
		
	private:
		int m_internalformat; //(char *)this + 20 | (DWORD *)this + 5
		
		GLuint m_textureid; //(char *)this + 40 | (DWORD *)this + 10
	}
	
} // namespace gl
	
} // namespace nmx