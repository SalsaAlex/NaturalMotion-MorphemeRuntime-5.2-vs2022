
namespace nmx
{
	
namespace gl
{
	
	struct Texture3dDesc
	{
	}
	
	//sizeof(Texture3d) == 48
	class Texture3d
	{
	public:
		Texture3d(const nmx::gl::Texture3dDesc& desc, const nmx::Image* image, nmx::gl::Device* device);
		~Texture3d(void);
		
		void clear(void);
		
		void setData(const nmx::Image& image);
		
	private:
		int m_internalformat; //(char *)this + 20 | (DWORD *)this + 5
		
		GLuint m_textureid; //(char *)this + 40 | (DWORD *)this + 10
	}
	
} // namespace gl
	
} // namespace nmx