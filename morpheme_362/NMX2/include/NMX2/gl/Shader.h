
namespace nmx
{
	
namespace gl
{
	
	class Shader
	{
	public:
		Shader(nmx::gl::ShaderType type, nmx::gl::Device* device);
		
	private:
		int m_unknown;
		nmx::gl::Device* m_device;
		std::string m_string1;
		std::string m_string2;
		std::wstring m_string3;
		int m_unknown2;
		nmx::gl::ShaderType m_type;
	}
	
} // namespace gl
	
} // namespace nmx