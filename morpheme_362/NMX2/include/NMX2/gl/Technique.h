
namespace nmx
{
	
namespace gl
{
	
	class Technique
	{
	public:
		bool hasNegatedMatrix(const nmx::Matrix& matrix);
		nmx::gl::Shader* makeFragmentShader(nmx::gl::Device* device, const wchar_t* glslpath);
		nmx::gl::Shader* makeVertexShader(nmx::gl::Device* device, const wchar_t* glslpath);
		nmx::gl::Shader* makeGeometryShader(nmx::gl::Device* device, const wchar_t* glslpath, uint32_t unknown1, nmx::gl::PrimitiveType primitivetype);
		
		nmx::gl::Program* makeVertexProgram(nmx::gl::Device* device, nmx::gl::Shader* vertshader, nmx::gl::Shader* fragshader, nmx::gl::Shader* geomshader);
		
	private:
		
	}
	
} // namespace gl
	
} // namespace nmx