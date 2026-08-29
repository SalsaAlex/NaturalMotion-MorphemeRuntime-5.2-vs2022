
namespace nmx
{
	
namespace gl
{
	
	class GeometryShader : public nmx::gl::Shader
	{
	public:
		GeometryShader(uint32_t unknown1, uint32_t unknown2, uint32_t unknown3, nmx::gl::Device* device);
		
		void attach(nmx::gl::Program* program);
		
	private:
		uint32_t m_unknown1;
		uint32_t m_unknown2;
		uint32_t m_unknown3;
	}
	
} // namespace gl
	
} // namespace nmx