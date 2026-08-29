
namespace nmx
{
	
namespace gl
{
	struct VertexElementDesc
	{
		int unknown[4];
	}
	
	//sizeof(VertexLayout) == 12
	class VertexLayout
	{
	public:
		VertexLayout(const nmx::gl::VertexElementDesc* desc, uint32_t numelements, nmx::gl::Program* shaderprogram, nmx::gl::Device* device);
		
		void bind(const nmx::gl::Program* shaderprogram, uint32_t offset, uint32_t unused, const nmx::gl::VertexBuffer** vertexbuffers, uint32_t *unknown1, uint32_t *unknown2);
		void unbind(const nmx::gl::Program* shaderprogram);
		
		int getMemoryUsage(void);
		
	private:
		
	}
	
} // namespace gl
	
} // namespace nmx