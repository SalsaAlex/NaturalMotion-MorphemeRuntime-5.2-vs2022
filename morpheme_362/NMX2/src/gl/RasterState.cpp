#include "gl/DepthStencilState.h"


namespace nmx
{
	
namespace gl
{
	
	RasterState::RasterState(const nmx::gl::RasterStateDesc& desc, nmx::gl::Device* device)
	{
		m_device = device;
		m_rasterstatelist = glGenLists(1);
		
		glNewList(m_rasterstatelist, GL_COMPILE);
		
		if(desc.polyfill == 1)
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		else
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		
		//undone
		
		glEndList();
	}
	RasterState::~RasterState(void)
	{
		glDeleteLists(&m_rasterstatelist, 1);
	}
	
	void RasterState::pop(void)
	{
		m_device->popRasterState(this);
	}
	void RasterState::push(void)
	{
		m_device->pushRasterState(this);
	}
	
} // namespace gl
	
} // namespace nmx