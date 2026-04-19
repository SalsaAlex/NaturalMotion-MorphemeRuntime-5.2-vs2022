#include "gl/DepthStencilState.h"


namespace nmx
{
	
namespace gl
{
	
	DepthStencilState::DepthStencilState(const nmx::gl::DepthStencilStateDesc& desc, nmx::gl::Device* device)
	{
		m_device = device;
		m_depthstencilstatelist = glGenLists(1);
		
		glNewList(m_depthstencilstatelist, GL_COMPILE);
		
		if(desc.depthtest)
			glEnable(GL_DEPTH_TEST);
		else
			glDisable(GL_DEPTH_TEST);
		
		//glDepthFunc(param[*((_DWORD *)a2 + 2)]);
		glDepthMask(desc.depthmask);
		
		if(stenciltest)
			glEnable(GL_STENCIL_TEST);
		else
			glDisable(GL_STENCIL_TEST);
		
		//undone
		
		glEndList();
	}
	DepthStencilState::~DepthStencilState(void)
	{
		if(m_depthstencilstatelist)
			glDeleteLists(&m_depthstencilstatelist, 1);
	}
	
	void DepthStencilState::pop(void)
	{
		m_device->popDepthStencilState(this);
	}
	void DepthStencilState::push(void)
	{
		m_device->pushDepthStencilState(this);
	}
	
} // namespace gl
	
} // namespace nmx