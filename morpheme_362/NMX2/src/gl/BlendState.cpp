#include "gl/BlendState.h"

namespace nmx
{
	
namespace gl
{
	
	BlendState::BlendState(const nmx::gl::BlendStateDesc& desc, nmx::gl::Device* device)
	{
		m_device = device;
		m_blendstatelist = glGenLists(1);
		
		glNewList(m_blendstatelist, GL_COMPILE);
		if(desc.m_unknown[2])
		{
			//undone
		}
		else
		{
			glDisable(GL_BLEND);
		}
		
		glEndList();
	}
	BlendState::~BlendState(void)
	{
		glDeleteLists(&m_blendstatelist, 1);
	}
		
	void BlendState::setBlendFactor(const float* factor)
	{
		if(!m_device->getCapabilities()->hasSecondaryColour())
			return;
		
		nmx::gl::getSharedGlewContext()->unknownfunc(factor[0], factor[1], factor[2], factor[3]);
	}
	
	void BlendState::pop(void)
	{
		m_device->popBlendState(this);
	}
	void BlendState::push(void)
	{
		m_device->pushBlendState(this);
	}
		
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