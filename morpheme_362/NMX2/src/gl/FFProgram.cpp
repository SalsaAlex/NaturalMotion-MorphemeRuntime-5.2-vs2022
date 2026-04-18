#include "FFProgram.h"


namespace nmx
{
	
namespace gl
{
	void FFProgram::bind(void)
	{
		glColor4fv(m_diffuse);
		
		if(m_lighting)
		{
			glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, m_shininess);
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, m_ambient);
			glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, m_diffuse);
			glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, m_specular);
			glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, m_emission);
			glEnable(GL_LIGHTING);
			glEnable(GL_LIGHT0);
			glEnable(GL_NORMALIZE);
		}
	}
	void FFProgram::unbind(void)
	{
		if(m_lighting)
		{
			glDisable(GL_LIGHTING);
			glDisable(GL_LIGHT0);
			glDisable(GL_NORMALIZE);
		}
	}
	
	void FFProgram::enableLighting(bool enable)
	{
		m_lighting = enable;
	}
	
	float FFProgram::getShininess(void)
	{
		return m_shininess;
	}
	nmx::Colour FFProgram::getSpecular(void)
	{
		return m_specular;
	}
	
	void FFProgram::setAmbient(const nmx::Colour& colour)
	{
		m_ambient = colour;
	}
	void FFProgram::setDiffuse(const nmx::Colour& colour)
	{
		m_diffuse = colour;
	}
	void FFProgram::setSpecular(const nmx::Colour& colour)
	{
		m_specular = colour
	}
	void FFProgram::setShininess(float shine)
	{
		m_shininess = shine;
	}
	
} // namespace gl
	
} // namespace nmx