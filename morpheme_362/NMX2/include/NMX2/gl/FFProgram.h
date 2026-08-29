
namespace nmx
{
	
namespace gl
{

	//sizeof(FFProgram) == 72 (check this)
	class FFProgram
	{
	public:
		void bind(void);
		void unbind(void);
		
		void enableLighting(bool enable);
		
		float getShininess(void);
		nmx::Colour getSpecular(void);
		
		void setAmbient(const nmx::Colour& colour);
		void setDiffuse(const nmx::Colour& colour);
		void setSpecular(const nmx::Colour& colour);
		void setShininess(float shine);
		

	private:
		nmx::Colour m_ambient = nmx::Colour(0.2, 0.2, 0.2, 1.0);
		nmx::Colour m_diffuse = nmx::Colour(0.6, 0.6, 0.6, 1.0);
		nmx::Colour m_specular = nmx::Colour(0.0, 0.0, 0.0, 1.0);
		nmx::Colour m_emission = nmx::Colour(0.0, 0.0, 0.0, 0.0);
		float m_shininess = 10.f;
		bool m_lighting = false;
	}
	
} // namespace gl
	
} // namespace nmx