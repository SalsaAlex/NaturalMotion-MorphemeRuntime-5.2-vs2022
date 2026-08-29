
namespace nmx
{
	
namespace gl
{
	
	//sizeof(Font) == 12
	class Font
	{
	public:
		Font(const nmx::String& name, uint32_t fontsize, nmx::gl::Device* device);
		~Font(void);
		
		const nmx::String* getFontName(void);
		uint32_t getFontSize(void);
		int getMemoryUsage(void);
		
		void renderText(const nmx::String& text);
		
		void setColour(const nmx::Colour& color);
		void setPosition2D(float x, float y);
		void setPosition2D(int x, int y);
		void setPosition3D(const nmx::Vector3& pos);
		
	private:
		
	}
	
} // namespace gl
	
} // namespace nmx