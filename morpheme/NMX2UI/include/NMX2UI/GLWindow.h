
namespace nmxui
{
	
	class GLWindow
	{
	public:
		void setCurrent(void);
		void render(void);
		
	private:
		HGLRC m_glcontext; //((HGLRC *)this + 97)
		HDC m_devicecontext; //((HDC *)this + 98)
		int m_width; //((_DWORD *)this + 99)
		int m_height; //((_DWORD *)this + 100)
		
	}
	
} //namespace nmxui