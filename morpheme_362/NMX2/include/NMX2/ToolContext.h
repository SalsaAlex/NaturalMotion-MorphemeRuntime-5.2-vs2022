

namespace nmx
{
	enum ToolContextFlags
	{
		
	}
	
	class ToolContext
	{
	public:
		ToolContext(nmx::ToolContextFlags flags);
		
		nmx::Viewport* asViewport(void);
		
		float width(void);
		float aspect(void);
		
		void setSize(uint32_t width, uint32_t height);
		void setCurrentToolByName(const wchar_t* toolname);
		
		void registerObserver(nmx::ToolObserver* observer);
		void unregisterObserver(nmx::ToolObserver* observer);
		
		void onBeginTool(void);
		void onEndTool(void);
		void onPassive(const nmx::ToolEvent& event);
		void onPress(const nmx::ToolEvent& event);
		void onRelease(const nmx::ToolEvent& event);
		void onTimer(const nmx::ToolEvent& event);
		void onWheel(const nmx::ToolEvent& event);
		void onDrag(const nmx::ToolEvent& event);
		
	private:
		uint32_t m_width;
		uint32_t m_height;
		int m_unknown1;
		int m_unknown2;
		nmx::ToolContextFlags m_createflags;
		int m_unknown3;
		int m_unknown4;
		int m_unknown5;
		nmtl::allocator* m_allocator;
		
	}
	
} // namespace nmx