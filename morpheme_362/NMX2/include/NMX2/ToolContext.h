

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
		
		void setCurrentToolByName(const wchar_t* toolname);
		
		void registerObserver(nmx::ToolObserver* observer);
		void unregisterObserver(nmx::ToolObserver* observer);
		
		virtual bool isViewport(void);
		
		virtual void onBeginTool(void);
		virtual void onEndTool(void);
		virtual void onPress(const nmx::ToolEvent& event);
		virtual void onRelease(const nmx::ToolEvent& event);
		virtual void onDrag(const nmx::ToolEvent& event);
		virtual void onWheel(const nmx::ToolEvent& event);
		virtual void onTimer(const nmx::ToolEvent& event);
		virtual void onPassive(const nmx::ToolEvent& event);
		
		virtual void setSize(uint32_t width, uint32_t height);
		
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