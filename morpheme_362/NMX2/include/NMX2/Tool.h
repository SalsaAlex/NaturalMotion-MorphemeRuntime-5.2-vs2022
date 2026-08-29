

namespace nmx
{
	
	class ToolEvent
	{
	public:
		enum WheelDirection
		{
			
		}
		enum MouseButton
		{
			NO_BUTTON = 0,
			LEFT_BUTTON = (1 << 0),
			RIGHT_BUTTON = (1 << 1),
			MIDDLE_BUTTON = (1 << 2)
		}
		
		MouseButton button(void);
		
		bool isHeld(MouseButton button);
		
		
	private:
		uint32_t m_buttonmask;
		MouseButton m_button; //button(void)
		int m_unknown3;
		WheelDirection m_wheeldir;
	}
	
	class Tool : public nmx::SettingsProvider
	{
	public:
		
	private:
		
	}
	
} // namespace nmx