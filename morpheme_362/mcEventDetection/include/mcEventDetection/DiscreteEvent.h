
namespace mcc
{
	class DiscreteEvent
	{
	public:
		void setStart(int start);
		void setEnd(int end);
		void setDuration(int duration);
		void setName(const std::wstring& name);
		void setUserData(int userdata);
		
		int getStart(void);
		int getEnd(void);
		int getDuration(void);
		std::wstring getName(void);
		int getUserData(void);
		
		virtual void clear(void);
		
	private:
		int m_iStart = 0;
		int m_iDuration = 0;
		std::wstring m_name;
		int m_userData = 0;
		
	}
	
	
} // namespace mcc