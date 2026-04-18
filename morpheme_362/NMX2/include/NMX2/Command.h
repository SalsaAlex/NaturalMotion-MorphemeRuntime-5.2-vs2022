

namespace nmx
{
	
	class Command : public nmx::SettingsProvider
	{
	public:
		static uint32_t ClassTypeId(void);
		
		static void Register(nmx::Module& module);
		static void Unregister(nmx::Module& module);
		
	private:
		
	}
	
} // namespace nmx