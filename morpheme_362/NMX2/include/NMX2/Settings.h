

namespace nmx
{
	//sizeof(Settings) == 8
	class Settings : public nmx::Database
	{
	public:
		Settings(const wchar_t* name);
		
		void addSettingsObserver(nmx::SettingsObserver& observer);
		void addSettingsPreObserver(nmx::SettingsPreObserver& preobserver);
		void removeSettingsObserver(nmx::SettingsObserver& observer);
		void removeSettingsPreObserver(nmx::SettingsPreObserver& preobserver);
		
		nmx::Node* getSetting(const wchar_t* settingname);
		void listSettings(nmx::StringArray& outlist);
		nmx::StringArray listSettings(void);
		
	private:
		
		
	}
	
} // namespace nmx