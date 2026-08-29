

namespace nmx
{
	class SettingsProvider
	{
	public:
		nmx::Settings* getSettingsDatabase(void);
		uint32_t getSettingsNodeTypeId(void);
		nmx::Node* internalGetSettingsNode(void);
		
		bool providesSettings(void);
		
		void registerSettings(void);
		void unregisterSettings(void);
		
	private:
		
		
	}
	
} // namespace nmx