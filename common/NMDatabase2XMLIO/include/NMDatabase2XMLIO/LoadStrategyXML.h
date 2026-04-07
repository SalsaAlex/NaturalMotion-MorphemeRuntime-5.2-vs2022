
namespace db2
{
	
	class LoadStrategyXML
	{
	public:
		void initLibrary();
		
		void onParseSuccess(); //virtual ?
		
		static bool isFileReadable(const wchar_t* file);
		
		int parse(void);
	}
	
	
	
}