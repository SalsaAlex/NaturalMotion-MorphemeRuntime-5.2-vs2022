
namespace db2
{
	
	void LoadStrategyXML::initLibrary()
	{
		
	}
	
	//static
	bool LoadStrategyXML::isFileReadable(const wchar_t* file)
	{
		char header[300];
		
		db2::FileInputStream fileinput(file, false);
		
		file.read(header, sizeof(header));
		
		if( strstr(header, "library=\"NMDatabase2\"") || strstr(header, "formatVersion=\"") )
			return true;

		return false;
	}
	
	int LoadStrategyXML::parse(void)
	{
		
	}
	
	
	
}