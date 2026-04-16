
namespace EAT
{
	
	//
	class BuildStampParser
	{
	public:
		bool parse(void);
		
		bool readDate(NMutils::MemoryStreamReader& reader);
		bool readTime(NMutils::MemoryStreamReader& reader);
		bool readBuildHostInfo(NMutils::MemoryStreamReader& reader);
		bool readSubversionInfo(NMutils::MemoryStreamReader& reader);
		bool readBuildNumber(NMutils::MemoryStreamReader& reader);
		bool readBuildSpec(NMutils::MemoryStreamReader& reader);
		bool readBuildID(NMutils::MemoryStreamReader& reader);
		bool readSkuID(NMutils::MemoryStreamReader& reader);
		bool readLicenseModel(NMutils::MemoryStreamReader& reader);
		
		int getBuildStampSize(void);
		
	private:
		void* m_buildStampData;
		EAT::BuildStampInfo* m_pStampInfo;
		
	}
	
} // namespace EAT