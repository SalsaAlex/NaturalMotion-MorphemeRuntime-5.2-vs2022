#include "EATBuildStampParser.h"


namespace EAT
{
	bool BuildStampParser::parse()
	{
		if( *(int*)m_buildStampData == 0)
			return;
		
		NMutils::MemoryStream memstream(
			NMutils::MemoryStream::DataOwnershipMode::kReferenceDataBlock,
			m_buildStampData,
			*(int*)m_buildStampData,
			false,
			nullptr);
		NMutils::MemoryStreamReader memstream_reader(memstream);
		
		if(memstream_reader.readInt())// skip size token
			return false;
		
		while(true)
		{
			char type;
			if(memstream_reader.readChar(type))
				return false;
			
			bool readresult = false;
			
			switch(type)
			{
				case 'D': readresult = readDate(memstream_render); break;
				case 'H': readresult = readBuildHostInfo(memstream_render); break;
				case 'I': readresult = readBuildSpec(memstream_render); break;
				case 'K': readresult = readBuildID(memstream_render); break;
				case 'L': readresult = readLicenseModel(memstream_render); break;
				case 'N': readresult = readBuildNumber(memstream_render); break;
				case 'S': readresult = readSubversionInfo(memstream_render); break;
				case 'T': readresult = readTime(memstream_render); break;
				case 'U': readresult = readSkuID(memstream_render); break;
			}
			
			if ( (type == 'E') )
				return true;
			else if (!readresult)
				return false;
		}
		return true;
	}
	
	int BuildStampParser::getBuildStampSize(void)
	{
		return *(int*)m_buildStampData;
	}
	
	bool BuildStampParser::readDate(NMutils::MemoryStreamReader& reader)
	{
		std::string date;
		if(reader.readString(date))
			return false;
		
		if(m_pStampInfo)
			m_pStampInfo->m_buildDate = wxString( NMutils::stringToWstring(date) );
		
		return true;
	}
	bool BuildStampParser::readTime(NMutils::MemoryStreamReader& reader)	
	{
		std::string time;
		if(reader.readString(time))
			return false;
		
		if(m_pStampInfo)
			m_pStampInfo->m_buildTime = wxString( NMutils::stringToWstring(time) );
		
		return true;
	}
	bool BuildStampParser::readBuildHostInfo(NMutils::MemoryStreamReader& reader)	
	{
		std::string date;
		if(reader.readString(date))
			return false;
		
		if(m_pStampInfo)
			m_pStampInfo->m_buildHost = wxString( NMutils::stringToWstring(date) );
		
		return true;
	}
	bool BuildStampParser::readSubversionInfo(NMutils::MemoryStreamReader& reader)	
	{
		//undone
		return false;
	}
	bool BuildStampParser::readBuildNumber(NMutils::MemoryStreamReader& reader)	
	{
		int buildnum;
		if(reader.readInt(buildnum))
			return false;
		
		m_pStampInfo->m_buildNum = wxString( NMutils::stringToWstring(buildnum) );
		
		return true;
	}
	bool BuildStampParser::readBuildSpec(NMutils::MemoryStreamReader& reader)	
	{
		std::string buildspec;
		if(reader.readString(buildspec))
			return false;
		
		if(m_pStampInfo)
			m_pStampInfo->m_buildSpec = wxString( NMutils::stringToWstring(buildspec) );
		
		return true;
	}
	bool BuildStampParser::readBuildID(NMutils::MemoryStreamReader& reader)	
	{
		std::string buildid;
		if(reader.readString(buildid))
			return false;
		
		if(m_pStampInfo)
			m_pStampInfo->m_buildId = wxString( NMutils::stringToWstring(buildid) );
		
		return true;
	}
	bool BuildStampParser::readSkuID(NMutils::MemoryStreamReader& reader)	
	{
		std::string skuid;
		if(reader.readString(skuid))
			return false;
		
		if(m_pStampInfo)
			m_pStampInfo->m_skuId = wxString( NMutils::stringToWstring(skuid) );
		
		return true;
	}
	bool BuildStampParser::readLicenseModel(NMutils::MemoryStreamReader& reader)	
	{
		std::string licensemodel;
		if(reader.readString(licensemodel))
			return false;
		
		if(m_pStampInfo)
			m_pStampInfo->m_licenseModel = wxString( NMutils::stringToWstring(licensemodel) );
		
		return true;
	}
	
} // namespace EAT