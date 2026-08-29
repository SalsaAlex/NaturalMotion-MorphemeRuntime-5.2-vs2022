#include "EATBuildStampInfo.h"


namespace EAT
{
	BuildStampInfo::BuildStampInfo()
	{
		m_buildDate = "Unknown date";
		m_buildTime = "Unknown time";
		m_buildCodeline = "Unknown codeline";
		m_buildHost = "Unknown build host";
		m_buildSpec = "Untyped";
		
		
		m_unknown1 = m_revisionNum = m_buildNum = -1;
	}
	
	bool BuildStampInfo::isValid(void)
	{
		return m_unknown1 >= 1;
	}
	wxString BuildStampInfo::getBuildDate(void)
	{
		return m_buildDate;
	}
	wxString BuildStampInfo::getBuildSpec(void)
	{
		return m_buildSpec;
	}
	wxString BuildStampInfo::getBuildID(void)
	{
		return m_buildId;
	}
	wxString BuildStampInfo::getSkuID(void)
	{
		return m_skuId;
	}
	wxString BuildStampInfo::getLicenseModel(void)
	{
		return m_licenseModel;
	}
	int BuildStampInfo::getRevisionNumber(void)
	{
		return m_revisionNum;
	}
	
} // namespace EAT