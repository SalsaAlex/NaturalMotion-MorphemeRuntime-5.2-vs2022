
namespace EAT
{
	
	//
	class BuildStampInfo
	{
	public:
		BuildStampInfo();
		
		bool isValid(void);
		wxString getBuildDate(void);
		wxString getBuildSpec(void);
		wxString getBuildID(void);
		wxString getSkuID(void);
		wxString getLicenseModel(void);
		int getRevisionNumber(void);
		
	private:
	
		wxString m_buildDate; //this
		wxString m_buildTime; //(_DWORD *)this + 1)
		wxString m_buildCodeline; //(_DWORD *)this + 2)
		wxString m_buildHost; //(_DWORD *)this + 3)
		wxString m_buildSpec; //(_DWORD *)this + 4)
		wxString m_buildId; //(_DWORD *)this + 5)
		wxString m_skuId; //(_DWORD *)this + 6)
		wxString m_licenseModel; //(_DWORD *)this + 7)
		int m_unknown1; //(_DWORD *)this + 8)
		int m_revisionNum; //(_DWORD *)this + 9)
		int m_buildNum; //(_DWORD *)this + 10)
	}
	
} // namespace EAT