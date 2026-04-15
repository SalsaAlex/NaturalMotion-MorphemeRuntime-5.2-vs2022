
namespace EAT
{
	
	//sizeof(EAT::EnvData) == 156
	class EnvData
	{
	public:
		EnvData(void* pBuildStampData, 
			const wxString& productId, 
			const wxString& appDisplayName, 
			int argc, wchar_t** argv, 
			int versionmajor, int versionminor, int versionrelease, int versionrevision, 
			wxString versionSpecialString, 
			const wxString& unknown1, 
			const wxString& vendor,
			unsigned long unknown2);
			
		//
		//	getters
		//
		
		wxRegConfig* 			getAppConfig(void) 						{ return m_appConfig; }
		wxString 				getAppDisplayNameAsLegalFilename(void);
		wxString 				getAppDocsDir(void);
		wxString 				getAppExeName(void) 					{ return m_appExeName; }
		LuaPlus::LuaObject 		getAppExeNameL(void);
		wxString 				getAppExecutableDir(void) 				{ return m_appExeDir; }
		LuaPlus::LuaObject 		getAppExecutableDirL(void);
		wxString 				getAppLongDisplayName(void) 			{ return m_appLongDisplayName; }
		wxString 				getAppPluginsDir(void);
		LuaPlus::LuaObject 		getAppPluginsDirL(void);
		wxString 				getAppResourcesDir(void);
		LuaPlus::LuaObject 		getAppResourcesDirL(void);
		wxString 				getAppRootDir(void) 					{ return m_devBuildCmd ? m_unknown1 : m_appExeDir; }
		LuaPlus::LuaObject 		getAppRootDirL(void);
		wxString 				getAppScriptsDir(void);
		LuaPlus::LuaObject 		getAppScriptsDirL(void);
		EAT::BuildStampInfo* 	getBuildStampInfo(void) 				{ return m_pStampInfoBuilder; }
		wxString 				getBuildToolsDir(void) 					{ return m_buildToolsDir; }
		wxString 				getCommonDocumentsDir(void) 			{ return m_commonDocumentsDir; }
		LuaPlus::LuaObject 		getCommonDocumentsDirL(void);
		wxString 				getCommonPluginsDir(void);
		LuaPlus::LuaObject 		getCommonPluginsDirL(void);
		wxString 				getCommonUserDataDir(void) 				{ return m_commonUserDataDir; }
		LuaPlus::LuaObject 		getCommonUserDataDirL(void);
		wxString 				getConfigRootKey(void) 					{ return m_configRootKey; }
		std::wstring 			getFullCommandLine(void) 				{ return std::wstring(m_fullCommandLine.c_str()); }
		wxString 				getFullVersionString(void);
		wxString 				getLocalUserDataDir(void) 				{ return m_localUserDataDir; }
		wxString 				getPluginConfigKey(void);
		wxString 				getPluginsDir(void);
		wxString 				getProductId(void) 						{ return m_productId; }
		wxString 				getRoamingUserDataDir(void) 			{ return m_roamingUserDataDir; }
		wxString 				getRootDir(void) 						{ return m_rootDir; } 
		LuaPlus::LuaObject 		getRootDirL(void);
		wxString 				getSessionIDString(void);
		wxString 				getSignaturesDir(void);
		wxString 				getUserDocumentsDir(void) 				{ return m_userDocumentsDir; }
		
		wxString 				getVendor(void) 						{ return m_vendor; }
		
		LuaPlus::LuaObject 		getVersionLua(void);
		int 					getVersionMajor(void) 					{ return m_versionMajor; }
		int 					getVersionMinor(void) 					{ return m_versionMinor; }
		wxString 				getVersionNumber(void);
		int 					getVersionRelease(void) 				{ return m_versionRelease; }
		int 					getVersionRevision(void) 				{ return m_versionRevision; }
		wxString 				getVersionSpecialString(void) 			{ return m_versionSpecialString; }
		
		
		//
		// boolean-ers
		//
		bool isDebugBuild(void) {
#ifdef _DEBUG
			return true;
#else
			return false;
#endif
		}
		bool isDevBuildCmd(void) { return m_devBuildCmd; }
		bool isCmdLineOptSet(const wchar_t* option);
		bool isCmdLineOptSet(const wchar_t* option, long& outvalue);
		bool isCmdLineOptSet(const wchar_t* option, wxString& outvalue);
		
		
		//
		// misc
		//
		void initLua(void);
		void incShutdownCount(void)
		{
			long l1, l2, l3;
			readSessionID(l1, l2, l3);
			writeSessionID(l1, l2, l3 + 1);
		}
		void incStartupCount(void)
		{
			long l1, l2, l3;
			readSessionID(l1, l2, l3);
			writeSessionID(l1, l2 + 1, l3);
		}
		void startUptimeTimer(void) { }
		void stopUptimeTimer(void);
		void testCrash(void)
		{
			__asm {
				xor eax, eax
				mov dword ptr [eax], 1
			}
		}
		
		static uint32_t calculateUniqueMachineID(void);
		void readSessionID(long &l1, long &l2, long &l3);
		void writeSessionID(long l1, long l2, long l3);
		
		void setAppPluginsDirBaseName(const wchar_t* basename) { m_appPluginsDirBaseName = basename; }
		
		
	private:
		
		wxString m_productId; //(DWORD*)this + 1
		wxString m_appDisplayName; //(DWORD*)this + 2
		wxString m_vendor; //(DWORD*)this + 3
		wxString m_rootDir; //(DWORD*)this + 4
		wxString m_buildToolsDir; //(DWORD*)this + 5
		wxString m_appExeName; //(DWORD*)this + 6
		wxString m_appExeDir; //(DWORD*)this + 7
		wxString m_unknown1; //(DWORD*)this + 8
		wxString m_appLongDisplayName; //(DWORD*)this + 9
		wxString m_localUserDataDir; //(DWORD*)this + 10
		wxString m_roamingUserDataDir; //(DWORD*)this + 11
		wxString m_commonUserDataDir; //(DWORD*)this + 12
		wxString m_userDocumentsDir; //(DWORD*)this + 13
		wxString m_commonDocumentsDir; //(DWORD*)this + 14
		wxString m_appPluginsDirBaseName; //(DWORD*)this + 15
		int m_versionMajor; //(DWORD*)this + 16
		int m_versionMinor; //(DWORD*)this + 17
		int m_versionRelease; //(DWORD*)this + 18
		int m_versionRevision; //(DWORD*)this + 19
		wxString m_versionSpecialString; //(DWORD*)this + 20
		EAT::BuildStampInfo* m_pStampInfoBuilder; //(DWORD*)this + 21
		int m_unknown2; //(DWORD*)this + 22
		int m_unknown3; //(DWORD*)this + 23
		int m_unknown4; //(DWORD*)this + 24
		int m_unknown5; //(DWORD*)this + 25
		int m_unknown6; //(DWORD*)this + 26
		int m_unknown7; //(DWORD*)this + 27
		int m_unknown8; //(DWORD*)this + 28
		int m_unknown9; //(DWORD*)this + 29
		int m_unknown10; //(DWORD*)this + 30
		int m_unknown11; //(DWORD*)this + 31
		wxCmdLineParser* m_cmdLineParser; //(DWORD*)this + 32
		void* m_unknown12; //(DWORD*)this + 33
		wxRegConfig* m_appConfig; //(DWORD*)this + 34
		wxString m_configRootKey; //(DWORD*)this + 35
		wxString m_fullCommandLine; //(DWORD*)this + 36
		bool m_devBuildCmd; //(DWORD*)this + 37
		unsigned long m_unknown13; //(DWORD*)this + 38
			
	}
	
} // namespace EAT