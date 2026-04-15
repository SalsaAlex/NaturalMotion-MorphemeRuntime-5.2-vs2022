#include "mcApp.h"

static bool ShowSplashImage(const wxString& splashimage_path, int waittime) // waittime is in miliseconds
{
	if( !EAT::gEAT.m_pEnvData->isDevBuild() &&  !EAT::gEAT.m_pEnvData->isCmdLineOptSet("nosplash") )
	{
		wxBitmap *splashimage = new wxBitmap();
		wxString fullpath = EAT::gEAT.m_pEnvData->getAppResourcesDir() + splashimage_path;
		
		bool gotimage = false;
		
		if ( wxFile::Exists(fullpath.c_str()) )
		{
			if( splashimage->LoadFile(fullpath) )
				gotimage = true;
		}
		if(gotimage)
		{
			// ??
			nmui::NMSplash* splashwindow = new nmui::NMSplash(splashimage, waittime, nullptr, -1, wxDefaultPosition, wxDefaultSize, (0x2000000 | 0x2), &g_pNMSplash);
			mcc::g_pNMSplash = splashwindow;
		}
	}
	return 1;
}

static void SetNMSplashStatusLabel(const wxString& label)
{
	if( g_pNMSplash)
		g_pNMSplash->setSplashStatusLabel(label);
}

namespace mcc
{


bool morphemeConnectApp::init()
{
	if(!m_member1->unknownfunc1(this))
	{
		MessageBox(nullptr, "A serious internal error has occurred.\nThe application cannot start.", "Internal Error", MB_ICONERROR);
		return false;
	}
	
	wxCmdLineParser cmdline_parser;
	cmdline_parser.Init();
	cmdline_parser.SetCmdLine(m_argc, m_argv);
	
	cmdline_parser.AddOption(
		"openFile",
		"openFile",
		"Opens the specified mcn file.",
	);
	cmdline_parser.AddSwitch(
		"nogui",
		"nogui",
		"Disables the user interface. Use in conjunction with -script."
	);
	cmdline_parser.AddOption(
		"script",
		"script",
		"Executes the specified script and then exits.",
	);
	cmdline_parser.AddOption(
		"log",
		"log",
		"Writes the application log to the specified location.",
	);
	
	if(cmdline_parser.Found("nogui")
	{
		m_bNoGUI = true;
	}
	else
	{
		mcc::gUIServices = m_pUIServices = new mcc::AppUIServices();
		g_nmxui_uiservices = new nmxui::UIServices();
		ShowSplashImage("splash.png", 3000);
	}
	
	if(!m_member1->unknownfunc2(this))
	{
		MessageBox(nullptr, "A serious internal error has occurred.\nThe application cannot start.", "Internal Error", MB_ICONERROR);
		return false;
	}
	
	EAT::gEAT.m_pEnvData->incStartupCount();
	mcu::logInfof("SessionID: %s", EAT::gEAT.m_pEnvData->getSessionIDString().c_str());
	
	char datebuffer[128];
	__time64_t sourceTime;
	tm* localtime = localtime64(&sourceTime);
	wcsftime(datebuffer, sizeof(datebuffer), "%H:%M:%S on %Y-%m-%d", localtime);
	mcu::logInfof("Application started at %s", datebuffer);
	
	EAT::gEAT.m_pEnvData->startUptimeTimer();
	
	EAT::gEAT.m_pAppMainLoop = new nmui::AppMainLoop();
	
	
}

} // namespace mcc