#include "nmhook_common.h"

//dll handle
static HMODULE g_DLLHandle;

//function typedefs
typedef int (*pfn_EAT_PluginManager_isLicensed)(void* this_, int a2, int a3, int a4, char* a5);

//raw handle to the function inside dll
pfn_EAT_PluginManager_isLicensed _EAT_PluginManager_isLicensed = nullptr;


//functions
int __fastcall isLicensed(void* this_, /*because compiler doesnt LISTEN to me*/void* dummy, unsigned int a2, int a3, int a4, bool& a5)
{
	a5 = 1;
	return 4;
}



//now actually do the hooks
BEGIN_DEFINE_HOOKFUNC(EATCore)

//get the functions,
_EAT_PluginManager_isLicensed = (pfn_EAT_PluginManager_isLicensed)GetProcAddress(g_DLLHandle, "?isLicensed@PluginManager@EAT@@QBE?AW4PluginState@12@IHW4LicencePurposes@LicencePlugin@2@AA_N@Z");

//and hook them
HookFunction(_EAT_PluginManager_isLicensed, isLicensed);


END_DEFINE_HOOKFUNC()