#include "nmhook_common.h"

//dll handle
static HMODULE g_DLLHandle;

//function typedefs
typedef bool (*pfn_EAT_EnvData_isDevBuildCmd)();

//raw handle to the function inside dll
pfn_EAT_EnvData_isDevBuildCmd _EAT_EnvData_isDevBuildCmd = nullptr;


//functions
bool __fastcall isDevBuildCmd(void* this_)
{
	return true;
}



//now actually do the hooks
BEGIN_DEFINE_HOOKFUNC(EATCore)

	//get the functions,
	_EAT_EnvData_isDevBuildCmd = (pfn_EAT_EnvData_isDevBuildCmd)GetProcAddress(g_DLLHandle, "?isDevBuildCmd@EnvData@EAT@@IAE_NXZ");
	
	//and hook them
    HookFunction(_EAT_EnvData_isDevBuildCmd, isDevBuildCmd);


END_DEFINE_HOOKFUNC()