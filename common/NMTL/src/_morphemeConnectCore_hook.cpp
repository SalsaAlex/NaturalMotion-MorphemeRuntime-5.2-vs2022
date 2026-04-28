#include "nmhook_common.h"

//dll handle
static HMODULE g_DLLHandle;

//function typedefs
typedef bool (*pfn_mcc_isEuphoriaEnabled)();

//raw handle to the function inside dll
pfn_mcc_isEuphoriaEnabled _mcc_isEuphoriaEnabled = nullptr;


//functions
bool __cdecl isEuphoriaEnabled()
{
    return true;
}



//now actually do the hooks
BEGIN_DEFINE_HOOKFUNC(morphemeConnectCore)

	//get the functions,
    _mcc_isEuphoriaEnabled = (pfn_mcc_isEuphoriaEnabled)GetProcAddress(g_DLLHandle, "?isEuphoriaEnabled@mcc@@YA_NXZ");
	
	//and hook them
    HookFunction(_mcc_isEuphoriaEnabled, isEuphoriaEnabled);
	
	
END_DEFINE_HOOKFUNC()
