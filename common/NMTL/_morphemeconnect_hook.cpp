
#define MORPHEME_CONNECT_HOOK

#ifdef MORPHEME_CONNECT_HOOK


#include <cstdlib>
#include <stdlib.h>
#include <stdint.h>
#include <cstdint>
#include <stdio.h>
#include <assert.h>
#include <string>
#include <vector>
#include <Windows.h>
#include <psapi.h>
#include "_morphemeconnect_hook/morphemeConnectCore_magicnums.h"
#include "MinHook/MinHook.h"

void Hook_morpheme_Functions();

void* g_pmorphemeConnectCoreBaseDLL = nullptr;
uint32_t g_morphemeConnectexe_processid = -1;
HANDLE g_morphemeConnectexe_handle = nullptr;
HMODULE g_morphemeConnectCoreHandle;

typedef bool (*pfn_mcc_isEuphoriaEnabled)();

pfn_mcc_isEuphoriaEnabled _mcc_isEuphoriaEnabled = nullptr;

//returns pointer to a section in string after the first occurence of character.
//nullptr if no character could be found
const char* ParseCharacter(const char* path, char character)
{
    while (true)
    {
        if (*path == '\0')
            break;
        path++;
        if (*path == character)
        {
            path++;
            return path;
        }
    }
    return nullptr;
}
const char* filename_frompath(const char* path)
{
    while (true)
    {
        if (!ParseCharacter(path, '\\'))
            return path;

        path = ParseCharacter(path, '\\');
    }
    return nullptr; //failed to get filename
}

void GetDLLName(const DEBUG_EVENT event, char* buffer, uint32_t buffersize)
{
    assert(event.dwDebugEventCode == LOAD_DLL_DEBUG_EVENT);

    char name[512];

    GetFinalPathNameByHandle(event.u.LoadDll.hFile, name, 512, 0);
    const char* filename = filename_frompath(name);
    uint32_t size = sizeof(name) - (filename - name);
    size = size > buffersize ? buffersize : size;

    strcpy_s(buffer, size, filename_frompath(filename));
}

HMODULE GetDLLInsideProcess(const HANDLE& processhandle, const char* dllname)
{
    HMODULE modules[1024];
    DWORD needed;

    if (!EnumProcessModules(processhandle, modules, sizeof(modules), &needed))
        return NULL;

    uint32_t numdlls = needed / sizeof(HMODULE);

    for (int i = 0; i < numdlls; i++)
    {
        char modulename[256];
        GetModuleBaseNameA(processhandle, modules[i], modulename, sizeof(modulename));

        if (_stricmp(modulename, dllname) == 0)
            return modules[i];
    }
    return NULL;
}

void* PoolFor_morphemeConnectCore_start()
{
    void* dllbasemem = nullptr;
    if (g_morphemeConnectCoreHandle = GetDLLInsideProcess(g_morphemeConnectexe_handle, "morphemeConnectCore.dll"))
        dllbasemem = (void*)g_morphemeConnectCoreHandle;

    //DEBUG_EVENT debugEvent;
    //WaitForDebugEvent(&debugEvent, INFINITE);
    //
    //if (debugEvent.dwDebugEventCode == LOAD_DLL_DEBUG_EVENT)
    //{
    //    char dllname[256];
    //    GetDLLName(debugEvent, dllname, sizeof(dllname));
    //    if (strstr(dllname, "morphemeConnectCore"))
    //    {
    //        dllbasemem = debugEvent.u.LoadDll.lpBaseOfDll;
    //        g_morphemeConnectCoreHandle = (HMODULE)dllbasemem;
    //        ContinueDebugEvent(
    //            debugEvent.dwProcessId,
    //            debugEvent.dwThreadId,
    //            DBG_CONTINUE);
    //        break;
    //    }
    //}
    //ContinueDebugEvent(
    //    debugEvent.dwProcessId,
    //    debugEvent.dwThreadId,
    //    DBG_CONTINUE);
    return dllbasemem;
}

void HookFunction(void* to_be_hooked, void* our_function)
{
    MH_CreateHook(to_be_hooked, our_function, nullptr);
    MH_EnableHook(to_be_hooked);
}

bool __cdecl isEuphoriaEnabled()
{
    return true; //uhhhmm.. yes the fuck is ?
}

void Hook_morpheme_Functions()
{
    //_mcc_isEuphoriaEnabled = (pfn_mcc_isEuphoriaEnabled)((uint8_t*)g_pmorphemeConnectCoreBaseDLL + g_mcc__isEuphoriaEnabled);
    _mcc_isEuphoriaEnabled = (pfn_mcc_isEuphoriaEnabled)GetProcAddress(g_morphemeConnectCoreHandle, "?isEuphoriaEnabled@mcc@@YA_NXZ");
    HookFunction(_mcc_isEuphoriaEnabled, isEuphoriaEnabled);
}

DWORD APIENTRY _morphemecore_hook_poll(LPVOID lpParam)
{
    MH_Initialize();

    while(true)
    {
        g_pmorphemeConnectCoreBaseDLL = PoolFor_morphemeConnectCore_start();

        if (g_pmorphemeConnectCoreBaseDLL)
        {
            Hook_morpheme_Functions();
            break;
        }
    }
    return 0;
}

BOOL DllMain(HINSTANCE hinstDLL, DWORD fdwReason, LPVOID lpvReserved)
{
    static bool hooked_morpheme = false;
    switch (fdwReason)
    {
        case DLL_PROCESS_ATTACH:
            if (!hooked_morpheme)
            {
                //cant do loops inside DllMain, make a thread instead
                hooked_morpheme = true;
                g_morphemeConnectexe_handle = GetCurrentProcess();
                CreateThread(nullptr, 0, _morphemecore_hook_poll, nullptr, 0, nullptr);
            }
        break;
    }
    return TRUE;
}


#endif