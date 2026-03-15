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
#include "morphemeConnectCore_magicnums.h"

void Hook_morpheme_Functions();

void* g_pmorphemeConnectCoreBaseDLL = nullptr;
uint32_t g_morphemeConnectexe_processid = -1;
HMODULE g_morphemeConnectCoreHandle;
PROCESS_INFORMATION g_morphemeConnectexe_info;

typedef bool (*pfn_mcc_isEuphoriaEnabled)();

pfn_mcc_isEuphoriaEnabled _mcc_isEuphoriaEnabled = nullptr;

bool StartLauncher()
{
    STARTUPINFO startinfo = {0};
    startinfo.cb = sizeof(STARTUPINFO);
    char cmd[] = "morphemeConnect.exe";

    if (!CreateProcess(cmd, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &startinfo, &g_morphemeConnectexe_info))
    {
        DWORD error = GetLastError();
        return 0;
    }
    g_morphemeConnectexe_processid = g_morphemeConnectexe_info.dwProcessId;

    return 1;
}

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

HMODULE GetDLLInsideProcess(const PROCESS_INFORMATION& processinfo, const char* dllname)
{
    HMODULE modules[1024];
    DWORD needed;

    if (!EnumProcessModules(processinfo.hProcess, modules, sizeof(modules), &needed))
        return NULL;

    uint32_t numdlls = needed / sizeof(HMODULE);

    for (int i = 0; i < numdlls; i++)
    {
        char modulename[256];
        GetModuleBaseNameA(processinfo.hProcess, modules[i], modulename, sizeof(modulename));

        if (_stricmp(modulename, dllname) == 0)
            return modules[i];
    }
    return NULL;
}

void* PoolFor_morphemeConnectCore_start()
{
    void* dllbasemem = nullptr;
    while (true)
    {
        if (g_morphemeConnectCoreHandle = GetDLLInsideProcess(g_morphemeConnectexe_info, "morphemeConnectCore.dll"))
            break;

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
    }
    dllbasemem = (void*)g_morphemeConnectCoreHandle;
    return dllbasemem;
}

void PoolFor_morphemeConnectCore_exit()
{
    while (true)
    {
        DEBUG_EVENT debugEvent;
        WaitForDebugEvent(&debugEvent, INFINITE);

        if (debugEvent.dwDebugEventCode == EXIT_PROCESS_DEBUG_EVENT)
        {
            if (debugEvent.dwProcessId == g_morphemeConnectexe_processid)
                break;
        }
        ContinueDebugEvent(
            debugEvent.dwProcessId,
            debugEvent.dwThreadId,
            DBG_CONTINUE);
    }
}

void Hook_morpheme_Functions()
{
    _mcc_isEuphoriaEnabled = (pfn_mcc_isEuphoriaEnabled)((uint8_t*)g_pmorphemeConnectCoreBaseDLL + g_mcc__isEuphoriaEnabled);
    assert(!_mcc_isEuphoriaEnabled());
}

int main()
{
	//launch morpheme 3.6.2
    if (!StartLauncher())
        return 0;

    g_pmorphemeConnectCoreBaseDLL = PoolFor_morphemeConnectCore_start();

    if (!g_pmorphemeConnectCoreBaseDLL)
        return 0;

    Hook_morpheme_Functions();

    PoolFor_morphemeConnectCore_exit();
}