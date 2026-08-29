

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

#ifdef CONNECT_3_6_2_HOOK

#include "nmhook_common.h"
#include "MinHook/MinHook.h"

#include "_global_hook.h"

static HANDLE g_morphemeConnectexe_handle;

static void NaturalmotionHooking_init()
{
    MH_Initialize();
}

static HMODULE GetDLLInsideProcess(const HANDLE& processhandle, const char* dllname)
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

HMODULE PoolForDll(const char* dllname)
{
    return GetDLLInsideProcess(g_morphemeConnectexe_handle, dllname);
}

void HookFunction(void* to_be_hooked, void* our_function)
{
    MH_CreateHook(to_be_hooked, our_function, nullptr);
    MH_EnableHook(to_be_hooked);
}

static void PatchDLLSignature()
{
    const uint32_t address = 0x527BC6;
    const uint32_t idapro_imagebase = 0x400000;
    const uint32_t offset = address - idapro_imagebase;

    const uint8_t instruction_firstbyte = 0xB0;
    const uint8_t instruction_lastbyte = 0x01;

    uint8_t* exe_base = (uint8_t*)GetModuleHandle(NULL);

    assert(exe_base[offset] == 0x8A);
    assert(exe_base[offset + 1] == 0xC3);

    DWORD oldProtect;

    //unlock
    if (VirtualProtect(exe_base + offset, 2, PAGE_EXECUTE_READWRITE, &oldProtect))
    {
        memcpy(exe_base + offset, &instruction_firstbyte, 1);
        memcpy(exe_base + offset + 1, &instruction_lastbyte, 1);

        //lock
        VirtualProtect(exe_base + offset, 2, oldProtect, &oldProtect);
    }
}

static DWORD APIENTRY _NM_hook_poll(LPVOID lpParam)
{
    NaturalmotionHooking_init();

    PatchDLLSignature();

    CALL_HOOKFUNC(EATCore);
    CALL_HOOKFUNC(mcEventDetection);
    CALL_HOOKFUNC(morphemeConnectCore);
    CALL_HOOKFUNC(NMDatabase);
    CALL_HOOKFUNC(NMDatabase2);
    CALL_HOOKFUNC(NMDatabase2UI);
    CALL_HOOKFUNC(NMDatabase2XMLIO);
    CALL_HOOKFUNC(NMDatabaseUI);
    CALL_HOOKFUNC(NMEditors);
    CALL_HOOKFUNC(NMLua);
    CALL_HOOKFUNC(NMLuaUI);
    CALL_HOOKFUNC(NMUtils);
    CALL_HOOKFUNC(NMWidgets);
    CALL_HOOKFUNC(NMX2);
    CALL_HOOKFUNC(NMX2Extensions);
    CALL_HOOKFUNC(NMX2Foundation);
    CALL_HOOKFUNC(NMX2PhysicsExtensions);
    CALL_HOOKFUNC(NMX2UI);
    CALL_HOOKFUNC(NMX2XMD);

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
            CreateThread(nullptr, 0, _NM_hook_poll, nullptr, 0, nullptr);
        }
        break;
    }
    return TRUE;
}

#else //CONNECT_3_6_2_HOOK

HMODULE PoolForDll(const char* dllname)
{
    return nullptr;
}

void HookFunction(void* to_be_hooked, void* our_function)
{
}

#endif //CONNECT_3_6_2_HOOK