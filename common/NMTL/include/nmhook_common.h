#pragma once

#include <cassert>
#include <Windows.h>

#define DECLARE_HOOKFUNC(namehere) extern void _##namehere##_hook()



#define BEGIN_DEFINE_HOOKFUNC(namehere) \
	void _##namehere##_hook() \
	{ \
		g_DLLHandle = PoolForDll(#namehere".dll"); \
		assert(g_DLLHandle); \

#define END_DEFINE_HOOKFUNC() }



#define CALL_HOOKFUNC(namehere) _##namehere##_hook() 

HMODULE PoolForDll(const char* dllname);

void HookFunction(void* to_be_hooked, void* our_function);

void GlobalHookOntoNaturalmotion(void); //executes all hooking