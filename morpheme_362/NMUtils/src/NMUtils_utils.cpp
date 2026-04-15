#include <string>
#include "NMUtils_utils.h"

namespace NMutils
{
	__declspec(dllexport) bool isWhiteSpace(wchar_t character){
		return (char)character == ' ' || (char)character == '\t' || (char)character == '\r' || (char)character == '\n';
	}
	
	__declspec(dllexport) void toLower(std::wstring& string){
		
	}
	__declspec(dllexport) void toUpper(std::wstring& string)	{
		
	}
	__declspec(dllexport) void toInitialLower(std::wstring& string)	{
		
	}
	__declspec(dllexport) void toInitialUpper(std::wstring& string)	{
		
	}
	__declspec(dllexport) void removeAllWhiteSpace(std::wstring& string)	{
		
	}
	
	__declspec(dllexport) std::string wstringToString(const std::wstring& wstring)	{
		std::string string;
		string.reserve(512);
		
		for (int i = 0; i < wstring.size(); ++i)
		{
			wchar_t c = wstring[i];
			if (c < 0x80)
				string += static_cast<char>(c);
		}

		return string;
	}
	__declspec(dllexport) std::string wstringToString(const wchar_t* wstring)	{
		std::string string;
		if(wstring)
		{
			for (int i = 0; wstring[i] != 0; ++i)
			{
				unsigned short c = wstring[i];
				if (c < 128)
					string += static_cast<char>(c);
			}
		}

		return string;
	}
	__declspec(dllexport) std::wstring stringToWstring(const std::string& string)	{
		std::wstring wstring;
		wstring.reserve(string.size());
		
		for (int i = 0; i < string.size(); ++i)
			wstring += (wchar_t)string[i];
		
		return wstring;
	}
	__declspec(dllexport) std::wstring stringToWstring(const char* string)	{
		std::wstring wstring;
		wstring.reserve(512);
		
		while (*string)
		{
			wstring += (wchar_t)*string;
			++string;
		}
		
		return wstring;
	}

	
	__declspec(dllexport) void* reallocWrapped(void* block, uint32_t size /*, Memory::eAllocationType alloctype*/){
		return realloc(block, size);
	}
	__declspec(dllexport) void* mallocWrapped(uint32_t size/*, Memory::eAllocationType alloctype*/){
		return malloc(size);
	}
	__declspec(dllexport) void* callocWrapped(uint32_t size/*, Memory::eAllocationType alloctype*/){
		return calloc(1, size);
	}
}//namespace NMutils