
namespace NMutils
{
	__declspec(dllexport) bool isWhiteSpace(wchar_t character);
	
	__declspec(dllexport) void toLower(std::wstring& string);
	__declspec(dllexport) void toUpper(std::wstring& string);
	__declspec(dllexport) void toInitialLower(std::wstring& string);
	__declspec(dllexport) void toInitialUpper(std::wstring& string);
	__declspec(dllexport) void removeAllWhiteSpace(std::wstring& string);
	
	__declspec(dllexport) std::string wstringToString(const std::wstring& wstring);
	__declspec(dllexport) std::string wstringToString(const wchar_t* wstring);
	__declspec(dllexport) std::wstring stringToWstring(const std::string& string);
	__declspec(dllexport) std::wstring stringToWstring(const char* string);
	
	__declspec(dllexport) void* reallocWrapped(void* block, uint32_t size /*, Memory::eAllocationType alloctype*/);
	__declspec(dllexport) void* mallocWrapped(uint32_t size/*, Memory::eAllocationType alloctype*/);
	__declspec(dllexport) void* callocWrapped(uint32_t size/*, Memory::eAllocationType alloctype*/);
}//namespace NMutils