
namespace nmx
{
	
	//sizeof(HostApplication) == 548
	class HostApplication
	{
	public:
		HostApplication(void);
		~HostApplication(void);
		
	private:
		// there's this reocurring pattern in constructor:
		//  *((_DWORD *)this + 124) = 0;
		// *((_DWORD *)this + 125) = 0;
		// *((_DWORD *)this + 126) = 0;
		// *((_DWORD *)this + 127) = nmtl::NMTLGetDefaultAllocator();
		//	...
		//
		//	perhaps a structure ?
		
		
		
		nmtl::allocator* m_allocator1; // (char *)this + 28 || (_DWORD *)this + 7
		nmtl::allocator* m_allocator2; // (char *)this + 44 || (_DWORD *)this + 11
		nmtl::allocator* m_allocator3; // (char *)this + 168 || (_DWORD *)this + 42
		nmtl::allocator* m_allocator4; // (char *)this + 184 || (_DWORD *)this + 46
		nmtl::allocator* m_allocator5; // (char *)this + 200 || (_DWORD *)this + 50
		nmtl::allocator* m_allocator6; // (char *)this + 216 || (_DWORD *)this + 54
		
		std::wstring m_string1; // (char *)this + 240 || (_DWORD *)this + 60
		std::wstring m_string2; // (char *)this + 268 || (_DWORD *)this + 67
		std::wstring m_string3; // (char *)this + 296 || (_DWORD *)this + 74
		std::wstring m_string4; // (char *)this + 324 || (_DWORD *)this + 81
		std::wstring m_string5; // (char *)this + 352 || (_DWORD *)this + 88
		std::wstring m_string6; // (char *)this + 380 || (_DWORD *)this + 95
		std::wstring m_string7; // (char *)this + 408 || (_DWORD *)this + 102
		
		nmtl::allocator* m_allocator7; // (char *)this + 464 || (_DWORD *)this + 116
		nmtl::allocator* m_allocator8; // (char *)this + 492 || (_DWORD *)this + 123
		nmtl::allocator* m_allocator9; // (char *)this + 508 || (_DWORD *)this + 127
		nmtl::allocator* m_allocator10; // (char *)this + 524 || (_DWORD *)this + 131
		nmtl::allocator* m_allocator11; // (char *)this + 540 || (_DWORD *)this + 135
	}
	
	extern HostApplication* g_application;
	
} // namespace nmx