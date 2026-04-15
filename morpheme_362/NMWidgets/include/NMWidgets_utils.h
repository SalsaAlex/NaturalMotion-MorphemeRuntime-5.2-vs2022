
namespace nmui
{
	bool GetDllPathname(LPCWSTR lpModuleName, wxString &pathname);
	wxString GetDirectoryFromPathname(const wxString& pathname);
	wxString GetFilenameFromPathname(const wxString& pathname);
	
	bool isPathHidden(LPCWSTR lpFileName);
	bool IsAnyPartOfPathHidden(const wxString& path);
	bool isStrInDelimitedStr(const wchar_t* string1, const wchar_t* string2, const wchar_t* string3);
	bool isAlpha(const wxString& string);
	bool isNumeric(const wxString& string);
	bool isAlphaNumeric(const wxString& string);
	bool isPathValid(const wxString& path);
	
	bool isDescendedFrom(wxWindow* child, wxWindow* supposedparent);
	
}//namespace nmui