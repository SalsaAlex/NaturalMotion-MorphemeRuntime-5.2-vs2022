
namespace nmlua
{
	
	class LuaArgumentTable
	{
	public:
		LuaArgumentTable(const LuaPlus::LuaObject& argtable);
		LuaArgumentTable(const LuaArgumentTable& argtable);
		
		bool 				getBoolean(const wchar_t* argname, bool defaultval, bool* result = nullptr);
		nmlua::LuaClass		*getClass(const wchar_t* argname, bool* result = nullptr);
		double 				getDouble(const wchar_t* argname, double defaultval, bool* result = nullptr);
		float 				getFloat(const wchar_t* argname, float defaultval, bool* result = nullptr);
		int 				getInteger(const wchar_t* argname, int defaultval, bool* result = nullptr);
		LuaPlus::LuaObject 	getObject(const wchar_t* argname, bool* result = nullptr);
		const wchar_t		*getString(const wchar_t* argname, const wchar_t* defaultval, bool* result = nullptr);
		LuaPlus::LuaObject 	getTable(void);
		
		bool isValid(void);
		
	private:
		LuaPlus::LuaObject m_luatable;
	}
	
} // namespace nmlua