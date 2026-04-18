
namespace nmlua
{
	
	class LuaClass
	{
	public:
		virtual const wchar_t* getClassNameStatic(void);
		virtual LuaPlus::LuaObject getMetatable(void);
		
	private:	
	}
	
} // namespace nmlua