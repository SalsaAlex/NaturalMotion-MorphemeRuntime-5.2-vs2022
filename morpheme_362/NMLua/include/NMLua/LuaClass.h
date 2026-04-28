
namespace nmlua
{
	
	class LuaClass
	{
	public:
		virtual const wchar_t* getClassNameStatic(void);
		virtual LuaPlus::LuaObject getMetatable(void);
		
	private:
		static LuaPlus::LuaObject* m_metatable;
	}
	
	typedef int(*)(LuaClass*, LuaPlus::LuaState*) pfn_MemberDispatch;
	
	//sizeof(LuaClassManager) == 84
	class LuaClassManager
	{
	public:
		LuaClassManager(LuaPlus::LuaState* luastate);
		
		~LuaClassManager() = default; //undone, i think
		
		static void addGarbageCollectorToMetatable(LuaPlus::LuaObject& metatable);
		void addClassCustomMember(const wchar_t* string1, const wchar_t* string2, pfn_MemberDispatch memberdispatch );
		
		static void buildMetatable_simpleTableNoGC(LuaPlus::LuaObject& luaobj);
		static void buildMetatable_simpleTableGC(LuaPlus::LuaObject& luaobj);
		
		void createLuaClassCustomMemberObject(pfn_MemberDispatch memberdispatch );
		void createLuaClassCustomScriptMemberObject(LuaPlus::LuaObject luaobj);
		void createClassMetatable(LuaPlus::LuaObject** luaobj, const wchar_t* string1, const wchar_t* string2, const wchar_t* string3);
		
		LuaPlus::LuaState* getLuaState(void);
		void* getLuaClass(LuaPlus::LuaObject luaobj);
		static void getOrCreateTable(LuaPlus::LuaObject luaobj, const wchar_t* string1);
		static void getMetatableItem(const wchar_t* string1, const wchar_t* string2, const wchar_t* string3);
		static void getClassBaseCmd(const wchar_t* string1);
		static void getClassFunctionsCmd(const wchar_t* string1);
		static void getClassReadAttributesCmd(const wchar_t* string1);
		static void getClassWriteAttributesCmd(const wchar_t* string1);
		static void getClassHelpCmd(const wchar_t* string1);
		static void getClassInstancesCmd(void);
		
		static void init(LuaPlus::LuaState* luastate);
		static bool isClassTypeCmd(const wchar_t* string1);
		static bool isClassCmd(LuaPlus::LuaObject luaobj);
		bool isClassRegistered(LuaClass* class);
		
		static pfn_MemberDispatch luaClassCustomMemberDispatcher(lua_State* luastate);
		static pfn_MemberDispatch luaClassCustomScriptMemberDispatcher(lua_State* luastate);
		
		static void* metatable_gcCmd(lua_State* luastate);
		
		void outputDebugTraceBack(void);
		
		bool pushLuaClass(const LuaClass* luaclass);
		
		void registerGlobalFunctions(void);
		void registerClass(LuaClass* luaclass);
		
		void throwLuaError(const wchar_t* errormsg);
		static void term(void);
		
		void unregisterClass(LuaClass* luaclass);
		
		
	private:
	}
	
	extern LuaClassManager* gLuaClassManager;
	
} // namespace nmlua