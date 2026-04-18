

namespace nmx
{
	
	class Node : public nmx::APIBase
	{
	public:
		static uint32_t ClassTypeId(void);
		
		static void Register(nmx::Module& module);
		static void Unregister(nmx::Module& module);
		
	private:
		
	}
	
} // namespace nmx