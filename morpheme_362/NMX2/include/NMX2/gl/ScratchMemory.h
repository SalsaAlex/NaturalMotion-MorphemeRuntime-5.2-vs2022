
namespace nmx
{
	
namespace gl
{
	class ScratchMemory
	{
	public:
		static void lock(uint32_t size);
		static void unlock(void);
		static void release(void);
		
	private:
		static void* m_memory;
		static uint32_t m_size;
		bool m_isLocked;
	}
	
} // namespace gl
	
} // namespace nmx