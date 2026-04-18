

namespace nmx
{
	
	class Scene : public nmx::Database
	{
	public:
		Scene(const wchar_t* name, uint32_t unknownint1);
		
		void addSceneObserver(nmx::SceneObserver& observer);
		void addScenePreObserver(nmx::ScenePreObserver& preobserver);
		
		
	private:
		
	}
	
} // namespace nmx