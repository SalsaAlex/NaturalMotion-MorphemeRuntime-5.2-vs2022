
namespace mcc
{
	
	class EuphoriaPersistentDataSection : public PersistentDataSection
	{
		EuphoriaPersistentDataSection(const EuphoriaPersistentDataSection datasection);
		EuphoriaPersistentDataSection(void);
		~EuphoriaPersistentDataSection(void);
		
		bool addEuphoriaNetwork(EuphoriaNetworkPersistentData* networkdata);
		bool addPersistentData(PersistentDataPart* datapart);
		bool removeEuphoriaNetwork(EuphoriaNetworkPersistentData* networkdata);
		bool removePersistentData(PersistentDataPart* datapart);
		
		void clear(void);
		
		EuphoriaNetworkPersistentData* getEuphoriaNetwork(uint index);
		
	}
	
}//namespace mcc