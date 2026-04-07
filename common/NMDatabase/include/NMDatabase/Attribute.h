
namespace db
{
	
	class Attribute
	{
	public:
		Attribute::AttribType 	getAttributeType(void);
		db::Database			*getDatabase(void);
		db::AttributeDef		*getDef(void);
		db::PointerAttribute 	*getDependentPointer(uint index);
		int 					getDependentPointerCount(void);
		char					*getDependentPointers(void); //this very likely returns a struct or a class.
		std::wstring			getEscapedName(void);
		Node 					*getGrandParentNode(void);
		int 					getIndex(void);
		Node 					*getLocalPathName(std::wstring & output);
		std::wstring 			getName(void);
		db::Attribute 			*getNext(void);
		db::Database 			*getOldDatabase(void);
		db::CompositeAttribute 	*getOldParentAttribute(void);
		Node 					*getOldParentNode(void);
		db::CompositeAttribute 	*getParentAttribute(void);
		Node 					*getParentNode(void);
		Attribute 				*getPrevious(void);
		db::CompositeAttribute 	*getRootParentAttribute(void);
		int 					getRuntimeID(void);
		void					*getUserData(void);
		
		bool 					hasFlag(int flag);
		bool 					hasGrandParentNode(const db::NodeDef& nodedef);
		bool 					hasInternalFlag(int internalflag);
		bool 					hasOldParentNode(const db::NodeDef& node);
		bool 					hasParentNode(const db::NodeDef& node);
		bool 					isAdded(void);
		bool 					isArrayElement(void);
		bool 					isDescendantOf(const db::Attribute* attrib);
		bool 					isEmbedded(void);
		bool 					isNodeType(const db::NodeDef& node);
		bool 					isPointedAt(void);
		bool 					isReferenced(void);
		bool 					isRemoved(void);
		bool 					isReparented(void);
		bool					wasDescendantOf(const db::Attribute* attrib);
		
		//turn on/off what?..
		void 					turnOn(int flag);
		void 					turnOff(int flag);
		void					updateValue(void);
		
	}
	
} // namespace db