
namespace nmxui
{
	//-------------------------------------
	//		NMXNodeExplorerNode
	//-------------------------------------
	
	bool NMXNodeExplorerNode::setName(const wchar_t* name)
	{
		nmx::ChangeBlock::ChangeBlock("Rename node"); //obs1: incomplete. obs2: make macro
		
		//incomplete
	}
	
	void NMXNodeExplorerNode::orderChildren(void)
	{
		//incomplete
	}
	
	bool NMXNodeExplorerNode::isRenamable(void)
	{
		//incomplete
	}
	
	void NMXNodeExplorerNode::getTypeName(wxString& string)
	{
		//m_Unknown = this[3]
		string = m_Unknown->getTypeString().c_str();
	}
	
	void NMXNodeExplorerNode::getName(wxString& string)
	{
		//m_Unknown = this[3]
		string = m_Unknown->getName().c_str();
	}
	
	
	
	
	//-------------------------------------
	//		SceneExplorerControl
	//-------------------------------------
	
	
	void SceneExplorerControl::handleContextMenuChoice(wxCommandEvent& event)
	{
		void* data = event.GetClientAssignedData();
		if(data)
		{
			nmx::ChangeBlock::ChangeBlock("SceneExplorer Callback"); //obs1: incomplete. obs2: make macro
			
			nmx::SelectionList selectionlist;
			this[260]->getSelectionList(selectionlist); //does this take a reference or a pointer ?
			//incomplete
			
		}
		else
		{
			//--pseudocode
			//wxEvent::GetId(a2);
			//nmui::MenuItem::getID(this[266]);
		}
	}
	
	
	
} //namespace nmxui