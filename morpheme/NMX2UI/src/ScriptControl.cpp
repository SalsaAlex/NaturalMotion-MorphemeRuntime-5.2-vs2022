
namespace nmxui
{
	
	void ScriptControl::Create(wxWindow* window)
	{
		nmxui::Control::Create(window);
		( (wxEvtHandler*)( (char*)this - 448 ) )->Connect(-1, wxEVT_SCRIPT_EDITOR_RUN, nmxui::ScriptControl::onRunScript, false, false);
	}
	
	void ScriptControl::onRunScript(EAT::ScriptEditorEvent& event)
	{
		if(!getCurrentEditor())
			return;
		int id = getCurrentEditor()->GetId();
		EAT::ScriptEditor::EditorData* data = getEditorData(id);
		if(!data)
			return;
		//incomplete
	}
	
} //namespace nmxui