
namespace nmxui
{
	
	void ScriptControl::Create(wxWindow* window)
	{
		nmxui::Control::Create(window);
		//m_pEvtHandler = (char*)this - 448
		m_pEvtHandler->Connect(-1, wxEVT_SCRIPT_EDITOR_RUN, nmxui::ScriptControl::OnRunScript, false, false); //false, false or 0, 0
	}
	
	void ScriptControl::UpdateDefaultSyntax(void)
	{
		updateDefaultSyntax(15); //EAT::ScriptEditor::updateDefaultSyntax. this lazily assumes ScriptControl inherits EAT::ScriptEditor
	}
	
	void ScriptControl::onRunScript(EAT::ScriptEditorEvent& event)
	{
		//incomplete
	}
	
	
} //namespace nmxui