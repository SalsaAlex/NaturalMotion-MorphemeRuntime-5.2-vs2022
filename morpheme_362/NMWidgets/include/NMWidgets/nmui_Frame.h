
namespace nmui
{
	class Frame : public wxFrame
	{
	public:
		Frame(
			wxWindow* window, 
			wxWindowID windowid, 
			const wxString &title, 
			const wxPoint &pos,
			const wxSize &size,
			long style,
			const wxString &name);
		
		void onActiveColourSchemeChanged(void) override;
		void enable(void) override;
		void Create(
			wxWindow* window, 
			wxWindowID windowid, 
			const wxString &title, 
			const wxPoint &pos,
			const wxSize &size,
			long style,
			const wxString &name);
	
	private:
		
	}
	
} // namespace nmui