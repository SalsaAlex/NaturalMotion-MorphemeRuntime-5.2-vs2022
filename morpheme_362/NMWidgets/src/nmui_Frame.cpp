#include "nmui_Frame.h"
#include "nmui_GUIServices.h"


namespace nmui
{
		Frame::Frame(
			wxWindow* window, 
			wxWindowID windowid, 
			const wxString &title, 
			const wxPoint &pos,
			const wxSize &size,
			long style,
			const wxString &name)
		{
			if(wxFrame::Create(window, windowid, title, pos, size, style, name))
				nmui::GUIServices::setDefaultFontAndColours(this, false);
		}
		
		void Frame::onActiveColourSchemeChanged(void)
		{
			//undone
		}
		void Frame::enable(void)
		{
			//undone
		}
		void Frame::Create(
			wxWindow* window, 
			wxWindowID windowid, 
			const wxString &title, 
			const wxPoint &pos,
			const wxSize &size,
			long style,
			const wxString &name)
		{
			if(wxFrame::Create(window, windowid, title, pos, size, style, name))
				nmui::GUIServices::setDefaultFontAndColours(this, false);
		}
} // namespace nmui