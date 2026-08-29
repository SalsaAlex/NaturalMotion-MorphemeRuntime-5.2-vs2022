
#include "wx\wx.h"

namespace nmui
{
	class Frame : public wxFrame
	{
	public:
		Frame(
			wxWindow* window,
			wxWindowID windowid,
			const wxString& title,
			const wxPoint& pos,
			const wxSize& size,
			long style,
			const wxString& name);

		virtual void onActiveColourSchemeChanged(void);
		virtual void enable(void);
		void Create(
			wxWindow* window,
			wxWindowID windowid,
			const wxString& title,
			const wxPoint& pos,
			const wxSize& size,
			long style,
			const wxString& name);

	private:

	};
	
} // namespace nmui