#include "nmui_GUIServices.h"


namespace nmui
{
	nmui::GUIServices* GUIServices::staticInstance = nullptr;


	GUIServices::GUIServices()
	{
		init();
	}
	
	void GUIServices::init()
	{
		setDefaultStandardColours();
		calculateDerivedColours();
		initBitmaps();
		initFonts();
		initBrushes();
		initPens();
	}
	void GUIServices::initBitmaps()
	{

	}
	void GUIServices::initFonts()
	{
		m_pStandardFont = 						new wxFont(8, 74, 90, 90, false, "Tahoma"); 		//243
		m_pStandardFontBold = 					new wxFont(8, 74, 90, 92, false, "Tahoma"); 		//244
		m_pStandardFontItalic = 				new wxFont(8, 74, 93, 90, false, "Tahoma");		//245
		m_pStandardFontBoldItalic = 			new wxFont(8, 74, 93, 92, false, "Tahoma");		//246
		m_pStandardFontFixedWidth = 			new wxFont(8, 74, 90, 90, false, "Courier New");	//247
		m_pStandardFontFixedWidthBold = 		new wxFont(8, 74, 90, 92, false, "Courier New");	//248
		m_pStandardFontFixedWidthItalic = 		new wxFont(8, 74, 93, 90, false, "Courier New");	//249
		m_pStandardFontFixedWidthBoldItalic = 	new wxFont(8, 74, 93, 92, false, "Courier New");	//250
		m_pLargeFont = 							new wxFont(8, 74, 90, 90, false, "Tahoma");		//251
		m_pLargeFontBold = 						new wxFont(8, 74, 90, 92, false, "Tahoma");		//252
		m_pSmallFont = 							new wxFont(7, 74, 90, 90, false, "Tahoma");		//253
		m_pSmallFontBold = 						new wxFont(7, 74, 90, 92, false, "Tahoma");		//254
		m_pVerySmallFont = 						new wxFont(8, 74, 90, 90, false, "Trebuchet MS");//255
	}
	void GUIServices::initBrushes()
	{

	}
	void GUIServices::initPens()
	{

	}
	
	void GUIServices::term()
	{
		termBitmaps();
		termFonts();
		termBrushes();
		termPens();
	}
	void GUIServices::termBitmaps()
	{

	}
	void GUIServices::termFonts()
	{

	}
	void GUIServices::termBrushes()
	{

	}
	void GUIServices::termPens()
	{

	}
	
	void GUIServices::update()
	{
		calculateDerivedColours();
		termBrushes();
		termPens();
		initBrushes();
		initPens();
	}

	void GUIServices::setDefaultStandardColours(void)
	{
		m_controlTextColour = wxSystemSettingsNative::GetColour(wxSystemColour::wxSYS_COLOUR_WINDOWTEXT);
		m_controlBackgroundColour = wxSystemSettingsNative::GetColour(wxSystemColour::wxSYS_COLOUR_WINDOW);
		m_controlMixedBackgroundColour = wxSystemSettingsNative::GetColour(wxSystemColour::wxSYS_COLOUR_3DSHADOW);
		m_controlErrorBackgroundColour = wxColour(255, 0, 0);
		m_controlSelectedTextColour = wxSystemSettingsNative::GetColour(wxSystemColour::wxSYS_COLOUR_HIGHLIGHTTEXT);
		m_controlSelectedBackgroundColour = wxSystemSettingsNative::GetColour(wxSystemColour::wxSYS_COLOUR_HIGHLIGHT);
		m_controlHeaderTextColour = wxSystemSettingsNative::GetColour(wxSystemColour::wxSYS_COLOUR_BTNTEXT);
		m_controlHeaderBackgroundColour = wxSystemSettingsNative::GetColour(wxSystemColour::wxSYS_COLOUR_BTNFACE);
		m_controlEdgeColour = wxSystemSettingsNative::GetColour(wxSystemColour::wxSYS_COLOUR_3DDKSHADOW);
		m_controlScrollbarColour = wxSystemSettingsNative::GetColour(wxSystemColour::wxSYS_COLOUR_SCROLLBAR);
		m_controlScrollbarHighlightColour = wxSystemSettingsNative::GetColour(wxSystemColour::wxSYS_COLOUR_SCROLLBAR);
		m_controlSplitterColour = wxSystemSettingsNative::GetColour(wxSystemColour::wxSYS_COLOUR_SCROLLBAR);
		m_controlSplitterHighlightColour = wxSystemSettingsNative::GetColour(wxSystemColour::wxSYS_COLOUR_SCROLLBAR);
		m_dialogTextColour = wxSystemSettingsNative::GetColour(wxSystemColour::wxSYS_COLOUR_BTNTEXT);
		m_dialogBackgroundColour = wxSystemSettingsNative::GetColour(wxSystemColour::wxSYS_COLOUR_BTNFACE);
		m_dialogSelectedTextColour = wxSystemSettingsNative::GetColour(wxSystemColour::wxSYS_COLOUR_HIGHLIGHTTEXT);
		m_dialogSelectedBackgroundColour = wxSystemSettingsNative::GetColour(wxSystemColour::wxSYS_COLOUR_HIGHLIGHT);
		m_dialogEdgeColour = wxSystemSettingsNative::GetColour(wxSystemColour::wxSYS_COLOUR_3DDKSHADOW);

	}
	void GUIServices::calculateDerivedColours(void)
	{
		//undone
	}


	void GUIServices::setDefaultFontAndColours(wxWindow* window, bool ControlMixedBackgroundColour)
	{
		window->SetFont(*getInstance()->getStandardFont());
		window->SetForegroundColour(getInstance()->getDialogTextColour());
		if (ControlMixedBackgroundColour)
		{
			window->SetBackgroundColour(getInstance()->getControlMixedBackgroundColour());
		}
		else
		{
			window->SetBackgroundColour(getInstance()->getDialogBackgroundColour());
		}
	}

	nmui::GUIServices* GUIServices::getInstance(void)
	{
		if (!nmui::GUIServices::staticInstance)
		{
			nmui::GUIServices::staticInstance = new nmui::GUIServices();
		}
		return nmui::GUIServices::staticInstance;
	}
	
	wxBitmap* GUIServices::getArrowDown(void){
		return m_pArrowDown;
	}
	wxBitmap* GUIServices::getArrowDownDisabled(void){
		return m_pArrowDownDisabled;
	}
	wxBitmap* GUIServices::getArrowLeft(void){
		return m_pArrowLeft;
	}
	wxBitmap* GUIServices::getArrowLeftDisabled(void){
		return m_pArrowLeftDisabled;
	}
	wxBitmap* GUIServices::getArrowRight(void){
		return m_pArrowRight;
	}
	wxBitmap* GUIServices::getArrowRightDisabled(void){
		return m_pArrowRightDisabled;
	}
	wxBitmap* GUIServices::getArrowUp(void){
		return m_pArrowUp;
	}
	wxBitmap* GUIServices::getArrowUpDisabled(void){
		return m_pArrowUpDisabled;
	}
	wxBitmap* GUIServices::getCheckMarkTick(void){
		return m_pCheckMarkTick;
	}
	wxBitmap* GUIServices::getDarkArrowRight(void){
		return m_pDarkArrowRight;
	}
	wxBitmap* GUIServices::getDeleteCross(void){
		return m_pDeleteCross;
	}
	wxBitmap* GUIServices::getDotsBlockHorizontal(void){
		return m_pDotsBlockHorizontal;
	}
	wxBitmap* GUIServices::getDotsBlockVertical(void){
		return m_pDotsBlockVertical;
	}
	wxBitmap* GUIServices::getTick(void){
		return m_pTick;
	}
	wxBitmap* GUIServices::getTinyArrowDown(void){
		return m_pTinyArrowDown;
	}
	wxBitmap* GUIServices::getTinyArrowDownDisabled(void){
		return m_pTinyArrowDownDisabled;
	}
	wxBitmap* GUIServices::getTinyArrowLeft(void){
		return m_pTinyArrowLeft;
	}
	wxBitmap* GUIServices::getTinyArrowLeftDisabled(void){
		return m_pTinyArrowLeftDisabled;
	}
	wxBitmap* GUIServices::getTinyArrowRight(void){
		return m_pTinyArrowRight;
	}
	wxBitmap* GUIServices::getTinyArrowRightDisabled(void){
		return m_pTinyArrowRightDisabled;
	}
	wxBitmap* GUIServices::getTinyArrowUp(void){
		return m_pTinyArrowUp;
	}
	wxBitmap* GUIServices::getTinyArrowUpDisabled(void){
		return m_pTinyArrowUpDisabled;
	}
	
	
	wxBrush* GUIServices::get2x2CheckerStippleBrush(void){
		return m_2x2CheckerStippleBrush;
	}
	wxBrush* GUIServices::get8x8CheckerStippleBrush(void){
		return m_8x8CheckerStippleBrush;
	}
	wxBrush* GUIServices::getControlBackgroundBrush(void){
		return m_controlBackgroundBrush;
	}
	wxBrush* GUIServices::getControlDarkTintBrush(void){
		return m_controlDarkTintBrush;
	}
	wxBrush* GUIServices::getControlDarkerTintBrush(void){
		return m_controlDarkerTintBrush;
	}
	wxBrush* GUIServices::getControlDarkestTintBrush(void){
		return m_controlDarkestTintBrush;
	}
	wxBrush* GUIServices::getControlDisabledTextBrush(void){
		return m_controlDisabledTextBrush;
	}
	wxBrush* GUIServices::getControlEdgeBrush(void){
		return m_controlEdgeBrush;
	}
	wxBrush* GUIServices::getControlFocusBrush(void){
		return m_controlFocusBrush;
	}
	wxBrush* GUIServices::getControlHeaderBackgroundBrush(void){
		return m_controlHeaderBackgroundBrush;
	}
	wxBrush* GUIServices::getControlLighterTintBrush(void){
		return m_controlLighterTintBrush;
	}
	wxBrush* GUIServices::getControlLightestTintBrush(void){
		return m_controlLightestTintBrush;
	}
	wxBrush* GUIServices::getControlMixedBackgroundBrush(void){
		return m_controlMixedBackgroundBrush;
	}
	wxBrush* GUIServices::getControlScrollbarBrush(void){
		return m_controlScrollbarBrush;
	}
	wxBrush* GUIServices::getControlScrollbarHighlightBrush(void){
		return m_controlScrollbarHighlightBrush;
	}
	wxBrush* GUIServices::getControlSelectedBackgroundBrush(void){
		return m_controlSelectedBackgroundBrush;
	}
	wxBrush* GUIServices::getControlSplitterBrush(void){
		return m_controlSplitterBrush;
	}
	wxBrush* GUIServices::getControlSplitterHighlightBrush(void){
		return m_controlSplitterHighlightBrush;
	}
	wxBrush* GUIServices::getDialogDarkTintBrush(void){
		return m_dialogDarkTintBrush;
	}
	wxBrush* GUIServices::getDialogDarkerTintBrush(void){
		return m_dialogDarkerTintBrush;
	}
	wxBrush* GUIServices::getDialogDarkestTintBrush(void){
		return m_dialogDarkestTintBrush;
	}
	wxBrush* GUIServices::getDialogDisabledTextBrush(void){
		return m_dialogDisabledTextBrush;
	}
	wxBrush* GUIServices::getDialogEdgeBrush(void){
		return m_dialogEdgeBrush;
	}
	wxBrush* GUIServices::getDialogFocusBrush(void){
		return m_dialogFocusBrush;
	}
	wxBrush* GUIServices::getDialogLighterTintBrush(void){
		return m_dialogLighterTintBrush;
	}
	wxBrush* GUIServices::getDialogLightestTintBrush(void){
		return m_dialogLightestTintBrush;
	}
	wxBrush* GUIServices::getDialogSelectedBackgroundBlendedBrush(void){
		return m_dialogSelectedBackgroundBlendedBrush;
	}
	wxBrush* GUIServices::getDialogSelectedBackgroundBrush(void){
		return m_dialogSelectedBackgroundBrush;
	}
	wxBrush* GUIServices::getDialogSelectedTextBrush(void){
		return m_dialogSelectedTextBrush;
	}
	wxBrush* GUIServices::getDialogTextBrush(void){
		return m_dialogTextBrush;
	}
	
	
	wxPen* GUIServices::getControlBlendedTextDottedPen(void){
		return m_controlBlendedTextDottedPen;
	}
	wxPen* GUIServices::getControlDarkShadowDottedPen(void){
		return m_controlDarkShadowDottedPen;
	}
	wxPen* GUIServices::getControlDarkTintPen(void){
		return m_controlDarkTintPen;
	}
	wxPen* GUIServices::getControlDarkerTintPen(void){
		return m_controlDarkerTintPen;
	}
	wxPen* GUIServices::getControlDarkestTintPen(void){
		return m_controlDarkestTintPen;
	}
	wxPen* GUIServices::getControlDisabledTextPen(void){
		return m_controlDisabledTextPen;
	}
	wxPen* GUIServices::getControlEdgePen(void){
		return m_controlEdgePen;
	}
	wxPen* GUIServices::getControlErrorBackgroundPen(void){
		return m_controlErrorBackgroundPen;
	}
	wxPen* GUIServices::getControlFocusPen(void){
		return m_controlFocusPen;
	}
	wxPen* GUIServices::getControlHeaderBackgroundPen(void){
		return m_controlHeaderBackgroundPen;
	}
	wxPen* GUIServices::getControlHeaderTextPen(void){
		return m_controlHeaderTextPen;
	}
	wxPen* GUIServices::getControlLighterTintPen(void){
		return m_controlLighterTintPen;
	}
	wxPen* GUIServices::getControlLightestTintPen(void){
		return m_controlLightestTintPen;
	}
	wxPen* GUIServices::getControlMixedBackgroundPen(void){
		return m_controlMixedBackgroundPen;
	}
	wxPen* GUIServices::getControlScrollbarHighlightPen(void){
		return m_controlScrollbarHighlightPen;
	}
	wxPen* GUIServices::getControlScrollbarPen(void){
		return m_controlScrollbarPen;
	}
	wxPen* GUIServices::getControlSelectedBackgroundPen(void){
		return m_controlSelectedBackgroundPen;
	}
	wxPen* GUIServices::getControlSelectedTextPen(void){
		return m_controlSelectedTextPen;
	}
	wxPen* GUIServices::getControlSelectedUnfocusedPen(void){
		return m_controlSelectedUnfocusedPen;
	}
	wxPen* GUIServices::getControlSelectedUnfocusedTextPen(void){
		return m_controlSelectedUnfocusedTextPen;
	}
	wxPen* GUIServices::getControlSplitterHighlightPen(void){
		return m_controlSplitterHighlightPen;
	}
	wxPen* GUIServices::getControlSplitterPen(void){
		return m_controlSplitterPen;
	}
	wxPen* GUIServices::getControlTargetPen(void){
		return m_controlTargetPen;
	}
	wxPen* GUIServices::getControlTextDottedPen(void){
		return m_controlTextDottedPen;
	}
	wxPen* GUIServices::getControlTextPen(void){
		return m_controlTextPen;
	}
	wxPen* GUIServices::getDialogDarkTintPen(void){
		return m_dialogDarkTintPen;
	}
	wxPen* GUIServices::getDialogDarkerTintDottedPen(void){
		return m_dialogDarkerTintDottedPen;
	}
	wxPen* GUIServices::getDialogDarkerTintPen(void){
		return m_dialogDarkerTintPen;
	}
	wxPen* GUIServices::getDialogDarkestTintPen(void){
		return m_dialogDarkestTintPen;
	}
	wxPen* GUIServices::getDialogDisabledTextPen(void){
		return m_dialogDisabledTextPen;
	}
	wxPen* GUIServices::getDialogEdgePen(void){
		return m_dialogEdgePen;
	}
	wxPen* GUIServices::getDialogFocusPen(void){
		return m_dialogFocusPen;
	}
	wxPen* GUIServices::getDialogLighterTintDottedPen(void){
		return m_dialogLighterTintDottedPen;
	}
	wxPen* GUIServices::getDialogLighterTintPen(void){
		return m_dialogLighterTintPen;
	}
	wxPen* GUIServices::getDialogLightestTintPen(void){
		return m_dialogLightestTintPen;
	}
	wxPen* GUIServices::getDialogSelectedBackgroundBlendedPen(void){
		return m_dialogSelectedBackgroundBlendedPen;
	}
	wxPen* GUIServices::getDialogSelectedBackgroundPen(void){
		return m_dialogSelectedBackgroundPen;
	}
	wxPen* GUIServices::getDialogSelectedTextPen(void){
		return m_dialogSelectedTextPen;
	}
	wxPen* GUIServices::getDialogTextPen(void){
		return m_dialogTextPen;
	}
	
	
	wxColour GUIServices::getControlBackgroundColour(void){
		return m_controlBackgroundColour;
	}
	wxColour GUIServices::getControlDarkTintColour(void){
		return m_controlDarkTintColour;
	}
	wxColour GUIServices::getControlDarkerTintColour(void){
		return m_controlDarkerTintColour;
	}
	wxColour GUIServices::getControlDarkestTintColour(void){
		return m_controlDarkestTintColour;
	}
	wxColour GUIServices::getControlDisabledTextColour(void){
		return m_controlDisabledTextColour;
	}
	wxColour GUIServices::getControlEdgeColour(void){
		return m_controlEdgeColour;
	}
	wxColour GUIServices::getControlErrorBackgroundColour(void){
		return m_controlErrorBackgroundColour;
	}
	wxColour GUIServices::getControlFocusColour(void){
		return m_controlFocusColour;
	}
	wxColour GUIServices::getControlHeaderBackgroundColour(void){
		return m_controlHeaderBackgroundColour;
	}
	wxColour GUIServices::getControlHeaderTextColour(void){
		return m_controlHeaderTextColour;
	}
	wxColour GUIServices::getControlLighterTintColour(void){
		return m_controlLighterTintColour;
	}
	wxColour GUIServices::getControlLightestTintColour(void){
		return m_controlLightestTintColour;
	}
	wxColour GUIServices::getControlMixedBackgroundColour(void){
		return m_controlMixedBackgroundColour;
	}
	wxColour GUIServices::getControlScrollbarColour(void){
		return m_controlScrollbarColour;
	}
	wxColour GUIServices::getControlScrollbarHighlightColour(void){
		return m_controlScrollbarHighlightColour;
	}
	wxColour GUIServices::getControlSelectedBackgroundColour(void){
		return m_controlSelectedBackgroundColour;
	}
	wxColour GUIServices::getControlSelectedTextColour(void){
		return m_controlSelectedTextColour;
	}
	wxColour GUIServices::getControlSelectedUnfocusedColour(void){
		return m_controlSelectedUnfocusedColour;
	}
	wxColour GUIServices::getControlSelectedUnfocusedTextColour(void){
		return m_controlSelectedUnfocusedTextColour;
	}
	wxColour GUIServices::getControlSplitterColour(void){
		return m_controlSplitterColour;
	}
	wxColour GUIServices::getControlSplitterHighlightColour(void){
		return m_controlSplitterHighlightColour;
	}
	wxColour GUIServices::getControlTextColour(void){
		return m_controlTextColour;
	}
	wxColour GUIServices::getDialogBackgroundColour(void){
		return m_dialogBackgroundColour;
	}
	wxColour GUIServices::getDialogDarkTintColour(void){
		return m_dialogDarkTintColour;
	}
	wxColour GUIServices::getDialogDarkerTintColour(void){
		return m_dialogDarkerTintColour;
	}
	wxColour GUIServices::getDialogDarkestTintColour(void){
		return m_dialogDarkestTintColour;
	}
	wxColour GUIServices::getDialogDisabledTextColour(void){
		return m_dialogDisabledTextColour;
	}
	wxColour GUIServices::getDialogEdgeColour(void){
		return m_dialogEdgeColour;
	}
	wxColour GUIServices::getDialogFocusColour(void){
		return m_dialogFocusColour;
	}
	wxColour GUIServices::getDialogLighterTintColour(void){
		return m_dialogLighterTintColour;
	}
	wxColour GUIServices::getDialogLightestTintColour(void){
		return m_dialogLightestTintColour;
	}
	wxColour GUIServices::getDialogSelectedBackgroundBlendedColour(void){
		return m_dialogSelectedBackgroundBlendedColour;
	}
	wxColour GUIServices::getDialogSelectedBackgroundColour(void){
		return m_dialogSelectedBackgroundColour;
	}
	wxColour GUIServices::getDialogSelectedTextColour(void){
		return m_dialogSelectedTextColour;
	}
	wxColour GUIServices::getDialogTextColour(void){
		return m_dialogTextColour;
	}
	
	
	int GUIServices::getControlTargetSize(void){
		return m_controlTargetPen->GetWidth();
	}
	
	
	wxFont* GUIServices::getFont(nmui::StaticTextFont fonttype)
	{
		switch(fonttype)
		{
			case nmui::StaticTextFont::StandardFontBold:	 //1
				return m_pStandardFontBold;
			case nmui::StaticTextFont::LargeFont:			//2
				return m_pLargeFont;
			case nmui::StaticTextFont::LargeFontBold:		//3
				return m_pLargeFontBold;
			case nmui::StaticTextFont::SmallFont:			//4
				return m_pSmallFont;
			case nmui::StaticTextFont::StandardFontItalic: //5
				return m_pStandardFontItalic;
			default:
				return m_pStandardFont;
		}
	}
	wxFont* GUIServices::getLargeFont(void)	{
		return m_pLargeFont;
	}
	wxFont* GUIServices::getLargeFontBold(void)	{
		return m_pLargeFontBold;
	}
	wxFont* GUIServices::getStandardFont(void)	{
		return m_pStandardFont;
	}
	wxFont* GUIServices::getStandardFontBold(void)	{
		return m_pStandardFontBold;
	}
	wxFont* GUIServices::getStandardFontBoldItalic(void)	{
		return m_pStandardFontBoldItalic;
	}
	wxFont* GUIServices::getStandardFontFixedWidth(void)	{
		return m_pStandardFontFixedWidth;
	}
	wxFont* GUIServices::getStandardFontFixedWidthBold(void)	{
		return m_pStandardFontFixedWidthBold;
	}
	wxFont* GUIServices::getStandardFontFixedWidthBoldItalic(void)	{
		return m_pStandardFontFixedWidthBoldItalic;
	}
	wxFont* GUIServices::getStandardFontFixedWidthItalic(void)	{
		return m_pStandardFontFixedWidthItalic;
	}
	wxFont* GUIServices::getStandardFontItalic(void)	{
		return m_pStandardFontItalic;
	}
	wxFont* GUIServices::getSmallFont(void)	{
		return m_pSmallFont;
	}
	wxFont* GUIServices::getSmallFontBold(void)	{
		return m_pSmallFontBold;
	}
	wxFont* GUIServices::getVerySmallFont(void)	{
		return m_pVerySmallFont;
	}

} // namespace nmui