
namespace nmui
{
	//sizeof(GUIServices) == 1028
	class GUIServices
	{
	public:
		GUIServices(void);
		
		void init(void);
		void initFonts();
		void initBrushes();
		void initPens();
				
		void term(void);
		void update(void);
		
		
		wxBitmap* getArrowDown(void);
		wxBitmap* getArrowDownDisabled(void);
		wxBitmap* getArrowLeft(void);
		wxBitmap* getArrowLeftDisabled(void);
		wxBitmap* getArrowRight(void);
		wxBitmap* getArrowRightDisabled(void);
		wxBitmap* getArrowUp(void);
		wxBitmap* getArrowUpDisabled(void);
		wxBitmap* getCheckMarkTick(void);
		wxBitmap* getDarkArrowRight(void);
		wxBitmap* getDeleteCross(void);
		wxBitmap* getDotsBlockHorizontal(void);
		wxBitmap* getDotsBlockVertical(void);
		wxBitmap* getTick(void);
		wxBitmap* getTinyArrowDown(void);
		wxBitmap* getTinyArrowDownDisabled(void);
		wxBitmap* getTinyArrowLeft(void);
		wxBitmap* getTinyArrowLeftDisabled(void);
		wxBitmap* getTinyArrowRight(void);
		wxBitmap* getTinyArrowRightDisabled(void);
		wxBitmap* getTinyArrowUp(void);
		wxBitmap* getTinyArrowUpDisabled(void);
		
		
		wxBrush* get2x2CheckerStippleBrush(void);
		wxBrush* get8x8CheckerStippleBrush(void);
		wxBrush* getControlBackgroundBrush(void);
		wxBrush* getControlDarkTintBrush(void);
		wxBrush* getControlDarkerTintBrush(void);
		wxBrush* getControlDarkestTintBrush(void);
		wxBrush* getControlDisabledTextBrush(void);
		wxBrush* getControlEdgeBrush(void);
		wxBrush* getControlFocusBrush(void);
		wxBrush* getControlHeaderBackgroundBrush(void);
		wxBrush* getControlLighterTintBrush(void);
		wxBrush* getControlLightestTintBrush(void);
		wxBrush* getControlMixedBackgroundBrush(void);
		wxBrush* getControlScrollbarBrush(void);
		wxBrush* getControlScrollbarHighlightBrush(void);
		wxBrush* getControlSelectedBackgroundBrush(void);
		wxBrush* getControlSplitterBrush(void);
		wxBrush* getControlSplitterHighlightBrush(void);
		wxBrush* getDialogDarkTintBrush(void);
		wxBrush* getDialogDarkerTintBrush(void);
		wxBrush* getDialogDarkestTintBrush(void);
		wxBrush* GUIServices::getDialogDisabledTextBrush(void);
		wxBrush* GUIServices::getDialogEdgeBrush(void);
		wxBrush* GUIServices::getDialogFocusBrush(void);
		wxBrush* GUIServices::getDialogLighterTintBrush(void);
		wxBrush* GUIServices::getDialogLightestTintBrush(void);
		wxBrush* GUIServices::getDialogSelectedBackgroundBlendedBrush(void);
		wxBrush* GUIServices::getDialogSelectedBackgroundBrush(void);
		wxBrush* GUIServices::getDialogSelectedTextBrush(void);
		wxBrush* GUIServices::getDialogTextBrush(void);
		
		
		wxPen* GUIServices::getControlBlendedTextDottedPen(void);
		wxPen* GUIServices::getControlDarkShadowDottedPen(void);
		wxPen* GUIServices::getControlDarkTintPen(void);
		wxPen* GUIServices::getControlDarkerTintPen(void);
		wxPen* GUIServices::getControlDarkestTintPen(void);
		wxPen* GUIServices::getControlDisabledTextPen(void);
		wxPen* GUIServices::getControlEdgePen(void);
		wxPen* GUIServices::getControlErrorBackgroundPen(void);
		wxPen* GUIServices::getControlFocusPen(void);
		wxPen* GUIServices::getControlHeaderBackgroundPen(void);
		wxPen* GUIServices::getControlHeaderTextPen(void);
		wxPen* GUIServices::getControlLighterTintPen(void);
		wxPen* GUIServices::getControlLightestTintPen(void);
		wxPen* GUIServices::getControlMixedBackgroundPen(void);
		wxPen* GUIServices::getControlScrollbarHighlightPen(void);
		wxPen* GUIServices::getControlScrollbarPen(void);
		wxPen* GUIServices::getControlSelectedBackgroundPen(void);
		wxPen* GUIServices::getControlSelectedTextPen(void);
		wxPen* GUIServices::getControlSelectedUnfocusedPen(void);
		wxPen* GUIServices::getControlSelectedUnfocusedTextPen(void);
		wxPen* GUIServices::getControlSplitterHighlightPen(void);
		wxPen* GUIServices::getControlSplitterPen(void);
		wxPen* GUIServices::getControlTargetPen(void);
		wxPen* GUIServices::getControlTextDottedPen(void);
		wxPen* GUIServices::getControlTextPen(void);
		wxPen* GUIServices::getDialogDarkTintPen(void);
		wxPen* GUIServices::getDialogDarkerTintDottedPen(void);
		wxPen* GUIServices::getDialogDarkerTintPen(void);
		wxPen* GUIServices::getDialogDarkestTintPen(void);
		wxPen* GUIServices::getDialogDisabledTextPen(void);
		wxPen* GUIServices::getDialogEdgePen(void);
		wxPen* GUIServices::getDialogFocusPen(void);
		wxPen* GUIServices::getDialogLighterTintDottedPen(void);
		wxPen* GUIServices::getDialogLighterTintPen(void);
		wxPen* GUIServices::getDialogLightestTintPen(void);
		wxPen* GUIServices::getDialogSelectedBackgroundBlendedPen(void);
		wxPen* GUIServices::getDialogSelectedBackgroundPen(void);
		wxPen* GUIServices::getDialogSelectedTextPen(void);
		wxPen* GUIServices::getDialogTextPen(void);
		
		
		wxColour GUIServices::getControlBackgroundColour(void);
		wxColour GUIServices::getControlDarkTintColour(void);
		wxColour GUIServices::getControlDarkerTintColour(void);
		wxColour GUIServices::getControlDarkestTintColour(void);
		wxColour GUIServices::getControlDisabledTextColour(void);
		wxColour GUIServices::getControlEdgeColour(void);
		wxColour GUIServices::getControlErrorBackgroundColour(void);
		wxColour GUIServices::getControlFocusColour(void);
		wxColour GUIServices::getControlHeaderBackgroundColour(void);
		wxColour GUIServices::getControlHeaderTextColour(void);
		wxColour GUIServices::getControlLighterTintColour(void);
		wxColour GUIServices::getControlLightestTintColour(void);
		wxColour GUIServices::getControlMixedBackgroundColour(void);
		wxColour GUIServices::getControlScrollbarColour(void);
		wxColour GUIServices::getControlScrollbarHighlightColour(void);
		wxColour GUIServices::getControlSelectedBackgroundColour(void);
		wxColour GUIServices::getControlSelectedTextColour(void);
		wxColour GUIServices::getControlSelectedUnfocusedColour(void);
		wxColour GUIServices::getControlSelectedUnfocusedTextColour(void);
		wxColour GUIServices::getControlSplitterColour(void);
		wxColour GUIServices::getControlSplitterHighlightColour(void);
		wxColour GUIServices::getControlTextColour(void);
		wxColour GUIServices::getDialogBackgroundColour(void);
		wxColour GUIServices::getDialogDarkTintColour(void);
		wxColour GUIServices::getDialogDarkerTintColour(void);
		wxColour GUIServices::getDialogDarkestTintColour(void);
		wxColour GUIServices::getDialogDisabledTextColour(void);
		wxColour GUIServices::getDialogEdgeColour(void);
		wxColour GUIServices::getDialogFocusColour(void);
		wxColour GUIServices::getDialogLighterTintColour(void);
		wxColour GUIServices::getDialogLightestTintColour(void);
		wxColour GUIServices::getDialogSelectedBackgroundBlendedColour(void);
		wxColour GUIServices::getDialogSelectedBackgroundColour(void);
		wxColour GUIServices::getDialogSelectedTextColour(void);
		wxColour GUIServices::getDialogTextColour(void);
		
		
		int GUIServices::getControlTargetSize(void);
		
		
		wxFont* getFont(nmui::StaticTextFont fonttype);
		wxFont* getLargeFont(void);
		wxFont* getLargeFontBold(void);
		wxFont* getStandardFont(void);
		wxFont* getStandardFontBold(void);
		wxFont* getStandardFontBoldItalic(void);
		wxFont* getStandardFontFixedWidth(void);
		wxFont* getStandardFontFixedWidthBold(void);
		wxFont* getStandardFontFixedWidthBoldItalic(void);
		wxFont* getStandardFontFixedWidthItalic(void);
		wxFont* getStandardFontItalic(void);
		wxFont* getSmallFont(void);
		wxFont* getSmallFontBold(void);
		wxFont* getVerySmallFont(void);
	
	private:
	
		wxColour m_controlTextColour; //(char *)this + 4 || (_DWORD *)this + 1
		wxColour m_controlBackgroundColour; //(char *)this + 20 || (_DWORD *)this + 5
		wxColour m_controlMixedBackgroundColour; //(char *)this + 36 || (_DWORD *)this + 9
		wxColour m_controlErrorBackgroundColour; //(char *)this + 52 || (_DWORD *)this + 13
		wxColour m_controlSelectedTextColour; //(char *)this + 68 || (_DWORD *)this + 17
		wxColour m_controlSelectedBackgroundColour; //(char *)this + 84 || (_DWORD *)this + 21
		wxColour m_controlSelectedUnfocusedColour; //(char *)this + 100 || (_DWORD *)this + 25
		wxColour m_controlSelectedUnfocusedTextColour; //(char *)this + 116 || (_DWORD *)this + 29
		wxColour m_controlHeaderTextColour; //(char *)this + 132 || (_DWORD *)this + 33
		wxColour m_controlHeaderBackgroundColour; //(char *)this + 148 || (_DWORD *)this + 37
		wxColour m_controlEdgeColour; //(char *)this + 164 || (_DWORD *)this + 41
		wxColour m_controlScrollbarColour; //(char *)this + 180 || (_DWORD *)this + 45
		wxColour m_controlSplitterColour; //(char *)this + 196 || (_DWORD *)this + 49
		wxColour m_dialogTextColour; //(char *)this + 212 || (_DWORD *)this + 53
		wxColour m_dialogBackgroundColour; //(char *)this + 228 || (_DWORD *)this + 57
		wxColour m_dialogSelectedTextColour; //(char *)this + 244 || (_DWORD *)this + 61
		wxColour m_dialogSelectedBackgroundColour; //(char *)this + 260 || (_DWORD *)this + 65
		wxColour m_dialogSelectedBackgroundBlendedColour; //(char *)this + 276 || (_DWORD *)this + 69
		wxColour m_dialogEdgeColour; //(char *)this + 292 || (_DWORD *)this + 73
		wxColour m_controlDisabledTextColour; //(char *)this + 308 || (_DWORD *)this + 77
		wxColour m_controlScrollbarHighlightColour; //(char *)this + 324 || (_DWORD *)this + 81
		wxColour m_controlSplitterHighlightColour; //(char *)this + 340 || (_DWORD *)this + 85
		wxColour m_dialogDisabledTextColour; //(char *)this + 356 || (_DWORD *)this + 89
		wxColour m_controlLightestTintColour; //(char *)this + 372 || (_DWORD *)this + 93
		wxColour m_controlLighterTintColour; //(char *)this + 388 || (_DWORD *)this + 97
		wxColour m_controlDarkTintColour; //(char *)this + 404 || (_DWORD *)this + 101
		wxColour m_controlDarkerTintColour; //(char *)this + 420 || (_DWORD *)this + 105
		wxColour m_controlDarkestTintColour; //(char *)this + 436 || (_DWORD *)this + 109
		wxColour m_dialogLightestTintColour; //(char *)this + 452 || (_DWORD *)this + 113
		wxColour m_dialogLighterTintColour; //(char *)this + 468 || (_DWORD *)this + 117
		wxColour m_dialogDarkTintColour; //(char *)this + 484 || (_DWORD *)this + 121
		wxColour m_dialogDarkerTintColour; //(char *)this + 500 || (_DWORD *)this + 125
		wxColour m_dialogDarkestTintColour; //(char *)this + 516 || (_DWORD *)this + 129
		wxColour m_controlFocusColour; //(char *)this + 532 || (_DWORD *)this + 133
		wxColour m_dialogFocusColour; //(char *)this + 548 || (_DWORD *)this + 137
		
		
		
		wxBrush* m_controlBackgroundBrush; //(char *)this + 568 || (_DWORD *)this + 142
		wxBrush* m_controlMixedBackgroundBrush; //(char *)this + 572 || (_DWORD *)this + 143
		void* m_unknown1; //(char *)this + 576 || (_DWORD *)this + 144
		void* m_unknown2; //(char *)this + 580 || (_DWORD *)this + 145
		wxBrush* m_controlSelectedBackgroundBrush; //(char *)this + 584 || (_DWORD *)this + 146
		void* m_unknown4; //(char *)this + 588 || (_DWORD *)this + 147
		void* m_unknown5; //(char *)this + 592 || (_DWORD *)this + 148
		void* m_unknown6; //(char *)this + 596 || (_DWORD *)this + 149
		wxBrush* m_controlHeaderBackgroundBrush; //(char *)this + 600 || (_DWORD *)this + 150
		wxBrush* m_controlEdgeBrush; //(char *)this + 604 || (_DWORD *)this + 151
		wxBrush* m_controlScrollbarBrush; //(char *)this + 608 || (_DWORD *)this + 152
		wxBrush* m_controlScrollbarHighlightBrush; //(char *)this + 612 || (_DWORD *)this + 153
		wxBrush* m_controlSplitterBrush; //(char *)this + 616 || (_DWORD *)this + 154
		wxBrush* m_controlSplitterHighlightBrush; //(char *)this + 620 || (_DWORD *)this + 155
		wxBrush* m_dialogTextBrush; //(char *)this + 624 || (_DWORD *)this + 156
		void* m_unknown7; //(char *)this + 628 || (_DWORD *)this + 157
		wxBrush* m_dialogSelectedTextBrush; //(char *)this + 632 || (_DWORD *)this + 158
		wxBrush* m_dialogSelectedBackgroundBrush; //(char *)this + 636 || (_DWORD *)this + 159
		wxBrush* m_dialogSelectedBackgroundBlendedBrush; //(char *)this + 640 || (_DWORD *)this + 160
		wxBrush* m_dialogEdgeBrush; //(char *)this + 644 || (_DWORD *)this + 161
		wxBrush* m_controlDisabledTextBrush; //(char *)this + 648 || (_DWORD *)this + 162
		wxBrush* m_dialogDisabledTextBrush; //(char *)this + 652 || (_DWORD *)this + 163
		wxBrush* m_controlLightestTintBrush; //(char *)this + 656 || (_DWORD *)this + 164
		wxBrush* m_controlLighterTintBrush; //(char *)this + 660 || (_DWORD *)this + 165
		wxBrush* m_controlDarkTintBrush; //(char *)this + 664 || (_DWORD *)this + 166
		wxBrush* m_controlDarkerTintBrush; //(char *)this + 668 || (_DWORD *)this + 167
		wxBrush* m_controlDarkestTintBrush; //(char *)this + 672 || (_DWORD *)this + 168
		wxBrush* m_dialogLightestTintBrush; //(char *)this + 676 || (_DWORD *)this + 169
		wxBrush* m_dialogLighterTintBrush; //(char *)this + 680 || (_DWORD *)this + 170
		wxBrush* m_dialogDarkTintBrush; //(char *)this + 684 || (_DWORD *)this + 171
		wxBrush* m_dialogDarkerTintBrush; //(char *)this + 688 || (_DWORD *)this + 172
		wxBrush* m_dialogDarkestTintBrush; //(char *)this + 692 || (_DWORD *)this + 173
		wxBrush* m_controlFocusBrush; //(char *)this + 696 || (_DWORD *)this + 174
		wxBrush* m_dialogFocusBrush; //(char *)this + 700 || (_DWORD *)this + 175
		wxBrush* m_2x2CheckerStippleBrush; //(char *)this + 704 || (_DWORD *)this + 176
		wxBrush* m_8x8CheckerStippleBrush; //(char *)this + 708 || (_DWORD *)this + 177
		
		
		
		wxPen* m_controlTextPen; //(char *)this + 712 || (_DWORD *)this + 178
		void* m_unknown36; //(char *)this + 716 || (_DWORD *)this + 179
		wxPen* m_controlMixedBackgroundPen; //(char *)this + 720 || (_DWORD *)this + 180
		wxPen* m_controlErrorBackgroundPen; //(char *)this + 724 || (_DWORD *)this + 181
		wxPen* m_controlSelectedTextPen; //(char *)this + 728 || (_DWORD *)this + 182
		wxPen* m_controlSelectedBackgroundPen; //(char *)this + 732 || (_DWORD *)this + 183
		wxPen* m_controlSelectedUnfocusedPen; //(char *)this + 736 || (_DWORD *)this + 184
		wxPen* m_controlSelectedUnfocusedTextPen; //(char *)this + 740 || (_DWORD *)this + 185
		wxPen* m_controlHeaderTextPen; //(char *)this + 744 || (_DWORD *)this + 186
		wxPen* m_controlHeaderBackgroundPen; //(char *)this + 748 || (_DWORD *)this + 187
		wxPen* m_controlEdgePen; //(char *)this + 752 || (_DWORD *)this + 188
		wxPen* m_controlScrollbarPen; //(char *)this + 756 || (_DWORD *)this + 189
		wxPen* m_controlScrollbarHighlightPen; //(char *)this + 760 || (_DWORD *)this + 190
		wxPen* m_controlSplitterPen; //(char *)this + 764 || (_DWORD *)this + 191
		wxPen* m_controlSplitterHighlightPen; //(char *)this + 768 || (_DWORD *)this + 192
		wxPen* m_controlTargetPen; //(char *)this + 772 || (_DWORD *)this + 193
		wxPen* m_dialogTextPen; //(char *)this + 776 || (_DWORD *)this + 194
		wxPen* m_dialogSelectedTextPen; //(char *)this + 784 || (_DWORD *)this + 196
		wxPen* m_dialogSelectedBackgroundPen; //(char *)this + 788 || (_DWORD *)this + 197
		wxPen* m_dialogSelectedBackgroundBlendedPen; //(char *)this + 792 || (_DWORD *)this + 198
		wxPen* m_dialogEdgePen; //(char *)this + 796 || (_DWORD *)this + 199
		wxPen* m_controlDisabledTextPen; //(char *)this + 800 || (_DWORD *)this + 200
		wxPen* m_dialogDisabledTextPen; //(char *)this + 804 || (_DWORD *)this + 201
		wxPen* m_controlLightestTintPen; //(char *)this + 808 || (_DWORD *)this + 202
		wxPen* m_controlLighterTintPen; //(char *)this + 812 || (_DWORD *)this + 203
		wxPen* m_controlDarkTintPen; //(char *)this + 816 || (_DWORD *)this + 204
		wxPen* m_controlDarkerTintPen; //(char *)this + 820 || (_DWORD *)this + 205
		wxPen* m_controlDarkestTintPen; //(char *)this + 824 || (_DWORD *)this + 206
		wxPen* m_dialogLightestTintPen; //(char *)this + 828 || (_DWORD *)this + 207
		wxPen* m_dialogLighterTintPen; //(char *)this + 832 || (_DWORD *)this + 208
		wxPen* m_dialogDarkTintPen; //(char *)this + 836 || (_DWORD *)this + 209
		wxPen* m_dialogDarkerTintPen; //(char *)this + 840 || (_DWORD *)this + 210
		wxPen* m_dialogDarkestTintPen; //(char *)this + 844 || (_DWORD *)this + 211
		wxPen* m_controlFocusPen; //(char *)this + 848 || (_DWORD *)this + 212
		wxPen* m_dialogFocusPen; //(char *)this + 852 || (_DWORD *)this + 213
		wxPen* m_dialogLighterTintDottedPen; //(char *)this + 856 || (_DWORD *)this + 214
		wxPen* m_dialogDarkerTintDottedPen; //(char *)this + 860 || (_DWORD *)this + 215
		wxPen* m_controlDarkShadowDottedPen; //(char *)this + 864 || (_DWORD *)this + 216
		wxPen* m_controlTextDottedPen; //(char *)this + 868 || (_DWORD *)this + 217
		wxPen* m_controlBlendedTextDottedPen; //(char *)this + 872 || (_DWORD *)this + 218
		
		
	
		wxBitmap* m_pTick; //(char *)this + 876 || (_DWORD *)this + 219
		wxBitmap* m_pIndeterminateCheckMarkTick; //(char *)this + 888 || (_DWORD *)this + 222
		wxBitmap* m_pDarkArrowRight; //(char *)this + 892 || (_DWORD *)this + 223
		wxBitmap* m_pArrowUp; //(char *)this + 896 || (_DWORD *)this + 224
		wxBitmap* m_pArrowDown; //(char *)this + 900 || (_DWORD *)this + 225
		wxBitmap* m_pArrowLeft; //(char *)this + 904 || (_DWORD *)this + 226
		wxBitmap* m_pArrowRight; //(char *)this + 908 || (_DWORD *)this + 227
		wxBitmap* m_pArrowUpDisabled; //(char *)this + 912 || (_DWORD *)this + 228
		wxBitmap* m_pArrowDownDisabled; //(char *)this + 916 || (_DWORD *)this + 229
		wxBitmap* m_pArrowLeftDisabled; //(char *)this + 920 || (_DWORD *)this + 230
		wxBitmap* m_pArrowRightDisabled; //(char *)this + 924 || (_DWORD *)this + 231
		wxBitmap* m_pTinyArrowUp; //(char *)this + 928 || (_DWORD *)this + 232
		wxBitmap* m_pTinyArrowDown; //(char *)this + 932 || (_DWORD *)this + 233
		wxBitmap* m_pTinyArrowLeft; //(char *)this + 936 || (_DWORD *)this + 234
		wxBitmap* m_pTinyArrowRight; //(char *)this + 940 || (_DWORD *)this + 235
		wxBitmap* m_pTinyArrowUpDisabled; //((char *)this + 944 || _DWORD *)this + 236
		wxBitmap* m_pTinyArrowDownDisabled; //(char *)this + 948 || (_DWORD *)this + 237
		wxBitmap* m_pTinyArrowLeftDisabled; //(char *)this + 952 || (_DWORD *)this + 238
		wxBitmap* m_pTinyArrowRightDisabled; //(char *)this + 956 || (_DWORD *)this + 239
		wxBitmap* m_pDotsBlockHorizontal; //(char *)this + 960 || (_DWORD *)this + 240
		wxBitmap* m_pDotsBlockVertical; //(char *)this + 964 || (_DWORD *)this + 241
		wxBitmap* m_pDeleteCross; //(char *)this + 968 || (_DWORD *)this + 242
		
		
	
		wxFont* m_pStandardFont; //(char *)this + 972 || (_DWORD *)this + 243
		wxFont* m_pStandardFontBold; //(char *)this + 976 || (_DWORD *)this + 244
		wxFont* m_pStandardFontItalic; //(char *)this + 980 || (_DWORD *)this + 245
		wxFont* m_pStandardFontBoldItalic; //(char *)this + 984 || (_DWORD *)this + 246
		wxFont* m_pStandardFontFixedWidth; //(char *)this + 988 || (_DWORD *)this + 247
		wxFont* m_pStandardFontFixedWidthBold; //(char *)this + 992 || (_DWORD *)this + 248
		wxFont* m_pStandardFontFixedWidthItalic; //(char *)this + 996 || (_DWORD *)this + 249
		wxFont* m_pStandardFontFixedWidthBoldItalic; //(char *)this + 1000 || (_DWORD *)this + 250
		wxFont* m_pLargeFont; //(char *)this + 1004 || (_DWORD *)this + 251
		wxFont* m_pLargeFontBold; //(char *)this + 1008 || (_DWORD *)this + 252
		wxFont* m_pSmallFont; //(char *)this + 1012 || (_DWORD *)this + 253
		wxFont* m_pSmallFontBold; //(char *)this + 1016 || (_DWORD *)this + 254
		wxFont* m_pVerySmallFont; //(char *)this + 1020 || (_DWORD *)this + 255
		void* m_unknown37; //(char *)this + 1024 || (_DWORD *)this + 256
	}
	
} // namespace nmui