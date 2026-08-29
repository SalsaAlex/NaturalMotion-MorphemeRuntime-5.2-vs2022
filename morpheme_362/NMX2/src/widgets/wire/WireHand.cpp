#include "widgets/wire/WireHand.h"


namespace nmx
{
	
namespace widgets
{
	DEFINE_WIDGET(WireHand,
				WireHandManager,
				eWidgetType_WireHand);
				
	DEFINE_WIDGETMANAGER(WireHandManager, eWidgetType_WireHand);
		
	
} // namespace widgets
	
} // namespace nmx