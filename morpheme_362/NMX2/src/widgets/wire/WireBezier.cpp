#include "widgets/wire/WireBezier.h"


namespace nmx
{
	
namespace widgets
{
	DEFINE_WIDGET(WireBezier,
				WireBezierManager,
				eWidgetType_WireBezier);
				
	DEFINE_WIDGETMANAGER(WireBezierManager, eWidgetType_WireBezier);
		
	
} // namespace widgets
	
} // namespace nmx