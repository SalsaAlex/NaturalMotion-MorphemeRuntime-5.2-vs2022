#include "widgets/wire/WireBox.h"


namespace nmx
{
	
namespace widgets
{
	DEFINE_WIDGET(WireBox,
				WireBoxManager,
				eWidgetType_WireBox);
				
	DEFINE_WIDGETMANAGER(WireBoxManager, eWidgetType_WireBox);
		
	
} // namespace widgets
	
} // namespace nmx