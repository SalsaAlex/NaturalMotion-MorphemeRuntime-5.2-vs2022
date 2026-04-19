#include "widgets\DeferredWidget.h"

namespace nmx
{
	
namespace widgets
{
	
	BEGIN_DECLARE_WIDGET(WireWedge)
		
	END_DECLARE_WIDGET();
	
	class WireWedgeData
	{
	public:
		
	private:
		
	};

	BEGIN_DECLARE_WIDGETMANAGER(WireWedgeManager)
	
		WireWedgeData* getNextWireWedgeData(uint32_t unknown1);
		
	END_DECLARE_WIDGETMANAGER();
	
} // namespace widgets
	
} // namespace nmx