#include "widgets\DeferredWidget.h"

namespace nmx
{
	
namespace widgets
{
	
	class SolidDisc : public nmx::widgets::DeferredWidget
	{
	public:
		DECLARE_WIDGET();

	private:
		
	}
	
	class SolidDiscManager : public nmx::widgets::DeferredWidgetManager
	{
	public:
		eWidgettype type(void) override;
		
	private:
		
	}
	
} // namespace widgets
	
} // namespace nmx