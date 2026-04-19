#include "widgets/WidgetBase.h"


namespace nmx
{
	
namespace widgets
{
#define BEGIN_DECLARE_WIDGET(widgetname) \
	class widgetname : public nmx::widgets::DeferredWidget \
	{ \
	public: \
		static nmx::widgets::DeferredWidget* creator(void); \
		static nmx::widgets::DeferredWidgetManager* createManager(void); \
		static void registerWidget(nmx::widgets::WidgetFactory& factory); 

#define END_DECLARE_WIDGET() \
	}



#define BEGIN_DECLARE_WIDGETMANAGER(widgetmanagername) \
	class widgetmanagername : public nmx::widgets::DeferredWidgetManager \
	{ \
	public: \
		eWidgettype type(void) override;



#define END_DECLARE_WIDGETMANAGER() \
	}\




#define DEFINE_WIDGET(widgetname, widgetmanagername, widgettype) \
		nmx::widgets::DeferredWidget* widgetname::creator(void) \
		{ \
			return new widgetname(); \
		} \
		nmx::widgets::DeferredWidgetManager* widgetname::createManager(void) \
		{ \
			return new widgetmanagername(); \
		} \
		void widgetname::registerWidget(nmx::widgets::WidgetFactory& factory) \
		{ \
			nmx::widgets::DeferredWidgetDesc widgetdesc(widgettype, false, widgetname::creator, widgetname::createManager); \
			factory.registerWidget(widgetdesc); \
		}


#define DEFINE_WIDGETMANAGER(widgetmanagername, widgettype) \
		eWidgettype widgetmanagername::type(void) \
		{ \
			return widgettype; \
		}
	
	
	typedef technique_constructor nmx::gl::Technique * (*)(void);
	enum eWidgettype
	{
		eWidgetType_Grid = 0,

		eWidgetType_WireArrow,
		eWidgetType_WireBezier,
		eWidgetType_WireBox,
		eWidgetType_WireCapsule,
		eWidgetType_WireCircle,
		eWidgetType_WireCone,
		eWidgetType_WireCylinder,
		eWidgetType_WireDisc,
		eWidgetType_WireFoot,
		eWidgetType_WireHand,
		eWidgetType_WireHead,
		eWidgetType_WireHemiSphere,
		eWidgetType_unknown1,
		eWidgetType_WirePelvis,
		eWidgetType_WireQuad,
		eWidgetType_WireRectangle,
		eWidgetType_WireSlopedCylinder,
		eWidgetType_WireSphere,
		eWidgetType_WireSpineOrient,
		eWidgetType_WireSphereSegment,
		eWidgetType_WireTriangle,
		eWidgetType_unknown2,
		eWidgetType_WireWedge,

		eWidgetType_SolidArrow,
		eWidgetType_SolidBox,
		eWidgetType_SolidCappedCone,
		eWidgetType_SolidCappedCylinder,
		eWidgetType_SolidCapsule,
		eWidgetType_SolidCircle,
		eWidgetType_SolidCone,
		eWidgetType_SolidCylinder,
		eWidgetType_SolidDisc,
		eWidgetType_SolidHemiSphere,
		eWidgetType_SolidQuad,
		eWidgetType_SolidRectangle,
		eWidgetType_SolidSlopedCylinder,
		eWidgetType_SolidSphere,
		eWidgetType_SolidTriangle,
		eWidgetType_SolidSixDofJoint,
		eWidgetType_SolidWedge,

		eWidgetType_WireOnSolidArrow,
		eWidgetType_WireOnSolidBox,
		eWidgetType_WireOnSolidCappedCone,
		eWidgetType_WireOnSolidCappedCylinder,
		eWidgetType_WireOnSolidCapsule,
		eWidgetType_WireOnSolidCircle,
		eWidgetType_WireOnSolidCone,
		eWidgetType_WireOnSolidCylinder,
		eWidgetType_WireOnSolidDisc,
		eWidgetType_WireOnSolidHemiSphere,
		eWidgetType_WireOnSolidQuad,
		eWidgetType_WireOnSolidRectangle,
		eWidgetType_WireOnSolidSlopedCylinder,
		eWidgetType_WireOnSolidSphere,
		eWidgetType_WireOnSolidTriangle,
		eWidgetType_WireOnSolidSixDofJoint,
		eWidgetType_WireOnSolidWedge,


	};
	
	//sizeof(DeferredWidget) == 48
	class DeferredWidget : public nmx::widgets::WidgetBase
	{
	public:

	private:

	};
	
	class DeferredWidgetManager
	{
	public:
		void init(nmx::gl::Device* device);
		virtual void update(nmx::gl::Device* device,
			const nmx::Ray& ray,
			nmtl::pod_vector<nmx::widgets::DepthSortedEntry>& depthsort1,
			nmtl::pod_vector<nmx::widgets::DepthSortedEntry>& depthsort2,
			nmtl::pod_vector<nmx::widgets::DeferredTransparentWidgetContainer>& transparentsort);
		void clear(nmx::widgets::DeferredWidgetContainer* container);
		void destory(nmx::gl::Device* device);

		void registerWidgetTechnique(nmx::gl::Technique::Api techapi, technique_constructor constructor);

		nmx::widgets::DeferredWidgetContainer* getContainer(uint32_t unknown1);

		bool render(const nmx::Viewport* viewport, nmx::widgets::DeferredWidgetContainer* container, bool unknownbool);

		virtual eWidgettype type(void) = 0;

	private:

	};
	
} // namespace widgets
	
} // namespace nmx