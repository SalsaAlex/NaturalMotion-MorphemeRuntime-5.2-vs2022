
namespace nmx
{
	
namespace gl
{

	//sizeof(RenderPass) == 112
	class RenderPass
	{
	public:
		RenderPass(uint32_t unknown1, 
			uint32_t unknown2, 
			nmx::gl::Device* device, 
			nmx::gl::RasterState* initialraster, 
			nmx::gl::DepthStencilState* initialdepthstencil,
			nmx::gl::BlendState* initialblend);
		~RenderPass(void);
		
		void renderInitial(const nmx::Viewport& viewport);
		void renderOverlay(const nmx::Viewport& viewport);
		void renderSolid(const nmx::Viewport& viewport);
		void renderPeel(const nmx::Viewport& viewport);
		void render(const nmx::Viewport& viewport);
		
		void initFaceCulling(nmx::gl::Device* device);
		void pushFaceCulling(const nmx::Viewport& viewport);
		void popFaceCulling(const nmx::Viewport& viewport);
		
		void update(const nmx::Viewport& viewport);
		
		void addDeferredObject(nmx::gl::DeferredRenderObject* object);
		
		bool isWirePass(void);
		bool isOverlayPass(void);
		bool isUnselectable(void);
		bool shouldClearDepthBuffer(void);
		
		void clear(void);
		void deleteInternal(void); //should we privatize this
		
		nmx::widgets::DeferredWidgetContainer* getWidgetContainer(uint32_t widgetId);

	private:
		
		
	}
	
} // namespace gl
	
} // namespace nmx