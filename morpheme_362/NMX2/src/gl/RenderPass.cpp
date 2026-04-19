#include "gl/RenderPass.h"

namespace nmx
{
	
namespace gl
{

	RenderPass::RenderPass(uint32_t unknown1, 
		uint32_t unknown2, 
		nmx::gl::Device* device, 
		nmx::gl::RasterState* initialraster, 
		nmx::gl::DepthStencilState* initialdepthstencil,
		nmx::gl::BlendState* initialblend)
	{
		//undone
		
		initFaceCulling(device);
	}
	~RenderPass(void)
	{
		clear();
		if(m_widgetcontainer)
			delete m_widgetcontainer;
		
		//undone
	}
	
	void RenderPass::renderInitial(const nmx::Viewport& viewport)
	{
		
	}
	void RenderPass::renderOverlay(const nmx::Viewport& viewport)
	{
		if ( shouldClearDepthBuffer() )
			glClear(GL_DEPTH_BUFFER_BIT);
	
		m_widgetcontainer->renderWidgets(viewport, false);
		glDisable(GL_POLYGON_STIPPLE);
		glDisable(GL_LINE_STIPPLE);
		glDisable(GL_LINE_SMOOTH);
		glDisable(GL_CULL_FACE);
	}
	void RenderPass::renderSolid(const nmx::Viewport& viewport)
	{
		
	}
	void RenderPass::renderPeel(const nmx::Viewport& viewport)
	{
		
	}
	void RenderPass::render(const nmx::Viewport& viewport)
	{
		
	}
			
	void RenderPass::initFaceCulling(nmx::gl::Device* device)
	{
		
	}
	void RenderPass::pushFaceCulling(const nmx::Viewport& viewport)
	{
		
	}
	void RenderPass::popFaceCulling(const nmx::Viewport& viewport)
	{
		
	}
	
	void RenderPass::update(const nmx::Viewport& viewport)
	{
		
	}
	
	void RenderPass::addDeferredObject(nmx::gl::DeferredRenderObject* object)
	{
		
	}
	
	bool RenderPass::isWirePass(void)
	{
		return m_renderflags & RENDERFLAGS_WIREPASS;
	}
	bool RenderPass::isOverlayPass(void)
	{
		return m_renderflags & RENDERFLAGS_OVERLAYPASS;
	}
	bool RenderPass::isUnselectable(void)
	{
		return m_renderflags & RENDERFLAGS_UNSELECTABLE;
	}
	bool RenderPass::shouldClearDepthBuffer(void)
	{
		return m_renderflags & RENDERFLAGS_CLEARDEPTHBUFFER;
	}
	
	void RenderPass::clear(void)
	{
		
	}
	void RenderPass::deleteInternal(void)
	{
		
	}
	
	nmx::widgets::DeferredWidgetContainer* RenderPass::getWidgetContainer(uint32_t widgetId)
	{
		m_widgetcontainer->containerFromWidgetId(widgetId);
	}
	
} // namespace gl
	
} // namespace nmx