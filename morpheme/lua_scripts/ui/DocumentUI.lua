------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

-- ensure certain scripts are loaded before this
require "ui/AssetManager/AssetManager.lua"
require "ui/AssetAttributeEditor.lua"
require "ui/AttributeEditor/AttributeEditor.lua"
require "ui/AttributeEditor/AnimationSetsAttributeEditor.lua"
require "ui/AttributeEditor/AnimationTakeEditor.lua"
require "ui/Network.lua"
require "ui/ScaleOptions.lua"
require "ui/ConnectCParam.lua"
require "ui/ConnectionNavigator.lua"
require "ui/tools/ToolSettingsPanel.lua"
--require "ui/Navigator.lua"

------------------------------------------------------------------------------------------------------------------------
local filterSelection = function(assetManager, selectedResources)
  local result = { }
  local hasTakes = null
  for resourceId in selectedResources do
    if anim.getResourceType(resourceId) == "take" then
      result[resourceId] = true
      hasTakes = true
    end
  end

  if not hasTakes then
    local currentAnimation = assetManager:getCurrentAnimation()
    result[currentAnimation] = true
  end

  return result
end

------------------------------------------------------------------------------------------------------------------------
local addControlParametersWindow = function(layoutManager)
  local controlPanel = layoutManager:addPanel{ name = "Controls", caption = "Controls", flags = "expand", proportion = 1 }
  
  controlPanel:beginFlexGridSizer{ cols = 1, flags = "expand", proportion = 1 }
    controlPanel:setFlexGridRowExpandable(1)
    controlPanel:setFlexGridColumnExpandable(1)
  
    controlPanel:addStockWindow{ type = "Controls", name = "Controls", flags = "expand", proportion = 1 }
  controlPanel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- createDocumentUI
------------------------------------------------------------------------------------------------------------------------
createDocumentUI = function()
  local layoutManager = ui.getWindow("MainFrame|LayoutManager")

  --Add main app windows.

  -- Create the log first so if other windows fail to add we can see why
  layoutManager:addStockWindow{ type = "Log", name = "Log" }

  -- Network Editor
  local networkEditor = safefunc(addNetwork, layoutManager)
  ui.setWantsSelectionContext(networkEditor)

  safefunc(addConnectControlParamWindow, layoutManager)
  safefunc(addConnectionNavigatorWindow, layoutManager)
  --safefunc(addNavigatorWindow, layoutManager)

  -- Network Viewport
  local networkViewPort = safefunc(addViewport, layoutManager)
  ui.setWantsSelectionContext(networkViewPort)

  layoutManager:addStockWindow{ type = "PreviewScript", name = "Preview Script" }
  layoutManager:addStockWindow{ type = "Palette", name = "Palette" }
  addControlParametersWindow(layoutManager)
  layoutManager:addStockWindow{ type = "LayerManager", name = "Layer Manager" }
  layoutManager:addStockWindow{ type = "AppScriptEditor", name = "App Script" }
  layoutManager:addStockWindow{ type = "LiveErrorLog", name = "Network Errors" }
  layoutManager:addStockWindow{ type = "RuntimeDebugging", name = "Runtime Debugging" }
  local assetManager = safefunc(addAssetManager, layoutManager)
  safefunc(addToolSettingsPanel, layoutManager)

  -- Functions to deal with chooser and file tree selection
  local getChooserSelection = function()
    -- getting the asset manager explicitly stops the callback being called on a dead
    -- reference to an asset manager window from a previously loaded mcn.
    local assetManager = ui.getWindow("MainFrame|LayoutManager|AssetManager")
    if assetManager then
      return filterSelection(assetManager, assetManager:getChooserSelection())
    end
    return { }
  end
  local setOnChooserSelectionChanged = function(theFunc) assetManager:setOnChooserSelectionChanged(theFunc) end
  local getFileTreeSelection = function() return filterSelection(assetManager, assetManager:getFileTreeSelection()) end
  local setOnFileTreeSelectionChanged = function(theFunc) assetManager:setOnFileTreeSelectionChanged(theFunc) end

  -- Context for the asset manager viewport
  local assetViewportContext = assetManager:getAssetManagerViewport();
  ui.setWantsSelectionContext(assetViewportContext)

  layoutManager:addStockWindow{ type = "BodyGroupEditor", name = "Body Groups", associatedContext = assetViewportContext }
  
  -- dont add collision editor if physics isnt enabled.
  if not mcn.isPhysicsDisabled() then
    layoutManager:addStockWindow{ type = "CollisionGroupEditor", name = "Collision Groups", associatedContext = assetViewportContext }
  end
  
  -- Context for the animationSets
  local animationSetsContext = assetManager:getAnimationSetsPanel()
  ui.setWantsSelectionContext(animationSetsContext)

  -- Context for the components
  local componentsContext = assetManager:getComponentsPanel()
  addAssetManagerComponentSection(componentsContext);
  ui.setWantsSelectionContext(componentsContext)

  -- Context for the animationFileTree
  local animationFileTreeContext = assetManager:getAnimationFileTreePanel()
  ui.setWantsSelectionContext(animationFileTreeContext)

  -- Context for the chooser
  local chooserContext = assetManager:getChooserPanel()
  ui.setWantsSelectionContext(chooserContext)

  -- Navigator (contextual)
  local navigator = layoutManager:addStockWindow{ type = "ContextualPanel", name = "Navigator" }
  safefunc(addNetworkNavigator, navigator, networkEditor)
  navigator:addStockWindow{ type = "AssetSceneExplorer", name = "AssetSceneExplorer", forContext = assetViewportContext }
  navigator:addStockWindow{ type = "ViewportSceneExplorer", name = "ViewportSceneExplorer", forContext = networkViewPort }

  -- Attribute Editor (contextual)
  local attributeEditor = layoutManager:addStockWindow{ type = "ContextualPanel", name = "AttributeEditor" }
  attributeEditor:addStockWindow{ type = "ViewportAttributeEditor", name = "ViewportAttributes", forContext = networkViewPort }
  safefunc(addAssetAttributeEditor, attributeEditor, assetViewportContext)
  safefunc(addComponentAttributeEditor, attributeEditor, componentsContext)
  safefunc(addAttributeEditor, attributeEditor, networkEditor)
  safefunc(addAnimationSetAttributeEditor, attributeEditor, animationSetsContext, assetManager)
  safefunc(addAnimationTakeEditor, attributeEditor, chooserContext, assetManager, getChooserSelection, setOnChooserSelectionChanged)
  safefunc(addAnimationTakeEditor, attributeEditor, animationFileTreeContext, assetManager, getFileTreeSelection, setOnFileTreeSelectionChanged)

  -- now init the user document ui if there is one.
  safefunc(userInitDocumentUI)
end