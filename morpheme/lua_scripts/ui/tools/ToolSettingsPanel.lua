------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------
require "ui/tools/ToolSettingsPages.lua"
require "ui/NMXAttributeWidgets.lua"

local lastViewportContext = "AssetManagerViewport"
------------------------------------------------------------------------------------------------------------------------
-- gets the currently selected viewport context, if none are currently selected returns
-- the last selected.
------------------------------------------------------------------------------------------------------------------------
local getCurrentViewport = function()
  -- the asset manager panel handles getShown properly, the asset manager viewport always returns true.
  --
  local assetManagerPanel = ui.getWindow("MainFrame|LayoutManager|AssetManager")
  if not assetManagerPanel then
    return nil
  end
  local assetManagerViewport = assetManagerPanel:getAssetManagerViewport()

  -- the preview panel handles getShown properly, the preview viewport always returns true.
  --
  local previewPanel = ui.getWindow("MainFrame|LayoutManager|PreviewViewport")
  if not previewPanel then
    return nil
  end
  local previewViewport = ui.getWindow("MainFrame|LayoutManager|PreviewViewport|Viewport|Viewport")

  local currentSelectionContext = ui.getCurrentContextName()
  if currentSelectionContext == "PreviewViewport" or
     currentSelectionContext == "AssetManagerViewport" then
    lastViewportContext = currentSelectionContext
  end

  if lastViewportContext == "AssetManagerViewport" then
    -- if the asset manager viewport was the last selected context but only the preview viewport is
    -- visible then return the preview viewport instead
    --
    if not assetManagerPanel:isShown() and previewPanel:isShown() then
      return previewViewport
    else
      return assetManagerViewport
    end
  elseif lastViewportContext == "PreviewViewport" then
    -- if the preview viewport was the last selected context but only the asset manager viewport is
    -- visible then return the asset manager viewport instead
    --
    if not previewPanel:isShown() and assetManagerPanel:isShown() then
      return assetManagerViewport
    else
      return previewViewport
    end
  end

  return nil
end

------------------------------------------------------------------------------------------------------------------------
-- get the currently selected tool for the last selected viewport
------------------------------------------------------------------------------------------------------------------------
local getCurrentTool = function()
  local viewportName = getCurrentViewport()

  local viewport = getCurrentViewport()
  if viewport then
    local currentTool = viewport:getCurrentTool()
    return currentTool
  end

  return nil
end

local currentTool
if not mcn.inCommandLineMode() then
  currentTool = getCurrentTool()
end

------------------------------------------------------------------------------------------------------------------------
-- updates the tool settings panel for the currently selected tool
------------------------------------------------------------------------------------------------------------------------
local updateToolSettingsPanel = function()
  local panel = ui.getWindow("MainFrame|LayoutManager|ToolSettings")
  if panel then
    local scrollPanel = panel:getChild("ToolSettingsPanel")
    if scrollPanel then
      local container = scrollPanel:getChild("ToolsRollupContainer")
      local rollup = container:getChild("CommonToolSettings")
      local rollupPanel = rollup:getPanel()
      
      local app = nmx.Application.new()
      local roamingSettings = app:getRoamingUserSettings()
      local globalRoamingSettings = roamingSettings:getNodeFromPath("|GlobalSettings")

      nmx.updateAttributeWidget(rollupPanel, globalRoamingSettings:findAttribute("ManipSize"))
      nmx.updateAttributeWidget(rollupPanel, globalRoamingSettings:findAttribute("ManipHandleSize"))

      local page = getToolSettingsPage(currentTool)
      if page then
        local pageRollup = container:getChild(string.format("%sSettings", currentTool))
        local pageRollupPanel = pageRollup:getPanel()
        safefunc(page.update, pageRollupPanel, page)
      end
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
-- rebuilds the tool settings panel for the currently selected tool
------------------------------------------------------------------------------------------------------------------------
rebuildToolSettingsPanel = function()
  local toolSettings = ui.getWindow("MainFrame|LayoutManager|ToolSettings")
  if toolSettings then
    local newTool = getCurrentTool()
    if currentTool ~= newTool then
      toolSettings:suspendLayout()
      toolSettings:freeze()
      toolSettings:clear()
      toolSettings:setBorder(0)

      local scrollPanel = toolSettings:addScrollPanel{
        name = "ToolSettingsPanel",
        flags = "expand;vertical",
        proportion = 1,
      }

      local container = scrollPanel:addRollupContainer{
        name = "ToolsRollupContainer",
        flags = "expand",
      }

      local rollup = container:addRollup{
        label = "Common Tool Settings",
        flags = "mainSection",
        name = "CommonToolSettings",
      }
      local rollupPanel = rollup:getPanel()
      rollupPanel:beginHSizer{ flags = "expand" }
        rollupPanel:addHSpacer(6)
        rollupPanel:setBorder(1)

        rollupPanel:beginFlexGridSizer{ flags = "expand", proportion = 1, cols = 2, }
        rollupPanel:setFlexGridColumnExpandable(2)
          local app = nmx.Application.new()
          local roamingSettings = app:getRoamingUserSettings()
          local globalRoamingSettings = roamingSettings:getNodeFromPath("|GlobalSettings")

          rollupPanel:addStaticText{ text = "Manipulator size", }
          nmx.addAttributeWidget(rollupPanel, globalRoamingSettings:findAttribute("ManipSize"))

          rollupPanel:addStaticText{ text = "Manipulator handle size", }
          nmx.addAttributeWidget(rollupPanel, globalRoamingSettings:findAttribute("ManipHandleSize"))
        rollupPanel:endSizer()
      rollupPanel:endSizer()

      local page = getToolSettingsPage(newTool)
      if page then
        local pageRollup = container:addRollup{
          label = page.title,
          flags = "mainSection",
          name = string.format("%sSettings", newTool),
        }
        local pageRollupPanel = pageRollup:getPanel()
        pageRollupPanel:beginHSizer{ flags = "expand" }
          pageRollupPanel:addHSpacer(6)
          pageRollupPanel:setBorder(1)

          safefunc(page.create, pageRollupPanel, page)
          safefunc(page.update, pageRollupPanel, page)
        pageRollupPanel:endSizer()
      end

      toolSettings:resumeLayout()
      toolSettings:rebuild()

      currentTool = newTool
    end

    updateToolSettingsPanel()
  end
end

------------------------------------------------------------------------------------------------------------------------
-- adds the tool settings panel
------------------------------------------------------------------------------------------------------------------------
addToolSettingsPanel = function(panelBuilder)
  local panel = panelBuilder:addPanel{ name = "ToolSettings", caption = "Tool Settings" }
  rebuildToolSettingsPanel()
end

------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------
defaultToolSettingsPageCreateFunction = function(panel, page)
  assert(type(page.settingsNode) == "string")
  assert(type(page.attributes) == "table")

  panel:beginFlexGridSizer{ flags = "expand", proportion = 1, cols = 2, }
  panel:setFlexGridColumnExpandable(2)
    local application = nmx.Application.new()
    local roamingSettings = application:getRoamingUserSettings()
    local toolSettings = roamingSettings:getNodeFromPath(page.settingsNode)
    if toolSettings then
      local count = table.getn(page.attributes)
      for i = 1, count do
        local attributeInfo = page.attributes[i]

        panel:addStaticText{ text = attributeInfo.displayName, }

        local attribute = toolSettings:findAttribute(attributeInfo.name)
        nmx.addAttributeWidget(panel, attribute)
      end
    else
      local message = string.format(
        "Could not find settings node '%s' when creating tool settings page '%s'",
        page.settingsNode,
        page.title)
      app.warning(message)
    end
  panel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------
defaultToolSettingsPageUpdateFunction = function(panel, page)
  assert(type(page.settingsNode) == "string")
  assert(type(page.attributes) == "table")

  local app = nmx.Application.new()
  local roamingSettings = app:getRoamingUserSettings()
  local toolSettings = roamingSettings:getNodeFromPath(page.settingsNode)

  if toolSettings then
    local count = table.getn(page.attributes)
    for i = 1, count do
      local attributeInfo = page.attributes[i]

      local attribute = toolSettings:findAttribute(attributeInfo.name)
      nmx.updateAttributeWidget(panel, attribute)
    end
  else
    local message = string.format(
      "Could not find settings node '%s' when updating tool settings page '%s'",
      page.settingsNode,
      page.title)
    app.warning(message)
  end
end

if not mcn.inCommandLineMode() then
  ------------------------------------------------------------------------------------------------------------------------
  -- rebuild the panel if the selected tool changes
  ------------------------------------------------------------------------------------------------------------------------
  if __toolSettingsPanelToolChangedHandler then
    unregisterEventHandler("mcSelectionContextChanged", __toolSettingsPanelToolChangedHandler)
    unregisterEventHandler("mcViewportToolChanged", __toolSettingsPanelToolChangedHandler)
  end

  ------------------------------------------------------------------------------------------------------------------------
  __toolSettingsPanelToolChangedHandler = function()
    rebuildToolSettingsPanel()
  end

  ------------------------------------------------------------------------------------------------------------------------
  registerEventHandler("mcSelectionContextChanged", __toolSettingsPanelToolChangedHandler)
  registerEventHandler("mcViewportToolChanged", __toolSettingsPanelToolChangedHandler)

  ------------------------------------------------------------------------------------------------------------------------
  -- update the panel if the preferences change
  ------------------------------------------------------------------------------------------------------------------------
  if __toolSettingsPreferencesChangedHandler then
    unregisterEventHandler("mcPreferencesChanged", __toolSettingsPreferencesChangedHandler)
  end

  ------------------------------------------------------------------------------------------------------------------------
  __toolSettingsPreferencesChangedHandler = function()
    updateToolSettingsPanel()
  end

  ------------------------------------------------------------------------------------------------------------------------
  registerEventHandler("mcPreferencesChanged", __toolSettingsPanelToolChangedHandler)
end