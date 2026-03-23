-- Copyright (c) 2011 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------
require "ui/tools/ToolSettingsPages.lua"
require "ui/tools/ToolSettingsPanel.lua"
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
local rayCastTypes = { }

------------------------------------------------------------------------------------------------------------------------
-- helper function for getting the ray cast tool settings node from roaming settings.
------------------------------------------------------------------------------------------------------------------------
local getRayCastToolSettingsNode = function()
  local app = nmx.Application:new()
  local roamingSettings = app:getRoamingUserSettings()

  return roamingSettings:getNodeFromPath("|RayCastToolSettingsNode")
end

------------------------------------------------------------------------------------------------------------------------
-- 
------------------------------------------------------------------------------------------------------------------------
local ensureValidRayCastTypeSelected = function()
  local selectedRayCastType = rayCastTool.getSelectedRayCastType()

  if type(rayCastTypes[selectedRayCastType]) ~= "table" then
    -- the stored selected type does not match any currently registered type so just select
    -- the first ray cast type in the rayCastTypes list
    --
    local name, firstRayCastType = next(rayCastTypes, nil)
    if type(firstRayCastType) == "nil" then
      rayCastTool.setSelectedRayCastType("")
    else
      rayCastTool.setSelectedRayCastType(name)
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: NAMESPACE
--| name: rayCastTool
--| title: Ray cast tool functions
--| brief: Morpheme:Connect ray cast tool functions are used to interact with the ray cast tool.
------------------------------------------------------------------------------------------------------------------------
rayCastTool = {
  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: boolean rayCastTool.addRayCastType(string rayCastTypeName, table rayCastTypeInfo)
  --| brief:
  --|   Add a new ray cast type used to interact with the character when previewing. rayCastTypeInfo must be of the form
  --|   { previewScriptFile = "file.lua", buildPreviewScript = function() end, createSettingsPanel = function() end, updateSettingsPanel = function() end, }.
  --|   previewScriptFile must point to a lua script that will be executed every time a ray cast of the new type is triggered,
  --|   the ray cast info will be accessible inside this script through a global table _RAYCASTEVENT.
  --|   buildPreviewScript is used to add any additional data to _RAYCASTEVENT before it is sent to the preview script state.
  --|   The other two functions are for building ui that is displayed in the tool settings panel.
  --|
  --| environments: GlobalEnv
  ----------------------------------------------------------------------------------------------------------------------
  addRayCastType = function(rayCastTypeName, rayCastTypeInfo)
    -- check the name is valid
    --
    assert(type(rayCastTypeName) == "string")
    assert(string.len(rayCastTypeName) > 0)

    -- check the info table is valid
    --
    assert(type(rayCastTypeInfo) == "table")
    
    assert(type(rayCastTypeInfo.previewScriptFile) == "string")
    assert(string.len(rayCastTypeInfo.previewScriptFile) > 0)
    
    assert(type(rayCastTypeInfo.buildPreviewScriptRayCast) == "function")
    
    assert(type(rayCastTypeInfo.createSettingsPanel) == "function")
    assert(type(rayCastTypeInfo.updateSettingsPanel) == "function")

    -- check there is no type already registered with this name
    --
    assert(rayCastTypes[rayCastTypeName] == nil)

    -- register the type
    --
    rayCastTypes[rayCastTypeName] = rayCastTypeInfo
  end,

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: nil rayCastTool.removeRayCastType(string rayCastTypeName)
  --| brief:
  --|   Remove a ray cast type previously added via rayCastTool.addRayCastType.
  --|
  --| environments: GlobalEnv
  ----------------------------------------------------------------------------------------------------------------------
  removeRayCastType = function(rayCastTypeName)
    -- check the name is valid
    --
    assert(type(rayCastTypeName) == "string")
    assert(string.len(rayCastTypeName) > 0)

    -- unregister the type
    --
    rayCastTypes[rayCastTypeName] = nil
  end,

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table rayCastTool.getRayCastTypes()
  --| brief:
  --|   Gets an array of available ray cast types.
  --|
  --| environments: GlobalEnv
  ----------------------------------------------------------------------------------------------------------------------
  getRayCastTypes = function()
    local types = { }
    for name, type in pairs(rayCastTypes) do
      table.insert(types, name)
    end
    return types
  end,

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: string rayCastTool.getSelectedRayCastType()
  --| brief:
  --|   Gets the name of the ray cast tools currently selected ray cast type.
  --|
  --| environments: GlobalEnv
  ----------------------------------------------------------------------------------------------------------------------
  getSelectedRayCastType = function()
    local toolSettings = getRayCastToolSettingsNode()
    local attribute = toolSettings:findAttribute("SelectedRayCastType")
    local selectedRayCastType = attribute:asString()
    if type(rayCastTypes[selectedRayCastType]) == "table" then
      return selectedRayCastType
    else
      return ""
    end
  end,

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: nil rayCastTool.setSelectedRayCastType(string rayCastType)
  --| brief:
  --|   Sets the the ray cast tools currently selected ray cast type.
  --|
  --| environments: GlobalEnv
  ----------------------------------------------------------------------------------------------------------------------
  setSelectedRayCastType = function(rayCastType)
    local toolSettings = getRayCastToolSettingsNode()
    local attribute = toolSettings:findAttribute("SelectedRayCastType")
    local database = attribute:getDatabase()

    if type(rayCastTypes[rayCastType]) == "table" then
      local status, cbRef = database:beginChangeBlock(getCurrentFileAndLine())
      attribute:setString(rayCastType)
      database:endChangeBlock(cbRef, changeBlockInfo("setting ray cast tool selected ray cast type"))
      
      -- make sure the ui is updated if it is the currently selected tool
      rebuildToolSettingsPanel()
    end
  end,
}

------------------------------------------------------------------------------------------------------------------------
-- register the ui for the RayCastTool and all its ray cast types
------------------------------------------------------------------------------------------------------------------------
removeToolSettingsPage("RayCastTool")
addToolSettingsPage(
  "RayCastTool",
  {
    title = "Ray Cast Tool Settings",

    --------------------------------------------------------------------------------------------------------------------
    -- create the tool settings panel for the ray cast tool
    --------------------------------------------------------------------------------------------------------------------
    create = function(panel, page)
      -- make sure there is a valid selection
      -- ensureValidRayCastTypeSelected()

      panel:beginVSizer{ flags = "expand", proportion = 1, }

        -- add the ray cast tool controls common to all ray cast types
        --
        panel:beginFlexGridSizer{ flags = "expand", cols = 2, }
        panel:setFlexGridColumnExpandable(2)

          local items = { }
          for name, type in pairs(rayCastTypes) do
            table.insert(items, name)
          end

          panel:addStaticText{ name = "RayCastTypeText", text = "Ray cast type", }
          local rayCastCombo = panel:addComboBox{
            name = "RayCastTypeComboBox",
            items = items,
            flags = "expand",
            proportion = 1,
            onChanged = function(self)
              rayCastTool.setSelectedRayCastType(self:getValue())
            end,
          }
        panel:endSizer()

        if table.getn(items) > 0 then
          local selectedRayCastTypeName = rayCastTool.getSelectedRayCastType()      
          if selectedRayCastTypeName == "" then
            selectedRayCastTypeName = items[1]
          else
            rayCastCombo:setValue(selectedRayCastTypeName)
          end

          if selectedRayCastTypeName ~= "" then
            local selectedRayCastTypeInfo = rayCastTypes[selectedRayCastTypeName]

            if type(selectedRayCastTypeInfo) == "table" then        
              local rollup = panel:addRollup{
                label = string.format("%s Type Settings", selectedRayCastTypeName),
                name = "RayCastTypeSettings",
                flags = "expand",
                proportion = 1,
              }
              local rollupPanel = rollup:getPanel()
              rollupPanel:beginHSizer{ flags = "expand", proportion = 1, }
                rollupPanel:addHSpacer(6)
                rollupPanel:setBorder(1)

                selectedRayCastTypeInfo.createSettingsPanel(rollupPanel, selectedRayCastTypeInfo)

                rollupPanel:endSizer()
              rollupPanel:endSizer()
            end
          end
        else
          rayCastCombo:enable(false)
        end

      panel:endSizer()
    end,

    --------------------------------------------------------------------------------------------------------------------
    -- update the tool settings panel for the ray cast tool
    --------------------------------------------------------------------------------------------------------------------
    update = function(panel, page)
      -- update the ray cast tool controls common to all ray cast types
      --
      local staticText = panel:getChild("RayCastTypeText")
      local comboBox = panel:getChild("RayCastTypeComboBox")
      local selectedRayCastTypeName = rayCastTool.getSelectedRayCastType()
      if selectedRayCastTypeName == "" then
        selectedRayCastTypeName = next(rayCastTypes, nil) or ""
      end

      if selectedRayCastTypeName ~= "" then
        staticText:enable(true)
        comboBox:enable(true)

        comboBox:setValue(selectedRayCastTypeName)

        local selectedRayCastTypeInfo = rayCastTypes[selectedRayCastTypeName]

        if type(selectedRayCastTypeInfo) == "table" then 
          local rollup = panel:getChild("RayCastTypeSettings")
          local rollupPanel = rollup:getPanel()

          local expectedRollupLabel = string.format("%s Type Settings", selectedRayCastTypeName)
          local actualRollupLabel = rollup:getLabel()

          if expectedRollupLabel ~= actualRollupLabel then
            -- the ray cast type selection has changed so we need to rebuild the panel
            --
            rollup:setLabel(string.format("%s Type Settings", selectedRayCastTypeName))

            rollupPanel:suspendLayout()
            rollupPanel:freeze()
            rollupPanel:clear()
      
            rollupPanel:beginHSizer{ flags = "expand", proportion = 1, }
              rollupPanel:addHSpacer(6)
              rollupPanel:setBorder(1)

              selectedRayCastTypeInfo.createSettingsPanel(rollupPanel, selectedRayCastTypeInfo)

              rollupPanel:endSizer()
            rollupPanel:endSizer()

            rollupPanel:resumeLayout()
            rollupPanel:rebuild()
          else
            -- just some settings changes so update the ui
            --
            selectedRayCastTypeInfo.updateSettingsPanel(rollupPanel, selectedRayCastTypeInfo)
          end
        end
      else
        staticText:enable(false)
        comboBox:enable(false)
      end
    end,
  }
)

------------------------------------------------------------------------------------------------------------------------
-- This is called when the ray cast tool is invoked and builds a table of information that is
-- passed to any preview script ray cast tool handler functions that are registered. The
-- table is serialized as a string as it internally has to pass between different lua states.
------------------------------------------------------------------------------------------------------------------------
local buildPreviewScriptRayCast = function(appScriptRayCast)
  local previewScriptRayCast = nil

  -- make sure the tool has a valid type selection
  --
  ensureValidRayCastTypeSelected()

  -- check if there is a valid ray cast type selected
  --
  local selectedRayCastTypeName = rayCastTool.getSelectedRayCastType()
  local selectedRayCastType = rayCastTypes[selectedRayCastTypeName]

  if type(selectedRayCastType) == "table" then
    -- build the type's ray cast
    --
    previewScriptRayCast = selectedRayCastType.buildPreviewScriptRayCast(appScriptRayCast, selectedRayCastType)
    assert(type(previewScriptRayCast) == "table")

    -- add some other members required by the boilerplate code included in the preview script
    --
    previewScriptRayCast.rayCastType = selectedRayCastTypeName
    previewScriptRayCast.previewScriptFile = selectedRayCastType.previewScriptFile
  else
    -- no ray cast type selected so just send the default data
    --
    previewScriptRayCast = table.clone(appScriptRayCast)
    previewScriptRayCast.rayCastType = ""
    previewScriptRayCast.previewScriptFile = ""
  end

  -- serialize the data ready for the other script state to handle
  --
  local serializedPreviewScriptRayCast = string.format("return %s", table.serialize(previewScriptRayCast))
  return serializedPreviewScriptRayCast
end

mcn.setBuildPreviewScriptRayCastCallback(buildPreviewScriptRayCast)