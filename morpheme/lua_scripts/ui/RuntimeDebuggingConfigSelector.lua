------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/PreferencesEditor/PreferencesEditor.lua"

if mcn.inCommandLineMode() then
  return
end

local repopulateComboBox = function(comboBox)
  assert(comboBox)

  local items = { "Off", }

  for _, t in ipairs(debugConfig.ls()) do
    table.insert(items, t:getName())
  end

  table.insert(items, "Configure...")

  comboBox:setItems(items)

  local activeDbgConfig = debugConfig.getActive()
  if activeDbgConfig then
    local activeDbgConfigName = activeDbgConfig:getName()
    comboBox:setSelectedItem(activeDbgConfigName)
  else
    comboBox:setSelectedItem("Off")
  end
end

local runtime_dbgconfig_selector_array = { }

addRuntimeDebuggingConfigSelector = function(mainFrame)

  mainFrame:setBorder(4)
  mainFrame:addStaticText{
    text = "Debugging:",
  }
  mainFrame:setBorder(3)

  local onComboChanged = function(self)
    local selection = self:getSelectedItem()

    if selection == "Configure..." then
      -- set back to the old selection before popping up the preferences so
      -- it doesn't display Configure... as the selected target
      local activeDbgConfig = debugConfig.getActive()
      if activeDbgConfig then
        local activeDbgConfigName = activeDbgConfig:getName()
        self:setSelectedItem(activeDbgConfigName)
      else
        self:setSelectedItem("Off")
      end

      showPreferencesDialog("Runtime debugging")
    elseif selection == "Off" then
      debugConfig.setActive(nil)
    else
      local t = debugConfig.find(selection)
      debugConfig.setActive(t)
    end
  end

  local comboBox = mainFrame:addComboBox{
    name = "RuntimeDebuggingConfigSelector",
    onChanged = onComboChanged,
    size = { width = 75 },
  }

  table.insert(runtime_dbgconfig_selector_array, comboBox)
  repopulateComboBox(comboBox)
end

--
local onRuntimeDebuggingConfigChange = function()
  for _, comboBox in ipairs(runtime_dbgconfig_selector_array) do
    repopulateComboBox(comboBox)
  end
end

registerEventHandler("mcRuntimeDebuggingConfigAdded", onRuntimeDebuggingConfigChange)
registerEventHandler("mcRuntimeDebuggingConfigRenamed", onRuntimeDebuggingConfigChange)
registerEventHandler("mcRuntimeDebuggingConfigRemoved", onRuntimeDebuggingConfigChange)

registerEventHandler("mcActiveRuntimeDebuggingConfigChanged", onRuntimeDebuggingConfigChange)

