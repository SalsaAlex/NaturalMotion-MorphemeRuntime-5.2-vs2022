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

  local items = { "None", }

  for _, t in ipairs(target.ls()) do
    table.insert(items, t:getName())
  end

  table.insert(items, "Manage...")

  comboBox:setItems(items)

  local activeRuntimeTarget = target.getActive()
  if activeRuntimeTarget then
    local activeTargetName = activeRuntimeTarget:getName()
    comboBox:setSelectedItem(activeTargetName)
  else
    comboBox:setSelectedItem("None")
  end
end

local runtime_target_selector_array = { }

addRuntimeTargetsSelector = function(mainFrame)

  mainFrame:setBorder(4)
  mainFrame:addStaticText{
    text = "Runtime Target:",
  }
  mainFrame:setBorder(3)

  local onComboChanged = function(self)
    local selection = self:getSelectedItem()

    if selection == "Manage..." then
      -- set back to the old selection before popping up the preferences so
      -- it doesn't display Manage... as the selected target
      local activeRuntimeTarget = target.getActive()
      if activeRuntimeTarget then
        local activeTargetName = activeRuntimeTarget:getName()
        self:setSelectedItem(activeTargetName)
      else
        self:setSelectedItem("None")
      end

      showPreferencesDialog("Runtime targets")
    elseif selection == "None" then
      target.setActive(nil)
    else
      local t = target.find(selection)
      target.setActive(t)
    end
  end

  local comboBox = mainFrame:addComboBox{
    name = "RuntimeTargetSelector",
    onChanged = onComboChanged,
    size = { width = 170 },
  }

  table.insert(runtime_target_selector_array, comboBox)

  repopulateComboBox(comboBox)
end

--
local onRuntimeTargetChange = function()
  for _, comboBox in ipairs(runtime_target_selector_array) do
    repopulateComboBox(comboBox)
  end
end

registerEventHandler("mcRuntimeTargetsAdded", onRuntimeTargetChange)
registerEventHandler("mcRuntimeTargetsRenamed", onRuntimeTargetChange)
registerEventHandler("mcRuntimeTargetsRemoved", onRuntimeTargetChange)
registerEventHandler("mcRuntimeTargetsReordered", onRuntimeTargetChange)

registerEventHandler("mcActiveRuntimeTargetChanged", onRuntimeTargetChange)