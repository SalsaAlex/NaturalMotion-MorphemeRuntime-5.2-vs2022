------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

local g_chosenInstances = nil

local refreshInstanceList = function(instances)
  -- Initialise the list of instances
  local list = ui.getWindow("InstanceChooser|InstanceList")
  list:clearRows()
  if instances then
    local i, v
    for i, v in ipairs(instances) do
      list:addRow(v.id .. " - " .. v.name)
    end
  end
end

local doCreateInstance = function()
  -- Return only one instance with an invalid ID
  g_chosenInstances = { -1 }
  ui.getWindow("InstanceChooser"):hide()
end

local doOK = function()
  local list = ui.getWindow("InstanceChooser|InstanceList")
  g_chosenInstances = list:getSelectedRows()
  ui.getWindow("InstanceChooser"):hide()
end

local doCancel = function()
  g_chosenInstances = nil
  ui.getWindow("InstanceChooser"):hide()
end

showInstanceChooser = function(instances, showCreateNewButton, multiSelect)
  if showCreateNewButton == nil then
    showCreateNewButton = true
  end
  
  if multiSelect == nil then
    multiSelect = true
  end
  
  local dlg = ui.getWindow("InstanceChooser")
  if not dlg then    
    dlg = ui.createModalDialog{ name = "InstanceChooser", resize = true, size = { width = 300 } }
  end
  
  dlg:clear()

  -- Ensure that the dlg was created correctly.
  if dlg:getName() ~= "InstanceChooser" then
    dlg = ui.getWindow("InstanceChooser")
    collectgarbage()
    return
  end

  listControlFlags = "expand"
  if multiSelect == true then
    listControlFlags = "expand;multiSelect"
  end
  
  local cap = "Choose Runtime Instance to connect to"
  if multiSelect then
    cap = "Choose Runtime Instance(s) to connect to"
  end

  dlg:beginVSizer()
    dlg:addStaticText{ text = cap }
    dlg:addListControl{ name = "InstanceList", flags = listControlFlags, proportion = 1 }
    dlg:beginHSizer{ flags = "expand" }
      if showCreateNewButton then
        dlg:addButton{ label = "Create Instance", onClick = doCreateInstance }
      end
      dlg:addPanel{ proportion = 1 }
      dlg:addButton{ label = "OK", onClick = doOK }
      dlg:addButton{ label = "Cancel", onClick = doCancel }
    dlg:endSizer()
  dlg:endSizer()

  local list = ui.getWindow("InstanceChooser|InstanceList")
  list:setOnItemActivated(doOK)

  refreshInstanceList(instances)

  dlg:show()
end

chooseRuntimeInstance = function(instances, showCreateNewButton, multiSelect)
  showInstanceChooser(instances, showCreateNewButton, multiSelect)
  return g_chosenInstances
end

if not mcn.inCommandLineMode() then
  -- This is useful debugging code that ensures that the dialogs get created each time this script is executed
  -- Useful when testing changes
  local dlg
  dlg = ui.getWindow("InstanceChooser")

  collectgarbage()
end