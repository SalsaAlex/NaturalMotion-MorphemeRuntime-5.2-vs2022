------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------


------------------------------------------------------------------------------------------------------------------------
-- Recursively build the body group tree.  This uses splitFilePath and listChildren to get the short
-- name and hierarchy of all body groups in the system.
------------------------------------------------------------------------------------------------------------------------

local refreshBodyGroupTree = function()
  -- Initialise the list of targets
  local tree = ui.getWindow("BodyGroups|GroupTree")
  if tree ~= nil then
    local rootItem = tree:getRoot()
    rootItem:clearChildren()
    rootItem:setValue("hidden")

    local children = listChildren("BodyGroups")
    for i, v in ipairs(children) do
        local _, shortName = splitNodePath(v)
        local childItem = rootItem:addChild(shortName)
        childItem:setUserDataString(v)
    end

    tree:selectItem(rootItem)
  end
end

local closeAddBodyGroupWindow = function()
  local dlg = ui.getWindow("AddBodyGroupWindow")
  dlg:hide()
end

------------------------------------------------------------------------------------------------------------------------
-- Create a new body group as the child of the selected body group. Called when the user
-- clicks Ok in the add dialog
------------------------------------------------------------------------------------------------------------------------
local addBodyGroup = function()
  local tree = ui.getWindow("BodyGroups|GroupTree")
  local value = ui.getWindow("AddBodyGroupWindow|NameText"):getValue()
  create("BodyGroup", "BodyGroups", value)
  closeAddBodyGroupWindow()
  refreshBodyGroupTree()
end

------------------------------------------------------------------------------------------------------------------------
-- Handle Renaming a body group
------------------------------------------------------------------------------------------------------------------------
local onGroupTreeRename = function(treeControl, itemRenamed)
  local newValue = itemRenamed:getValue()
  local oldValue = itemRenamed:getUserDataString()
  local newPath = rename(oldValue, newValue)
  -- rename causes a refreshBodyGroupTree
  -- itemRenamed invalid after this point
end

------------------------------------------------------------------------------------------------------------------------
-- Shows a small dialog with a name field, OK and cancel buttons.
------------------------------------------------------------------------------------------------------------------------
local showAddBodyGroup = function()
 local dlg = ui.getWindow("AddBodyGroupWindow")
 if not dlg then
  dlg = ui.createModalDialog{ name = "AddBodyGroupWindow", caption = "Add Body Group", resize = false, size = { width = 200, height = 80 } }
  dlg:beginVSizer()
    dlg:beginHSizer{ flags = "expand" }
     dlg:addStaticText{ text = "Name" }
     dlg:addTextBox{ name = "NameText", flags = "expand" }
    dlg:endSizer()
    dlg:beginHSizer{ flags = "expand" }
      dlg:addStretchSpacer{ proportion = 1 }
      dlg:addButton{ label = "OK", onClick = addBodyGroup }
      dlg:addButton{ label = "Cancel", onClick = closeAddBodyGroupWindow }
    dlg:endSizer()
  dlg:endSizer()
 end
 ui.getWindow("AddBodyGroupWindow|NameText"):setValue("")
 dlg:show()
end

------------------------------------------------------------------------------------------------------------------------
-- Remove the selected body group.
------------------------------------------------------------------------------------------------------------------------
local removeBodyGroup = function()
  local tree = ui.getWindow("BodyGroups|GroupTree")
  local curItem = tree:getSelectedItem()
  if (curItem ~= nil) then
  
    -- Wrap the the asset manager and network scene in a change block so that synchronisation is delayed until the 
    -- body group has been removed from db1
    local assetManagerScene = nmx.Application.new():getSceneByName("AssetManager")
    
    local status, cbRef = assetManagerScene:beginChangeBlock(getCurrentFileAndLine())
    
    local bodyGroupName = curItem:getValue()

    delete(("BodyGroups|" .. bodyGroupName))
    refreshBodyGroupTree()
    
    assetManagerScene:endChangeBlock(cbRef, changeBlockInfo("Remove body group " .. bodyGroupName))
    
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Hide the main body groups editor
------------------------------------------------------------------------------------------------------------------------
local closeMainWindow = function()
  local dlg = ui.getWindow("BodyGroups")
  dlg:hide()
end

------------------------------------------------------------------------------------------------------------------------
-- Clear the attribute editor to stop a crash in the widget
------------------------------------------------------------------------------------------------------------------------
local clearBodyGroupsAttributeWidget = function()
  collectgarbage()
end

------------------------------------------------------------------------------------------------------------------------
-- The Main Body Groups Editor. A tree control with add and remove buttons
------------------------------------------------------------------------------------------------------------------------
showBodyGroups = function()

  local dlg = ui.getWindow("BodyGroups")
  if not dlg then
    dlg = ui.createModelessDialog{
        name = "BodyGroups",
        caption = "Body Groups",
        size = { width = 250, height = 300 },
        centre = true,
        resize = true
    }

    dlg:beginHSizer{ flags = "expand", proportion = 1 }

      dlg:beginVSizer{ flags = "expand", proportion = 1 }

      dlg:beginHSizer()
        dlg:addButton{ label = "Add", onClick = showAddBodyGroup }
        dlg:addButton{ label = "Remove", onClick = removeBodyGroup }
      dlg:endSizer()

      dlg:addTreeControl{
        name = "GroupTree",
        flags = "expand;rename;hideRoot",
        proportion = 1,
        onItemRenamed = onGroupTreeRename }
      dlg:endSizer()
    dlg:endSizer()

  end

  refreshBodyGroupTree()
  dlg:show()
end

------------------------------------------------------------------------------------------------------------------------
-- Register the appropriate events for the Body Groups Editor.
------------------------------------------------------------------------------------------------------------------------
registerEventHandler("mcFileCloseBegin", clearBodyGroupsAttributeWidget)

registerEventHandler("mcBodyGroupCreated", refreshBodyGroupTree)
registerEventHandler("mcBodyGroupDestroyed", refreshBodyGroupTree)
registerEventHandler("mcBodyGroupRenamed", refreshBodyGroupTree)

