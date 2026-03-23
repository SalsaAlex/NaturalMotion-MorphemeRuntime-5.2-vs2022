------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

--[[

Function
========
This script creates a UI window that lists the inputs and ouputs of the selected node.
By double clicking you can select the corrsponding input or output

--]]

local inList = nil
local outList = nil
local currentSelectionLabel = nil
local parent = nil
local panel = nil

local refreshWindow = function()
  local windowExists = false

  -- check they exist before trying to clear them
  if inList and type(inList.clearRows) == "function" then
    inList:clearRows()
    windowExists = true
  end

  if outList and type(outList.clearRows) == "function" then
    outList:clearRows()
    windowExists = true
  end

  if currentSelectionLabel and type(currentSelectionLabel.setLabel) == "function" then
    currentSelectionLabel:setLabel(" ")
    windowExists = true
  end

  if not windowExists then
    return
  end

  inGoButton:enable(false)
  outGoButton:enable(false)

  local sel = ls("Selection")
  if table.getn(sel) == 1 then
    local obj = sel[1]
    local name
    parent, name = splitNodePath(obj)
    currentSelectionLabel:setLabel(name)

    local inCons = listConnections{ Object = obj, Upstream = true, Downstream = false }
    local outCons = listConnections{ Object = obj, Upstream = false, Downstream = true }
    for i, v in inCons do
      local _, name = splitNodePath(v)
      inList:addRow(name)
    end
    for i, v in outCons do
      local _, name = splitNodePath(v)
      outList:addRow(name)
    end
  elseif table.getn(sel) == 0 then
    parent = nil
    currentSelectionLabel:setLabel("Nothing Selected")
  else
    parent = nil
    currentSelectionLabel:setLabel("Multiple Objects Selected")
  end

  panel:rebuild()
end

------------------------------------------------------------------------------------------------------------------------
local onItemChosen = function(self, item)
  if parent == nil then
    select(self:getItemValue(item, 1))
  else
    select(parent .. "|"..self:getItemValue(item, 1))
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Make selection mutually exclusive between the two lists
------------------------------------------------------------------------------------------------------------------------
local isChangingSelection = false
local onInputSelected = function(self, item)
  if not isChangingSelection then
    isChangingSelection = true
    outList:clearSelection()
    outGoButton:enable(false)
    inGoButton:enable(true)
    isChangingSelection = false
  end
end

local onOutputSelected = function(self, item)
  if not isChangingSelection then
    isChangingSelection = true
    inList:clearSelection()
    inGoButton:enable(false)
    outGoButton:enable(true)
    isChangingSelection = false
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Connection Navigator Window
------------------------------------------------------------------------------------------------------------------------
addConnectionNavigatorWindow = function(layoutManager)
  panel = layoutManager:addPanel{ name = "ConnectionNav", caption = "Connection Navigator", flags = "expand", proportion = 1 }
  currentSelectionLabel = panel:addStaticText{ text = " ", font = "largebold" }
  panel:beginVSizer{ flags = "expand", proportion = 1 }
    panel:beginHSizer{ flags = "expand" }
      panel:addStaticText{ text = "Input Connections", flags = "expand", proportion = 1 }
      inGoButton = panel:addButton{ label = "Go", size = { width = 74 } }
    panel:endSizer()
    inList = panel:addListControl{ name = "inList", flags = "expand", proportion = 1 }
    panel:beginHSizer{ flags = "expand" }
      panel:addStaticText{ text = "Output Connections", flags = "expand", proportion = 1 }
      outGoButton = panel:addButton{ label = "Go", size = { width = 74 } }
    panel:endSizer()
    outList = panel:addListControl{ name = "outList", flags = "expand", proportion = 1 }
  panel:endSizer()

  inList:setOnItemActivated(onItemChosen)
  outList:setOnItemActivated(onItemChosen)

  inList:setOnSelectionChanged(onInputSelected)
  outList:setOnSelectionChanged(onOutputSelected)

  inGoButton:setOnClick(function() onItemChosen(inList, inList:getSelectedRow()) end)
  outGoButton:setOnClick(function() onItemChosen(outList, outList:getSelectedRow()) end)

  refreshWindow()
end

if not mcn.inCommandLineMode() then
  registerEventHandler("mcSelectionChange", refreshWindow)
end
