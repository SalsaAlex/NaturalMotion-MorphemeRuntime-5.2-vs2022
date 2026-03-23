------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

local filterStrings = { }
local filterControl;
local navigator;
local currentFilterColumn = 1

local kColumns =
{
  { name = "Type", isVisible = false, minWidth = 30, width = 80, filter = true },
  { name = "Manifest", isVisible = false, minWidth = 30, width = 80, filter = false },
}

------------------------------------------------------------------------------------------------------------------------
local doText = function(column, nodeName, nodeType, nodePath)
  if column == 1 then
    return nodeName
  elseif column == 2 then
    return nodeType
  elseif column == 3 then
    return getType(nodePath)
  elseif column == 4 and getType(nodePath) == "AnimWithEvents" then
    local animationTake = getAttribute(nodePath, "AnimationTake", getSelectedAssetManagerAnimSet())
    local path, file = splitFilePath(animationTake.filename)
    return file
  end
  return ""
end

------------------------------------------------------------------------------------------------------------------------
local imageCache = nil
local doIcon = function(nodeName, nodeType, nodePath)
  if imageCache == nil then
    imageCache = {
      ["BlendTree"]             = app.loadImage("BlendTreeIcon16x16.png"),
      ["PhysicsBlendTree"]      = app.loadImage("PhysicsBlendTreeIcon16x16.png"),
      ["StateMachine"]          = app.loadImage("StateMachineIcon16x16.png"),
      ["PhysicsStateMachine"]   = app.loadImage("PhysicsStateMachineIcon16x16.png"),
      ["BehaviourStateMachine"] = app.loadImage("BehaviourStateMachineIcon16x16.png"),
      ["BlendNode"]             = app.loadImage("BlendTreeNodeIcon16x16.png"),
      ["Transition"]            = app.loadImage("TransitionIcon16x16.png"),
      ["Condition"]             = app.loadImage("ConditionIcon16x16.png"),
    }
  end
  return imageCache[nodeType]
end

------------------------------------------------------------------------------------------------------------------------
local doFilter = function(nodeName, nodeType, nodePath)

  -- If there are no filterstrings then we just don't filter
  if table.getn(filterStrings) == 0 then
    return true
  end

  -- Otherwise look for a match in the filter strings
  nodeName = string.lower(doText(currentFilterColumn, nodeName, nodeType, nodePath))
  for _, v in ipairs(filterStrings) do
    if not string.find(nodeName, v) then
      return false
    end
  end

  return true
end

------------------------------------------------------------------------------------------------------------------------
local onFilterChanged = function()

  -- When the filter changes we get a new set of lowercase strings to match against
  local value = filterControl:getValue()
  filterStrings = splitString(string.lower(value), " ")

  -- And mark the filter as changed so everything will be refiltered
  navigator:filterChanged()
end

------------------------------------------------------------------------------------------------------------------------
local doContextualMenu = function(menu, nodeName, nodeType, nodePath)
  menu:addItem{ label = "Edit name...", onClick = function() navigator:editItem(nodePath) end }
  menu:addItem{ label = "Expand All", onClick = function() navigator:expandAll(nodePath) end }
  menu:addItem{ label = "Collapse All", onClick = function() navigator:collapseAll(nodePath) end }
end

------------------------------------------------------------------------------------------------------------------------
local findColumn = function(name)
  local numColumns = navigator:getNumColumns()
  for i = 1, numColumns do
    if navigator:getColumnHeaderText(i) == name then
      return i
    end
  end

  return 0
end

------------------------------------------------------------------------------------------------------------------------
local toggleVisiblity = function(name)
  return function()
    local columnIndex = findColumn(name)
    local isVisible = navigator:isColumnShown(columnIndex)
    navigator:showColumn(columnIndex, not isVisible)
    navigator:showColumnHeader(navigator:getNumColumnsShown() > 1)
  end
end

------------------------------------------------------------------------------------------------------------------------
local setFilterColumn = function(columnIndex)
  return function()
    currentFilterColumn = columnIndex
    local label = string.format("Search (%s)", navigator:getColumnHeaderText(columnIndex))
    filterControl:setLabel(label)
    -- And mark the filter as changed so everything will be refiltered
    navigator:filterChanged()
  end
end

------------------------------------------------------------------------------------------------------------------------
local doOnOptionsMenu = function(menu, nodeName, nodeType, nodePath)
  menu:clear()
  menu:addItem{ label = "Columns", enable = false }
  for _, column in ipairs(kColumns) do
    local ischecked = navigator:isColumnShown(findColumn(column.name))
    menu:addCheckedItem{ label = "  " .. column.name, checked = ischecked, onClick = toggleVisiblity(column.name) }
  end
end

------------------------------------------------------------------------------------------------------------------------
local doOnFilterMenu = function(menu, nodeName, nodeType, nodePath)
  menu:clear()
  menu:addItem{ label = "Filter by", enable = false }
  local columnNumber = 1
  local ischecked = columnNumber == currentFilterColumn
  menu:addCheckedItem{ label = "  Name", checked = ischecked, onClick = setFilterColumn(columnNumber) }

  for _, column in ipairs(kColumns) do
    if column.filter then
      columnNumber = columnNumber + 1
      ischecked = columnNumber == currentFilterColumn
      menu:addCheckedItem{ label = "  " .. column.name, checked = ischecked, onClick = setFilterColumn(columnNumber) }
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Object creation functions.
------------------------------------------------------------------------------------------------------------------------
addNetworkNavigator = function(contextualPanel, forContext)
  filterStrings = { } -- clear the filter strings otherwise if you open an old doc you will get the last filter used
  currentFilterColumn = 1

  local panel = contextualPanel:addPanel{ name = "NetworkNavigator", caption = "Navigator", forContext = forContext }
  panel:beginVSizer{ flags = "expand", proportion = 1}

    local optionsWidget = app.loadImage("OptionsWidget.png");

    -- Add a filter widget
    panel:beginHSizer{ flags = "expand", proportion = 0}
      filterControl = panel:addFilterControl{
        name = "Filter",
        flags = "expand",
        proportion = 1,
        cancelImage = app.loadImage("CancelWidget.png"),
        searchImage = app.loadImage("SearchWidgetWithPopup.png"),
        onChanged = onFilterChanged,
        onMenu = doOnFilterMenu,
      }
      panel:addHSpacer(2)
      panel:addButton{
        name = "OptionsButton",
        label = "",
        image = optionsWidget,
        flags = "expand;alpha",
        onMenu = doOnOptionsMenu,
        proportion = 0,
      }
    panel:endSizer()

    -- Add a navigator list
    panel:setBorder(0)
    navigator = panel:addStockWindow{
      type = "Navigator",
      name = "Navigator",
      flags = "expand",
      proportion = 1,
      filter = doFilter,
      text = doText,
      image = doIcon,
      contextualMenu = doContextualMenu,
    }

    navigator:setColumnHeaderText(1, "Name")

    -- Add all the columns in kColumns
    local columnNumber = 1
    navigator:setColumnAutosize(columnNumber, false)
    navigator:setColumnAllowResize(columnNumber, true)
    navigator:setColumnWidth(columnNumber, 100)
    navigator:setColumnMinWidth(columnNumber, 30)

    for _, column in ipairs(kColumns) do
      columnNumber = columnNumber + 1
      navigator:insertColumn(columnNumber, column.name)
      navigator:showColumn(columnNumber, column.isVisible)
      navigator:setColumnAutosize(columnNumber, false)
      navigator:setColumnAllowResize(columnNumber, true)
      navigator:setColumnMinWidth(columnNumber, column.minWidth)
      navigator:setColumnWidth(columnNumber, column.width)
    end
    navigator:showColumnHeader(navigator:getNumColumnsShown() > 1)
    setFilterColumn(1);

  panel:endSizer()
  return panel
end
