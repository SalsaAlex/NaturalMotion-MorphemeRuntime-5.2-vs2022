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
This script creates a UI window that has a searchable tree respresentation of the network

TODO LIST:
Change the on deleted callback
  Use the trunk implementation that uses change sets
  Iterate through the removed nodes in the change set building up a set of hierarchies of removed nodes
  The top of each of these hierarchies will be in the DB
  Construct a node path for the removed node based on the node in the DB and the stack of removed objects
  Call the lua callback with the removed object's path.

Store the expanded / collapsed state when adding/removing/renaming nodes.
  Will require extra (collapsedState) data to be stored as (at least for deleting objects) we need to rebuild the entire tree.
  Likely to need a onCollapse/Expand tree item callback.
  NOTE: If we remove the need to rebuild the tree when deleting nodes then collapse state is easier to manage, as you can modify sections of the tree directly rather than a full rebuild.

Improve the Type filter:
  Group the entries by manifestGroup - requires Lua API for this info
  Use the manifest display name - requires Lua API for this info
  Correctly implement toggling of 'parent types' eg show or hide all IK nodes.
  Add 'All Transitions', 'All Transit from transitions', Physics Compositors, and other hard coded types

State Machines:
  sort SM entries by type (transits, transits from transits, states) Have them as seperate tree sub items based on types.
  Transition groups? Add as sub trees?

--]]

local mainPanel = nil
local navigatorTree = nil
local searchTextBox = nil
local typeFilterTree = nil
local caseSpecific = false
local lastBoldItem = nil
local needsRebuild = false
local hiddenTypes = { }

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
local buildAncestry = function(nodePath)
  -- Build a table of ancestors
  local ancestors = { }
  local pos = 0
  local lastPos = 0
  while true do
    pos = string.find(nodePath, "|", lastPos + 1)
    if pos == nil then
      table.insert(ancestors, string.sub(nodePath, lastPos + 1, string.len(nodePath)))
      break
    end
    table.insert(ancestors, string.sub(nodePath, lastPos + 1, pos - 1))
    lastPos = pos
  end
  return ancestors
end

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
local shouldDisplayNode = function(nodePath, searchString)
  if nodePath == nil then return false end

  -- Ignore control parameters
  local parent, node = splitNodePath(nodePath)
  if parent == "ControlParameters" then return false end

  -- Ignore Requests
  local parent, node = splitNodePath(nodePath)
  if parent == "Requests" then return false end

  -- Ignore Layers
  local parent, node = splitNodePath(nodePath)
  if parent == "Layers" then return false end

  -- don't need to worry if it's a condition on a transition.
  local type, manifestType = getType(nodePath)
  if (manifestType == "Condition") then return false end

  -- See if this type has been filtered out
  for _, v in hiddenTypes do
    if (type == v) then return false end
  end

  if (searchString ~= "") then
    local _, compareString = splitNodePath(nodePath)
    if not caseSpecific then
      compareString = string.upper(compareString)
      searchString = string.upper(searchString)
    end
    if string.find(compareString, searchString) == nil then
      return false
    end
  end

  return true
end

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
local addItemToTree = function(nodePath, tree)
  -- Build a table of ancestors
  local ancestors = buildAncestry(nodePath)

  -- Loop through all the components of the path, adding them if they don't exist
  local currentItem = tree:getRoot()
  local currentPath = ""
  for i, currentName in ipairs(ancestors) do
    if currentPath == "" then
      currentPath = currentName
    else
      currentPath = currentPath .. "|" .. currentName
    end
    nextItem = currentItem:findChildByValue(currentName)
    if (nextItem == nil) then
      nextItem = currentItem:addChild(currentName)
      nextItem:setColumnValue(1, currentName)
      local type, manifestType = getType(currentPath)
      nextItem:setColumnValue(2, type)
      nextItem:setUserDataString(currentPath)
      if (getCurrentGraph() == currentPath) then
        nextItem:setBold(true)
        lastBoldItem = nextItem
      end
      -- TODO: Default to collapsed items?
      --currentItem:collapse()
    end

    -- move on to next item
    currentItem = nextItem
  end
end

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
local rebuildList = function()
  assert(needsRebuild)

  navigatorTree:getRoot():clearChildren()
  lastBoldItem = nil

  -- populate the tree view with the full contents
  local currentItem = navigatorTree:getRoot()
  currentItem:setUserDataString("")

  -- Iterate through all the nodes adding them to the list
  local nodes = ls()
  table.sort(nodes)
  for i, nodePath in ipairs(nodes) do
    if shouldDisplayNode(nodePath, searchTextBox:getValue()) then
      addItemToTree(nodePath, navigatorTree)
    end
  end

  if (getCurrentGraph() == "") then
    navigatorTree:getRoot():setBold(true)
  else
    navigatorTree:getRoot():setBold(false)
  end

  needsRebuild = false
end

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
local requestRebuild = function()
  if not needsRebuild then
    needsRebuild = true
    mainPanel:addIdleCallback(rebuildList)
  end
end

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
local addNewNode = function(nodePath)
  if searchTextBox and shouldDisplayNode(nodePath, searchTextBox:getValue()) then
    addItemToTree(nodePath, navigatorTree)
  end
end

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
local renameNode = function(oldPath, newPath)
  requestRebuild()
  --[[
  -- TODO: Would be better to work on a single item, but need to make sure selection and boldness is correct
  if shouldDisplayNode(oldPath, searchTextBox:getValue()) then
    removeItemFromTree(oldPath, navigatorTree)
  end

  if shouldDisplayNode(oldPath, searchTextBox:getValue()) then
    addItemToTree(newPath, navigatorTree)
  end
  --]]

end

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
local onResultActivated = function(self, item)
  local nodePath = item:getUserDataString()
  local parent, node = splitNodePath(nodePath)
  select(nodePath)
  setCurrentGraph(parent)

  -- TODO: make sure then nodepath is visible, could be a util function
  --local x, y = getNodePosition(nodePath)
  --setGraphTransform(parent, x, y, 1)
  -- check script for network view. Ensure visible (or zoom to sleection) may be in there.

end

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
local isChangingSelection = false

local onResultSelectionChanged = function(self)
  if not isChangingSelection then
    isChangingSelection = true
    local selection = self:getSelectedItems()
    local s = { }
    for i, v in ipairs(selection) do
      table.insert(s, v:getUserDataString())
    end
    select(s)
    isChangingSelection = false
  end
end

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
local onNetworkPanelSelectionChange = function()
  if navigatorTree and (not isChangingSelection) then
    isChangingSelection = true
    foundAnItem = false
    navigatorTree:clearSelection()

    -- Loop over all objects in the selection
    local selection = ls("Selection")
    table.sort(selection)
    for _, v in ipairs(selection) do
      local ancestors = buildAncestry(v)
      local currentItem = navigatorTree:getRoot()
      for _, a in ipairs(ancestors) do
        currentItem = currentItem:findChildByValue(a)
      end

      if currentItem ~= nil and currentItem:getUserDataString() == v then
        navigatorTree:selectItem(currentItem)
        if (foundAnItem == false) then
          navigatorTree:ensureItemVisible(currentItem)
          foundAnItem =true
        end
      else
        app.warning("Couldn't find item: "..v)
        -- this is because the selection has changed but the tree has not been updated yet.
      end
    end
    isChangingSelection = false
  end
end

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
local onNetworkPanelGraphChange = function()
  if navigatorTree then
    -- Clear the bold on the previous item (if any)
    if (lastBoldItem ~= nil) then
      lastBoldItem:setBold(false)
    end

    -- Set the bold on the new graph
    local graph = getCurrentGraph()
    local ancestors = buildAncestry(graph)
    local currentItem = navigatorTree:getRoot()
    for _, v in ipairs(ancestors) do
      currentItem = currentItem:findChildByValue(v)
    end

    -- Possible we don't find the item if the filter has filtered it out, or it is the root.
    if (currentItem ~= nil) then
      currentItem:setBold(true)
      navigatorTree:ensureItemVisible(currentItem)
    end

    -- Store the item for next time
    lastBoldItem = currentItem

    -- TODO, for some reason the selection isn't updated here.
    onNetworkPanelSelectionChange()
  end
end

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
local populateTypeFilterTree = function(tree)
  local child
  local manifestItem = tree:getRoot():addChild("Manifest Entries")
  manifestItem:setColumnValue(2, "Manifest Entries")
  manifestItem:setColumnCheckbox(1, 4) -- check

  -- TODO: would be _much_ better to be able to get the 'group' of the manifest entry and use that to split the blend nodes into categories.
  local blendNodeItem = manifestItem:addChild("Blend Nodes")
  blendNodeItem:setColumnValue(2, "Blend Nodes")
  blendNodeItem:setColumnCheckbox(1, 4) -- check

  local types = listTypes("BlendNode")
  table.sort(types)
  for _, v in ipairs(types) do
    child = blendNodeItem:addChild(v)
    child:setColumnValue(2, v)
    child:setColumnCheckbox(1, 4) -- check
  end

  local transitionItem = manifestItem:addChild("Transitions")
  transitionItem:setColumnValue(2, "Transitions")
  transitionItem:setColumnCheckbox(1, 4) -- check

  local types = { }
  for _, v in pairs(listTypes("Transition")) do table.insert(types, v) end
  table.sort(types)
  for _, v in ipairs(listTypes("Transition")) do
    child = transitionItem:addChild(v)
    child:setColumnValue(2, v)
    child:setColumnCheckbox(1, 4) -- check
  end
end

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
local onTypeFilterChanged = function(tree, itemActivated)
  if (itemActivated:getColumnCheckbox(1) == 3) then
    for i, v in ipairs(hiddenTypes) do
      -- already exists in list. Should not happen, but just return
      if (v == itemActivated:getColumnValue(2)) then
        app.warning(itemActivated:getColumnValue(2) .. " already exists in the hidden types")
        return
      end
    end
    table.insert(hiddenTypes, itemActivated:getColumnValue(2))
    requestRebuild()
  else
    for i, v in ipairs(hiddenTypes) do
      if (v == itemActivated:getColumnValue(2)) then
        table.remove(hiddenTypes, i)
        requestRebuild()
        return
      end
    end
  end

end

------------------------------------------------------------------------------------------------------------------------
-- Find Window
------------------------------------------------------------------------------------------------------------------------
addNavigatorWindow = function(layoutManager)
  mainPanel = layoutManager:addPanel{ name = "Navigator", caption = "NavigatorPrototype", flags = "expand", proportion = 1 }
  mainPanel:freeze()
  mainPanel:beginVSizer{ flags = "expand", proportion = 1 }
    mainPanel:beginHSizer{ flags = "expand" }
      mainPanel:addStaticText{ text = "Find:" }
      searchTextBox = mainPanel:addTextBox{ name = "searchString", flags = "expand", proportion = 1 }
    mainPanel:endSizer()

    local rollup = mainPanel:addRollup{ label = "Filters", flags = "mainSection;expand", name = "filterRollup" }
    local rollPanel = rollup:getPanel()
    rollPanel:beginVSizer{ flags = "expand", proportion = 1 }
      typeFilterTree = rollPanel:addTreeControl{
        name = "typeFilterTree",
        flags = "expand;hideRoot;multiSelect",
        numColumns = 2,
        treeControlColumn = 2,
        size = { height = 150 }
      }
      typeFilterTree:setColumnWidth(1, 20)
    rollPanel:endSizer()

    navigatorTree = mainPanel:addTreeControl{
      name = "resultList",
      flags = "expand;multiSelect",
      proportion = 1,
      numColumns = 2,
      columnNames = { "Name", "Type" },
      treeControlColumn = 1
      }

  mainPanel:endSizer()
  mainPanel:rebuild()

  -- set up search
  searchTextBox:setOnChanged(requestRebuild)
  searchTextBox:setOnEnter(requestRebuild)
  navigatorTree:setOnItemActivated(onResultActivated)
  navigatorTree:setOnSelectionChanged(onResultSelectionChanged)

  -- set up the type filter
  typeFilterTree:setOnCheckboxChanged(onTypeFilterChanged)
  populateTypeFilterTree(typeFilterTree)

  -- Set up the string for the root.
  navigatorTree:getRoot():setUserDataString("")
  requestRebuild()
end

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
--[[
-- Disable all functinality for now
if not mcn.inCommandLineMode() then
  registerEventHandler("mcSelectionChange", onNetworkPanelSelectionChange)
  registerEventHandler("mcCurrentGraphChange", onNetworkPanelGraphChange)
  registerEventHandler("mcNodeCreated", addNewNode)
  registerEventHandler("mcEdgeCreated", addNewNode)
  -- Can't get a path to the destroyed object so must do complete rebuild.
  registerEventHandler("mcNodeDestroyed", requestRebuild)
  registerEventHandler("mcEdgeDestroyed", requestRebuild)
  registerEventHandler("mcNodeRenamed", renameNode)
  registerEventHandler("mcEdgeRenamed", renameNode)
end
--]]