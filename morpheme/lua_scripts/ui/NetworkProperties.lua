------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- Network Properties dialog
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- Widgets in the control
------------------------------------------------------------------------------------------------------------------------
local networkNodeCountText
local networkDepthText
local nodesByTypeList
local unusedCParamsList
local unusedRequestsList
local animUsageList
local sortAnimsByName = true
local reverseAnimSort = false

------------------------------------------------------------------------------------------------------------------------
-- Recusing through the network
------------------------------------------------------------------------------------------------------------------------
local networkNodeCount
local networkDepth
local nodeTypeUsage
local currentDepth

local processNode
processNode = function(currNode)

  if currNode then

    -- Now process all children
    children = listChildren(currNode)
    if (children) then
      for i, child in ipairs(children) do
        -- Recurse in
        currentDepth = currentDepth + 1
        processNode(child)
      end
    end

    -- now recurse out
    if (currentDepth > networkDepth) then networkDepth = currentDepth end
  else
    error("nil value for curr node")
  end
  currentDepth = currentDepth - 1
end

local getNetworkStats = function()

  local allItems = ls()
  for _,currNode in allItems do
    if (getType(currNode) ~= "BlendTree") and
       (getType(currNode) ~= "PhysicsBlendTree") and
       (getType(currNode) ~= "ControlParameters") and
       (getType(currNode) ~= "OutputControlParameters") then
      -- increment the count of this node type
      if (nodeTypeUsage[getType(currNode)] == nil) then
        nodeTypeUsage[getType(currNode)] = 1
      else
        nodeTypeUsage[getType(currNode)] = nodeTypeUsage[getType(currNode)] + 1
      end

      networkNodeCount = networkNodeCount + 1
    end
  end

  -- recurse through the network building up a list of all data
  currentDepth = 0
  processNode("")
end

------------------------------------------------------------------------------------------------------------------------
-- Getting and deleting unused CParams
------------------------------------------------------------------------------------------------------------------------
local getUnusedCParams = function()
  -- generate a list of unused control parameters
  local unused = { }
  local numUnused = 0

  -- list all control parameters
  local allUsed = ls("ControlParameter")

  -- list unconnected control parameters first
  local i, cparam
  for i, cparam in ipairs(allUsed) do
    -- Treat any control parameters that are connected to references as 'used' even if the reference doesn't do anything with them
    if (table.getn(listConnections({ Object = cparam, ResolveReferences = false })) == 0) then
      numUnused = numUnused + 1
      _, unused[numUnused] = splitNodePath(cparam)
    end
  end

  -- list unused control parameters by subtracting cParams used by transition conditions from unconnected ones
  local cParamConditions = { ls("ControlParamTest"), ls("ControlParamInRange")}
  local v
  local index = 0
  for i, v in cParamConditions do
    local j, condition
    for j, condition in ipairs(cParamConditions[i]) do
      index = index + 1
      _, referencedParam = splitNodePath(getAttribute(condition, "ControlParameter"))
      for k, unusedParam in unused do
        if (referencedParam == unusedParam) then
          unused[k] = nil
          break
        end
      end
    end
  end
  
  -- TODO: Need to make sure that ouput control parameters are correctly flagged as being used if they have an input connection to themselves.

  return unused
end

local deleteSelectedCParams = function()
  unusedCParamsList:getSelectedRows()
  local i, v
  local itemsToRemove = { }
  local numItemsToRemove = 0;
  -- Build a list of things to delete
  for i, v in pairs(unusedCParamsList:getSelectedRows()) do
    numItemsToRemove = numItemsToRemove + 1
    itemsToRemove[numItemsToRemove] = unusedCParamsList:getItemValue(v, 1)
  end
  -- now do the deletion
  for i, v in pairs(itemsToRemove) do
    delete("ControlParameters|"..v)
    local j = 1
    while (unusedCParamsList:getItemValue(j, 1) ~= v) do
      j = j + 1
    end
    unusedCParamsList:removeRow(j)
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Getting and deleting unused Requests
------------------------------------------------------------------------------------------------------------------------
local getUnusedRequests = function()
  -- generate a list of unused requests
  local unused ={ }
  local numUnused = 0
  local isUsed = { }

  -- get a list of all request being used in the system
  local allUsed = ls("MessageCondition")
  for i, request in ipairs(allUsed) do
    local requestPath = getAttribute(request, "Message")
    isUsed[requestPath] = true
  end

  -- get a list of all request that are in the system (used or not)
  local allRequests = ls ("Request")
  for i, request in ipairs(allRequests) do
    -- if they dont match, remove them
    if (isUsed[request] == nil) then
      numUnused = numUnused + 1
      _, unused[numUnused] = splitNodePath(request)
    end
  end
  return unused
end

local deleteSelectedRequests = function()
  unusedCParamsList:getSelectedRows()
  local itemsToRemove = { }
  local numItemsToRemove = 0;
  -- Build a list of things to delete
  for i, v in pairs(unusedRequestsList:getSelectedRows()) do
    numItemsToRemove = numItemsToRemove + 1
    itemsToRemove[numItemsToRemove] = unusedRequestsList:getItemValue(v, 1)
  end
  -- now do the deletion
  for i, v in pairs(itemsToRemove) do
    delete("Requests|"..v)
    local j = 1
    while (unusedRequestsList:getItemValue(j, 1) ~= v) do
      j = j + 1
    end
    unusedRequestsList:removeRow(j)
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Getting animation file usage stats
------------------------------------------------------------------------------------------------------------------------
local getAnimFileStats = function()
  local animSets = listAnimSets()
  local numAnimSets = table.getn(animSets)
  local animList = { }
  local animUsage = { }

  for i, v in ipairs(ls("AnimWithEvents")) do
    for asIdx, asVal in animSets do
      local animationTake = getAttribute(v, "AnimationTake", asVal)
      local name = animationTake.filename .. "|"..animationTake.takename
      if (animList[name] == nil) then
        animList[name] = 1
      else
        animList[name] = animList[name] + 1
      end
    end
  end

  -- construct a sortable table from the list of anims
  for i, v in pairs(animList) do
    table.insert(animUsage, { name = i, count = v })
  end

  -- sort the table appropriately
  if sortAnimsByName then  
    if reverseAnimSort then
      table.sort(animUsage, function(a, b) return a.name > b.name end)
    else
      table.sort(animUsage, function(a, b) return a.name < b.name end)
    end
  else
    if reverseAnimSort then
      table.sort(animUsage, function(a, b) return a.count > b.count end)
    else
      table.sort(animUsage, function(a, b) return a.count < b.count end)
    end
  end

  return animUsage
end

local refreshAnimList = function()
  animUsageList:clearRows()
  local animUsage = getAnimFileStats()
  for i, v in ipairs(animUsage) do
    rowData = { v.name, string.format("%i", v.count) }
    animUsageList:addRow(rowData)
  end
end

local onSortAnimsByName = function()
  if sortAnimsByName then
    reverseAnimSort = not reverseAnimSort
  else
    sortAnimsByName = true
  end
  refreshAnimList()
end

local onSortAnimsByNumber = function()
    if not sortAnimsByName then
    reverseAnimSort = not reverseAnimSort
  else
    sortAnimsByName = false
  end
  refreshAnimList()
end

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
showNetworkProperties = function()

  local dlg = ui.getWindow("NetworkProperties")
  if not dlg then
    dlg = ui.createModalDialog{ name = "NetworkProperties", caption = "Network Properties", resize = true, size = { width = 600, height = 400 } }

    dlg:beginHSizer{ }
      dlg:beginVSizer{ flags = "expand", proportion = 0 }
        -- Stats
        dlg:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 0 }
          dlg:setFlexGridColumnExpandable(2)
          dlg:addStaticText{ text = "Number of Nodes" }
          networkNodeCountText = dlg:addStaticText{ }
          dlg:addStaticText{ text = "Max Depth of network" }
          networkDepthText = dlg:addStaticText{ }
        dlg:endSizer()
        -- Nodes by type
        dlg:addStaticText{ text = "Number of Nodes by type" }
        nodesByTypeList = dlg:addListControl{ flags = "expand", proportion = 1, columnNames = { "Node Type", "Count" } };
        -- Unused CParams
        dlg:beginHSizer{ flags = "expand" }
          dlg:addStaticText{ text = "Unused Control Parameters", proportion = 1 }
          dlg:addButton{ label = "Delete Selected", onClick = deleteSelectedCParams }
        dlg:endSizer()
        unusedCParamsList = dlg:addListControl{ flags = "expand;multiSelect", proportion = 1 };
        -- Unused Requests
        dlg:beginHSizer{ flags = "expand" }
          dlg:addStaticText{ text = "Unused Messages", flags = "expand", proportion = 1 }
          dlg:addButton{ label = "Delete Selected", onClick = deleteSelectedRequests }
        dlg:endSizer()
        unusedRequestsList = dlg:addListControl{ flags = "expand;multiSelect", proportion = 1 };

      dlg:endSizer()

      dlg:beginVSizer{ flags = "expand", proportion = 1 }
        -- anim list
        dlg:beginHSizer{ flags = "expand" }
          dlg:addStaticText{ text = "Animations", proportion = 1 }
          dlg:addButton{ label = "Sort By Name", onClick = onSortAnimsByName }
          dlg:addButton{ label = "Sort By Number", onClick = onSortAnimsByNumber }
        dlg:endSizer()
        animUsageList = dlg:addListControl{ flags = "expand", proportion = 1, columnNames = { "Anim Name", "Count" } };
      dlg:endSizer()
    dlg:endSizer()
  end

  -- Clear out any stale data
  networkNodeCount = 0
  networkDepth =0

  unusedCParams = { }
  nodeTypeUsage = { }

  nodesByTypeList:clearRows()
  unusedCParamsList:clearRows()
  unusedRequestsList:clearRows()
  animUsageList:clearRows()

  -- General network stats
  getNetworkStats()
  networkNodeCountText:setLabel(string.format("%i", networkNodeCount))
  networkDepthText:setLabel(string.format("%i", networkDepth))
  for i, v in pairs(nodeTypeUsage) do
    rowData = { i, string.format("%i", v) }
    nodesByTypeList:addRow(rowData)
  end

  -- Unused CParams
  local unusedCParams = getUnusedCParams()
  for i, v in pairs(unusedCParams) do
    unusedCParamsList:addRow(v)
  end

  -- Unused Requests
  local unusedRequests = getUnusedRequests()
  for i, v in pairs(unusedRequests) do
    unusedRequestsList:addRow(v)
  end

  -- Animation Files
  refreshAnimList()

  dlg:show()

end

-- This is useful debugging code that ensures that the dialogs get created each time this script is executed
-- Useful when testing changes
--[[
local dlg
dlg = ui.getWindow("NetworkProperties")
if dlg then
  dlg = nil
  collectgarbage()
end

showNetworkProperties()
--]]

