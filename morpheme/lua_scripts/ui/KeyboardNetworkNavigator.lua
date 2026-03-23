------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

-- only look at nodes in the current graph and in the requested direction
------------------------------------------------------------------------------------------------------------------------
getCurrentNodesInGraph = function(nodes, selectedNodeX, selectedNodeY, SelectedNode, direction)
local filteredNodes = { }
local selectedNodeWidth, selectedNodeHeight = getNodeSize(SelectedNode)
for i, value in ipairs(nodes) do
  if (value ~= SelectedNode) then
    if canNavigate(value) then
    local horizontalPos, verticalPos = getNodePosition(value)
    local width, height = getNodeSize(value)

      if (selectedNodeY > verticalPos + (height / 2)) and direction == "UP" then
      filteredNodes[table.getn(filteredNodes) + 1] = value

      elseif (selectedNodeY + (selectedNodeHeight / 2) < verticalPos) and direction == "DOWN" then
      filteredNodes[table.getn(filteredNodes) + 1] = value

      elseif (selectedNodeX + (selectedNodeWidth / 2) < horizontalPos) and direction == "RIGHT" then
      filteredNodes[table.getn(filteredNodes) + 1] = value

      elseif (selectedNodeX > horizontalPos + (width / 2)) and direction == "LEFT" then
      filteredNodes[table.getn(filteredNodes) + 1] = value
      end

    end
  end
end
 return filteredNodes
end -- function

-- get the closest node in the predefined direction using "hypotenuse + offset orthogonal to the direction" to find the best match
------------------------------------------------------------------------------------------------------------------------
findClosestNode = function(nodes, SelectedNode, Direction)

local selectedNodeWidth, selectedNodeHeight = getNodeSize(SelectedNode)
local selectedNodeX, selectedNodeY = getNodePosition(SelectedNode)

local selectedCenterX = selectedNodeX + (selectedNodeWidth / 2)
local selectedCenterY = selectedNodeY + (selectedNodeHeight / 2)

-- use the first node in the list as a reference
local refNodeX, refNodeY = getNodePosition(nodes[1])
local refCatA = math.abs(selectedNodeX - refNodeX)
local refCatB = math.abs(selectedNodeY - refNodeY)
local closestDistance = math.sqrt((refCatA * refCatA) + (refCatB * refCatB)) + refCatA + refCatB
local closestNode = nodes[1]

local width, height, centerX, centerY, catA, catB, tmpDistance = 0.0
for i, value in ipairs(nodes) do
topLeftX, topLeftY = getNodePosition(value)
width, height = getNodeSize(value)
centerX = topLeftX + (width / 2)
centerY = topLeftY + (height / 2)

 if (Direction == "RIGHT") then
   catA = math.abs(topLeftX - (selectedNodeX + selectedNodeWidth))
   catB = math.abs(selectedCenterY - centerY)
   tmpDistance = math.sqrt((catA * catA) + (catB * catB))
 elseif (Direction == "LEFT") then
   catA = math.abs(selectedNodeX - (topLeftX + width))
   catB = math.abs(selectedCenterY - centerY)
   tmpDistance = math.sqrt((catA * catA) + (catB * catB))
 elseif (Direction == "UP") then
   catA = math.abs(topLeftX - selectedNodeX)
   catB = math.abs(selectedNodeY - (topLeftY + height))
   tmpDistance = math.sqrt((catA * catA) + (catB * catB))
 elseif (Direction == "DOWN") then
   catA = math.abs(topLeftX - selectedNodeX)
   catB = math.abs(topLeftY - (selectedNodeY + selectedNodeHeight))
   tmpDistance = math.sqrt((catA * catA) + (catB * catB))
 end

  if ((tmpDistance) < closestDistance) then
    closestDistance = tmpDistance
    closestNode = value
  end
end

return closestNode
end -- function

-- shift the graph to make sure the selcted node is in view
------------------------------------------------------------------------------------------------------------------------
ensureNodeIsInView = function(node)
local network = ui.getWindow("MainFrame|LayoutManager|Network|Network")
local size = network:getSize()
local myX, myY = getNodePosition(node)
screenX, screenY = mcn.graphToScreenPos(getCurrentGraph(), myX, myY)

local panX, panY = 0 , 0
if (screenX < 25) then panX = math.abs(screenX) + 50 end
if (screenX > (size.width - 100)) then panX = (-(screenX - size.width)) - 200 end
if (screenY < 25) then panY = math.abs(screenY) + 50 end
if (screenY > (size.height - 100)) then panY = (-(screenY - size.height)) - 200 end

currentGraphX, currentGraphY, zoom = getGraphTransform(getCurrentGraph())
setGraphTransform(getCurrentGraph(), currentGraphX + panX, currentGraphY + panY, zoom)
end -- function

------------------------------------------------------------------------------------------------------------------------
selectFrstNavigatableNode = function(nodes)
  for index, node in ipairs(nodes) do
    _, subtype = getType(node)
    if canNavigate(node) then
      select(node)
      ensureNodeIsInView(node)
    end
  end
end  -- function

------------------------------------------------------------------------------------------------------------------------
canNavigate = function(node)
local type, subtype = getType(node)
return (subtype ~= "Transition") and (subtype ~= nil)
end  -- function

------------------------------------------------------------------------------------------------------------------------
selectNodeUp = function()
local graphPath = getCurrentGraph()
local graphNodes = listChildren(graphPath)
local selection = ls("Selection")
if table.getn(selection) == 1 and table.getn(graphNodes) > 1 then
  -- make sure we select a navigatable node in our current graph
  if (getParent(selection[1]) ~= graphPath) or not canNavigate(selection[1]) then
    select(selection[1], false)
    selectFrstNavigatableNode(graphNodes)
    return
  end

  local topLeftX, topLeftY = getNodePosition(selection[1])
  local interestNodes = getCurrentNodesInGraph(graphNodes, topLeftX, topLeftY, selection[1], "UP")
  if table.getn(interestNodes) > 0 then
    local closestNode = findClosestNode(interestNodes, selection[1], "UP")
    select(closestNode)
    ensureNodeIsInView(closestNode)
  end
end

end -- function

------------------------------------------------------------------------------------------------------------------------
selectNodeDown = function()
local graphPath = getCurrentGraph()
local graphNodes = listChildren(graphPath)
local selection = ls("Selection")
if table.getn(selection) == 1 and table.getn(graphNodes) > 1 then
  -- make sure we select a navigatable node in our current graph
  if (getParent(selection[1]) ~= graphPath) or not canNavigate(selection[1]) then
    select(selection[1], false)
    selectFrstNavigatableNode(graphNodes)
    return
  end
  local topLeftX, topLeftY = getNodePosition(selection[1])
  local interestNodes = getCurrentNodesInGraph(graphNodes, topLeftX, topLeftY, selection[1], "DOWN")
  if table.getn(interestNodes) > 0 then
    local closestNode = findClosestNode(interestNodes, selection[1], "DOWN")
    select(closestNode)
    ensureNodeIsInView(closestNode)
  end
end

end -- function

------------------------------------------------------------------------------------------------------------------------
selectNodeLeft = function()
local graphPath = getCurrentGraph()
local graphNodes = listChildren(graphPath)
local selection = ls("Selection")
if table.getn(selection) == 1 and table.getn(graphNodes) > 1 then
  -- make sure we select a navigatable node in our current graph
  if (getParent(selection[1]) ~= graphPath) or not canNavigate(selection[1]) then
    select(selection[1], false)
    selectFrstNavigatableNode(graphNodes)
    return
  end
  local topLeftX, topLeftY = getNodePosition(selection[1])
  local interestNodes = getCurrentNodesInGraph(graphNodes, topLeftX, topLeftY, selection[1], "LEFT")
  if table.getn(interestNodes) > 0 then
    local closestNode = findClosestNode(interestNodes, selection[1], "LEFT")
    select(closestNode)
    ensureNodeIsInView(closestNode)
  end
end

end -- function

------------------------------------------------------------------------------------------------------------------------
selectNodeRight = function()
local graphPath = getCurrentGraph()
local graphNodes = listChildren(graphPath)
local selection = ls("Selection")
if table.getn(selection) == 1 and table.getn(graphNodes) > 1 then
  -- make sure we select a navigatable node in our current graph
  if (getParent(selection[1]) ~= graphPath) or not canNavigate(selection[1]) then
    select(selection[1], false)
    selectFrstNavigatableNode(graphNodes)
    return
  end
  local topLeftX, topLeftY = getNodePosition(selection[1])
  local interestNodes = getCurrentNodesInGraph(graphNodes, topLeftX, topLeftY, selection[1], "RIGHT")
  if table.getn(interestNodes) > 0 then
    local closestNode = findClosestNode(interestNodes, selection[1], "RIGHT")
    select(closestNode)
    ensureNodeIsInView(closestNode)
  end
end

end -- function

------------------------------------------------------------------------------------------------------------------------
navigateToChildState = function()
local selection = ls("Selection")
if (table.getn(selection) == 1) then
  if table.getn(listChildren(selection[1])) > 0 then
    setCurrentGraph(selection[1])
  end
 end
end -- function

------------------------------------------------------------------------------------------------------------------------
navigateToParentState = function()
local currentState = getCurrentGraph()
if currentState ~= "" then
    local parent, _ = splitNodePath(currentState)
    setCurrentGraph(parent)
    select(currentState)
end
end -- function

------------------------------------------------------------------------------------------------------------------------
zoomIn = function()
max = 10.0
currentGraphX, currentGraphY, zoomFactor = getGraphTransform(getCurrentGraph())
newZoom = zoomFactor + 0.1
setGraphTransform(getCurrentGraph(), currentGraphX, currentGraphY, math.min(newZoom, max))
end

------------------------------------------------------------------------------------------------------------------------
zoomOut = function()
min = 0.1
currentGraphX, currentGraphY, zoomFactor = getGraphTransform(getCurrentGraph())
newZoom = zoomFactor - 0.1
setGraphTransform(getCurrentGraph(), currentGraphX, currentGraphY, math.max(newZoom, min))
end

------------------------------------------------------------------------------------------------------------------------
setupNetworkNavigator = function()
  local network = ui.getWindow("MainFrame|LayoutManager|Network|Network")
  network:addAccelerator("Up", selectNodeUp)
  network:addAccelerator("Down", selectNodeDown)
  network:addAccelerator("Left", selectNodeLeft)
  network:addAccelerator("Right", selectNodeRight)
  network:addAccelerator("PGDN", navigateToChildState)
  network:addAccelerator("PGUP", navigateToParentState)
  network:addAccelerator("KP_ADD", zoomIn)
  network:addAccelerator("KP_SUBTRACT", zoomOut)
end -- function

