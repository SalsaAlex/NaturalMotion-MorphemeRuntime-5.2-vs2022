------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- Find out if the Network is optimal for conversion
------------------------------------------------------------------------------------------------------------------------
local validateNetworkExpand = function(object)
  local error        = false
  local errorMessage = ""
  if( object == nil ) then
    return nil, ""
  end
  -- Validate X & Y values depending on the number of pins connected
  local nodeCountX = getAttribute(object, "XValue")
  local nodeCountY = getAttribute(object, "YValue")
  
  local expectedConnections = nodeCountX * nodeCountY
  local numberOfConnections = 0
  
  local index = 0
  local baseName = "Source"
  while true do
    local pin = string.format("%s.%s%d", object, baseName, index)
    if not pinExists(pin) then
      break
    end
    if isConnected(pin) then
      numberOfConnections = numberOfConnections + 1
    end
    index = index + 1
  end  
  
  local connections = listConnections{
    Object     = object,
    Downstream = false,
    Upstream   = true,
    ResolveReferences = true
  }
  
  if expectedConnections ~= numberOfConnections then
    error = true
    errorMessage = "The number of connected pins does not match the expected X and Y values."
  end
  -- Check control parameters are connected
  if (not isConnected(string.format("%s.WeightX", object)) or
     not isConnected(string.format("%s.WeightY", object))) then
      error = true
      errorMessage = "All the control parameters must be connected."
  end
  if error then
    return nil, errorMessage
  end
  return true
end

------------------------------------------------------------------------------------------------------------------------
-- Get the name of the BlendNxM node connected to the output
------------------------------------------------------------------------------------------------------------------------
local getBlendNxMNameExpand = function(object)
  local nodeType
  for i = 1, table.getn(object) do
    nodeType = getType(object[i])
    if (nodeType == "BlendNxM") then
      local connections = listConnections{
        Object = object[i],
        Downstream = false,
        Upstream = true,
        ResolveReferences = true
      }
      if (connections ~= nil) and
        (table.getn(connections) > 0) then
        local matchEvents = true
        if (getAttribute(object[i], "TimeStretchMode") == 0) then
          matchEvents = false
        end
        return object[i], matchEvents
      end
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Returns a list with all the animations in the correct order
------------------------------------------------------------------------------------------------------------------------
local getListOfPinsExpand = function(object)
  local inputAnims    = { }
  local controlParams = { }
  local resultPin     = ""
  
  local index = 0
  local baseName = "Source"
  while true do
    local pin = string.format("%s.%s%d", object, baseName, index)
    if not pinExists(pin) then
      break
    end
    
    -- Input pins
    local connections = listConnections{
      Object     = pin,
      Downstream = false,
      Upstream   = true,
      ResolveReferences = true
    }
      
    if table.getn(connections) > 0 then
      table.insert(inputAnims, connections[1])
    end
    
    index = index + 1
  end  
  
  -- Control parameters
  local weightXInput =  listConnections{
    Object     = string.format("%s.WeightX", object),
    Downstream = false,
    Upstream   = true,
    ResolveReferences = true
  }  
  table.insert(controlParams, weightXInput[1])
  
  local weightYInput =  listConnections{
    Object     = string.format("%s.WeightY", object),
    Downstream = false,
    Upstream   = true,
    ResolveReferences = true
  }
  table.insert(controlParams, weightYInput[1])
  
  -- Result pin
  local result = listConnections{
    Object     = object,
    Downstream = true,
    Upstream   = false,
    ResolveReferences = true
  }
  if (table.getn(result) > 0) then
    resultPin = result[1] .. ".Source" -- Filter node
  else
    resultPin = getParent(object) .. ".Result" -- Output
  end
  return inputAnims, controlParams, resultPin
end

------------------------------------------------------------------------------------------------------------------------
-- Returns a list of the BlendNxM attributes
------------------------------------------------------------------------------------------------------------------------
local getListOfAttributesExpand = function(object, matchEvents)
  local list   = { }
  local xValue = getAttribute(object, "XValue")
  local yValue = getAttribute(object, "YValue")

  table.insert(list, getAttribute(object, "SphericallyInterpolateTrajectoryPosition")) -- 1
  table.insert(list, getAttribute(object, "AlwaysBlendTrajectoryAndTransforms")) -- 2
  table.insert(list, getAttribute(object, "AlwaysBlendEvents")) -- 3
  table.insert(list, getAttribute(object, "BlendWeightsX")) -- 4
  table.insert(list, getAttribute(object, "BlendWeightsY")) -- 5
  table.insert(list, getAttribute(object, "TimeStretchMode")) -- 6
  if (matchEvents) then
    table.insert(list, getAttribute(object, "Loop")) -- 7
    table.insert(list, getAttribute(object, "StartEventIndex")) -- 8
    table.insert(list, getAttribute(object, "DurationEventBlendPassThrough")) -- 9
    table.insert(list, getAttribute(object, "DurationEventBlendIgnoreEventOrder")) -- 10
    table.insert(list, getAttribute(object, "DurationEventBlendSameUserData")) -- 11
    table.insert(list, getAttribute(object, "DurationEventBlendOnOverlap")) -- 12
    table.insert(list, getAttribute(object, "DurationEventBlendWithinRange")) -- 13
  end
  return list, xValue, yValue
end

------------------------------------------------------------------------------------------------------------------------
-- Create and configute the new blend N nodes
------------------------------------------------------------------------------------------------------------------------
local createBlendNodesExpand = function(inputAnims, attributes, controlParams, resultPin, xValue, yValue)
  local listNodes = { }
  local sourcePin
  local destPin
  local blendNodes  = 3
  local matchEvents = (table.getn(attributes) > 4)
  -- Create four new nodes
  local parentNode = getParent(inputAnims[1])
  local node1      = create("BlendN", parentNode)
  local node2      = create("BlendN", parentNode)
  local node3      = create("BlendN", parentNode)
  local node4
  local node5
  local node6
  local node7
  table.insert(listNodes, node1)
  table.insert(listNodes, node2)
  table.insert(listNodes, node3)
  if (xValue == 3) then
    node4 = create("BlendN", parentNode)
    table.insert(listNodes, node4)
    blendNodes = 4
  elseif(xValue == 4) then
    node4 = create("BlendN", parentNode)
    node5 = create("BlendN", parentNode)
    table.insert(listNodes, node4)
    table.insert(listNodes, node5)
    blendNodes = 5
  elseif(xValue == 5) then
    node4 = create("BlendN", parentNode)
    node5 = create("BlendN", parentNode)
    node6 = create("BlendN", parentNode)
    table.insert(listNodes, node4)
    table.insert(listNodes, node5)
    table.insert(listNodes, node6)
    blendNodes = 6
  end
  -- Set the position of the nodes
  if (blendNodes == 3) then
    setPositionXNode(inputAnims[1], node1, 350)
    setPositionYNode(node1, node2, 350)
    setPositionXNode(inputAnims[3], node3, 600)
  elseif(blendNodes == 4) then
    setPositionXNode(inputAnims[1], node1, 350)
    setPositionYNode(node1, node2, 250)
    setPositionYNode(node2, node3, 250)
    setPositionXNode(node2, node4, 200)
  elseif(blendNodes == 5) then
    setPositionXNode(inputAnims[1], node1, 350)
    setPositionYNode(node1, node2, 300)
    setPositionYNode(node2, node3, 300)
    setPositionYNode(node3, node4, 300)
    setPositionXNode(node2, node5, 350)
  elseif(blendNodes == 6) then
    setPositionXNode(inputAnims[1], node1, 350)
    setPositionYNode(node1, node2, 300)
    setPositionYNode(node2, node3, 300)
    setPositionYNode(node3, node4, 300)
    setPositionYNode(node4, node5, 300)
    setPositionXNode(node2, node6, 350)
  elseif (blendNodes == 7) then
    setPositionXNode(inputAnims[1], node1, 350)
    setPositionYNode(node1, node2, 250)
    setPositionYNode(node2, node3, 250)
    setPositionYNode(node3, node4, 250)
    setPositionXNode(node2, node5, 200)
    setPositionYNode(node5, node6, 250)
    setPositionXNode(node5, node7, 200)
  end

  -- Set the connections
  local j      = 1
  local source = 0
  for i = 0, table.getn(inputAnims) - 1 do
    destPin   = string.format("%s.Source%s", listNodes[j], source)
    sourcePin = string.format("%s.Result", inputAnims[i + 1])
    connect(sourcePin, destPin)
    source = source + 1
    if (math.mod(i + 1, yValue)) == 0 then
      j      = j + 1
      source = 0
    end
  end
  -- Connect the parent node
  if (blendNodes <= 6) then
    for i = 0, blendNodes - 2 do
      sourcePin = string.format("%s.Result", listNodes[i + 1])
      destPin   = string.format("%s.Source%s", listNodes[blendNodes], i)
      connect(sourcePin, destPin)
    end
  end
  -- Connect the output
  sourcePin   = string.format("%s.Result", listNodes[blendNodes])
  connect(sourcePin, resultPin)
  -- Connect the control parameters
  for i = 1, blendNodes - 1 do
    sourcePin = string.format("%s.Result", controlParams[2])
    destPin   = string.format("%s.Weight", listNodes[i])
    connect(sourcePin, destPin)
  end
  sourcePin = string.format("%s.Result", controlParams[1])
  destPin   = string.format("%s.Weight", listNodes[blendNodes])
  connect(sourcePin, destPin)
  
    -- Set the attributes of the nodes, must be done after the connections so that the source weights can be validated corretcly
  for i = 1, table.getn(listNodes) do
    setAttribute(listNodes[i] .. ".SphericallyInterpolateTrajectoryPosition", attributes[1])
    setAttribute(listNodes[i] .. ".AlwaysBlendTrajectoryAndTransforms", attributes[2])
    setAttribute(listNodes[i] .. ".AlwaysBlendEvents", attributes[3])
    setAttribute(listNodes[i] .. ".SourceWeightDistribution", 0) -- Set to custom distribution
    setAttribute(listNodes[i] .. ".SourceWeights", attributes[5]) -- BlendWeightY
    setAttribute(listNodes[i] .. ".TimeStretchMode", attributes[6])
    if (matchEvents) then -- Match Events attributes
      setAttribute(listNodes[i] ..".Loop", attributes[7])
      setAttribute(listNodes[i] ..".StartEventIndex", attributes[8])
      setAttribute(listNodes[i] ..".DurationEventBlendPassThrough", attributes[9])
      setAttribute(listNodes[i] ..".DurationEventBlendIgnoreEventOrder", attributes[10])
      setAttribute(listNodes[i] ..".DurationEventBlendSameUserData", attributes[11])
      setAttribute(listNodes[i] ..".DurationEventBlendOnOverlap", attributes[12])
      setAttribute(listNodes[i] ..".DurationEventBlendWithinRange", attributes[13])
    end
  end
  setAttribute(listNodes[table.getn(listNodes)] .. ".SourceWeightDistribution", 0) -- Set to custom distribution
  setAttribute(listNodes[table.getn(listNodes)] ..".SourceWeights", attributes[4]) -- BlendWeightX
end

-- Sets the position of a node relative to other node with same Y ------------
setPositionXNode = function(sourceNode, destNode, distance)
  local sourceX, sourceY = getNodePosition(sourceNode)
  setNodePosition(destNode, sourceX + distance, sourceY)
end

-- Sets the position of a node relative to other node with same X ------------
setPositionYNode = function(sourceNode, destNode, distance)
  local sourceX, sourceY = getNodePosition(sourceNode)
  setNodePosition(destNode, sourceX, sourceY + distance)
end

------------------------------------------------------------------------------------------------------------------------
-- Enable or disable the Expand button depending on the objects selected
------------------------------------------------------------------------------------------------------------------------
local enableExpandButton = function()
  local expandButton = ui.getWindow("MainFrame|LayoutManager|Network|ToolBar|ExpandNodeNxM")
  local selection    = ls("Selection")
  local found        = false
  local index        = 0
  local nodeType
  while ((index < table.getn(selection)) and (found == false)) do
    index = index + 1
    nodeType = getType(selection[index])
    if (nodeType == "BlendNxM") then
      found = true
    end
  end
  if (found) then
    expandButton:enable(true)
  else
    expandButton:enable(false)
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Determine if you can collapse to NxM
------------------------------------------------------------------------------------------------------------------------
canExpandNxMNode = function(optionalObject)
  local selection    = ls("Selection")

  if (optionalObject ~= nil) then
    local hasObj = false
    for i, v in ipairs(selection) do
      if v == optionalObject then
         hasObj = true
         break
       end
    end
    if not hasObj then
      table.insert(selection, optionalObject)
    end
  end
  
  local found = false
  local index = 0
  local nodeType
  while ((index < table.getn(selection)) and (found == false)) do
    index = index + 1
    nodeType = getType(selection[index])
    if (nodeType == "BlendNxM") then
      return true
    end
  end

  return false
end

------------------------------------------------------------------------------------------------------------------------
-- EXPAND BLEND NODE NxM TO HIERARCHY
------------------------------------------------------------------------------------------------------------------------
expandNxMNode = function(optionalObject)
  local selection = ls("Selection")

  if (optionalObject ~= nil) then
    local hasObj = false
    for i, v in ipairs(selection) do
      if v == optionalObject then
         hasObj = true
         break
       end
    end
    if not hasObj then
      table.insert(selection, optionalObject)
    end
  end

  local nxm, matchEvents    = getBlendNxMNameExpand(selection)
  local valid, errorMessage = validateNetworkExpand(nxm)
  if (valid) then
    local inputAnims, controlParams, resultPin  = getListOfPinsExpand(nxm)
    local attributes, xValue, yValue            = getListOfAttributesExpand(nxm, matchEvents)
    undoBlock(
        function()
      delete(nxm)
      createBlendNodesExpand(inputAnims, attributes, controlParams, resultPin, xValue, yValue)
    end)
  else
    ui.showErrorMessageBox("Not possible to expand. "..errorMessage, "Expand error")
  end
end