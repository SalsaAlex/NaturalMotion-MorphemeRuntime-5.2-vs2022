------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- Returns the BlendNode which result pin is connected to the Output
------------------------------------------------------------------------------------------------------------------------
local lastBlendNode = function(object)
  local index  = 0
  local found  = false
  local nodeType
  while ((found == false) and
         (index < table.getn(object))) do
    index    = index + 1
    nodeType = getType(object[index])
    if (string.find(nodeType, "Blend") ~= nil) then
      local inputConnections = listConnections{
                               Object = object[index],
                               Downstream = false,
                               Upstream = true,
                               ResolveReferences = true
                             }
      if (table.getn(inputConnections) > 0) then
        nodeType = getType(inputConnections[1])
        if (string.find(nodeType, "Blend") ~= nil) then
          -- Case 2x4
          local outputConnections = listConnections{
                                    Object = object[index],
                                    Downstream = true,
                                    Upstream = false,
                                    ResolveReferences = true
                                  }
          if ((outputConnections[1] == nil) or
              (string.find(getType(outputConnections[1]), "Blend") == nil))then
            found = true
          end
        end
      end
    end
  end
  return object[index]
end

------------------------------------------------------------------------------------------------------------------------
-- Recursive function to reach the anim nodes of selected nodes. Used by getListOfConnections
------------------------------------------------------------------------------------------------------------------------
local getAnimationList
getAnimationList = function(object, selection)
  local listAnims   = { }
  local index       = 1
  local nodeType
  local connections = listConnections{
    Object = object,
    Downstream = false,
    Upstream = true,
    ResolveReferences = true
  }
  while connections[index] ~= nil do
    local isInput = false
  
    specificNodeType, nodeType = getType(connections[index])
    if (string.find(specificNodeType, "Blend") ~= nil) then
      local found = false
      for i = 1, table.getn(selection) do -- only include nodes that are selected
        if (selection[i] == connections[index]) then
          table.insert(listAnims, getAnimationList(connections[index], selection))
          found = true
          break
        end
      end
      if not found then
        isInput = true
      end
    elseif nodeType == "BlendNode" and not isOperatorNode(connections[index]) then
      isInput = true
    end
    -- control parameters and operator nodes not handled...
    
    if isInput then    
      table.insert(listAnims, connections[index])
    end
    
    index = index + 1
  end
  return listAnims
end

------------------------------------------------------------------------------------------------------------------------
-- Validate the attributes of the blend nodes
------------------------------------------------------------------------------------------------------------------------
local validateAttributes = function(object)
  local errorMessage = ""
  local parentNode   = lastBlendNode(object)
  local index        = 1
  local matchEvents  = true
  local nodeType
  local numBlendNodes = 0

  if (parentNode == nil) then
    errorMessage = "Network not suitable for collapse."
    return nil, errorMessage
  end

  for i = 1, table.getn(object) do
    nodeType = getType(object[i])
    if (string.find(nodeType, "Blend") ~= nil) then
      numBlendNodes = numBlendNodes + 1
    end
  end
  if (numBlendNodes < 3) then
    errorMessage = "Network not suitable for collapse."
    return nil, errorMessage
  else 
    if (numBlendNodes > 7) then
      numBlendNodes = 7
    end
  end

  if (getAttribute(parentNode, "TimeStretchMode") == 0) then
    matchEvents = false
  end

  -- SLERP + MatchEvents attributes
  local slerp = getAttribute(parentNode, "SphericallyInterpolateTrajectoryPosition")
  local alwaysBlendTrajectoryAndTransforms = getAttribute(parentNode, "AlwaysBlendTrajectoryAndTransforms")
  local alwaysBlendEvents = getAttribute(parentNode, "AlwaysBlendEvents")
  local timeStretch = getAttribute(parentNode, "TimeStretchMode")
  local loop
  local startIndex
  local passThrough
  local eventOrder
  local userData
  local onOverlap
  local withinRange

  if (matchEvents) then
    loop        = getAttribute(parentNode, "Loop")
    startIndex  = getAttribute(parentNode, "StartEventIndex")
    passThrough = getAttribute(parentNode, "DurationEventBlendPassThrough")
    eventOrder  = getAttribute(parentNode, "DurationEventBlendIgnoreEventOrder")
    userData    = getAttribute(parentNode, "DurationEventBlendSameUserData")
    onOverlap   = getAttribute(parentNode, "DurationEventBlendOnOverlap")
    withinRange = getAttribute(parentNode, "DurationEventBlendWithinRange")
  end
  nodeType = getType(object[index])
  while (index < table.getn(object))do
    if ((string.find(nodeType, "Blend") ~= nil) and (object[index] ~= parentNode))then
      if (slerp~= getAttribute(object[index], "SphericallyInterpolateTrajectoryPosition")) then
        errorMessage = string.format("Nodes %s and %s have different slerp values", parentNode, object[index])
        return nil, errorMessage
      end
      if (timeStretch~= getAttribute(object[index], "TimeStretchMode")) then
          errorMessage = string.format("Nodes %s and %s have different TimeStretchMode values", parentNode, object[index])
          return nil, errorMessage
      end
      if (alwaysBlendTrajectoryAndTransforms ~= getAttribute(object[index], "AlwaysBlendTrajectoryAndTransforms")) then
          errorMessage = string.format("Nodes %s and %s have different Always blend trajectory and transforms values", parentNode, object[index])
          return nil, errorMessage
      end
      if (alwaysBlendEvents ~= getAttribute(object[index], "AlwaysBlendEvents")) then
          errorMessage = string.format("Nodes %s and %s have different Always blend events values", parentNode, object[index])
          return nil, errorMessage
      end
      if (matchEvents) then
        if (loop~= getAttribute(object[index], "Loop")) then
          errorMessage = string.format("Nodes %s and %s have different Loop values", parentNode, object[index])
          return nil, errorMessage
        end
        if (startIndex~= getAttribute(object[index], "StartEventIndex")) then
          errorMessage = string.format("Nodes %s and %s have different StartEventIndex values", parentNode, object[index])
          return nil, errorMessage
        end
        if (passThrough~= getAttribute(object[index], "DurationEventBlendPassThrough")) then
          errorMessage = string.format("Nodes %s and %s have different DurationEventBlendPassThrough values", parentNode, object[index])
          return nil, errorMessage
        end
        if (eventOrder~= getAttribute(object[index], "DurationEventBlendIgnoreEventOrder")) then
          errorMessage = string.format("Nodes %s and %s have different DurationEventBlendIgnoreEventOrder values", parentNode, object[index])
          return nil, errorMessage
        end
        if (userData~= getAttribute(object[index], "DurationEventBlendSameUserData")) then
          errorMessage = string.format("Nodes %s and %s have different DurationEventBlendSameUserData values", parentNode, object[index])
          return nil, errorMessage
        end
        if (onOverlap~= getAttribute(object[index], "DurationEventBlendOnOverlap")) then
          errorMessage = string.format("Nodes %s and %s have different DurationEventBlendOnOverlap values", parentNode, object[index])
          return nil, errorMessage
        end
        if (withinRange~= getAttribute(object[index], "DurationEventBlendWithinRange")) then
          errorMessage = string.format("Nodes %s and %s have different DurationEventBlendWithinRange values", parentNode, object[index])
          return nil, errorMessage
        end
      end
    end
    index = index + 1
    nodeType = getType(object[index])
  end
  -- Blend Weights inter nodes
  index = 2
  nodeType = getType(object[index])
  while (object[index] ~= nil) and (string.find(nodeType, "Blend") ~= nil) and (index < table.getn(object) -1) do
    local blend = table.serialize(getAttribute(object[1], "SourceWeights"))
    local blend2 = table.serialize(getAttribute(object[index], "SourceWeights"))
    if ((object[index] ~= parentNode) and (blend ~= blend2)) then
      for i = 1, 4 do
        if (blend[i] ~= blend2[i]) then
          if ((i == 1) or ((i > 1) and (blend[i] > blend[i - 1]) and (blend2[i] > blend2[i - 1]))) then
            errorMessage = string.format("Nodes %s and %s have different Blend Weight values", object[1], object[index])
            return nil, errorMessage
          end
        end
      end
    end
    index = index + 1
    nodeType = getType(object[index])
  end
  return true
end

------------------------------------------------------------------------------------------------------------------------
-- Returns a list with all the animations in the correct order
------------------------------------------------------------------------------------------------------------------------
local getListOfConnections = function(object)
  local list      = { }
  local resultPin = ""
  -- list of blend nodes selected
  local blendNodes = { }
  for i = 1, table.getn(object) do
    if (string.find(getType(object[i]), "Blend") ~= nil) then
      table.insert(blendNodes, object[i])
    end
  end
  -- Animations
  local lastblend = lastBlendNode(blendNodes)
  
  local anims = getAnimationList(lastblend, object)
  
  for i = 1, table.getn(anims) do
    local aux = { }
    aux = anims[i]
    for j = 1, table.getn(aux) do
      table.insert(list, aux[j])
    end
  end
  -- Result pin of the parent node
  local parentNode = lastBlendNode(blendNodes)
  local result     = listConnections{
                       Object     = parentNode,
                       Downstream = true,
                       Upstream   = false,
                       ResolveReferences = true
                     }
  if (table.getn(result) > 0) then
    resultPin = result[1] .. ".Source" -- Filter node
  else
    resultPin = getParent(object[1]) .. ".Result" -- Output
  end
  return list, resultPin
end

------------------------------------------------------------------------------------------------------------------------
-- Returns a list with all the control parameters in the correct order
------------------------------------------------------------------------------------------------------------------------
local getListOfControlParams = function(object)
  local list = { }
  
  local lastBlend = lastBlendNode(object)
  
  local yWeight = listConnections{
    Object = lastBlend .. ".Weight",
    Upstream = true,
    Downstream = true,
    ResolveReferences = false,
    Pins = true
  }
  
  local connectionsOutNode = listConnections{
    Object = lastBlend,
    Upstream = true,
    Downstream = false,
    ResolveReferences = false
  }
  
  local xWeight = listConnections{
    Object = connectionsOutNode[1] .. ".Weight",
    Upstream = true,
    Downstream = true,
    ResolveReferences = false,
    Pins = true
  }
  
  table.insert(list, xWeight[1])
  table.insert(list, yWeight[1])
  
  return list
end

------------------------------------------------------------------------------------------------------------------------
-- Returns a list of the BlendNxM attributes
------------------------------------------------------------------------------------------------------------------------
local getListOfAttributes = function(object)
  local list        = { }
  local matchEvents = true
  local parentNode  = lastBlendNode(object)
  local connections = listConnections{
    Object = parentNode,
    Downstream = false,
    Upstream = true,
    ResolveReferences = true
  }

  table.insert(list, getAttribute(parentNode, "SphericallyInterpolateTrajectoryPosition")) -- 1
  table.insert(list, getAttribute(parentNode, "AlwaysBlendTrajectoryAndTransforms")) -- 2
  table.insert(list, getAttribute(parentNode, "AlwaysBlendEvents")) -- 3
  
  -- 4 & 5
  if (string.find(getType(parentNode), "BlendN") ~= nil) then
    table.insert(list, getAttribute(parentNode, "SourceWeights"))
  else
    local blendWeigths = getAttribute(parentNode, "BlendWeights")
    for i = 3, 11 do
      blendWeigths[i] = 1.0
    end
    table.insert(list, blendWeigths)
  end
  if (string.find(getType(connections[1]), "BlendN") ~= nil) then
    table.insert(list, getAttribute(connections[1], "SourceWeights"))
  else
    local blendWeigths = getAttribute(connections[1], "BlendWeights")
    for i = 3, 11 do
      blendWeigths[i] = 1.0
    end
    table.insert(list, blendWeigths)
  end
  
  table.insert(list, getAttribute(parentNode, "TimeStretchMode")) -- 6
  if (getAttribute(parentNode, "TimeStretchMode") == 0) then
    matchEvents = false
  end
  if (matchEvents) then
    table.insert(list, getAttribute(parentNode, "Loop")) -- 7
    table.insert(list, getAttribute(parentNode, "StartEventIndex")) -- 8
    table.insert(list, getAttribute(parentNode, "DurationEventBlendPassThrough")) -- 9
    table.insert(list, getAttribute(parentNode, "DurationEventBlendIgnoreEventOrder")) -- 10
    table.insert(list, getAttribute(parentNode, "DurationEventBlendSameUserData")) -- 11
    table.insert(list, getAttribute(parentNode, "DurationEventBlendOnOverlap")) -- 12
    table.insert(list, getAttribute(parentNode, "DurationEventBlendWithinRange")) -- 13
  end

  return list
end

------------------------------------------------------------------------------------------------------------------------
-- Sets the position of a blend node relative to an animation node
------------------------------------------------------------------------------------------------------------------------
local setPositionNxM = function(anim, blendNode)
  local animX, animY = getNodePosition(anim)
  local posX = animX + 500
  local posY = animY - 350
  setNodePosition(blendNode, posX, posY)
end

------------------------------------------------------------------------------------------------------------------------
-- Create and fully connect a BlendNxM Node
------------------------------------------------------------------------------------------------------------------------
local createNxM = function(anims, controlParams, attributes, resultPin, xValue, yValue)
  local sourcePin
  local destPin
  local matchEvents = (table.getn(attributes) > 4)
  local nodeNxM
  -- Create our new node
  nodeNxM = create("BlendNxM", getParent(anims[1]))
  setPositionNxM(anims[table.getn(anims)], nodeNxM)
  -- Set the attributes
  setAttribute(nodeNxM ..".XValue", xValue)
  setAttribute(nodeNxM ..".YValue", yValue)
  setAttribute(nodeNxM ..".SphericallyInterpolateTrajectoryPosition", attributes[1])
  setAttribute(nodeNxM ..".AlwaysBlendTrajectoryAndTransforms", attributes[2])
  setAttribute(nodeNxM ..".AlwaysBlendEvents", attributes[3])
  setAttribute(nodeNxM ..".BlendWeightsX", attributes[4])
  setAttribute(nodeNxM ..".BlendWeightsY", attributes[5])
  setAttribute(nodeNxM ..".TimeStretchMode", attributes[6])
  if (matchEvents) then
    setAttribute(nodeNxM ..".Loop", attributes[7])
    setAttribute(nodeNxM ..".StartEventIndex", attributes[8])
    setAttribute(nodeNxM ..".DurationEventBlendPassThrough", attributes[9])
    setAttribute(nodeNxM ..".DurationEventBlendIgnoreEventOrder", attributes[10])
    setAttribute(nodeNxM ..".DurationEventBlendSameUserData", attributes[11])
    setAttribute(nodeNxM ..".DurationEventBlendOnOverlap", attributes[12])
    setAttribute(nodeNxM ..".DurationEventBlendWithinRange", attributes[13])
  end
  nodeNxMPins  = listPins(nodeNxM)
  -- Connect the anim pins
  for index = 1, 16 do
    if (nodeNxMPins[index] ~= nil) then
      if (anims[index] ~= nil) then
        destPin   = string.format("%s.%s", nodeNxM, nodeNxMPins[index])
        sourcePin = string.format("%s.Result", anims[index])
        connect(sourcePin, destPin)
      end
    end
  end

  -- Connect the control parameter pins
  destPin   = string.format("%s.WeightX", nodeNxM)
  connect(controlParams[2], destPin)
  destPin   = string.format("%s.WeightY", nodeNxM)
  connect(controlParams[1], destPin)

  -- Connect the result of our new node to the Output
  sourcePin   = string.format("%s.Result", nodeNxM)
  connect(sourcePin, resultPin)
  
end

------------------------------------------------------------------------------------------------------------------------
-- Delete all the Blend trees in the Network
------------------------------------------------------------------------------------------------------------------------
local deleteAllBlendTrees = function(object)
  local nodeType
  for index = 0, table.getn(object) do
    if (object[index] ~= nil) then
      nodeType = getType(object[index])
      if string.find(nodeType, "Blend") ~= nil then
        delete(object[index])
      end
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Returns the X and Y values of the structuret
------------------------------------------------------------------------------------------------------------------------
local getNxMSize = function(object, numAnims)
  if numAnims > 16 then
    return 0, 0
  end
  
  local xValue = 2
  local yValue = 2
  local parentNode     = lastBlendNode(object)
  local parentNodePins = listConnections{
                             Object = parentNode,
                             Downstream = false,
                             Upstream = true,
                             ResolveReferences = true
                           }
  if (numAnims == 6) then
    if (table.getn(parentNodePins) - 1 >= 3) then
      xValue = 3
      yValue = 2
    else
      xValue = 2
      yValue = 3
    end
  elseif(numAnims == 8) then
    local childNodePins  = listConnections{
                             Object = parentNodePins[1],
                             Downstream = false,
                             Upstream = true,
                             ResolveReferences = true
                           }
    if (table.getn(childNodePins) - 1 >= 4) then
      xValue = 2
      yValue = 4
    else
      xValue = 4
      yValue = 2
    end
  elseif(numAnims == 9) then
    xValue = 3
    yValue = 3
  elseif(numAnims == 10) then
    if (table.getn(parentNodePins) - 1 >= 5) then
      xValue = 5
      yValue = 2
    else
      xValue = 2
      yValue = 5
    end
  elseif(numAnims == 12) then
    if (table.getn(parentNodePins) - 1 >= 4) then
      xValue = 4
      yValue = 3
    else
      xValue = 3
      yValue = 4
    end
  elseif(numAnims == 15) then
    if (table.getn(parentNodePins) - 1 >= 5) then
      xValue = 5
      yValue = 3
    else
      xValue = 3
      yValue = 5
    end
  elseif(numAnims == 16) then
    xValue = 4
    yValue = 4
  end
  return xValue, yValue
end

------------------------------------------------------------------------------------------------------------------------
-- Determine if every blend node is properly connected 
------------------------------------------------------------------------------------------------------------------------
isNetworkConnected = function(object)
  local list = {}
  local index = 1
  while (index <= table.getn(object)) do
    node = object[index]
    local nodeType = getType(node)
    if (string.find(nodeType, "Blend") ~= nil) then
      local connectionsIn = listConnections{
        Object = node,
        Downstream = false,
        Upstream = true,
        ResolveReferences = true
      }
      -- Has minimum 2 sources
      local numConnections = table.getn(connectionsIn)
      if (numConnections<2) then
        return false
      end
    end
    index = index + 1
  end
  
  local controlParams = getListOfControlParams(object)
  return table.getn(controlParams) == 2
end

------------------------------------------------------------------------------------------------------------------------
-- Determine if you can collapse to NxM
------------------------------------------------------------------------------------------------------------------------
canCollapseToNxMNode = function(optionalObject)
  local selection     = ls("Selection")
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

  local nBlendNodes    = 0
  local index          = 1
  while (index < table.getn(selection)) do
    local nodeType = getType(selection[index])
    -- Count valid blend nodes to collapse
    if (string.find(nodeType, "Blend") ~= nil) and  
       (string.find(nodeType, "BlendNxM") == nil) and -- not BlendNxM
       (string.find(nodeType, "FeatherBlend") == nil) and -- not FeatherBlend
       (string.find(nodeType, "BlendTree") == nil) then -- not BlendTree
      nBlendNodes = nBlendNodes + 1
    end
    index = index + 1
  end
  nBlendNodes = nBlendNodes + 1
  
  if (nBlendNodes < 3) or (nBlendNodes > 6) then
    return false
  end

  return isNetworkConnected(selection)
end

------------------------------------------------------------------------------------------------------------------------
-- COLLAPSE INTO NODE BLEND NxM
------------------------------------------------------------------------------------------------------------------------
collapseNxMNode = function(optionalObject)
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
  local valid, errorMessage  = validateAttributes(selection)
  if (valid) then
    local inputAnims, resultPin = getListOfConnections(selection)
    local controlParams         = getListOfControlParams(selection)
    local attributes            = getListOfAttributes(selection)
    local xValue, yValue        = getNxMSize(selection, table.getn(inputAnims))
    
    if xValue*yValue > 16 or xValue == 0 or yValue == 0 then
      ui.showMessageBox("The selection has too many inputs to create a BlendNxM node", "ok")
    else    
      undoBlock(
          function()
        deleteAllBlendTrees(selection)
        createNxM(inputAnims, controlParams, attributes, resultPin, xValue, yValue)
      end)
    end
  else
    ui.showErrorMessageBox("Not possible to collapse. "..errorMessage, "Collapse error")
  end
end