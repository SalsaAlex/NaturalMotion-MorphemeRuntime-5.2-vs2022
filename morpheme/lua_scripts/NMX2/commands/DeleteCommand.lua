------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- DistributeMassCommand.lua
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- Lazily create the nmxUtils table.
--
if nmxUtils == nil
then
  nmxUtils = { }
end

local transformHasShapeType = function(transform, type)
  local child = transform:getFirstChild()
  while child ~= nil do
    if child:is(nmx.sgShapeNode.ClassTypeId()) then
    local shape = child:getDataNode()
      if shape:is(type) then
        return true
      end
    end
    child = child:getNextSibling()
  end
  return false
end

local transformHasNoShape = function(transform)
  -- a transform cant not have a shape if it isnt a transform
  if transform:is(nmx.sgTransformNode.ClassTypeId()) == false then
    return false
  end
  
  local child = transform:getFirstChild()
  while child ~= nil do
    if child:is(nmx.sgShapeNode.ClassTypeId()) then
      return false
    end
    child = child:getNextSibling()
  end
  return true
end

local countNodes = function(root, classTypeId)
  local it = nmx.NodeIterator.new(root, classTypeId)
  local result = 0
  while it:next() do
    result = result + 1
  end
  
  return result
end

local getDeleteCommandForNode = function(db, node)
  -- only the root offset frame should be deletable
  if transformHasShapeType(node,nmx.Application.new():lookupTypeId("OffsetFrameNode")) and
      not transformHasShapeType(node:getParent(),nmx.Application.new():lookupTypeId("OffsetFrameNode")) then
    return "Core", "Generic Delete"

  elseif transformHasShapeType(node,nmx.Application.new():lookupTypeId("InteractionProxyNode")) then
    return "morpheme:connect", "Delete Interaction Proxy"
    
  elseif transformHasShapeType(node,nmx.PhysicsJointNode.ClassTypeId()) then
    return "Physics Tools", "Weld To Parent"
    
  elseif node:is(nmx.BodyGroupNode.ClassTypeId()) then
    return "Group", "Delete body group"

  elseif node:is(nmx.CollisionGroupNode.ClassTypeId()) then
    return "Core", "Generic Delete"

  elseif transformHasShapeType(node,nmx.JointNode.ClassTypeId()) then
    return "morpheme:connect", "Delete Joint"
  
  elseif node:is(nmx.RefNode.ClassTypeId()) then
    return "Group", "Remove From Group"
    
  elseif node:is(nmx.Application.new():lookupTypeId("LimbNode")) then
    return "Group", "Remove From Group"
    
  elseif transformHasShapeType(node,nmx.GeometryNode.ClassTypeId()) and db:getName() == "AssetManager" then
    return
    
  elseif transformHasShapeType(node,nmx.JointLimitNode.ClassTypeId()) then
    return "Physics Tools", "Delete Limit"
    
  elseif transformHasShapeType(node, nmx.EnvironmentPhysicsConstraintNode.ClassTypeId()) then
    return "Physics Tools", "DeleteSimplePhysicsNode"
    
  elseif transformHasShapeType(node, nmx.PhysicsBodyNode.ClassTypeId()) then
    return "Physics Tools", "DeleteSimplePhysicsNode"
    
  elseif transformHasShapeType(node, nmx.TriggerVolumeNode.ClassTypeId()) then
    return "Core", "Generic Delete"
    
  elseif transformHasShapeType(node, nmx.PhysicsVolumeNode.ClassTypeId()) then
    return "Physics Tools", "DeleteSimplePhysicsNode"
    
  elseif transformHasShapeType(node, nmx.CharacterStartPointNode.ClassTypeId())
      and db:getName() ~= "AssetManager"
      and (countNodes(db:getRoot(), nmx.CharacterStartPointNode.ClassTypeId()) > 1) then
    return "Core", "Generic Delete"
    
  elseif node:is(nmx.ExtremityNode.ClassTypeId()) then
    return "Limb", "Delete limb"
  end
end

local fixSelectionList = function(db, selectionList)
  local fixedSelectionList = nmx.SelectionList.new()
  local sl = nmx.SelectionList.new()
  
  -- used to store nodes which should be removed later as a node which references them is also selected
  local postRemoveNodes = {}
  
  for i=1,selectionList:size() do
    local node = selectionList:getNode(i)
    sl:clear()
    sl:add(node)
    
    -- find the correct delete command
    local namespace, name = getDeleteCommandForNode(db, node)
    if namespace ~= nil and name ~= nil then
      if nmx.Application.new():canRunCommand(namespace, name, db, sl) == true then
        fixedSelectionList:add(node)
      end
    end
    
    -- remove referenced nodes if their reference is also selected
    if node:is(nmx.RefNode.ClassTypeId()) then
      local connectedNode = node:getConnectedNode()
      
      -- if the linked node is an sgShape, test its transform
      if connectedNode:is(nmx.sgShapeNode.ClassTypeId()) then
        if selectionList:isSelected(connectedNode) or selectionList:isSelected(connectedNode:getParent()) then
          table.insert(postRemoveNodes, connectedNode)
          table.insert(postRemoveNodes, connectedNode:getParent())
        end     
        
      -- if the linked node is a shape, test its sgShape and also transform
      elseif connectedNode:is(nmx.ShapeNode.ClassTypeId()) then
        if selectionList:isSelected(connectedNode) or selectionList:isSelected(connectedNode:getFirstInstance())  or selectionList:isSelected(connectedNode:getFirstInstance():getParent()) then
          table.insert(postRemoveNodes, connectedNode)
          table.insert(postRemoveNodes, connectedNode:getFirstInstance())
          table.insert(postRemoveNodes, connectedNode:getFirstInstance():getParent())
        end        
        
      -- otherwise if its an sgTransform, just test it.
      elseif connectedNode:is(nmx.sgTransformNode.ClassTypeId()) then
        if selectionList:isSelected(connectedNode) then
          table.insert(postRemoveNodes, connectedNode)
        end
      end
    end
  end
  
  -- now we will iterate over the fixed list and remove referenced nodes which also have their reference selected
  -- this is due to the way the navigator won't let a user select the refnode individually, which will be fixed later
  for i,v in ipairs(postRemoveNodes) do
    if v ~= nil then
      fixedSelectionList:remove(v)
    end
  end
  
  return fixedSelectionList
end

executeDeleteCommand = function(db, selectionList)
  local fixedSelectionList = fixSelectionList(db, selectionList)
  if fixedSelectionList:size() ~= 0 then
    local sl = nmx.SelectionList.new()
    
    for i=1,fixedSelectionList:size() do
      local node = fixedSelectionList:getNode(i)
          
      -- test to see that a different delete in the loop hasn't already deleted this node
      -- testing this flag is faster than sorting all the nodes in lua.
      if not node:isDestroyed() then
        -- find the correct delete command
        local namespace, name = getDeleteCommandForNode(db, node)
        
        if namespace ~= nil and name ~= nil then
          sl:clear();
          sl:add(node)
          local commandReturn = nmx.Application.new():runCommand(namespace, name, db, sl):asString()
          if commandReturn ~= "kSuccess" then
            -- command failed
            nmx.Application.new():logError("Attempting to run the command " .. namespace .. " " .. name .. " on the current selection failed with \"" .. commandReturn .. "\".")
            return
          end
        else
          nmx.Application.new():logError("No command found to delete the current selection")
        end
      end
    end
  else
    nmx.Application.new():logWarning("Cannot run the delete command on the given selection")
  end
end

-- Create a lua string which implements the command.
-- This essentially just delegates to a function.
--
local callbackImplementation = [[executeDeleteCommand(database, selectionList)]]

-- Create a new callback object with the implementation provided by
-- callbackImplementation.
--
local app = nmx.Application.new()
local scriptManager = app:getScriptManager()
local callback, errorString = scriptManager:createCallback(
  "lua",
  nmx.Application.ScriptedCommandsSignature(),
  callbackImplementation
)

local associatedTypes = nmx.UIntArray.new()
associatedTypes:push_back(nmx.Node.ClassTypeId())

-- Register the scripted command create transform node.
--
app:registerScriptedCommand(
  "Core",
  "Delete",
  "Delete the current selection",
  callback,
  associatedTypes,
  nmx.TransformNode.ClassTypeId()
  )

