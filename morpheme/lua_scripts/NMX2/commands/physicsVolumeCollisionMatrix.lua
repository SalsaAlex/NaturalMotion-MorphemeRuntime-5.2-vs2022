------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- physicsBodyCollisionMatrix.lua
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- Lazily create the nmxUtils table.
--
if nmxUtils == nil then
  nmxUtils = { }
end

-- Create a lua string which implements the create command.
-- This essentially just delegates to a function.
--
getPhysicsShape = function(a)
  local child = a:getFirstChild()
  while child ~= nil do
    if child:is(nmx.sgShapeNode.ClassTypeId()) and
       child:getDataNode() ~= nil and
       child:getDataNode():is(nmx.PhysicsBodyNode.ClassTypeId()) then
      return child
    end
    child = child:getNextSibling()
  end
end

collisionStatus = function(a, b)
  if a == nil or b == nil then
    return ""
  end
  local rigRoot = nmx.RigInfoNode.getRigRoot(a:getParent())
  if rigRoot == nil then
    return ""
  end

  local collisionSet
  local rootChild = rigRoot:getFirstChild()
  while rootChild ~= nil do
    if (rootChild:is(nmx.CollisionSetNode.ClassTypeId()) == true) then
      collisionSet = rootChild
    end
    rootChild = rootChild:getNextSibling()
  end

  -- loop through collision groups
  local child = collisionSet:getFirstChild()
  while child ~= nil do
    if (child:is(nmx.CollisionGroupNode.ClassTypeId()) == true) and
       child:hasGroupNode(getPhysicsShape(a)) and child:hasGroupNode(getPhysicsShape(b)) then
      return "Disabled"
    end
    child = child:getNextSibling()
  end
  return ""
end

nodesAreEqual = function(a, b)
  local statusA, pathA = a:getPath(a:getDatabase():getRoot())
  local statusB, pathB = b:getPath(b:getDatabase():getRoot())

  return pathA == pathB
end

isCollisionImplicitlyDisabled = function(a, b)
  if nodesAreEqual(a:getParent():getParent(), b:getParent()) then
    return true
  end
  if nodesAreEqual(a:getParent(), b:getParent()) then
    return true
  end
  return false
end

local callbackImplementation = [[
  -- function to update UI, defined later as it used local variabled defined later
  local updateUI = function() end

  local physicsBodies = { }
  local names = { "" }

  local collisionResult = function(a, b)
    if isCollisionImplicitlyDisabled(a, b) or isCollisionImplicitlyDisabled(b, a) then
      return "Implicitly Disabled"
    end
    return tostring(collisionStatus(a, b))
  end

  local addToList = function(node)
    if node == nil then
      return
    end

    local physicsBody = nmx.PhysicsBodyNode.getPhysicsBody(node)
    if physicsBody == nil then
      if nmx.PhysicsVolumeNode.getPhysicsVolume(node) ~= nil then
        node = node:getParent()
      else
        return
      end
    end
    
    -- Don't add nodes already in the list
    local nodeName = node:getName()
    local alreadyIn = false
    for i, v in pairs(names) do
      if v == nodeName then 
        alreadyIn = true
        break
      end
    end
    
    if alreadyIn == false then
      table.insert(names, nodeName)
      table.insert(physicsBodies, node)
    end
  end

  local addCollisionGroup = function(group)
    local child = group:getFirstChild()
    while child ~= nil do
      if child:is(nmx.RefNode.ClassTypeId()) == true then
        addToList(child:getConnectedNode():getParent())
      end
      child = child:getNextSibling()
    end
  end

  local expandList = function(list)
    for i=1, (list:size()) do
      local listNode = list:getNode(i)
      if listNode:is(nmx.sgTransformNode.ClassTypeId()) then
        addToList(list:getNode(i))
      elseif listNode:is(nmx.RefNode.ClassTypeId()) then
        addToList(list:getNode(i):getConnectedNode():getParent())
      elseif listNode:is(nmx.CollisionGroupNode.ClassTypeId()) then
        addCollisionGroup(list:getNode(i))
      elseif listNode:is(nmx.CollisionSetNode.ClassTypeId()) then
        local child = list:getNode(i):getFirstChild()
        while child ~= nil do
          if child:is(nmx.CollisionGroupNode.ClassTypeId()) == true then
            addCollisionGroup(child)
          end
          child = child:getNextSibling()
        end
      end
    end
  end
  expandList(selectionList)

  -- Create a new modal dialog for setting import options.
  local dlg = ui.createModalDialog{ caption = "Physics Matrix" }
  dlg:beginVSizer{ flags = "expand" }
  local scrollArea = dlg:addScrollPanel{ flags = "expand;both", proportion = 8, size = { width = 700, height = 500 } }

  -- make grid sizer, and add local var controls to it
  scrollArea:beginVSizer{ flags = "expand" }

  local list = scrollArea:addListControl{ columnNames = names, numRows = (table.getn(names)-1), numColumns = table.getn(names), flags = "expand" }

    for i, v in ipairs(physicsBodies) do
      list:setItemValue(i, j, v:getName())

      -- Add names of cells
      for j, w in ipairs(physicsBodies) do
        if j < i then
          list:setItemValue(i, j+1, collisionResult(v, w))
        else
          list:setItemValue(i, j+1, "")
        end
      end
    end

  scrollArea:endSizer()
  dlg:endSizer()
  updateUI()
  dlg:show()
  dlg = nil
  ]]

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
associatedTypes:push_back(nmx.PhysicsVolumeNode.ClassTypeId())

local physicEnabled = not mcn.isPhysicsDisabled or not mcn.isPhysicsDisabled()
if physicEnabled then
  app:registerScriptedCommand(
    "PhysicsTools",
    "Physics Volume Collision Matrix",
    "Shows information about physics volumes",
    callback,
    associatedTypes,
    nmx.TransformNode.ClassTypeId()
    )
end
