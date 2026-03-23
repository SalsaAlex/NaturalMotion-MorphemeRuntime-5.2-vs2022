-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold,
-- licensed or commercially exploited in any manner without the
-- written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential
-- information of NaturalMotion and may not be disclosed to any
-- person nor used for any purpose not expressly approved by
-- NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

-- Get hold of the application
--
local app = nmx.Application.new()
local scriptManager = app:getScriptManager()
local callbackManager = app:getCallbackManager()

------------------------------------------------------------------------------------------------------------------------
-- Associated nodes for physics nodes with child engine specific data
------------------------------------------------------------------------------------------------------------------------
addAssociatedPhysicsEngineDataNodes = function(node, associatedNodes)
  local selectedPhysicsEngine = preferences.get("PhysicsEngine")

  local physicsEngineDataClassId = nmx.PhysicsEngineDataNode.ClassTypeId()
  local euphoriaEnabled = not mcn.isEuphoriaDisabled()

  local child = node:getFirstChild(physicsEngineDataClassId)
  while child do
    if child:is(physicsEngineDataClassId) then
      local physicsEngine = child:getPhysicsEngineName()

      if physicsEngine == selectedPhysicsEngine then
        -- add the selected physics engine data
        --
        associatedNodes:push_back(child)
      elseif euphoriaEnabled and physicsEngine == "Euphoria" then
        -- always add euphoria if it is enabled
        --
        associatedNodes:push_back(child)
      end
    end

    child = child:getNextSibling()
  end
end

addAssociatedEnvironmentBodyNodes = function(node, associatedNodes)
  local childVolumes = node:findAttributeArray("ChildVolumes")
  for i = 1, childVolumes:size() do
    local attr = childVolumes:getAttribute(i)
    local volumeAttr = attr:getInput()
    if volumeAttr:isValid() then
      local volumeTx = volumeAttr:getNode()
      if volumeTx and volumeTx:is(nmx.sgTransformNode.ClassTypeId()) then
        -- loop through the the volume-sgTransform's child sgShapes.
        local child = volumeTx:getFirstChild()
        while child do
          if child:is(nmx.sgShapeNode.ClassTypeId()) then
            local data = child:getDataNode()
            if data then
              if data:is(nmx.PhysicsVolumeNode.ClassTypeId()) then
                associatedNodes:push_back(data)
              elseif data:is(nmx.PrimitiveNode.ClassTypeId()) or data:is(nmx.GeometryNode.ClassTypeId()) then
                associatedNodes:push_back(data)

                -- get the material for the primitive/geometry shape
                local material = child:getMaterial()
                if material then
                  associatedNodes:push_back(material)
                end
              end
            end
          end

          child = child:getNextSibling()
        end
      end
    end
  end
end

addPhysicsBodyAssociatedDesiredOrder = function(desiredOrder)
  desiredOrder:push_back('PrimitiveNode') -- Cylinder/Sphere/Box
  desiredOrder:push_back('GeometryNode') -- Meshes
  desiredOrder:push_back('PhysicsVolumeNode')
  desiredOrder:push_back('PhysicsBodyNode')
end

-- The callback function body
--
local associatedNodesCallback = scriptManager:createCallback(
  "lua",
  nmx.CallbackManager.AssociatedNodesCallbackSignature(),
  [[
    addAssociatedPhysicsEngineDataNodes(node, associatedNodes)

    addPhysicsBodyAssociatedDesiredOrder(desiredOrder)
  ]]
)

local environmentbodyAssociatedNodesCallback = scriptManager:createCallback(
  "lua",
  nmx.CallbackManager.AssociatedNodesCallbackSignature(),
  [[
    addAssociatedPhysicsEngineDataNodes(node, associatedNodes)
    addAssociatedEnvironmentBodyNodes(node, associatedNodes)

    addPhysicsBodyAssociatedDesiredOrder(desiredOrder)
  ]]
)

-- Unregister any previous callbacks
callbackManager:unRegisterAssociatedNodesCallback("PhysicsVolumeNode")
callbackManager:unRegisterAssociatedNodesCallback("PhysicsHingeNode")
callbackManager:unRegisterAssociatedNodesCallback("PhysicsTwistSwingNode")
callbackManager:unRegisterAssociatedNodesCallback("PhysicsRagdollLimitNode")
callbackManager:unRegisterAssociatedNodesCallback("PhysicsBodyNode")
callbackManager:unRegisterAssociatedNodesCallback("EnvironmentPhysicsBodyNode")
callbackManager:unRegisterAssociatedNodesCallback("EnvironmentPhysicsConstraintNode")
callbackManager:unRegisterAssociatedNodesCallback("CharacterControllerNode")

-- Register the new callback
callbackManager:registerAssociatedNodesCallback("PhysicsVolumeNode", associatedNodesCallback)
callbackManager:registerAssociatedNodesCallback("PhysicsHingeNode", associatedNodesCallback)
callbackManager:registerAssociatedNodesCallback("PhysicsTwistSwingNode", associatedNodesCallback)
callbackManager:registerAssociatedNodesCallback("PhysicsRagdollLimitNode", associatedNodesCallback)
callbackManager:registerAssociatedNodesCallback("PhysicsBodyNode", associatedNodesCallback)
callbackManager:registerAssociatedNodesCallback("EnvironmentPhysicsBodyNode", environmentbodyAssociatedNodesCallback)
callbackManager:registerAssociatedNodesCallback("EnvironmentPhysicsConstraintNode", associatedNodesCallback)
callbackManager:registerAssociatedNodesCallback("CharacterControllerNode", associatedNodesCallback)
