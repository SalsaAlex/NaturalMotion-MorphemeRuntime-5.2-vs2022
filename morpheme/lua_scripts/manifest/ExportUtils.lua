------------------------------------------------------------------------------------------------------------------------
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

------------------------------------------------------------------------------------------------------------------------
-- This function searches for a child transform with a PhysicsBody on it
-- and returns true if one is found.
------------------------------------------------------------------------------------------------------------------------
local hasTransformWithPhysicsVolume = function(childJoint)
  -- Finding the physics body
  --
  local transformChild = childJoint:getFirstChild()
  while transformChild ~= nil do
    if transformChild:is(nmx.sgTransformNode.ClassTypeId()) then
      if nmx.PhysicsBodyNode.getPhysicsBody(transformChild) ~= nil then
        local childVolume = transformChild:getFirstChild()

        while childVolume ~= nil do
          if childVolume:is(nmx.sgTransformNode.ClassTypeId()) and
             nmx.PhysicsVolumeNode.getPhysicsVolume(childVolume) ~= nil then
            return true
          end

          childVolume = childVolume:getNextSibling()
        end
      end
    end
    transformChild = transformChild:getNextSibling()
  end

  return false
end

------------------------------------------------------------------------------------------------------------------------
-- This function checks that a joint has a physics joint limit
------------------------------------------------------------------------------------------------------------------------
local hasPhysicsJointLimit = function(nmxJoint)
  if nmx.PhysicsJointNode.getPhysicsJointLimit(nmxJoint, false) then
    return true
  end
  return false
end

------------------------------------------------------------------------------------------------------------------------
-- This function iterates through all the physics joints in the scene
-- retrieving only the ones that have physics volume children.
------------------------------------------------------------------------------------------------------------------------
getPhysicsRigJointsForExport = function(node, requiresPhysicsBody, warningsAndErrors)
  local joints = { }
  local jointCount = 0

  -- Find all the relevant physics joint descendants of 'node'.
  --
  local nodes = nmx.NodeIterator.new(node, nmx.sgTransformNode.ClassTypeId())
  local rootTransform = nil
  nodes:includeRoot(true)

  while nodes:next() do
    local childTransform = nodes:node()

    -- We only want to add physics joint nodes.
    --
    if childTransform:transformNodeIs(nmx.TransformNode.ClassTypeId()) or
       childTransform:transformNodeIs(nmx.PosQuatTransformNode.ClassTypeId()) or
       childTransform:transformNodeIs("HierarchyNode") then

      -- If the transform has any transform bodies on it then add it to the table of physics joints.
      --
      if hasTransformWithPhysicsVolume(childTransform) then
        local isRootJoint = jointCount == 0

        local lastValidVolume = childTransform
        local parent = childTransform:getParent()
        while parent and hasTransformWithPhysicsVolume(parent) do
          lastValidVolume = parent
          parent = parent:getParent()
        end

        if rootTransform and not lastValidVolume:is(rootTransform) then
          table.insert(
            warningsAndErrors.errors,
            string.format("Multiple physics roots found in rigExport, at joints '%s' and '%s'", rootTransform:getName(), lastValidVolume:getName()))
        end

        rootTransform = lastValidVolume

        -- root joint will never have a joint limit
        --
        if not isRootJoint and not hasPhysicsJointLimit(childTransform) then
          table.insert(
            warningsAndErrors.errors,
            string.format("PhysicsJoint '%s' has physics volumes but no physics joint limit.", childTransform:getName()))
        end

        jointCount = jointCount + 1
        joints[jointCount] = childTransform
      elseif hasPhysicsJointLimit(childTransform) then
        table.insert(
          warningsAndErrors.errors,
          string.format("PhysicsJoint '%s' has a physics joint limit but no physics volumes.", childTransform:getName()))
      end

    -- Skip over any nodes that are not PhysicsJoint nodes.
    --
    elseif not requiresPhysicsBody then
      nodes:skipChildren(true)
    end
  end

  return joints
end

------------------------------------------------------------------------------------------------------------------------
-- Raise all the physics joints and update the scene to get valid transforms.
-- The physics rig may not have been not have been rendered in which case the transforms could be in an invalid state,
-- by raising the physics joints and calling update we ensure that they are valid.
------------------------------------------------------------------------------------------------------------------------
preparePhysicsRigForExport = function(scene, physicsJointTxInstances)
  local physicsJointCount = table.getn(physicsJointTxInstances)

  for i = 1, physicsJointCount do
    local physicsJointTxInstance = physicsJointTxInstances[i]
    physicsJointTxInstance:recursiveRaise()
  end

  scene:update()
end

------------------------------------------------------------------------------------------------------------------------
-- Restore the rig back to the state it was in before the export
-- Make sure the physics joints aren't left raised as that will cause them to be needlessly computed all the time.
------------------------------------------------------------------------------------------------------------------------
restorePhysicsRigAfterExport = function(scene, physicsJointTxInstances)
  local physicsJointCount = table.getn(physicsJointTxInstances)

  for i = 1, physicsJointCount do
    local physicsJointTxInstance = physicsJointTxInstances[i]
    physicsJointTxInstance:recursiveLower()
  end

  scene:update()
end

validatePhysicsRig = function(set)
  local app = nmx.Application.new()
  local scene = app:getSceneByName("AssetManager")

  -- get these 
  local physicsSceneRoot = anim.getPhysicsRigSceneRoot(scene, set)

  local errorsReturn = { errors = { }, warnings = { } }

  getPhysicsRigJointsForExport(physicsSceneRoot, false, errorsReturn)

  return errorsReturn
end

validatePhysicsRigAndInformUser = function(set)
  local errorsAndWarnings = validatePhysicsRig(set)
  
  local log = ""
    
  local numErrors = table.getn(errorsAndWarnings.errors)
  if numErrors ~= 0  then
    log = log .. "The physics rig has " .. tostring(numErrors) .. " errors"
  
    for _,v in pairs(errorsAndWarnings.errors) do
      log = log .. "\n - " .. v   
    end
  end
  
  local numWarnings = table.getn(errorsAndWarnings.warnings)
  if numWarnings ~= 0  then
    log = log .. "The physics rig has " .. tostring(numWarnings) .. " warnings"
  
    for _,v in pairs(errorsAndWarnings.warnings) do
      log = log .. "\n - " .. v   
    end
  end
  
  if log ~= "" then
    ui.showMessageBox(log, "ok")
  end
end