------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2012 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold,
-- licensed or commercially exploited in any manner without the
-- written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential
-- information of NaturalMotion and may not be disclosed to any
-- person nor used for any purpose not expressly approved by
-- NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------
require "manifest/ExportUtils.lua"
require "manifest/export/PhysicsExport.lua"
require "manifest/export/PhysicsPartExport.lua"
require "manifest/export/PhysicsJointExport.lua"
require "manifest/export/PhysicsCollisionGroupExport.lua"
------------------------------------------------------------------------------------------------------------------------

local MaxJointLimit = 64

-- this allows the file to be reexecute from within connect without having to restart
-- it must be a global for this to work
__physicsRigExportHandlers = __physicsRigExportHandlers or { }

------------------------------------------------------------------------------------------------------------------------
local getIndexInArray = function(objects, object)
  for i, current in ipairs(objects) do
    if object:is(current) then
      return i
    end
  end

  return -1
end

------------------------------------------------------------------------------------------------------------------------
-- This function does the actual exporting of the rigExport data from connect into the xml file
------------------------------------------------------------------------------------------------------------------------
local onPhysicsRigExport = function(rigExport, animSetName, physicsRigSceneRootPath, collisionSetsPath, sceneIndex, sourceFileName, previousExportGUID, currentGUID, options)
  options.shouldWrite = true

  local warningsAndErrors = { }
  warningsAndErrors.errors = { }
  warningsAndErrors.warnings = { }

  if previousExportGUID ~= "" and previousExportGUID == currentGUID then
    -- The GUID stored in the currently exported asset is the same as the asset of the rigExport held im morpheme, don't export again.
    -- inform connect not to overwrite the old rigExport
    --
    options.shouldWrite = false
    return warningsAndErrors
  end

  -- ensure that the scene is valid
  --
  
  app.warning("sceneIndex " .. tostring(sceneIndex))
  app.warning("animSetName " .. animSetName)
  app.warning("physicsRigSceneRootPath " .. physicsRigSceneRootPath)
  app.warning("collisionSetsPath " .. collisionSetsPath)
  
  local scene = nmx.Application.new():getScene(sceneIndex)
  if scene == nil then
    table.insert(
      warningsAndErrors.errors,
      string.format("Could not find scene with index '%d' when exporting physics rig '%s' for animation set '%s'.", sceneIndex, sourceFileName, animSetName))
    return warningsAndErrors
  end

  local physicsRigRootTxInstance = scene:getNodeFromPath(physicsRigSceneRootPath)
  local collisionSets = scene:getNodeFromPath(collisionSetsPath)

  -- Ensure that the physics root is a valid
  --
  if physicsRigRootTxInstance == nil or scene == nil then
    table.insert(
      warningsAndErrors.errors,
      string.format("Could not find physics rig root '%s' when exporting physics rig '%s' for animation set '%s'.", physicsRigSceneRootPath, sourceFileName, animSetName))
    return warningsAndErrors
  end

  -- export all physics joints with bodies under the root as parts and associated shapes
  --
  local physicsJointTxInstances = getPhysicsRigJointsForExport(physicsRigRootTxInstance, false, warningsAndErrors)
  if table.getn(warningsAndErrors.errors) > 0 then
    return warningsAndErrors
  end

  local physicsJointCount = table.getn(physicsJointTxInstances)

  -- validate the physics joints, must have at least one physics joint and every physics joint must have an animation equivilent
  --
  if physicsJointCount == 0 then
    table.insert(
      warningsAndErrors.errors,
      string.format("The physics rig '%s' contains no physics joints.", sourceFileName))
    return warningsAndErrors
  end

  if physicsJointCount > MaxJointLimit then
    table.insert(
      warningsAndErrors.errors,
      string.format("The physics rig '%s' has %d physics joints, the maximum number of allowed joints is %d", sourceFileName, physicsJointCount, MaxJointLimit))    
    return warningsAndErrors
  end

  for partIndex = 1, physicsJointCount do
    local physicsJointTxInstance = physicsJointTxInstances[partIndex]
    local physicsJointName = physicsJointTxInstance:getName()

    if not anim.getRigJoint(scene, animSetName, physicsJointName) then
      table.insert(
        warningsAndErrors.errors,
        string.format("Unable to find matching joint '%s' in animation rig for animation set '%s'", physicsJointName, animSetName))
      return warningsAndErrors
    end
  end

  preparePhysicsRigForExport(scene, physicsJointTxInstances)

  -- Export all the physics parts.
  --
  local inverseRootMatrix = physicsRigRootTxInstance:getWorldMatrix()
  inverseRootMatrix:invert()
  
  app.warning("inverserootmatrix")
  for key, value in pairs(nmx.matrixToTable(inverseRootMatrix)) do
    app.warning("," .. value)
  end

  local parts = { }
  for partIndex = 1, physicsJointCount do
    local physicsJointTxInstance = physicsJointTxInstances[partIndex]
    local physicsJoint = nmx.PhysicsJointNode.getPhysicsJoint(physicsJointTxInstance)

    -- is this joint a trajectory helper
    --
    if physicsJointTxInstance:findTag("TrajectoryCalculationJoint") then
      rigExport:addTrajectoryCalcMarkupPart(partIndex)
    end

    local partExport = exportPhysicsPart(rigExport, inverseRootMatrix, physicsJointTxInstance, partIndex, warningsAndErrors)
    if table.getn(warningsAndErrors.errors) > 0 then
      restorePhysicsRigAfterExport(scene, physicsJointTxInstances)
      return warningsAndErrors
    end
    
    -- does this joint contain the physics root offset,
    -- this must be done after the partExport is created otherwise the call to setPhysicsRootPart will fail
    --
    local physicsRootOffset = physicsJoint:getFirstChild("PhysicsRootOffsetNode")
    if physicsRootOffset then
      local rootOffsetTranslation = physicsRootOffset:getTranslation()
      local rootOffsetRotation = nmx.Quat.new()
      physicsRootOffset:getRotation(rootOffsetRotation)

      rigExport:setPhysicsRootPart(partIndex, nmx.vector3ToTable(rootOffsetTranslation), nmx.vector3ToTable(rootOffsetRotation))
    end

    -- add the part to the list of created parts so joint limits and disabled collision sets can reference them later.
    table.insert(parts, partExport)
  end

  -- export all the physics joint limits
  --
  for partIndex = 1, physicsJointCount do
    local physicsJointTxInstance = physicsJointTxInstances[partIndex]

    local physicsJointLimit = nmx.PhysicsJointNode.getPhysicsJointLimit(physicsJointTxInstance, false)

    -- only the root part is not allowed to have a joint limit
    --
    if not physicsJointLimit and partIndex ~= 1 then
      table.insert(
        warningsAndErrors.errors,
        string.format("PhysicsJoint '%s' in physics rig '%s' has no PhysicsJointLimit.", physicsJointTxInstance:getName(), sourceFileName))
    end

    local parentTxInstance = physicsJointTxInstance:getParent()
    local parentPartIndex = getIndexInArray(physicsJointTxInstances, parentTxInstance)
    local childPartIndex = getIndexInArray(physicsJointTxInstances, physicsJointTxInstance)
    
    if parentPartIndex == -1 then
      table.insert(
        warningsAndErrors.warnings,
        string.format("PhysicsJointLimit '%s' in physics rig '%s' has no parent PhysicsJoint, exporting of this limit was skipped.", physicsJointTxInstance:getName(), sourceFileName))
    elseif childPartIndex == -1 then
      table.insert(
        warningsAndErrors.warnings,
        string.format("PhysicsJointLimit '%s' in physics rig '%s' has no child PhysicsJoint, exporting of this limit was skipped.", physicsJointTxInstance:getName(), sourceFileName))
    else
      local jointExport = exportPhysicsJoint(rigExport, physicsJointLimit, parentPartIndex, childPartIndex, warningsAndErrors)

      if not mcn.isEuphoriaDisabled() then
        local physicsSoftJointLimit = nmx.PhysicsJointNode.getPhysicsJointLimit(physicsJointTxInstance, true)

        if physicsSoftJointLimit ~= nil then
          -- check the soft limit is of the same type as the hard limit
          --
          if not physicsSoftJointLimit:is(physicsJointLimit:getTypeId()) then
            local message = string.format(
              "Mismatched limit types for PhysicsJoint '%s', PhysicsJointLimit type '%s' does not match PhysicsSoftJointLimit type '%s'",
              physicsJointTxInstance:getName(),
              physicsJointLimit:getTypeString(),
              physicsSoftJointLimit:getTypeString())
            table.insert(warningsAndErrors.errors, message)
          end

          exportPhysicsSoftJointLimit(jointExport, physicsSoftJointLimit, warningsAndErrors)
        end
      end

      if table.getn(warningsAndErrors.errors) > 0 then
        restorePhysicsRigAfterExport(scene, physicsJointTxInstances)
        return warningsAndErrors
      end
    end
  end

  -- export all the collision sets
  --
  if collisionSets ~= nil then
    local collisionGroup = collisionSets:getFirstChild()
    while collisionGroup ~= nil do
      if collisionGroup:is(nmx.CollisionGroupNode.ClassTypeId()) then
        exportPhysicsCollisionGroup(rigExport, collisionGroup, physicsJointTxInstances, parts)
      end
      collisionGroup = collisionGroup:getNextSibling()
    end
  end

  restorePhysicsRigAfterExport(scene, physicsJointTxInstances)

  return warningsAndErrors
end

-- register the function to handle the rigExport export event
--
if __physicsRigExportHandlers.export ~= nil then
  unregisterEventHandler("mcPhysicsRigExport", __physicsRigExportHandlers.export)
end
registerEventHandler("mcPhysicsRigExport", onPhysicsRigExport)
__physicsRigExportHandlers.export = onPhysicsRigExport

------------------------------------------------------------------------------------------------------------------------
-- This function is used by anim.getPhysicsChannelNames() to return the physics joint names for a given physics rig.
------------------------------------------------------------------------------------------------------------------------
local onGetPhysicsRigJointNames = function(sceneIndex, physicsRigSceneRootPath)
  local app = nmx.Application.new()
  local scene = app:getScene(sceneIndex)
  local physicsRigRootTxInstance = scene:getNodeFromPath(physicsRigSceneRootPath)
  
  if physicsRigRootTxInstance then
    local warningsAndErrors = {
      warnings = { },
      errors = { }
    }
    local physicsJointTxInstances = getPhysicsRigJointsForExport(physicsRigRootTxInstance, false, warningsAndErrors)
    local physicsJointCount = table.getn(physicsJointTxInstances)

    local physicsJointNames = { }
    table.setn(physicsJointNames, physicsJointCount)

    for i = 1, physicsJointCount do
      local physicsJointTxInstance = physicsJointTxInstances[i]
      physicsJointNames[i] =  physicsJointTxInstance:getName()
    end

    return physicsJointNames
  end
  
  return nil
end

-- register the function to handle the physics rigExport joint name event.
--
if __physicsRigExportHandlers.channelNames ~= nil then
  unregisterEventHandler("mcGetPhysicsRigChannelNames", __physicsRigExportHandlers.channelNames)
end
registerEventHandler("mcGetPhysicsRigChannelNames", onGetPhysicsRigJointNames)
__physicsRigExportHandlers.channelNames = onGetPhysicsRigJointNames

------------------------------------------------------------------------------------------------------------------------
-- This function is used by anim.getPhysicsRigSize() to return the physics joint count for a given physics rig.
------------------------------------------------------------------------------------------------------------------------
local onGetPhysicsRigSize = function(physicsRigSceneRootPath)
  local app = nmx.Application.new()
  local scene = app:getScene(sceneIndex)
  local physicsRigRootTxInstance = scene:getNodeFromPath(physicsRigSceneRootPath)

  if physicsRigRootTxInstance then
    local physicsJointTxInstances = getPhysicsRigJointsForExport(physicsRigRootTxInstance)
    return table.getn(physicsJointTxInstances)
  end

  return 0
end

-- register the function to handle the physics rigExport joint name event
--
if __physicsRigExportHandlers.rigSize ~= nil then
  unregisterEventHandler("mcGetPhysicsRigSize", __physicsRigExportHandlers.rigSize)
end
registerEventHandler("mcGetPhysicsRigSize", onGetPhysicsRigSize)
__physicsRigExportHandlers.rigSize = onGetPhysicsRigSize