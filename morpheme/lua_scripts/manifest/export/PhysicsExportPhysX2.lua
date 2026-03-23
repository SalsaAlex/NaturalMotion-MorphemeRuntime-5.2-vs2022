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

------------------------------------------------------------------------------------------------------------------------
local getShapeMinimumDimension = function(shape)
  local type = shape:getTypeId()
  if type == nmx.BoxNode.ClassTypeId() then
    return math.min(shape:getWidth(), shape:getHeight(), shape:getDepth())
  elseif type == nmx.CapsuleNode.ClassTypeId() then
    return math.min(shape:getRadius(), shape:getHeight())
  elseif type == nmx.CylinderNode.ClassTypeId() then
    return math.min(shape:getRadius(), shape:getHeight())
  elseif type == nmx.SphereNode.ClassTypeId() then
    return shape:getRadius()
  end
  return 0.0
end

local exporterPhysX2 = {
  ----------------------------------------------------------------------------------------------------------------------
  exportPart = function(physicsBody, partExport, warningsAndErrors)
    local physicsEngineData = physicsBody:getFirstChild("PhysX2BodyDataNode")

    if physicsEngineData then
      local solverIterationCount = physicsEngineData:findAttribute("SolverIterationCount"):asInt()
      partExport:setUIntAttribute("solverIterationCount", solverIterationCount)

      local usesSleepEnergyThreshold = physicsEngineData:findAttribute("UseSleepEnergyThreshold"):asBool()
      partExport:setBoolAttribute("usesSleepEnergyThreshold", usesSleepEnergyThreshold)

      local sleepEnergyThreshold = physicsEngineData:findAttribute("SleepEnergyThreshold"):asFloat()
      partExport:setDoubleAttribute("sleepEnergyThreshold", sleepEnergyThreshold)

      if physicsBody:findAttribute("EnableMaxAngularVelocity"):asBool() then
        partExport:setDoubleAttribute("maxAngularVelocity", physicsBody:findAttribute("MaxAngularVelocity"):asFloat())
      else
        partExport:setDoubleAttribute("maxAngularVelocity", -1)
      end
    else
      table.insert(
        warningsAndErrors.warnings,
        string.format("PhysicsBody '%s' has no PhysX2 attributes, using default values.\n", physicsBody:getName()))

      partExport:setUIntAttribute("solverIterationCount", 4)
      partExport:setBoolAttribute("usesSleepEnergyThreshold", false)
      partExport:setDoubleAttribute("sleepEnergyThreshold", 0.00005)
      partExport:setDoubleAttribute("maxAngularVelocity", -1)
    end
  end,
  
 ----------------------------------------------------------------------------------------------------------------------
  supportsPerVolumeAuthoring = function()
      return true
  end,
  
  ----------------------------------------------------------------------------------------------------------------------
  exportShape = function(physicsJointTxInstance, physicsVolume, shape, shapeExport, warningsAndErrors)
    local physicsEngineData = physicsVolume:getFirstChild("PhysX2VolumeDataNode")

    if physicsEngineData then
      local dynamicFriction = physicsEngineData:findAttribute("DynamicFriction"):asFloat()
      shapeExport:setDoubleAttribute("dynamicFriction", dynamicFriction)

      local frictionCombineMode = physicsEngineData:findAttribute("FrictionCombineMode"):asString()
      shapeExport:setStringAttribute("frictionCombineMode", frictionCombineMode)

      local restitutionCombineMode = physicsEngineData:findAttribute("RestitutionCombineMode"):asString()
      shapeExport:setStringAttribute("restitutionCombineMode", restitutionCombineMode)

      local disableStrongFriction = physicsEngineData:findAttribute("DisableStrongFriction"):asBool()
      shapeExport:setBoolAttribute("disableStrongFriction", disableStrongFriction)

      local skinWidth = physicsEngineData:findAttribute("SkinWidth"):asFloat()
      if getShapeMinimumDimension(shape) < skinWidth then
        table.insert(
          warningsAndErrors.warnings,
          string.format("PhysicsVolume '%s' owned by PhysicsJoint '%s' has a skin width larger than the smallest part size.\n", physicsVolume:getName(), physicsJointTxInstance:getName()))
      end

      shapeExport:setDoubleAttribute("skinwidth", skinWidth)
    else
      table.insert(
        warningsAndErrors.warnings,
        string.format("PhysicsVolume '%s' owned by PhysicsJoint '%s' has no PhysX2 attributes, using default values.\n", physicsVolume:getName(), physicsJointTxInstance:getName()))

      shapeExport:setDoubleAttribute("dynamicFriction", 1.0)
      shapeExport:setStringAttribute("frictionCombineMode", "Multiply")
      shapeExport:setStringAttribute("restitutionCombineMode", "Multiply")
      shapeExport:setBoolAttribute("disableStrongFriction", false)
      shapeExport:setDoubleAttribute("skinwidth", 0.01)
    end
  end,

  ----------------------------------------------------------------------------------------------------------------------
  exportJoint = function(physicsJointLimit, jointExport, warningsAndErrors)
    if physicsJointLimit:is("PhysicsTwistSwingNode") then
      local physicsEngineData = physicsJointLimit:getFirstChild("PhysX2TwistSwingDataNode")

      if physicsEngineData then
        jointExport:setBoolAttribute("useSlerpDrive", physicsEngineData:findAttribute("DriveType"):asInt() == 0) -- SLERPDrive

        jointExport:setDoubleAttribute("swingDriveSpring", physicsEngineData:findAttribute("SwingDriveSpring"):asFloat())
        jointExport:setDoubleAttribute("swingDriveDamping", physicsEngineData:findAttribute("SwingDriveDamping"):asFloat())

        jointExport:setDoubleAttribute("twistDriveSpring", physicsEngineData:findAttribute("TwistDriveSpring"):asFloat())
        jointExport:setDoubleAttribute("twistDriveDamping", physicsEngineData:findAttribute("TwistDriveDamping"):asFloat())

        jointExport:setDoubleAttribute("slerpDriveSpring", physicsEngineData:findAttribute("SLERPDriveSpring"):asFloat())
        jointExport:setDoubleAttribute("slerpDriveDamping", physicsEngineData:findAttribute("SLERPDriveDamping"):asFloat())

        jointExport:setBoolAttribute("useAccelerationSprings", physicsEngineData:findAttribute("DriveSpringIsAcceleration"):asBool())
      else
        table.insert(
          warningsAndErrors.warnings,
          string.format("PhysicsJointLimit '%s' has no PhysX2 attributes, using default values.\n", physicsJointLimit:getName()))

        jointExport:setBoolAttribute("useSlerpDrive", true)

        jointExport:setDoubleAttribute("swingDriveSpring", 0.0)
        jointExport:setDoubleAttribute("swingDriveDamping", 0.0)

        jointExport:setDoubleAttribute("twistDriveSpring", 0.0)
        jointExport:setDoubleAttribute("twistDriveDamping", 0.0)

        jointExport:setDoubleAttribute("slerpDriveSpring", 0.0)
        jointExport:setDoubleAttribute("slerpDriveDamping", 0.0)

        jointExport:setBoolAttribute("useAccelerationSprings", false)
      end
    elseif physicsJointLimit:is("PhysicsHingeNode") then
      local physicsEngineData = physicsJointLimit:getFirstChild("PhysX2HingeDataNode")

      if physicsEngineData then
        jointExport:setBoolAttribute("useSlerpDrive", false)

        jointExport:setDoubleAttribute("twistDriveSpring", physicsEngineData:findAttribute("DriveSpring"):asFloat())
        jointExport:setDoubleAttribute("twistDriveDamping", physicsEngineData:findAttribute("DriveDamping"):asFloat())

        jointExport:setDoubleAttribute("swingDriveSpring", 0.0)
        jointExport:setDoubleAttribute("swingDriveDamping", 0.0)

        jointExport:setDoubleAttribute("slerpDriveSpring", physicsEngineData:findAttribute("DriveSpring"):asFloat())
        jointExport:setDoubleAttribute("slerpDriveDamping", physicsEngineData:findAttribute("DriveDamping"):asFloat())

        jointExport:setBoolAttribute("useAccelerationSprings", physicsEngineData:findAttribute("DriveSpringIsAcceleration"):asBool())
      else
        table.insert(
          warningsAndErrors.warnings,
          string.format("PhysicsJointLimit '%s' has no PhysX2 attributes, using default values.\n", physicsJointLimit:getName()))

        jointExport:setBoolAttribute("useSlerpDrive", false)

        jointExport:setDoubleAttribute("swingDriveSpring", 0.0)
        jointExport:setDoubleAttribute("swingDriveDamping", 0.0)

        jointExport:setDoubleAttribute("twistDriveSpring", 0.0)
        jointExport:setDoubleAttribute("twistDriveDamping", 0.0)

        jointExport:setDoubleAttribute("slerpDriveSpring", 0.0)
        jointExport:setDoubleAttribute("slerpDriveDamping", 0.0)

        jointExport:setBoolAttribute("useAccelerationSprings", false)
      end
    else
      table.insert(
        warningsAndErrors.errors,
        string.format("PhysicsJointLimit '%s' of type '%s' is not supported by PhysX2", physicsJointLimit:getName(), physicsJointLimit:getTypeString()))
    end
  end,

  ----------------------------------------------------------------------------------------------------------------------
  exportController = function(target, controllerData)
    local extraPhysX2Info = controllerData:getFirstChild("PhysX2CharacterControllerDataNode")
    target:setDoubleAttribute("StepHeight",extraPhysX2Info:findAttribute("StepHeight"):asFloat())
  end
}

if getPhysicsDriverExporter("PhysX2") then
  unregisterPhysicsDriverExporter("PhysX2")
end
registerPhysicsDriverExporter("PhysX2", exporterPhysX2)