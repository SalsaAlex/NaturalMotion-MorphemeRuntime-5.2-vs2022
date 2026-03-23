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
-- converts PhysicsJointNode.CombineModes to the string value required by export
------------------------------------------------------------------------------------------------------------------------
local combineModeEnumToString = function(intValue)
  if intValue == nmx.PhysicsJointNode.CombineModes.kMultiply then
    return "CM_MULTIPLY"
  elseif intValue == nmx.PhysicsJointNode.CombineModes.kAverage then
    return "CM_AVERAGE"
  elseif intValue == nmx.PhysicsJointNode.CombineModes.kMin then
    return "CM_MIN"
  elseif intValue == nmx.PhysicsJointNode.CombineModes.kMax then
    return "CM_MAX"
  end
  return "CM_MULTIPLY"
end

local exporterPhysX3 = {
  ----------------------------------------------------------------------------------------------------------------------
  exportPart = function(physicsBody, partExport, warningsAndErrors)
    local physicsEngineData = physicsBody:getFirstChild("PhysX3BodyDataNode")

    if physicsEngineData then
      local positionSolverIterationCount = physicsEngineData:findAttribute("PositionSolverIterationCount"):asInt()
      partExport:setUIntAttribute("positionSolverIterationCount", positionSolverIterationCount)
      local velocitySolverIterationCount = physicsEngineData:findAttribute("VelocitySolverIterationCount"):asInt()
      partExport:setUIntAttribute("velocitySolverIterationCount", velocitySolverIterationCount)
      local inertiaSphericalisation = physicsEngineData:findAttribute("InertiaSphericalisation"):asFloat()
      partExport:setDoubleAttribute("inertiaSphericalisation", inertiaSphericalisation)

      local nonArticulationSleepThreshold = physicsEngineData:findAttribute("NonArticulationSleepThreshold"):asFloat()
      partExport:setDoubleAttribute("sleepEnergyThreshold", nonArticulationSleepThreshold)

      local maxContactOffsetIncrease = physicsEngineData:findAttribute("MaxContactOffsetIncrease"):asFloat()
      partExport:setDoubleAttribute("maxContactOffsetIncrease", maxContactOffsetIncrease)

      if physicsBody:findAttribute("EnableMaxAngularVelocity"):asBool() then
        local maxAngularVelocity = physicsBody:findAttribute("MaxAngularVelocity"):asFloat()
        partExport:setDoubleAttribute("maxAngularVelocity", maxAngularVelocity)
      else
        partExport:setDoubleAttribute("maxAngularVelocity", -1)
      end
    else
      table.insert(
        warningsAndErrors.warnings,
        string.format("body '%s' has no PhysX3 attributes, using default values.\n", physicsBody:getName()))

      partExport:setUIntAttribute("positionSolverIterationCount", 4)
      partExport:setUIntAttribute("velocitySolverIterationCount", 2)
      partExport:setDoubleAttribute("inertiaSphericalisation", 0.5)
      partExport:setDoubleAttribute("sleepEnergyThreshold", 0.00005)
      partExport:setDoubleAttribute("maxContactOffsetIncrease", 0.0)
      partExport:setDoubleAttribute("maxAngularVelocity", -1)
    end
  end,

  ----------------------------------------------------------------------------------------------------------------------
  exportShape = function(physicsJoint, physicsVolume, physicsShape, shapeExport, warningsAndErrors)
    local physicsEngineData = physicsVolume:getFirstChild("PhysX3VolumeDataNode")

    if physicsEngineData then
      local dynamicFriction = physicsEngineData:findAttribute("DynamicFriction"):asFloat()
      shapeExport:setDoubleAttribute("dynamicFriction", dynamicFriction)

      local frictionCombineMode = physicsEngineData:findAttribute("FrictionCombineMode"):asString()
      shapeExport:setStringAttribute("frictionCombineMode", frictionCombineMode)

      local restitutionCombineMode = physicsEngineData:findAttribute("RestitutionCombineMode"):asString()
      shapeExport:setStringAttribute("restitutionCombineMode", restitutionCombineMode)

      local disableStrongFriction = physicsEngineData:findAttribute("DisableStrongFriction"):asBool()
      shapeExport:setBoolAttribute("disableStrongFriction", disableStrongFriction)

      local contactOffset = physicsEngineData:findAttribute("ContactOffset"):asFloat()
      shapeExport:setDoubleAttribute("contactOffset", contactOffset)

      local restOffset = physicsEngineData:findAttribute("RestOffset"):asFloat()
      shapeExport:setDoubleAttribute("restOffset", restOffset)
    else
      local db = physicsJoint:getDatabase()
      local status, physicsJointPath = physicsJoint:getPath(db:getRoot())

      table.insert(
        warningsAndErrors.warnings,
        string.format("PhysicsVolume '%s' owned by PhysicsJoint '%s' has no PhysX3 attributes, using default values.\n", physicsVolume:getName(), physicsJointPath))

      shapeExport:setDoubleAttribute("dynamicFriction", 1.0)
      shapeExport:setStringAttribute("frictionCombineMode", "Multiply")
      shapeExport:setStringAttribute("restitutionCombineMode", "Multiply")
      shapeExport:setBoolAttribute("disableStrongFriction", false)
      shapeExport:setDoubleAttribute("contactOffset", 0.01)
      shapeExport:setDoubleAttribute("restOffset", 0.0)
    end
  end,
  
 ----------------------------------------------------------------------------------------------------------------------
  supportsPerVolumeAuthoring = function()
      return true
  end,
  
  ----------------------------------------------------------------------------------------------------------------------
  exportJoint = function(physicsJointLimit, jointExport, warningsAndErrors)
    if physicsJointLimit:is("PhysicsTwistSwingNode") then
      local physicsEngineData = physicsJointLimit:getFirstChild("PhysX3TwistSwingDataNode")
      
      if physicsEngineData then
        jointExport:setBoolAttribute(
          "useSlerpDrive",
          physicsEngineData:findAttribute("NonArticulationDriveType"):asInt() == 0) -- kNonArticulationSLERPDrive

        -- this needs to be fixed up when the runtime supports non-articulation rigs
        --
        jointExport:setDoubleAttribute("slerpDriveSpring", physicsEngineData:findAttribute("NonArticulationSLERPDriveSpring"):asFloat())
        jointExport:setDoubleAttribute("slerpDriveDamping", physicsEngineData:findAttribute("NonArticulationSLERPDriveDamping"):asFloat())

        jointExport:setDoubleAttribute("swingDriveSpring", physicsEngineData:findAttribute("NonArticulationSwingDriveSpring"):asFloat())
        jointExport:setDoubleAttribute("swingDriveDamping", physicsEngineData:findAttribute("NonArticulationSwingDriveDamping"):asFloat())

        jointExport:setDoubleAttribute("twistDriveSpring", physicsEngineData:findAttribute("NonArticulationTwistDriveSpring"):asFloat())
        jointExport:setDoubleAttribute("twistDriveDamping", physicsEngineData:findAttribute("NonArticulationTwistDriveDamping"):asFloat())

        jointExport:setBoolAttribute("useAccelerationSprings", physicsEngineData:findAttribute("NonArticulationDriveSpringIsAcceleration"):asBool())
      else
        table.insert(
          warningsAndErrors.warnings,
          string.format("PhysicsJointLimit '%s' has no PhysX3 attributes, using default values.\n", physicsJointLimit:getName()))

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
      local physicsEngineData = physicsJointLimit:getFirstChild("PhysX3HingeDataNode")
      
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
          string.format("PhysicsJointLimit '%s' has no PhysX3 attributes, using default values.\n", physicsJointLimit:getName()))

        jointExport:setBoolAttribute("useSlerpDrive", true)

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
    local extraPhysX3Info = controllerData:getFirstChild("PhysX3CharacterControllerDataNode")
    target:setDoubleAttribute("StepHeight",extraPhysX3Info:findAttribute("StepHeight"):asFloat())
  end
}

if getPhysicsDriverExporter("PhysX3") then
  unregisterPhysicsDriverExporter("PhysX3")
end
registerPhysicsDriverExporter("PhysX3", exporterPhysX3)