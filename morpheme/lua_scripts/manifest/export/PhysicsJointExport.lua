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
require "manifest/export/PhysicsExport.lua"
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
local exportPhysicsSixDOFJoint = function(physicsTwistSwingLimit, jointExport, rotationOffset, warningsAndErrors)
  -- export Swing1 limit
  --
  local swing1Active = physicsTwistSwingLimit:findAttribute("Swing1Active"):asBool()
  jointExport:setStringAttribute("swing1Motion", swing1Active and "MOTION_LIMITED" or "MOTION_FREE")

  local swing1Angle = physicsTwistSwingLimit:findAttribute("Swing1Angle"):asDouble()
  
  if swing1Angle >= math.pi then
    table.insert(
      warningsAndErrors.warnings,
      string.format("PhysicsTwistSwing limit '%s' Swing1Angle must be less than 180.", physicsTwistSwingLimit:getName()))
  end

  jointExport:setDoubleAttribute("swing1Limit", swing1Angle)

  -- export Swing2 limit
  --
  local swing2Active = physicsTwistSwingLimit:findAttribute("Swing2Active"):asBool()
  jointExport:setStringAttribute("swing2Motion", swing2Active and "MOTION_LIMITED" or "MOTION_FREE")

  local swing2Angle = physicsTwistSwingLimit:findAttribute("Swing2Angle"):asDouble()

  if swing2Angle >= math.pi then
    table.insert(
      warningsAndErrors.warnings,
      string.format("PhysicsTwistSwing limit '%s' Swing2Angle must be less than 180.", physicsTwistSwingLimit:getName()))
  end

  jointExport:setDoubleAttribute("swing2Limit", swing2Angle)

  -- validate Swing1Angle and Swing2Angle won't cause instabilities
  --
  if swing1Active and swing2Active then
    local swingRatio = swing1Angle / swing2Angle

    if swingRatio < 0.1 or swingRatio > 10 then
      table.insert(
        warningsAndErrors.warnings,
        string.format("PhysicsTwistSwing limit '%s' Swing1Angle and Swing2Angle ratio is very large, this may cause instability", physicsTwistSwingLimit:getName()))
    end
  end

  -- export Twist limit
  --
  local twistActive = physicsTwistSwingLimit:findAttribute("TwistActive"):asBool()
  jointExport:setStringAttribute("twistMotion", twistActive and "MOTION_LIMITED" or "MOTION_FREE")

  -- as the frame has been adjusted the effective twist offset is 0 so set twist limit high and low accordingly
  --
  local halfTwistAngle = 0.5 * physicsTwistSwingLimit:findAttribute("TwistAngle"):asDouble()
  jointExport:setDoubleAttribute("twistLimitLow", -halfTwistAngle)
  jointExport:setDoubleAttribute("twistLimitHigh", halfTwistAngle)
end

------------------------------------------------------------------------------------------------------------------------
local exportPhysicsHingeJoint = function(physicsHingeLimit, jointExport, rotationOffset, warningsAndErrors)
  -- Set the swing limits up as quite small.
  --
  jointExport:setStringAttribute("swing1Motion", "MOTION_LIMITED")
  jointExport:setDoubleAttribute("swing1Limit", 0.1)

  jointExport:setStringAttribute("swing2Motion", "MOTION_LIMITED")
  jointExport:setDoubleAttribute("swing2Limit", 0.1)

  -- export Twist limit
  --
  local twistActive = physicsHingeLimit:findAttribute("TwistActive"):asBool()
  jointExport:setStringAttribute("twistMotion", twistActive and "MOTION_LIMITED" or "MOTION_FREE")

  -- as the frame has been adjusted the effective twist offset is 0 so set twist limit high and low accordingly
  --
  local halfTwistAngle = 0.5 * physicsHingeLimit:findAttribute("TwistAngle"):asDouble()
  jointExport:setDoubleAttribute("twistLimitLow", -halfTwistAngle)
  jointExport:setDoubleAttribute("twistLimitHigh", halfTwistAngle)
end

------------------------------------------------------------------------------------------------------------------------
local exportPhysicsRagdollJoint = function(physicsRagdollLimit, jointExport, warningsAndErrors)
  jointExport:setDoubleAttribute("coneAngle", physicsRagdollLimit:findAttribute("ConeAngle"):asDouble())
  jointExport:setDoubleAttribute("planeMinAngle", physicsRagdollLimit:findAttribute("PlaneMinAngle"):asDouble())
  jointExport:setDoubleAttribute("planeMaxAngle", physicsRagdollLimit:findAttribute("PlaneMaxAngle"):asDouble())

  -- as the frame has been adjusted the effective twist offset is 0 so set twist limit high and low accordingly
  --
  local halfTwistAngle = 0.5 * physicsRagdollLimit:findAttribute("TwistAngle"):asDouble()
  jointExport:setDoubleAttribute("twistMinAngle", -halfTwistAngle)
  jointExport:setDoubleAttribute("twistMaxAngle", halfTwistAngle)
end

------------------------------------------------------------------------------------------------------------------------
exportPhysicsJoint = function(rigExport, physicsJointLimit, parentPartIndex, childPartIndex, warningsAndErrors)
  local jointExport = nil

  -- create the joint export class
  --
  local jointName = physicsJointLimit:getName()
  if physicsJointLimit:is("PhysicsTwistSwingNode") then
    jointExport = rigExport:createJoint(jointName, "6DOF", parentPartIndex, childPartIndex)
  elseif physicsJointLimit:is("PhysicsHingeNode") then
    jointExport = rigExport:createJoint(jointName, "6DOF", parentPartIndex, childPartIndex)
  elseif physicsJointLimit:is("PhysicsRagdollLimitNode") then
    jointExport = rigExport:createJoint(jointName, "Ragdoll", parentPartIndex, childPartIndex)
  else
    table.insert(
      warningsAndErrors.errors,
      string.format("cannot export PhysicsJointLimit '%s' with unsupported type '%s'.", jointName, physicsJointLimit:getTypeString()))
    return nil
  end

  local physicsJointLimitInstances = nmx.Nodes.new()
  physicsJointLimit:getInstances(physicsJointLimitInstances)

  if physicsJointLimitInstances:empty() then
    table.insert(
      warningsAndErrors.errors,
      string.format("cannot export PhysicsJointLimit '%s' with no sgShapeNode instances.", jointName))
    return jointExport
  end

  if physicsJointLimitInstances:size() > 1 then
    table.insert(
      warningsAndErrors.warnings,
      string.format("PhysicsJointLimit '%s' has more than one sgShapeNode instance, only the first instance will be exported.", jointName))
  end

  -- build the local parent frame for the joint
  --
  local physicsJointLimitInstance = physicsJointLimitInstances:at(1)
  local parentTxInstance = physicsJointLimitInstance:getParent()

  local parentLocalMatrix = parentTxInstance:getLocalMatrix()
  jointExport:setMatrix34Attribute("localMatrix", nmx.matrixToTable(parentLocalMatrix))

  -- build the child frame for the joint
  --
  local rotationOffset = physicsJointLimit:getRotationOffset()
  local length = rotationOffset:normalise()

  if length > 1.05 or length < 0.95 then
    table.insert(
      warningsAndErrors.warnings,
      string.format("PhysicsJointLimit '%s' has an invalid rotation offset, the exported offset has been normalised.", jointName))
  end

  -- If the joint has a twist offset make sure the child frame is adjusted by the offset.
  -- The twist offset allows for nicer authoring of joint limits but often generates values
  -- outside the valid range of twist limits for physics engines.
  --
  local twistOffsetAttr = physicsJointLimit:findAttribute("TwistOffset")
  if twistOffsetAttr:isValid() then
    local twistOffset = twistOffsetAttr:asDouble()
    local twistOffsetQuat = nmx.Quat.new(nmx.Vector3.new(1.0, 0.0, 0.0), -twistOffset)
    rotationOffset:multiply(twistOffsetQuat)
  end

  local offsetMatrix = nmx.Matrix.new(rotationOffset)
  jointExport:setMatrix34Attribute("offsetMatrix", nmx.matrixToTable(offsetMatrix))

  if physicsJointLimit:is("PhysicsTwistSwingNode") then
    exportPhysicsSixDOFJoint(physicsJointLimit, jointExport, rotationOffset, warningsAndErrors)
  elseif physicsJointLimit:is("PhysicsHingeNode") then
    exportPhysicsHingeJoint(physicsJointLimit, jointExport, rotationOffset, warningsAndErrors)
  elseif physicsJointLimit:is("PhysicsRagdollLimitNode") then
    exportPhysicsRagdollJoint(physicsJointLimit, jointExport, warningsAndErrors)
  end

  -- export driver specific data
  --
  local physicsEngine = preferences.get("PhysicsEngine")
  local physicsDriverExporter = getPhysicsDriverExporter(physicsEngine)
  physicsDriverExporter.exportJoint(physicsJointLimit, jointExport, warningsAndErrors)

  local hasDriveScalingInfo = false
  if not mcn.isEuphoriaDisabled() then
    local euphoriaData = physicsJointLimit:getFirstChild("EuphoriaJointLimitDataNode")

    if euphoriaData then
      hasDriveScalingInfo = true
      jointExport:setDoubleAttribute("driveStiffnessScale", euphoriaData:findAttribute("DriveStiffnessScale"):asDouble())
      jointExport:setDoubleAttribute("driveDampingScale", euphoriaData:findAttribute("DriveDampingScale"):asDouble())
      jointExport:setDoubleAttribute("driveMinDampingScale", euphoriaData:findAttribute("DriveMinDampingScale"):asDouble())
      jointExport:setDoubleAttribute("driveCompensation", euphoriaData:findAttribute("DriveCompensation"):asDouble())
    else
      table.insert(
        warningsAndErrors.warnings,
        string.format("Limit '%s' is missing Euphoria specific data, using defaults.", jointName))
    end
  end

  if not hasDriveScalingInfo then
    jointExport:setDoubleAttribute("driveStiffnessScale", 1.0)
    jointExport:setDoubleAttribute("driveDampingScale", 1.0)
    jointExport:setDoubleAttribute("driveMinDampingScale", 1.0)
    jointExport:setDoubleAttribute("driveCompensation", 0.0)
  end
  
  return jointExport
end

------------------------------------------------------------------------------------------------------------------------
exportPhysicsSoftJointLimit = function(jointExport, physicsSoftJointLimit, warningsAndErrors)
  local physicsSoftJointLimitInstances = nmx.Nodes.new()
  physicsSoftJointLimit:getInstances(physicsSoftJointLimitInstances)

  if physicsSoftJointLimitInstances:empty() then
    table.insert(
      warningsAndErrors.errors,
      string.format("cannot export PhysicsSoftJointLimit '%s' with no sgShapeNode instances.", physicsSoftJointLimit:getName()))
    return jointExport
  end

  if physicsSoftJointLimitInstances:size() > 1 then
    table.insert(
      warningsAndErrors.warnings,
      string.format("PhysicsSoftJointLimit '%s' has more than one sgShapeNode instance, only the first instance will be exported.", physicsSoftJointLimit:getName()))
  end

  -- build the local parent frame for the joint
  --
  local physicsSoftJointLimitInstance = physicsSoftJointLimitInstances:at(1)
  local parentTxInstance = physicsSoftJointLimitInstance:getParent()

  local parentLocalMatrix = parentTxInstance:getLocalMatrix()
  jointExport:setMatrix34Attribute("softLimitLocalMatrix", nmx.matrixToTable(parentLocalMatrix))

  -- build the child frame for the joint
  --
  local rotationOffset = physicsSoftJointLimit:getRotationOffset()
  local length = rotationOffset:normalise()

  if length > 1.05 or length < 0.95 then
    table.insert(
      warningsAndErrors.warnings,
      string.format("PhysicsJointLimit '%s' has an invalid rotation offset, the exported offset has been normalised.", physicsJointLimit:getName()))
  end

  local offsetMatrix = nmx.Matrix.new(rotationOffset)
  jointExport:setMatrix34Attribute("softLimitOffsetMatrix", nmx.matrixToTable(offsetMatrix))

  local offsetQuat = physicsSoftJointLimit:findAttribute("RotationOffset"):asQuat()

  if physicsSoftJointLimit:is("PhysicsSoftTwistSwingNode") then
    if physicsSoftJointLimit:findAttribute("Swing1Active"):asBool() then
      jointExport:setBoolAttribute("useSoftSwing1", true)

      local swing1SoftLimit = physicsSoftJointLimit:findAttribute("Swing1Angle"):asDouble()
      if swing1SoftLimit >= math.pi then
        table.insert(warningsAndErrors.warnings, physicsSoftJointLimit:getName() .. " has an invalid swing 1 angle on soft limit. It should be less then 180.")
      end
      jointExport:setDoubleAttribute("swing1SoftLimit", swing1SoftLimit)
    else
      jointExport:setBoolAttribute("useSoftSwing1", false)
    end

    if physicsSoftJointLimit:findAttribute("Swing2Active"):asBool() then
      jointExport:setBoolAttribute("useSoftSwing2", true)

      local swing1Active = physicsSoftJointLimit:findAttribute("Swing1Active"):asBool()
      local swing2Angle = physicsSoftJointLimit:findAttribute("Swing2Angle"):asDouble()
      local swing1Angle = physicsSoftJointLimit:findAttribute("Swing1Angle"):asDouble()

      -- Validate that swing1Angle and swing2Angle do differ by more than a factor of 10
      if swing1Active and swing1Angle ~= nil and swing2Angle ~= nil then
        local swingRatio = swing1Angle / swing2Angle

        if swingRatio > 10.0 or swingRatio < 0.1 then
          table.insert(warningsAndErrors.warnings, physicsSoftJointLimit:getName() .. " swing1 and swing2 angle ratio is very large which may cause instability")
        end
      end

      local swing2SoftLimit = physicsSoftJointLimit:findAttribute("Swing2Angle"):asDouble()
      if swing2SoftLimit >= math.pi then
        table.insert(warningsAndErrors.warnings, physicsSoftJointLimit:getName() .. " has an invalid swing 2 angle on soft limit. It should be less then 280.")
      end
      jointExport:setDoubleAttribute("swing2SoftLimit", swing2SoftLimit)
    else
      jointExport:setBoolAttribute("useSoftSwing2", false)
    end

    if physicsSoftJointLimit:findAttribute("TwistActive"):asBool() then
      jointExport:setBoolAttribute("useSoftTwist", true)
      local softTwistAngle = physicsSoftJointLimit:findAttribute("TwistAngle"):asDouble()
      if softTwistAngle >= math.pi * 2 then
        table.insert(warningsAndErrors.warnings, physicsSoftJointLimit:getName() .. " has an invalid twist angle. It should be less than 360.")
      end

      -- here we adjust the rotation offset, removing the twist offset from the matrix, this allows us to have the full range of twist angles
      -- otherwise physx limitations dont allow twist low and high values of > or < (-)PI
      local offset = physicsSoftJointLimit:findAttribute("TwistOffset"):asDouble()
      local offsetAdjustQuat = nmx.Quat.new(nmx.Vector3.new(1.0, 0.0, 0.0), -offset)
      offsetQuat:multiply(offsetAdjustQuat)

      local halfAngle = 0.5 * softTwistAngle
      local lowerSoftLimit = -halfAngle
      local upperSoftLimit = halfAngle
      jointExport:setDoubleAttribute("twistSoftLimitLow", lowerSoftLimit)
      jointExport:setDoubleAttribute("twistSoftLimitHigh", upperSoftLimit)
    else
      jointExport:setBoolAttribute("useSoftTwist", false)
    end
  end

  jointExport:setMatrix34Attribute("softLimitOffsetMatrix", nmx.matrixToTable(nmx.Matrix.new(offsetQuat)))
  jointExport:setDoubleAttribute("softLimitStrength", physicsSoftJointLimit:findAttribute("Strength"):asDouble())
end