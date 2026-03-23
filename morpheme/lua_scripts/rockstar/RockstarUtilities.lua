------------------------------------------------------------------------------------------------------------------------
-- some utility functions for manipulating nmx objects
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
--| namespace: rockstar
--| brief: Contains all the functions required for rockstar rig authoring.
------------------------------------------------------------------------------------------------------------------------
rockstar = rockstar or {}

------------------------------------------------------------------------------------------------------------------------
--| signature: number copysign(number value, number sign)
--| brief: Equivalent to "sign * abs(value)"
------------------------------------------------------------------------------------------------------------------------
local copysign = function(value, sign)
  if sign > 0 then
    if value > 0 then
      return value
    else
      return -value
    end
  else
    if value > 0 then
      return -value
    else
      return value
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
--| signature: number safeacos(number value)
------------------------------------------------------------------------------------------------------------------------
local safeacos = function(value)
  if value > -1.0 and value < 1.0 then
    return math.acos(value)
  elseif value >= 1.0 then
    return 0.0
  else
    return 3.14159
  end
end

------------------------------------------------------------------------------------------------------------------------
-- brief: list of all effector names used by euphoria
--  the order the effectors are specified here is the order they will be exported, this order should not be changed
--  unless the rockstar euphoria loading code is changed as well.
------------------------------------------------------------------------------------------------------------------------
rockstar.euphoriaEffectorNames = {
  "hip_left",
  "knee_left",
  "ankle_left",
  "hip_right",
  "knee_right",
  "ankle_right",
  "spine_0",
  "spine_1",
  "spine_2",
  "spine_3",
  "clavicle_jnt_left",
  "shoulder_left",
  "elbow_left",
  "wrist_left",
  "clavicle_jnt_right",
  "shoulder_right",
  "elbow_right",
  "wrist_right",
  "neck_lower",
  "neck_upper",
}

------------------------------------------------------------------------------------------------------------------------
--| signature: string rockstar.getRockstarName(sgTransformNode nodeTxInstance)
------------------------------------------------------------------------------------------------------------------------
rockstar.getRockstarName = function(nodeTxInstance)
  local nodeName = nodeTxInstance:getName()

  if nodeTxInstance:getChildDataNode(nmx.PhysicsJointLimitNode.ClassTypeId()) then
    return string.gsub(nodeName, "Limit", "PhysicsJointLimit")
  elseif nmx.PhysicsBodyNode.getPhysicsBody(nodeTxInstance) then
    return string.gsub(nodeName, "Body", "PhysicsBody")
  end

  return nodeName
end

------------------------------------------------------------------------------------------------------------------------
--| signature: boolean rockstar.physicsJointIsLeaf(
--|   sgTransformNode  physicsJointTxInstance,
--|   PhysicsJointNode physicsJoint)
------------------------------------------------------------------------------------------------------------------------
rockstar.physicsJointIsLeaf = function(physicsJointTxInstance, physicsJoint)
  -- assert all arguments are valid
  --
  assert(type(physicsJointTxInstance) == "userdata" and physicsJointTxInstance:is(nmx.sgTransformNode.ClassTypeId()), "bad argument #1 to 'rockstar.hasPhysicsJointChildren'")
  assert(type(physicsJoint) == "userdata" and physicsJoint:isDerivedFrom(nmx.PhysicsJointNode.ClassTypeId()), "bad argument #2 to 'rockstar.hasPhysicsJointChildren'")

  local childTxInstance = physicsJointTxInstance:getFirstChild()
  while childTxInstance do
    if childTxInstance:is("sgTransformNode") then
      local childPhysicsJoint = nmx.PhysicsJointNode.getPhysicsJoint(childTxInstance)

      if childPhysicsJoint then
        return false
      end
    end
    childTxInstance = childTxInstance:getNextSibling()
  end
  
  return true
end

------------------------------------------------------------------------------------------------------------------------
--| signature: boolean rockstar.physicsJointHasPhysicsVolume(
--|   sgTransformNode  physicsJointTxInstance,
--|   PhysicsJointNode physicsJoint)
------------------------------------------------------------------------------------------------------------------------
rockstar.physicsJointHasPhysicsVolume = function(physicsJointTxInstance, physicsJoint)
  -- assert all arguments are valid
  --
  assert(type(physicsJointTxInstance) == "userdata" and physicsJointTxInstance:is(nmx.sgTransformNode.ClassTypeId()), "bad argument #1 to 'rockstar.hasPhysicsJointChildren'")
  assert(type(physicsJoint) == "userdata" and physicsJoint:isDerivedFrom(nmx.PhysicsJointNode.ClassTypeId()), "bad argument #2 to 'rockstar.hasPhysicsJointChildren'")

  local physicsBodyTxInstance, physicsBody = rockstar.getPhysicsBody(physicsJointTxInstance, physicsJoint)
  if physicsBodyTxInstance and physicsBody then
    local physicsVolumeTxInstance, physicsVolume = rockstar.getPhysicsVolume(physicsBodyTxInstance, physicsBody)
    if physicsVolumeTxInstance and physicsVolume then
      return true
    end
  end

  return false
end

------------------------------------------------------------------------------------------------------------------------
--| signature: sgTransformNode, PhysicsJointLimitNode rockstar.getPhysicsJointLimit(
--|   sgTransformNode  physicsJointTxInstance,
--|   PhysicsJointNode physicsJoint)
--|
--| brief: Returns the PhysicsJointLimit sgTransformNode and PhysicsJointLimit for the given PhysicsJointNode.
------------------------------------------------------------------------------------------------------------------------
rockstar.getPhysicsJointLimit = function(physicsJointTxInstance, physicsJoint)
  -- assert all arguments are valid
  --
  assert(type(physicsJointTxInstance) == "userdata" and physicsJointTxInstance:is(nmx.sgTransformNode.ClassTypeId()), "bad argument #1 to 'rockstar.getPhysicsJointLimit'")
  assert(type(physicsJoint) == "userdata" and physicsJoint:isDerivedFrom(nmx.PhysicsJointNode.ClassTypeId()), "bad argument #2 to 'rockstar.getPhysicsJointLimit'")
  
  -- loop through all the PhysicsJointNodes children looking for the PhysicsJointLimitNode
  --
  local childTxInstance = physicsJointTxInstance:getFirstChild()
  while childTxInstance do
    if childTxInstance:is("sgTransformNode") then
      local childPhysicsJointLimitNode = childTxInstance:getChildDataNode(nmx.PhysicsJointLimitNode.ClassTypeId())

      -- check we found a PhysicsJointLimitNode
      --
      if childPhysicsJointLimitNode then
        -- check the PhysicsJointLimitNode we found is the one connected to the PhysicsJointNode
        --
        local expectedPhysicsJointLimit = physicsJoint:getPhysicsLimit(false)
        if childPhysicsJointLimitNode:is(expectedPhysicsJointLimit) then
          return childTxInstance, childPhysicsJointLimitNode
        end
      end
    end
    childTxInstance = childTxInstance:getNextSibling()
  end

  return nil, nil
end

------------------------------------------------------------------------------------------------------------------------
--| signature: sgTransformNode, PhysicsBodyNode rockstar.getPhysicsBody(
--|   sgTransformNode  physicsJointTxInstance,
--|   PhysicsJointNode physicsJoint)
--|
--| brief: Returns the PhysicsBody sgTransformNode and PhysicsBody for the given PhysicsJointNode.
------------------------------------------------------------------------------------------------------------------------
rockstar.getPhysicsBody = function(physicsJointTxInstance, physicsJoint)
  -- assert all arguments are valid
  --
  assert(type(physicsJointTxInstance) == "userdata" and physicsJointTxInstance:is(nmx.sgTransformNode.ClassTypeId()), "bad argument #1 to 'rockstar.getPhysicsJointLimit'")
  assert(type(physicsJoint) == "userdata" and physicsJoint:isDerivedFrom(nmx.PhysicsJointNode.ClassTypeId()), "bad argument #2 to 'rockstar.getPhysicsJointLimit'")

  local physicsBodyTxInstance = physicsJointTxInstance:getFirstChild()
  while physicsBodyTxInstance do
    if physicsBodyTxInstance:is("sgTransformNode") then
      local physicsBody = nmx.PhysicsBodyNode.getPhysicsBody(physicsBodyTxInstance)

      if physicsBody then        
        return physicsBodyTxInstance, physicsBody
      end
    end
    physicsBodyTxInstance = physicsBodyTxInstance:getNextSibling()
  end

  return nil, nil
end

------------------------------------------------------------------------------------------------------------------------
--| signature: sgTransformNode, PhysicsVolumeNode rockstar.getPhysicsVolume(
--|   sgTransformNode physicsBodyTxInstance,
--|   PhysicsBodyNode physicsBody)
--|
--| brief: Returns the PhysicsBody sgTransformNode and PhysicsBody for the given PhysicsBodyNode.
------------------------------------------------------------------------------------------------------------------------
rockstar.getPhysicsVolume = function(physicsBodyTxInstance, physicsBody)
  -- assert all arguments are valid
  --
  assert(type(physicsBodyTxInstance) == "userdata" and physicsBodyTxInstance:is(nmx.sgTransformNode.ClassTypeId()), "bad argument #1 to 'rockstar.getPhysicsVolume'")
  assert(type(physicsBody) == "userdata" and physicsBody:isDerivedFrom(nmx.PhysicsBodyNode.ClassTypeId()), "bad argument #2 to 'rockstar.getPhysicsVolume'")

  local physicsVolumeTxInstance = physicsBodyTxInstance:getFirstChild()
  while physicsVolumeTxInstance do
    if physicsVolumeTxInstance:is("sgTransformNode") then
      local physicsVolume = nmx.PhysicsVolumeNode.getPhysicsVolume(physicsVolumeTxInstance)

      if physicsVolume then        
        return physicsVolumeTxInstance, physicsVolume
      end
    end
    physicsVolumeTxInstance = physicsVolumeTxInstance:getNextSibling()
  end

  return nil, nil
end

------------------------------------------------------------------------------------------------------------------------
--| signature: CustomPhysicsEngineDataNode rockstar.getRockstarPhysicsEngineData(Node physicsNode)
--|
--| brief: Returns the associated rockstar physics engine data for a node if there is any.
------------------------------------------------------------------------------------------------------------------------
rockstar.getRockstarPhysicsEngineData = function(physicsNode)
  -- assert all arguments are valid
  --
  assert(type(physicsNode) == "userdata" and physicsNode:isDerivedFrom(nmx.Node.ClassTypeId()), "bad argument #1 to 'rockstar.getRockstarPhysicsEngineData'")

  local physicsEngineData = physicsNode:getFirstChild("CustomPhysicsEngineDataNode")
  while physicsEngineData do
    if physicsEngineData:is("CustomPhysicsEngineDataNode") and
       physicsEngineData:getPhysicsEngineName() == "Rockstar" then
      return physicsEngineData
    end

    physicsEngineData = physicsEngineData:getNextSibling()
  end

  return nil
end

------------------------------------------------------------------------------------------------------------------------
--| signature: Matrix rockstar.calculateJointLimitWorldMovingFrame(
--|   sgTransformNode       physicsJointLimitTxInstance,
--|   PhysicsJointLimitNode physicsJointLimit)
------------------------------------------------------------------------------------------------------------------------
rockstar.calculateJointLimitWorldMovingFrame = function(physicsJointLimitTxInstance, physicsJointLimit)
  local rotationOffsetQuat = physicsJointLimit:getRotationOffset()
  local worldMovingFrame = nmx.Matrix.new(rotationOffsetQuat)

  local jointLocalFrame = physicsJointLimit:getJointLocalMatrix() 
  worldMovingFrame:multiply(jointLocalFrame)

  local parentWorldFrame = physicsJointLimit:getParentWorldMatrix()
  worldMovingFrame:multiply(parentWorldFrame)
  
  return worldMovingFrame
end

------------------------------------------------------------------------------------------------------------------------
--| signature: nil rockstar.writeTwistSwingLimitAngleData(
--|   table                 exportWriter,
--|   integer               indent,
--|   sgTransformNode       physicsJointLimitTxInstance,
--|   PhysicsJointLimitNode physicsJointLimit)
------------------------------------------------------------------------------------------------------------------------
rockstar.writeTwistSwingLimitAngleData = function(exportWriter, indent, physicsJointLimitTxInstance, physicsJointLimit)
  assert(physicsJointLimit:is(nmx.PhysicsTwistSwingNode.ClassTypeId()))
  
  -- get the limit frame and limit moving frame in world space
  --
  local worldFrame = physicsJointLimitTxInstance:getWorldMatrix()
  local worldMovingFrame = rockstar.calculateJointLimitWorldMovingFrame(physicsJointLimitTxInstance, physicsJointLimit)

  local worldMovingFrameXAxis = worldMovingFrame:xAxis()
  local worldMovingFrameYAxis = worldMovingFrame:yAxis()
  local worldMovingFrameZAxis = worldMovingFrame:zAxis()

  local worldFrameYAxis = worldFrame:yAxis()
  local worldFrameZAxis = worldFrame:zAxis()

  -- work out the lean offsets so the joint limits are asymmetric around the main axis
  --
	-- copysign appears to be equivalent to sign * abs(value)
	--
  local lean2Offset = copysign(1, worldMovingFrameXAxis:dot(worldFrameYAxis)) * safeacos(worldMovingFrameYAxis:dot(worldFrameYAxis))

  local rotatedWorldFrame = nmx.Matrix.new(worldFrame)
  local rotation = nmx.Quat.new()
  rotation:fromAxisAngle(worldFrameZAxis, lean2Offset)
  rotatedWorldFrame:multiply(nmx.Matrix.new(rotation))

  local rotatedWorldFrameXAxis = rotatedWorldFrame:xAxis()
	
  local lean1Offset = copysign(1, worldMovingFrameZAxis:dot(rotatedWorldFrameXAxis)) * safeacos(worldMovingFrameXAxis:dot(rotatedWorldFrameXAxis))

	local reverseSwing1 = physicsJointLimit:findAttribute("ReverseFirstLeanMotor"):asBool()
	if(reverseSwing1) then lean1Offset = lean1Offset * -1 end
  local swing1Angle = physicsJointLimit:findAttribute("Swing1Angle"):asFloat()
  local minfirstleanangle = lean1Offset - swing1Angle
  exportWriter:write(indent, "<minfirstleanangle>", minfirstleanangle, "</minfirstleanangle>")
  local maxfirstleanangle = lean1Offset + swing1Angle
  exportWriter:write(indent, "<maxfirstleanangle>", maxfirstleanangle, "</maxfirstleanangle>")

	local reverseSwing2 = physicsJointLimit:findAttribute("ReverseSecondLeanMotor"):asBool()
	if(reverseSwing2) then lean2Offset = lean2Offset * -1 end
  local swing2Angle = physicsJointLimit:findAttribute("Swing2Angle"):asFloat()
  local minsecondleanangle = -swing2Angle + lean2Offset
  exportWriter:write(indent, "<minsecondleanangle>", minsecondleanangle, "</minsecondleanangle>")
  local maxsecondleanangle = swing2Angle + lean2Offset
  exportWriter:write(indent, "<maxsecondleanangle>", maxsecondleanangle, "</maxsecondleanangle>")

  -- adjust twist to export in terms of min and max angles
  --
  local twistAngle = physicsJointLimit:findAttribute("TwistAngle"):asFloat()
  local twistOffset = physicsJointLimit:findAttribute("TwistOffset"):asFloat()
	local reverseTwist = physicsJointLimit:findAttribute("ReverseTwistMotor");
	if(reverseTwist) then twistOffset = twistOffset * -1 end
  local mintwistangle = -0.5 * twistAngle - twistOffset
  exportWriter:write(indent, "<mintwistangle>", mintwistangle, "</mintwistangle>")
  local maxtwistangle = 0.5 * twistAngle - twistOffset
  exportWriter:write(indent, "<maxtwistangle>", maxtwistangle, "</maxtwistangle>")
end

------------------------------------------------------------------------------------------------------------------------
--| signature: nil rockstar.writeHingeLimitAngleData(
--|   table                 exportWriter,
--|   integer               indent,
--|   sgTransformNode       physicsJointLimitTxInstance,
--|   PhysicsJointLimitNode physicsJointLimit)
------------------------------------------------------------------------------------------------------------------------
rockstar.writeHingeLimitAngleData = function(exportWriter, indent, physicsJointLimitTxInstance, physicsJointLimit)
  --assert(physicsJointLimit:is(nmx.PhysicsHingeNode.ClassTypeId()))
  --1dof joints changed to 3dof joints with swing1 and swing2 switched off
  --  The hinge joint in morpheme has different axes(?) and cannot be edited into the correct configuration
  assert((physicsJointLimit:is(nmx.PhysicsTwistSwingNode.ClassTypeId()) and not (physicsJointLimit:findAttribute("Swing1Active"):asBool() or physicsJointLimit:findAttribute("Swing2Active"):asBool())))
  -- adjust twist to export in terms of min and max angles
  --
  local twistAngle = physicsJointLimit:findAttribute("TwistAngle"):asFloat()
  local twistOffset = physicsJointLimit:findAttribute("TwistOffset"):asFloat()

  local mintwistangle = -0.5 * twistAngle + twistOffset
  exportWriter:write(indent, "<minangle>", mintwistangle, "</minangle>")
  local maxtwistangle = 0.5 * twistAngle + twistOffset
  exportWriter:write(indent, "<maxangle>", maxtwistangle, "</maxangle>")
end

------------------------------------------------------------------------------------------------------------------------
--| signature: nil writeCustomRockstarProperties(table exportWriter, integer indent, CustomPhysicsEngineDataNode rockstarPhysicsData)
------------------------------------------------------------------------------------------------------------------------
rockstar.writeCustomRockstarProperties = function(exportWriter, indent, rockstarPhysicsData)
  -- find the index of the last non-dynamic attribute
  --
  local physicsEngineNameAttr = rockstarPhysicsData:findAttribute("PhysicsEngineName")
  local lastElementIndex = physicsEngineNameAttr:elementIndex()
  local excludes = {
    ["EffectorName"] = true,
  }

  -- every attribute after that index will be a custom one and should be written out
  --
  for i = lastElementIndex + 1, rockstarPhysicsData:getNumElements() - 1 do
    local attribute = rockstarPhysicsData:getAttribute(i)
    local attributeName = attribute:getName()
    local attributeTypeId = attribute:getTypeId()

    if not excludes[attributeName] then
      -- get the value
      --
      local value = nil
      if attributeTypeId == nmx.AttributeTypeId.kInt then
        value = attribute:asInt()
      elseif attributeTypeId == nmx.AttributeTypeId.kFloat then
        value = attribute:asFloat()
      elseif attributeTypeId == nmx.AttributeTypeId.kBool then
        value = attribute:asBool()
        value = tostring(value)
      end

      -- write the property out
      --
      local tagName = string.lower(attributeName)
      exportWriter:write(indent, "<", tagName, ">", value, "</", tagName, ">")
    end
  end
end