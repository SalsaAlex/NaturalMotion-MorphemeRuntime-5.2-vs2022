------------------------------------------------------------------------------------------------------------------------
-- Functions for writing out rockstar ragdoll data.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- include other required scripts
------------------------------------------------------------------------------------------------------------------------
require "rockstar/RockstarUtilities.lua"

------------------------------------------------------------------------------------------------------------------------
--| signature: number copysign(number value, number sign)
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
--| signature: nil writeNodePath(table exportWriter, integer indent, Node node)
------------------------------------------------------------------------------------------------------------------------
local writeNodePath = function(exportWriter, indent, node)
  -- assert all arguments are valid
  --
  assert(type(exportWriter) == "table", "bad argument #1 to 'writeNodePath'")
  assert(type(indent) == "number", "bad argument #2 to 'writeNodePath'")
  assert(type(node) == "userdata", "bad argument #3 to 'writeNodePath'")
  
  -- get the path from the root node to this node
  --
  local database = node:getDatabase()
  local rootNode = database:getRoot()
  local status, nodePath = node:getPath(rootNode)

  -- strip the first three nodes off of the node path
  -- the path should start at the root of the physics rig no the root of the scene
  --
  local position = string.find(nodePath, "|")
  position = string.find(nodePath, "|", position + 1)
  position = string.find(nodePath, "|", position + 1)
  nodePath = string.sub(nodePath, position)

  -- write out the node path
  --
  exportWriter:write(indent, "<path>", nodePath, "</path>")
end

------------------------------------------------------------------------------------------------------------------------
--| signature: nil writeNativeProperty(table exportWriter, integer indent, string propertyName, string property)
--| signature: nil writeNativeProperty(table exportWriter, integer indent, string propertyName, number property)
--| signature: nil writeNativeProperty(table exportWriter, integer indent, string propertyName, boolean property)
------------------------------------------------------------------------------------------------------------------------
local writeNativeProperty = function(exportWriter, indent, propertyName, property)
  -- assert all arguments are valid
  --
  assert(type(exportWriter) == "table", "bad argument #1 to 'writeNativeProperty'")
  assert(type(indent) == "number", "bad argument #2 to 'writeNativeProperty'")
  assert(type(propertyName) == "string", "bad argument #3 to 'writeNativeProperty'")
  assert(type(property) == "number" or type(property) == "string" or type(property) == "boolean", "bad argument #4 to 'writeNativeProperty'")

  if type(property) == "boolean" then
    local value = property and "True" or "False"
    exportWriter:write(indent, "<", propertyName, ">", value, "</", propertyName, ">")
  else
    exportWriter:write(indent, "<", propertyName, ">", property, "</", propertyName, ">")
  end
end

------------------------------------------------------------------------------------------------------------------------
--| signature: nil writeVectorProperty(table exportWriter, integer indent, string propertyName, Vector3 property)
------------------------------------------------------------------------------------------------------------------------
local writeVectorProperty = function(exportWriter, indent, propertyName, property)
  -- assert all arguments are valid
  --
  assert(type(exportWriter) == "table", "bad argument #1 to 'writeVectorProperty'")
  assert(type(indent) == "number", "bad argument #2 to 'writeVectorProperty'")
  assert(type(propertyName) == "string", "bad argument #3 to 'writeVectorProperty'")
  assert(type(property) == "userdata", "bad argument #4 to 'writeVectorProperty'")

  -- begin the vector
  --
  exportWriter:write(
    indent,
    "<", propertyName, ".X>", property:getX(), "</", propertyName, ".X>  <",
         propertyName, ".Y>", property:getY(), "</", propertyName, ".Y>  <",
         propertyName, ".Z>", property:getZ(), "</", propertyName, ".Z>"
  )

end

------------------------------------------------------------------------------------------------------------------------
--| signature: nil writeTransformProperty(table exportWriter, integer indent, string propertyName, Matrix property)
------------------------------------------------------------------------------------------------------------------------
local writeTransformProperty = function(exportWriter, indent, propertyName, property)
  -- assert all arguments are valid
  --
  assert(type(exportWriter) == "table", "bad argument #1 to 'writeTransformProperty'")
  assert(type(indent) == "number", "bad argument #2 to 'writeTransformProperty'")
  assert(type(propertyName) == "string", "bad argument #3 to 'writeTransformProperty'")
  assert(type(property) == "userdata", "bad argument #4 to 'writeTransformProperty'")

  -- begin the transform
  --
  exportWriter:write(indent, "<", propertyName, ">")
  indent = indent + 1

  local xAxis = property:xAxis()
  exportWriter:write(indent, "<xAxis_x>", xAxis:getX(), "</xAxis_x>  <xAxis_y>", xAxis:getY(), "</xAxis_y>  <xAxis_z>", xAxis:getZ(), "</xAxis_z>")

  local yAxis = property:yAxis()
  exportWriter:write(indent, "<yAxis_x>", yAxis:getX(), "</yAxis_x>  <yAxis_y>", yAxis:getY(), "</yAxis_y>  <yAxis_z>", yAxis:getZ(), "</yAxis_z>")

  local zAxis = property:zAxis()
  exportWriter:write(indent, "<zAxis_x>", zAxis:getX(), "</zAxis_x>  <zAxis_y>", zAxis:getY(), "</zAxis_y>  <zAxis_z>", zAxis:getZ(), "</zAxis_z>")

  local translation = property:translation()
  exportWriter:write(indent, "<trans_x>", translation:getX(), "</trans_x>  <trans_y>", translation:getY(), "</trans_y>  <trans_z>", translation:getZ(), "</trans_z>")

  -- end the transform
  --
  indent = indent - 1
  exportWriter:write(indent, "</", propertyName, ">")
end

------------------------------------------------------------------------------------------------------------------------
--| signature: nil rockstar.writeRagdollPhysicsJoint(
--|   table            exportWriter,
--|   integer          indent,
--|   sgTransformNode  physicsJointTxInstance,
--|   PhysicsJointNode physicsJoint)
------------------------------------------------------------------------------------------------------------------------
local writeRagdollPhysicsJoint = function(exportWriter, indent, physicsJointTxInstance, physicsJoint)
  -- assert all arguments are valid
  --
  assert(type(exportWriter) == "table", "bad argument #1 to 'rockstar.writeRagdollPhysicsJoint'")
  assert(type(indent) == "number", "bad argument #2 to 'rockstar.writeRagdollPhysicsJoint'")
  assert(type(physicsJointTxInstance) == "userdata" and physicsJointTxInstance:is(nmx.sgTransformNode.ClassTypeId()), "bad argument #3 to 'rockstar.writeRagdollPhysicsJoint'")
  assert(type(physicsJoint) == "userdata" and physicsJoint:isDerivedFrom(nmx.PhysicsJointNode.ClassTypeId()), "bad argument #4 to 'rockstar.writeRagdollPhysicsJoint'")

  -- begin PhysicsJoint tag
  --
  exportWriter:write(indent, "<PhysicsJoint>")
  indent = indent + 1

  writeNativeProperty(exportWriter, indent, "name", rockstar.getRockstarName(physicsJointTxInstance))
  writeNodePath(exportWriter, indent, physicsJointTxInstance)
  writeTransformProperty(exportWriter, indent, "localFrame", physicsJointTxInstance:getLocalMatrix())
  writeTransformProperty(exportWriter, indent, "worldFrame", physicsJointTxInstance:getWorldMatrix())

  -- begin PhysicsJoint tag
  --
  indent = indent - 1
  exportWriter:write(indent, "</PhysicsJoint>")
end

-- this rotation required to orient capsules down the y-axis instead of the z-axis
--
local capsuleOrientationTransform = nmx.Matrix.new()
capsuleOrientationTransform:set3x3ToXRotation(0.5 * math.pi)

------------------------------------------------------------------------------------------------------------------------
--| signature: nil rockstar.writeRagdollPhysicsBody(
--|   table            exportWriter,
--|   integer          indent,
--|   sgTransformNode  physicsBodyTxInstance,
--|   PhysicsBodyNode  physicsBody)
------------------------------------------------------------------------------------------------------------------------
local writeRagdollPhysicsBody = function(exportWriter, indent, physicsBodyTxInstance, physicsBody)
  -- assert all arguments are valid
  --
  assert(type(exportWriter) == "table", "bad argument #1 to 'rockstar.writeRagdollPhysicsBody'")
  assert(type(indent) == "number", "bad argument #2 to 'rockstar.writeRagdollPhysicsBody'")
  assert(type(physicsBodyTxInstance) == "userdata" and physicsBodyTxInstance:is(nmx.sgTransformNode.ClassTypeId()), "bad argument #3 to 'rockstar.writeRagdollPhysicsBody'")
  assert(type(physicsBody) == "userdata" and physicsBody:isDerivedFrom(nmx.PhysicsBodyNode.ClassTypeId()), "bad argument #4 to 'rockstar.writeRagdollPhysicsBody'")

  -- begin PhysicsBody tag
  --
  exportWriter:write(indent, "<PhysicsBody>")
  indent = indent + 1

  writeNativeProperty(exportWriter, indent, "name", rockstar.getRockstarName(physicsBodyTxInstance))
  writeNodePath(exportWriter, indent, physicsBodyTxInstance)

  local physicsVolumeTxInstance, physicsVolume = rockstar.getPhysicsVolume(physicsBodyTxInstance, physicsBody)
  assert(physicsVolumeTxInstance and physicsVolume, string.format("PhysicsBodyNode '%s' has no associated PhysicsVolumeNode", physicsBodyTxInstance:getName()))
  local primitive = physicsVolume:getShape()

  local localFrame = physicsVolumeTxInstance:getLocalMatrix()
  local worldFrame = physicsVolumeTxInstance:getWorldMatrix()

  if primitive:is(nmx.CapsuleNode.ClassTypeId()) then
    -- write capsule specific properties
    --
    writeNativeProperty(exportWriter, indent, "shape", "Capsule")
    writeNativeProperty(exportWriter, indent, "length", primitive:getHeight())
    writeNativeProperty(exportWriter, indent, "radius", primitive:getRadius())

    -- make sure capsules are oriented down the correct axis
    --
    localFrame:multiply(capsuleOrientationTransform)
    worldFrame:multiply(capsuleOrientationTransform)
  elseif primitive:is(nmx.BoxNode.ClassTypeId()) then
    -- write box specific properties
    --
    writeNativeProperty(exportWriter, indent, "shape", "Box")
    writeNativeProperty(exportWriter, indent, "height", primitive:getHeight())
    writeNativeProperty(exportWriter, indent, "length", primitive:getDepth())
    writeNativeProperty(exportWriter, indent, "width", primitive:getWidth())
  else
    assert(false, string.format("Unsupported primitive type '%s'", primitive:getTypeString()))
  end

  local rockstarPhysicsData = rockstar.getRockstarPhysicsEngineData(physicsVolume)
  --Mass
  if rockstarPhysicsData then
    rockstar.writeCustomRockstarProperties(exportWriter, indent, rockstarPhysicsData)
  end

  writeTransformProperty(exportWriter, indent, "localFrame", physicsVolumeTxInstance:getLocalMatrix())
  writeTransformProperty(exportWriter, indent, "worldFrame", physicsVolumeTxInstance:getWorldMatrix())

  -- end PhysicsBody tag
  --
  indent = indent - 1
  exportWriter:write(indent, "</PhysicsBody>")
end

------------------------------------------------------------------------------------------------------------------------
--| signature: nil writeRagdollPhysicsJointLimit(
--|   table                 exportWriter,
--|   integer               indent,
--|   sgTransformNode       physicsJointLimitTxInstance,
--|   PhysicsJointLimitNode physicsJointLimit)
------------------------------------------------------------------------------------------------------------------------
local writeRagdollPhysicsJointLimit = function(exportWriter, indent, physicsJointLimitTxInstance, physicsJointLimit)
  -- assert all arguments are valid
  --
  assert(type(exportWriter) == "table", "bad argument #1 to 'rockstar.writeRagdollPhysicsJointLimit'")
  assert(type(indent) == "number", "bad argument #2 to 'rockstar.writeRagdollPhysicsJointLimit'")
  assert(type(physicsJointLimitTxInstance) == "userdata" and physicsJointLimitTxInstance:is(nmx.sgTransformNode.ClassTypeId()), "bad argument #3 to 'rockstar.writeRagdollPhysicsJointLimit'")
  assert(type(physicsJointLimit) == "userdata" and physicsJointLimit:isDerivedFrom(nmx.PhysicsJointLimitNode.ClassTypeId()), "bad argument #4 to 'rockstar.writeRagdollPhysicsJointLimit'")

  -- begin PhysicsJointLimit tag
  --
  exportWriter:write(indent, "<PhysicsJointLimit>")
  indent = indent + 1

  writeNativeProperty(exportWriter, indent, "name", rockstar.getRockstarName(physicsJointLimitTxInstance))
  writeNodePath(exportWriter, indent, physicsJointLimitTxInstance)

  --if physicsJointLimit:is(nmx.PhysicsHingeNode.ClassTypeId()) then
  --1dof joints changed to 3dof joints with swing1 and swing2 switched off
  --  The hinge joint in morpheme has different axes(?) and cannot be edited into the correct configuration
  if (physicsJointLimit:is(nmx.PhysicsTwistSwingNode.ClassTypeId()) and not (physicsJointLimit:findAttribute("Swing1Active"):asBool() or physicsJointLimit:findAttribute("Swing2Active"):asBool())) then
    -- export PhysicsHingeNode properties
    --
    writeNativeProperty(exportWriter, indent, "type", "Hinge")
    --not needed for r* human input
    local rockstarPhysicsData = rockstar.getRockstarPhysicsEngineData(physicsJointLimit)
    writeNativeProperty(exportWriter, indent, "twistAngle", physicsJointLimit:findAttribute("TwistAngle"):asFloat())
    writeNativeProperty(exportWriter, indent, "twistOffset", physicsJointLimit:findAttribute("TwistOffset"):asFloat())
    --output later as worldFixedFrame trans
    local worldFixedFrameTrans = physicsJointLimitTxInstance:getWorldMatrix():translation()
    writeVectorProperty(exportWriter, indent, "axisposition", worldFixedFrameTrans)
    --output later as worldFixedFrame xAxis
    local worldFixedFrameXAxis = physicsJointLimitTxInstance:getWorldMatrix():xAxis()
    writeVectorProperty(exportWriter, indent, "axisdirection", worldFixedFrameXAxis)
    writeNativeProperty(exportWriter, indent, "limitenabled", rockstarPhysicsData:findAttribute("LimitEnabled"):asBool())
    
    rockstar.writeHingeLimitAngleData(exportWriter, indent, physicsJointLimitTxInstance, physicsJointLimit)
    
    --not needed for r* human input
    writeNativeProperty(exportWriter, indent, "createeffector", rockstarPhysicsData:findAttribute("CreateEffector"):asBool())
    writeNativeProperty(exportWriter, indent, "defaultleanforcecap", rockstarPhysicsData:findAttribute("DefaultLeanForceCap"):asFloat())
    writeNativeProperty(exportWriter, indent, "defaultmusclestiffness", rockstarPhysicsData:findAttribute("DefaultMuscleStiffness"):asFloat())
    writeNativeProperty(exportWriter, indent, "defaultmusclestrength", rockstarPhysicsData:findAttribute("DefaultMuscleStrength"):asFloat())
    writeNativeProperty(exportWriter, indent, "defaultmuscledamping", rockstarPhysicsData:findAttribute("DefaultMuscleDamping"):asFloat())

    -- export limit frames common to all joint types
    -- hinge requires a transformation
    local hingeOrientationTransform = nmx.Matrix.new()
    hingeOrientationTransform:set3x3ToZRotation(-0.5 * math.pi)

    local localFrame = physicsJointLimitTxInstance:getLocalMatrix()
    writeTransformProperty(exportWriter, indent, "localFrame", localFrame)
    
    local worldFixedFrame = physicsJointLimitTxInstance:getWorldMatrix()
    writeTransformProperty(exportWriter, indent, "worldFixedFrame", worldFixedFrame)

    local worldMovingFrame = rockstar.calculateJointLimitWorldMovingFrame(physicsJointLimitTxInstance, physicsJointLimit)
    writeTransformProperty(exportWriter, indent, "worldMovingFrame", worldMovingFrame)
  elseif physicsJointLimit:is(nmx.PhysicsTwistSwingNode.ClassTypeId()) then
    -- export PhysicsTwistSwingNode properties
    --
    writeNativeProperty(exportWriter, indent, "type", "BallSocket")
    --not needed for r* human input
    local rockstarPhysicsData = rockstar.getRockstarPhysicsEngineData(physicsJointLimit)
    writeNativeProperty(exportWriter, indent, "swing1Angle", physicsJointLimit:findAttribute("Swing1Angle"):asFloat())
    writeNativeProperty(exportWriter, indent, "swing2Angle", physicsJointLimit:findAttribute("Swing2Angle"):asFloat())
    writeNativeProperty(exportWriter, indent, "twistAngle", physicsJointLimit:findAttribute("TwistAngle"):asFloat())
    writeNativeProperty(exportWriter, indent, "twistOffset", physicsJointLimit:findAttribute("TwistOffset"):asFloat())
    --output later as worldFixedFrame trans
    local worldFixedFrameTrans = physicsJointLimitTxInstance:getWorldMatrix():translation()
    writeVectorProperty(exportWriter, indent, "axisposition", worldFixedFrameTrans)
    --output later as worldMovingFrame xAxis
    local worldMovingFrameXAxis = rockstar.calculateJointLimitWorldMovingFrame(physicsJointLimitTxInstance, physicsJointLimit):xAxis()
    writeVectorProperty(exportWriter, indent, "axisdirection", worldMovingFrameXAxis)
    writeNativeProperty(exportWriter, indent, "limitenabled", rockstarPhysicsData:findAttribute("LimitEnabled"):asBool())
    writeNativeProperty(exportWriter, indent, "createeffector", rockstarPhysicsData:findAttribute("CreateEffector"):asBool())
    --output later as worldMovingFrame zAxis
    local worldMovingFrameZAxis = rockstar.calculateJointLimitWorldMovingFrame(physicsJointLimitTxInstance, physicsJointLimit):zAxis()
    writeVectorProperty(exportWriter, indent, "leandirection", worldMovingFrameZAxis)

    rockstar.writeTwistSwingLimitAngleData(exportWriter, indent, physicsJointLimitTxInstance, physicsJointLimit)
    --not needed for r* human input
    --mmmtodo could output them all using rockstar.writeCustomRockstarProperties but would mean bigger file/wrong order
    writeNativeProperty(exportWriter, indent, "reversefirstleanmotor", rockstarPhysicsData:findAttribute("ReverseFirstLeanMotor"):asBool())
    writeNativeProperty(exportWriter, indent, "reversesecondleanmotor", rockstarPhysicsData:findAttribute("ReverseSecondLeanMotor"):asBool())
    writeNativeProperty(exportWriter, indent, "reversetwistmotor", rockstarPhysicsData:findAttribute("ReverseTwistMotor"):asBool())
    writeNativeProperty(exportWriter, indent, "softlimitfirstleanmultiplier", rockstarPhysicsData:findAttribute("SoftLimitFirstLeanMultiplier"):asFloat())
    writeNativeProperty(exportWriter, indent, "softlimitsecondleanmultiplier", rockstarPhysicsData:findAttribute("SoftLimitSecondLeanMultiplier"):asFloat())
    writeNativeProperty(exportWriter, indent, "softlimittwistmultiplier", rockstarPhysicsData:findAttribute("SoftLimitTwistMultiplier"):asFloat())
    writeNativeProperty(exportWriter, indent, "defaultleanforcecap", rockstarPhysicsData:findAttribute("DefaultLeanForceCap"):asFloat())
    writeNativeProperty(exportWriter, indent, "defaulttwistforcecap", rockstarPhysicsData:findAttribute("DefaultTwistForceCap"):asFloat())
    writeNativeProperty(exportWriter, indent, "defaultmusclestiffness", rockstarPhysicsData:findAttribute("DefaultMuscleStiffness"):asFloat())
    writeNativeProperty(exportWriter, indent, "defaultmusclestrength", rockstarPhysicsData:findAttribute("DefaultMuscleStrength"):asFloat())
    writeNativeProperty(exportWriter, indent, "defaultmuscledamping", rockstarPhysicsData:findAttribute("DefaultMuscleDamping"):asFloat())


    -- export limit frames common to all joint types
    --
    local localFrame = physicsJointLimitTxInstance:getLocalMatrix()
    writeTransformProperty(exportWriter, indent, "localFrame", localFrame)

    local worldFixedFrame = physicsJointLimitTxInstance:getWorldMatrix()
    writeTransformProperty(exportWriter, indent, "worldFixedFrame", worldFixedFrame)

    local worldMovingFrame = rockstar.calculateJointLimitWorldMovingFrame(physicsJointLimitTxInstance, physicsJointLimit)
    writeTransformProperty(exportWriter, indent, "worldMovingFrame", worldMovingFrame)
  else
    assert(false, string.format("Unsupported PhysicsJointLimitNode type '%s'", physicsJointLimit:getTypeString()))
  end

  -- end PhysicsJointLimit tag
  --
  indent = indent - 1
  exportWriter:write(indent, "</PhysicsJointLimit>")
end

------------------------------------------------------------------------------------------------------------------------
--| signature: nil writeRagdollCollisionSets(
--|   table            exportWriter,
--|   integer          indent,
--|   CollisionSetNode physicsCollisionSets)
------------------------------------------------------------------------------------------------------------------------
local writeRagdollCollisionSets = function(exportWriter, indent, physicsCollisionSets)
  local physicsCollisionSet = physicsCollisionSets:getFirstChild("CollisionGroupNode")
  while physicsCollisionSet do
    local shapeTxInstances = nmx.sgTransformNodes.new()
    physicsCollisionSet:getGroupNodes(shapeTxInstances)

    if not shapeTxInstances:empty() then
      exportWriter:write(indent, "<CollisionSet>")
      indent = indent + 1

      writeNativeProperty(exportWriter, indent, "name", physicsCollisionSet:getName())

      -- use a table as as set to ensure that no duplicate parts are exported.
      --
      for i = 1, shapeTxInstances:size() do
        local shapeTxInstance = shapeTxInstances:at(i)
        writeNativeProperty(exportWriter, indent, "item", rockstar.getRockstarName(shapeTxInstance))
      end

      indent = indent - 1
      exportWriter:write(indent, "</CollisionSet>")
    end

    physicsCollisionSet = physicsCollisionSet:getNextSibling("CollisionGroupNode")
  end
end

------------------------------------------------------------------------------------------------------------------------
--| signature: nil writeRagdollPhysicsJointChildren(
--|   table            exportWriter,
--|   integer          depth,
--|   sgTransformNode  physicsJointTxInstance,
--|   PhysicsJointNode physicsJoint,
--|   sgTransformNodes exportedPhysicsJointTxInstances)
------------------------------------------------------------------------------------------------------------------------
local writeRagdollPhysicsJointChildren = nil
writeRagdollPhysicsJointChildren = function(exportWriter, indent, physicsJointTxInstance, physicsJoint, exportedPhysicsJointTxInstances)
  -- assert all arguments are valid
  --
  assert(type(exportWriter) == "table", "bad argument #1 to 'exportRockstarRagdollPhysicsJoint'")
  assert(type(indent) == "number", "bad argument #2 to 'exportRockstarRagdollPhysicsJoint'")
  assert(type(physicsJointTxInstance) == "userdata" and physicsJointTxInstance:is(nmx.sgTransformNode.ClassTypeId()), "bad argument #3 to 'exportRockstarRagdollPhysicsJoint'")
  assert(type(physicsJoint) == "userdata" and physicsJoint:isDerivedFrom(nmx.PhysicsJointNode.ClassTypeId()), "bad argument #4 to 'exportRockstarRagdollPhysicsJoint'")
  assert(type(exportedPhysicsJointTxInstances) == "table", "bad argument #4 to 'exportRockstarRagdollPhysicsJoint'")

  -- write out children here
  -- begin child object section for the PhysicsJointNode
  --
  exportWriter:write(indent + 1, "<ChildObjects>")
  
  local childJointTxInstances = {}
  local childJoints = {}

  -- write child physics joints
  --
  local childTxInstance = physicsJointTxInstance:getFirstChild()
  while childTxInstance do
    if childTxInstance:is("sgTransformNode") then
      local childPhysicsJoint = nmx.PhysicsJointNode.getPhysicsJoint(childTxInstance)
      if childPhysicsJoint then
        if not rockstar.physicsJointHasPhysicsVolume(childTxInstance, childPhysicsJoint) and
           not rockstar.physicsJointIsLeaf(physicsJointTxInstance, physicsJoint) then
          assert(false, string.format("PhysicsJointNode '%s' has no associated PhysicsVolumeNode", physicsJointTxInstance:getName()))
        end

        -- write out child PhysicsJointNode
        --
        writeRagdollPhysicsJoint(exportWriter, indent + 2, childTxInstance, childPhysicsJoint)
        writeRagdollPhysicsJointChildren(exportWriter, indent + 2, childTxInstance, childPhysicsJoint, exportedPhysicsJointTxInstances)

        table.insert(exportedPhysicsJointTxInstances, childTxInstance)
        table.insert(childJointTxInstances, childTxInstance)
        table.insert(childJoints, childPhysicsJoint)
      end
    end
    childTxInstance = childTxInstance:getNextSibling()
  end

  -- write out the child PhysicsJointLimitNodes
  --
  for i = 1, table.getn(childJointTxInstances) do
    local physicsJointLimitTxInstance, physicsJointLimit = rockstar.getPhysicsJointLimit(childJointTxInstances[i], childJoints[i])
    if physicsJointLimitTxInstance and physicsJointLimit then
      writeRagdollPhysicsJointLimit(exportWriter, indent + 2, physicsJointLimitTxInstance, physicsJointLimit)
    end
  end

  -- begin child PhysicsBodyNodes section, the indent of 3 is to match the rockstar ragdoll format
  --
  exportWriter:write(indent + 3, "<ChildObjects>")

    -- write out the PhysicsBodyNode for the root PhysicsJointNode
    --
    local physicsBodyTxInstance, physicsBody = rockstar.getPhysicsBody(physicsJointTxInstance, physicsJoint)
    writeRagdollPhysicsBody(exportWriter, indent + 4, physicsBodyTxInstance, physicsBody)

  -- end child PhysicsBodyNodes section
  --
  exportWriter:write(indent + 3, "</ChildObjects>")

  -- end child object section for the PhysicsJointNode
  --
  exportWriter:write(indent + 1, "</ChildObjects>")
end

------------------------------------------------------------------------------------------------------------------------
--| signature: nil rockstar.writeRagdoll(
--|   table            exportWriter,
--|   sgTransformNode  rootPhysicsJointTxInstance,
--|   PhysicsJointNode rootPhysicsJoint,
--|   sgTransformNodes exportedPhysicsJointTxInstances)
------------------------------------------------------------------------------------------------------------------------
rockstar.writeRagdoll = function(exportWriter, rootPhysicsJointTxInstance, rootPhysicsJoint, exportedPhysicsJointTxInstances)
  -- assert all arguments are valid
  --
  assert(type(exportWriter) == "table", "bad argument #1 to 'rockstar.writeRagdoll'")
  assert(type(rootPhysicsJointTxInstance) == "userdata" and rootPhysicsJointTxInstance:is(nmx.sgTransformNode.ClassTypeId()), "bad argument #2 to 'rockstar.writeRagdoll'")
  assert(type(rootPhysicsJoint) == "userdata" and rootPhysicsJoint:isDerivedFrom(nmx.PhysicsJointNode.ClassTypeId()), "bad argument #3 to 'rockstar.writeRagdoll'")
  assert(type(exportedPhysicsJointTxInstances) == "table", "bad argument #4 to 'rockstar.writeRagdoll'")

  -- export the ragdoll
  --
  local indent = 0
  exportWriter:write(indent, "<LODGroup>")
  exportWriter:write(indent, "<NMAgent>", 0, "</NMAgent>")
  exportWriter:write(0, "")

  -- begin LOD 0 export
  --
  exportWriter:write(indent, "<PhysicsCharacter>")
  indent = indent + 1

  exportWriter:write(indent, "<LOD>", 0, "</LOD>")

  exportWriter:write(indent, "<ChildObjects>")
  indent = indent + 1

  -- recursively export the joints, joint limits and bodies
  --
  if not rockstar.physicsJointHasPhysicsVolume(rootPhysicsJointTxInstance, rootPhysicsJoint) then
    assert(false, string.format("Physics rig root PhysicsJointNode '%s' has no associated PhysicsVolumeNode", rootPhysicsJointTxInstance:getName()))
  end

  table.insert(exportedPhysicsJointTxInstances, rootPhysicsJointTxInstance)
  writeRagdollPhysicsJoint(exportWriter, indent, rootPhysicsJointTxInstance, rootPhysicsJoint)
  writeRagdollPhysicsJointChildren(exportWriter, indent, rootPhysicsJointTxInstance, rootPhysicsJoint, exportedPhysicsJointTxInstances)

  indent = indent - 1
  exportWriter:write(indent, "</ChildObjects>")

  -- begin collision sets export
  --
  exportWriter:write(indent, "<CollisionSets>")
  indent = indent + 1

  local physicsRigRootTx = rootPhysicsJoint:getParent()
  local collisionSets = physicsRigRootTx:getFirstChild("CollisionSetNode")
  writeRagdollCollisionSets(exportWriter, indent, collisionSets)

  -- end collision sets export
  --
  indent = indent - 1
  exportWriter:write(indent, "</CollisionSets>")

  -- end LOD 0 export
  --
  indent = indent - 1
  exportWriter:write(indent, "</PhysicsCharacter>")
  exportWriter:write(0, "")

  -- end ragdoll export
  --
  exportWriter:write(indent, "</LODGroup>")
end
