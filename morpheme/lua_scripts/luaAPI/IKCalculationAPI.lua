------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- recursively finds a descendent joint of the root joint given by name
------------------------------------------------------------------------------------------------------------------------
local findJointByName = function(root, name)
  local jointIt = nmx.NodeIterator.new(root, nmx.sgShapeNode.ClassTypeId())
  while jointIt:next() do
    local sgShape = jointIt:node()
    local dataNode = sgShape:getDataNode();
    if dataNode and dataNode:is(nmx.JointNode.ClassTypeId()) and dataNode:getName() == name then
      return sgShape:getParent()
    end
  end
  return nil
end

------------------------------------------------------------------------------------------------------------------------
-- Calculate the hinge axis of two joints in the rig of the specified set
-- the two joints are the parent and grand parent joints of the joint passed in.
-- Returns a table with x, y, z values.
------------------------------------------------------------------------------------------------------------------------
calculateHingeAxis = function(set, endJointName)
  local scene = nmx.Application.new():getSceneByName("AssetManager")
  local rootObj = anim.getRigSceneRoot(scene, set)
  local endJoint = findJointByName(rootObj, endJointName)

  if endJoint then
     -- The hinge axis is the cross product of the vectors along the two bones
     -- This vector is in the space of the root joint
    local hingeJoint = endJoint:getParent()
    local rootJoint = hingeJoint:getParent()
    if hingeJoint and rootJoint then
      local rootTrans = rootJoint:getWorldMatrix():translation()
      local hingeTrans = hingeJoint:getWorldMatrix():translation()
      local endTrans = endJoint:getWorldMatrix():translation()
      local v1 = nmx.Vector3.new(hingeTrans)
      local v2 = nmx.Vector3.new(endTrans)
      v1 = v1:subtract(rootTrans)
      v2 = v2:subtract(hingeTrans)
      v1:normalise()
      v2:normalise()
      local crossVec = nmx.Vector3.new()
      crossVec:cross(v1, v2)
      crossVec:normalise()
      local rootMat = rootJoint:getWorldMatrix()
      crossVec = rootMat:inverseRotateVector(crossVec)
      return crossVec
    end
  end

  return nmx.Vector3.new(1, 0, 0)
end

------------------------------------------------------------------------------------------------------------------------
-- Calculate the height of the given joint along an up vector
------------------------------------------------------------------------------------------------------------------------
calculateJointHeight = function(set, jointName, upVector)
  local scene = nmx.Application.new():getSceneByName("AssetManager")
  local rootObj = anim.getRigSceneRoot(scene, set)
  local joint = findJointByName(rootObj, jointName)

  if joint then
    return joint:getWorldMatrix():translation():dot(upVector)
  end

  return 0
end

------------------------------------------------------------------------------------------------------------------------
-- Calculate an up vector in the local space of a joint
------------------------------------------------------------------------------------------------------------------------
calculateLocalUpVector = function(set, jointName, upVector)
  local scene = nmx.Application.new():getSceneByName("AssetManager")
  local rootObj = anim.getRigSceneRoot(scene, set)
  local joint = findJointByName(rootObj, jointName)

  if joint then
    return joint:getWorldMatrix():inverseRotateVector(upVector)
  end

  return nmx.Vector3.new(0, 1, 0)
end

------------------------------------------------------------------------------------------------------------------------
-- Calculate a vector perpendicular to the given up vector and to the direction from
-- the reference joint to the child joint, or the world z-axis if the child joint isn't
-- found, in the coordinate frame of the reference joint
------------------------------------------------------------------------------------------------------------------------
calculateLevelVector = function(set, referenceJointName, childJointName, upVector)
  local scene = nmx.Application.new():getSceneByName("AssetManager")
  local rootObj = anim.getRigSceneRoot(scene, set)
  local referenceJoint = findJointByName(rootObj, referenceJointName)
  local childJoint = findJointByName(rootObj, childJointName)

  if referenceJoint then

    local forwardVector = nmx.Vector3.new(0, 0, 1)
    if childJoint then
      local refWorldMatrix = referenceJoint:getWorldMatrix()
      local childWorldMatrix = childJoint:getWorldMatrix()
      forwardVector = childWorldMatrix:translation():subtract(refWorldMatrix:translation())
    end

    local sidewaysWorldVector = nmx.Vector3.new()
    sidewaysWorldVector:cross(upVector, forwardVector)
    sidewaysWorldVector:normalise()

    -- Very roundabout way of doing inverseRotateVector using the limited maths API
    return referenceJoint:getWorldMatrix():inverseRotateVector(sidewaysWorldVector);
  end

  return nmx.Vector3.new(1, 0, 0)
end

------------------------------------------------------------------------------------------------------------------------
-- Calculate a hinge rotation parallel to the ground (perpendicular to the up vector) and
-- to the bone between two joints.  So it is an axis around which the baseJoint should
-- rotate to move the childJoint vertically, specified in the referenceJoint's coordinate
-- frame.
------------------------------------------------------------------------------------------------------------------------
calculateLiftAxis = function(set, referenceJointName, baseJointName, childJointName, upVector)
  local scene = nmx.Application.new():getSceneByName("AssetManager")
  local rootObj = anim.getRigSceneRoot(scene, set)
  local referenceJoint = findJointByName(rootObj, referenceJointName)
  local baseJoint = findJointByName(rootObj, baseJointName)
  local childJoint = findJointByName(rootObj, childJointName)

  if referenceJoint and baseJoint and childJoint then

    -- First, get the local axis of rotation in the baseJoint's frame
    local baseWorldMatrix = baseJoint:getWorldMatrix()
    local childWorldMatrix = childJoint:getWorldMatrix()
    local childRelativePosition = childWorldMatrix:translation():subtract(baseWorldMatrix:translation())
    local worldAxis = nmx.Vector3.new()
    worldAxis:cross(upVector, childRelativePosition)
    worldAxis:normalise()

    -- Put into the reference joint's coordinate frame
    return referenceJoint:getWorldMatrix():inverseRotateVector(worldAxis)
  end

  return nmx.Vector3.new(1, 0, 0)
end

