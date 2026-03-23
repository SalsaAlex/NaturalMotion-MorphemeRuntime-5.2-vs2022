------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- scales the given list of object attributes by scale factor
------------------------------------------------------------------------------------------------------------------------
local scaleObjectAttributes = function(object, attributeNames, scaleFactor)
  assert(object)
  assert(type(attributeNames) == "table")
  assert(type(scaleFactor) == "number")

  for _, attributeName in ipairs(attributeNames) do
    local attribute = object:findAttribute(attributeName)

    if attribute:isValid() then
      local attributeIsEditable = attribute:isEditable()
      attribute:setEditable(true)

      local attributeType = attribute:getTypeId()
      if attributeType == nmx.AttributeTypeId.kInt then
        local value = attribute:asInt()
        value = value * scaleFactor
        local result = attribute:setInt(value)
        if not result then
          app.warning(string.format("scalePhysicsRig(): failed to scale attribute '%s' of object '%s'", attributeName, object:getName()))
        end
      elseif attributeType == nmx.AttributeTypeId.kFloat then
        local value = attribute:asFloat()
        value = value * scaleFactor
        local result = attribute:setFloat(value)
        if not result then
          app.warning(string.format("scalePhysicsRig(): failed to scale attribute '%s' of object '%s'", attributeName, object:getName()))
        end
      elseif attributeType == nmx.AttributeTypeId.kVector3 then
        local value = attribute:asVector3()
        value:setScaledVector(value, scaleFactor)
        local result = attribute:setVector3(value)
        if not result then
          app.warning(string.format("scalePhysicsRig(): failed to scale attribute '%s' of object '%s'", attributeName, object:getName()))
        end
      else
        -- unhandled attribute type
        assert(false)
      end

      attribute:setEditable(attributeIsEditable)
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
-- recurse through the rig scaling any appropriate objects
------------------------------------------------------------------------------------------------------------------------
local recursivelyScaleRig
recursivelyScaleRig = function(app, object, scaleFactor)
  assert(object)
  assert(type(scaleFactor) == "number")
  assert(scaleFactor > 0.0)

  local characterControllerId = app:lookupTypeId("CharacterControllerNode")
  local interactionProxyId = app:lookupTypeId("InteractionProxyNode")

  local shouldScaleTransform = false

  local child = object:getFirstChild()
  while child ~= nil do
  
    if child:is(nmx.BoxNode.ClassTypeId()) then
      scaleObjectAttributes(
        child,
        { "Width", "Height", "Depth", },
        scaleFactor)
        
    elseif child:is(nmx.CapsuleNode.ClassTypeId()) then
      scaleObjectAttributes(
        child,
        { "Height", "Radius", },
        scaleFactor)
        
    elseif child:is(nmx.SphereNode.ClassTypeId()) then
      scaleObjectAttributes(
        child,
        { "Radius", },
        scaleFactor)
        
    elseif child:is(nmx.PhysicsVolumeNode.ClassTypeId()) then
      scaleObjectAttributes(
        child,
        { "SkinWidth", },
        scaleFactor)
      shouldScaleTransform = true
        
    elseif child:is(nmx.OffsetFrameTransformNode.ClassTypeId()) then
      scaleObjectAttributes(
        child,
        { "LocalTranslation" },
        scaleFactor)
        
    elseif child:is(nmx.ReverseOffsetFrameNode.ClassTypeId()) then
      scaleObjectAttributes(
        child,
        { "OffsetLocalTranslation" },
        scaleFactor)
      
    elseif child:is(interactionProxyId) or  child:is(characterControllerId) then
      shouldScaleTransform = true
    end


    if child:is(nmx.TransformBaseNode.ClassTypeId()) or
       child:is(nmx.JointNode.ClassTypeId()) or
       child:is(nmx.PhysicsBodyNode.ClassTypeId()) then
      recursivelyScaleRig(app, child, scaleFactor)
    end

    child = child:getNextSibling()
  end

  if shouldScaleTransform and object:is(nmx.TransformNode.ClassTypeId()) then
    scaleObjectAttributes(
      object,
      { "Translation", },
      scaleFactor
    )
  end
end

local scalePhysicsRigBindPose = function(root, scaleFactor)
  -- Scale the bind pose
  local hierarchyIterator = nmx.NodeIterator.new(root, nmx.HierarchyNode.ClassTypeId())
  if hierarchyIterator:next() then
    local hierarchy = hierarchyIterator:node()
    hierarchy:scaleBindPoseTranslations(scaleFactor)
  else
    error("Cannot find the HierarchyNode in physics rig")
  end
end

local scaleAnimationRigPoses = function(root, scaleFactor)
  local scaleVec = nmx.Vector3.new(scaleFactor, scaleFactor, scaleFactor)
  -- Scale the bind pose
  local it = nmx.NodeIterator.new(root, nmx.RigInfoNode.ClassTypeId())
  if it:next() then
    local rigInfo = it:node()
    local retargetPose = rigInfo:findAttributeArray("RigRetargetPose")
    
    local size = retargetPose:size()
    for i = 1,size do
      local attr = retargetPose:getAttribute(i)
      local mat = attr:asMatrix()
      
      local translation = mat:translation()
      
      translation:multiplyElements(scaleVec)
      
      mat:setRow(translation, 3)
      
      attr:setMatrix(mat)
    end
  else
    error("Cannot find the HierarchyNode in physics rig")
  end
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean scalePhysicsRig(nmx.Scene scene, string set, number scaleFactor, boolean scaleRig = true, boolean scaleBindPose = false)
--| brief: Scales an animation set's physics rig by scaleFactor.
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
scalePhysicsRig = function(scene, set, scaleFactor, scaleRig, scaleBindPose)
  if scaleRig == nil then scaleBindPose = true end
  if scaleBindPose == nil then scaleBindPose = false end

  if type(set) ~= "string" then
    error(string.format("scalePhysicsRig : invalid parameter 2 expected 'string' got '%s'.", type(set)))
  end

  if type(scaleFactor) ~= "number" then
    error(string.format("scalePhysicsRig : invalid parameter 3 expected 'number' got '%s'.", type(scaleFactor)))
  end

  if scaleFactor <= 0.0 then
    error(string.format("scalePhysicsRig : scale factor must be a positive number, not '%f'.", scaleFactor))
  end

  if not (scaleRig or scaleBindPose) then
    error("scalePhysicsRig : scaleRig and scaleBindPose are both false")
  end

  local root = anim.getPhysicsRigDataRoot(scene, set)
  if root then
    local blockStatus, cbRef = scene:beginChangeBlock(getCurrentFileAndLine())

    local app = nmx.Application.new()

    local status, err

    if scaleBindPose then
      status, err = pcall(scalePhysicsRigBindPose, root, scaleFactor)
    end

    if status and scaleRig then
      status, err = pcall(recursivelyScaleRig, app, root, scaleFactor)
    end

    if status then
      scene:endChangeBlock(cbRef, changeBlockInfo("scaling physics rig for set %q by a factor of %f", set, scaleFactor))
    else
      print(err)
      scene:rollback(cbRef)
    end
    return true
  end

  return false
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean scaleAnimationRig(nmx.Scene scene, string set, number scaleFactor)
--| brief: Scales an animation set's physics rig by scaleFactor.
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
scaleAnimationRig = function(scene, set, scaleFactor)
  if type(set) ~= "string" then
    error(string.format("scaleAnimationRig : invalid parameter 2 expected 'string' got '%s'.", type(set)))
  end

  if type(scaleFactor) ~= "number" then
    error(string.format("scaleAnimationRig : invalid parameter 3 expected 'number' got '%s'.", type(scaleFactor)))
  end

  if scaleFactor <= 0.0 then
    error(string.format("scaleAnimationRig : scale factor must be a positive number, not '%f'.", scaleFactor))
  end

  local root = anim.getRigDataRoot(scene, set)
  if root then
    local blockStatus, cbRef = scene:beginChangeBlock(getCurrentFileAndLine())

    local app = nmx.Application.new()

    local status, err
    status, err = pcall(scaleAnimationRigPoses, root, scaleFactor)

    if status then
      status, err = pcall(recursivelyScaleRig, app, root, scaleFactor)
    end

    if status then
      scene:endChangeBlock(cbRef, changeBlockInfo("scaling animation rig for set %q by a factor of %f", set, scaleFactor))
    else
      print(err)
      scene:rollback(cbRef)
    end
    return true
  end

  return false
end