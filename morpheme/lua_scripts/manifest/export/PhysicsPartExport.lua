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
-- For every physics joint in the table passed in this function exports a
-- partExport with all child physics bodies added to the parts volumeExport as the appropriate
-- shapeExport type.
------------------------------------------------------------------------------------------------------------------------
exportPhysicsPart = function(rigExport, inverseRootMatrix, physicsJointTxInstance, partIndex, warningsAndErrors)
  -- create the part for export
  --
  local partName = physicsJointTxInstance:getName()
  local partExport = rigExport:createPart(partName)
  local hasChildVolume = false

  -- look for a body plan tag on this joint node and export it if it exists.
  --
  partExport:setStringAttribute("BodyPlanTag", partName)
  
  -- is this joint a trajectory helper
  --
  if physicsJointTxInstance:findTag("TrajectoryCalculationJoint") then
    rigExport:addTrajectoryCalcMarkupPart(partIndex)
  end

  -- Get hold of the physics body for this joint.
  --
  local physicsBodyTxInstance = nil
  local physicsBody = nil
  local childInstance = physicsJointTxInstance:getFirstChild()
  while childInstance do
    if childInstance:is("sgTransformNode") then
      physicsBody = childInstance:getChildDataNode(nmx.PhysicsBodyNode.ClassTypeId())
      if physicsBody then
        physicsBodyTxInstance = childInstance
      end
    end
    childInstance = childInstance:getNextSibling()
  end

  -- error if there is no body
  --
  local physicsBody = nmx.PhysicsJointNode.getPhysicsBody(physicsJointTxInstance)
  if physicsBody == nil then
    table.insert(
      warningsAndErrors.errors,
      string.format("Unable to find PhysicsBody for PhysicsJoint '%s'.", physicsJointTxInstance:getName()))

    return partExport
  end

  partExport:setAngularDamping(physicsBody:findAttribute("AngularDamping"):asFloat())
  partExport:setLinearDamping(physicsBody:findAttribute("LinearDamping"):asFloat())

  -- export driver specific data
  --
  local physicsEngine = preferences.get("PhysicsEngine")
  local physicsDriverExporter = getPhysicsDriverExporter(physicsEngine)
  physicsDriverExporter.exportPart(physicsBody, partExport, warningsAndErrors)
  
  -- some engines don't allow volumes to have different attributes.  Warn if users vary these in the UI
  local supportsPerVolumeAuthoring = physicsDriverExporter.supportsPerVolumeAuthoring()
  local commonDensity, commonRestitution, commonFriction = nil, nil, nil 
  local differentVolumeAttributes = false
  
  -- Add all the child physics bodies as shapes to the volumeExport.
  --
  local hasChildVolume = false
  local volumeExport = partExport:getPhysicsVolume()

  childInstance = physicsBodyTxInstance:getFirstChild()
  while childInstance ~= nil do
    if childInstance:is(nmx.sgTransformNode.ClassTypeId()) and
       (childInstance:transformNodeIs(nmx.TransformNode.ClassTypeId()) or childInstance:transformNodeIs(nmx.PosQuatTransformNode.ClassTypeId())) then
      -- If there is a PhysicsBody as the PhysicsJoint's child then create the relevant shapeExport.
      --
      local physicsVolumeTxInstance = childInstance
      local physicsVolume = nmx.PhysicsVolumeNode.getPhysicsVolume(childInstance)
      local shapeInstance = nmx.PhysicsVolumeNode.getSgShape(childInstance)
      if physicsVolume and shapeInstance then
        hasChildVolume = true

        local shape = shapeInstance:getDataNode()
        local type = shape:getTypeId()

        -- World transform out
        --
        local transform = nmx.Matrix.new(physicsJointTxInstance:getWorldMatrix())
        transform:multiply(inverseRootMatrix)
        transform = nmx.matrixToTable(transform)
        partExport:setTransform(transform)
		
		app.warning("part transform")
		for key, value in pairs(transform) do
			app.warning("," .. value)
		end

        local shapeExport = nil
        local childLocalMatrix = nmx.matrixToTable(physicsVolumeTxInstance:getLocalMatrix())

        if type == nmx.BoxNode.ClassTypeId() then
          -- Box
          local dimensions = {
            x = shape:findAttribute("Width"):asFloat(),
            y = shape:findAttribute("Height"):asFloat(),
            z = shape:findAttribute("Depth"):asFloat()
          }
          shapeExport = volumeExport:createBox(childLocalMatrix, dimensions)
        elseif type == nmx.CapsuleNode.ClassTypeId() then
          -- Capsule
          local capsuleRadius = shape:findAttribute("Radius"):asFloat()
          local capsuleHeight = shape:findAttribute("Height"):asFloat()
          shapeExport = volumeExport:createCapsule(childLocalMatrix, capsuleRadius, capsuleHeight)
        elseif type == nmx.CylinderNode.ClassTypeId() then
          -- Cylinder
          local cylinderRadius = shape:findAttribute("Radius"):asFloat()
          local cylinderHeight = shape:findAttribute("Height"):asFloat()
          shapeExport = volumeExport:createCylinder(childLocalMatrix, cylinderRadius, cylinderHeight)
        elseif type == nmx.SphereNode.ClassTypeId() then
          -- Sphere
          local sphereRadius = shape:findAttribute("Radius"):asFloat()
          shapeExport = volumeExport:createSphere(childLocalMatrix, sphereRadius)
        else
          table.insert(
            warningsAndErrors.errors,
            string.format("Encountered unsupported shape type '%s' for PhysicsJoint '%s'.", shape:getTypeString(), physicsJointTxInstance:getName()))

          return partExport
        end

        local curDensity = physicsVolume:findAttribute("Density"):asFloat()
        local curRestitution = physicsVolume:findAttribute("Restitution"):asFloat()
        local curFriction = physicsVolume:findAttribute("Friction"):asFloat()
        shapeExport:setDoubleAttribute("density", curDensity )
        shapeExport:setDoubleAttribute("restitution", curRestitution)
        shapeExport:setDoubleAttribute("friction", curFriction)
        
        -- detect per volume attributes
        if(commonDensity == nil) then 
          commonDensity, commonRestitution, commonFriction = curDensity, curRestitution, curFriction
        elseif(curDensity ~= commonDensity or curRestitution ~= commonRestitution or curFriction ~= commonFriction) then 
          differentVolumeAttributes = true; 
        end
        
        -- export driver specific shape data
        --
        physicsDriverExporter.exportShape(physicsJointTxInstance, physicsVolume, shape, shapeExport, warningsAndErrors)

        partExport:setHasCollision(true)
      end
    end

    childInstance = childInstance:getNextSibling()
  end

  if not supportsPerVolumeAuthoring and differentVolumeAttributes then 
  table.insert(
      warningsAndErrors.warnings,
      string.format("PhysicsJoint '%s' has volumes with different attributes.  The physics engine must have identical values (for instance density) for all child volumes.", physicsJointTxInstance:getName()))
  end 
  
  if not hasChildVolume then
    table.insert(
      warningsAndErrors.errors,
      string.format("PhysicsJoint '%s' has no shapes, all exported PhysicsJoints must have at least one child shape.", physicsJointTxInstance:getName()))
  end

  return partExport
end