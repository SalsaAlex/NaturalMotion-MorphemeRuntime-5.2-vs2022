------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
local physicsNodeTypesSet = {
  ["EnvironmentPhysicsConstraintNode"] = true,
  ["PhysicsMassPropertiesNode"] = true,
  ["PhysicsJointLimitNode"] = true,
  ["PhysicsSoftTwistSwingNode"] = true,
  ["PhysicsJointNode"] = true,
  ["PreviewPhysicsVolumeNode"] = true,
  ["PreviewPhysicsJointNode"] = true,
  ["PreviewGhostPhysicsVolumeNode"] = true,
  ["PreviewGhostPhysicsJointNode"] = true,
  -- PhysicsVolumeNode and PhysicsBodyNode not included as they can be created in non-physics build
}

------------------------------------------------------------------------------------------------------------------------
local euphoriaNodeTypesSet = {
  ["LocatorNode"] = true,
  ["InteractionProxyNode"] = true,
  ["ReachLimitNode"] = true,
  ["SelfAvoidanceNode"] = true,
}

------------------------------------------------------------------------------------------------------------------------
local kinectNodeTypesSet = {
}

------------------------------------------------------------------------------------------------------------------------
local Disabled = 0
local Hidden = 1
local Shown = 2
local Editable = 3

------------------------------------------------------------------------------------------------------------------------
local addPreset = function(label, settings)
  -- label must be a string of greater than zero length
  if type(label) ~= "string" or string.len(label) == 0 then
    return false
  end

  -- settings must be a string containing valid lua code
  if type(settings) ~= "table" then
    return false
  end

  local app = nmx.Application.new()
  local roamingUserSettings = app:getRoamingUserSettings()

  local presets = roamingUserSettings:getSetting("|Presets")
  local presetTypeId = app:lookupTypeId("PresetNode")
  local preset = roamingUserSettings:createNode(presetTypeId, label, presets)

  preset:findAttribute("Camera"):setString(settings.camera)
  preset:findAttribute("DisplayMode"):setString(settings.displayMode)
  preset:findAttribute("ColourMode"):setString(settings.colourMode)

  local setType = function(sf, typeId, show)
    if show then
      sf:addNodeType(typeId)
    else
      sf:removeNodeType(typeId)
    end
  end

  -- default is editable for all types
  settings.filters.Node = Editable

  local physicsDisabled = mcn.isPhysicsDisabled()
  local euphoriaDisabled = mcn.isEuphoriaDisabled()
  local kinectDisabled = mcn.isKinectDisabled()

  local displayFilter = nmx.SelectionFilter.new()
  local selectionFilter = nmx.SelectionFilter.new()

  for typeName, typeState in pairs(settings.filters) do
    local typeId = app:lookupTypeId(typeName)

    -- check this was a valid type
    if app:lookupTypename(typeId) == typeName then
      -- make types specific to disabled skus hidden and not selectable
      if physicsNodeTypesSet[typeName] and physicsDisabled then
        typeState = Disabled
      end

      if euphoriaNodeTypesSet[typeName] and euphoriaDisabled then
        typeState = Disabled
      end

      if kinectNodeTypesSet[typeName] and kinectDisabled then
        typeState = Disabled
      end

      local isShown = false
      if typeState == Shown or typeState == Editable then
        isShown = true
      end
      setType(displayFilter, typeId, isShown)

      local isSelectable = false
      -- if the type is hidden it is still selectable so when it is unhidden the user
      -- does not have to then make it selectable as well
      if typeState == Editable or typeState == Hidden then
        isSelectable = true
      end
      setType(selectionFilter, typeId, isSelectable)
    end
  end

  local serialisedDisplayFilter = nmx.IntArray.new()
  displayFilter:serialise(serialisedDisplayFilter)
  preset:findAttribute("DisplayFilter"):setIntArray(serialisedDisplayFilter)

  local serialisedSelectionFilter = nmx.IntArray.new()
  selectionFilter:serialise(serialisedSelectionFilter)
  preset:findAttribute("SelectionFilter"):setIntArray(serialisedSelectionFilter)

  return true
end

------------------------------------------------------------------------------------------------------------------------
-- adds a default set of presets
------------------------------------------------------------------------------------------------------------------------
local addDefaultPresets = function()
  local app = nmx.Application.new()
  local roamingUserSettings = app:getRoamingUserSettings()

  local status, cbRef = roamingUserSettings:beginChangeBlock(getCurrentFileAndLine())

  local euphoriaTypesEditable = Editable
  local euphoriaTypesShown = Shown
  if mcn.isEuphoriaDisabled() then
    euphoriaTypesEditable = Hidden
    euphoriaTypesShown = Hidden
  end

  addPreset(
    "Default",
    {
      
      camera = "|LogicalRoot|Cameras|PerspectiveCameraTransform|PerspectiveCamera",
      displayMode = "Normal", colourMode = "Object",
      filters = {
        JointNode = Editable,
        MorphemeMainSkeletonNode = Editable,
        JointLimitNode = Editable,
        BlendFrameTransformLocatorNode = Editable,
        MorphemePreviewSkeletonNode = Editable,
        MeshNode = Shown,
        OffsetFrameNode = Hidden,
        EnvironmentPhysicsConstraintNode = Editable,
        PhysicsBodyNode = Editable,
        PhysicsMassPropertiesNode = Shown,
        PhysicsVolumeNode = Editable,
        PhysicsJointLimitNode = Editable,
        PhysicsSoftTwistSwingNode = euphoriaTypesEditable,
        PhysicsJointNode = Editable,
        ReachLimitNode = Hidden,
        LocatorNode = euphoriaTypesEditable,
        InteractionProxyNode = euphoriaTypesEditable,
        CharacterControllerNode = Hidden,
        CharacterStartPointNode = Editable,
        SelfAvoidanceNode = euphoriaTypesEditable,
        PreviewPhysicsVolumeNode = Hidden,
        PreviewPhysicsJointNode = Hidden,
        PreviewGhostPhysicsVolumeNode = Hidden,
        PreviewGhostPhysicsJointNode = Hidden,
        TriggerVolumeNode = Hidden,
      },
    }
  )

  addPreset(
    "Animation Preview",
    {
      camera = "|LogicalRoot|Cameras|PerspectiveCameraTransform|PerspectiveCamera",
      displayMode = "Normal", colourMode = "Object",
      filters = {
        JointNode = Hidden,
        MorphemeMainSkeletonNode = Hidden,
        JointLimitNode = Hidden,
        BlendFrameTransformLocatorNode = Hidden,
        MorphemePreviewSkeletonNode = Hidden,
        MeshNode = Shown,
        OffsetFrameNode = Hidden,
        PhysicsBodyNode = Hidden,
        EnvironmentPhysicsConstraintNode = Hidden,
        PhysicsMassPropertiesNode = Hidden,
        PhysicsVolumeNode = Hidden,
        PhysicsJointLimitNode = Hidden,
        PhysicsSoftTwistSwingNode = Hidden,
        PhysicsJointNode = Hidden,
        ReachLimitNode = Hidden,
        LocatorNode = Hidden,
        InteractionProxyNode = Hidden,
        CharacterControllerNode = Hidden,
        CharacterStartPointNode = Hidden,
        SelfAvoidanceNode = Hidden,
        PreviewPhysicsVolumeNode = Hidden,
        PreviewPhysicsJointNode = Hidden,
        PreviewGhostPhysicsVolumeNode = Hidden,
        PreviewGhostPhysicsJointNode = Hidden,
        TriggerVolumeNode = Hidden,
      },
    }
  )

  if not mcn.isPhysicsDisabled() then    
    addPreset(
      "Physics Preview",
      {
        camera = "|LogicalRoot|Cameras|PerspectiveCameraTransform|PerspectiveCamera",
        displayMode = "Normal", colourMode = "Object",
        filters = {
          JointNode = Hidden,
          MorphemeMainSkeletonNode = Hidden,
          JointLimitNode = Hidden,
          BlendFrameTransformLocatorNode = Hidden,
          MorphemePreviewSkeletonNode = Hidden,
          MeshNode = Hidden,
          OffsetFrameNode = Hidden,
          PhysicsBodyNode = Hidden,
          EnvironmentPhysicsConstraintNode = Hidden,
          PhysicsMassPropertiesNode = Hidden,
          PhysicsVolumeNode = Editable,
          PhysicsJointLimitNode = Editable,
          PhysicsSoftTwistSwingNode = euphoriaTypesEditable,
          PhysicsJointNode = Editable,
          ReachLimitNode = Hidden,
          LocatorNode = Hidden,
          InteractionProxyNode = Hidden,
          CharacterControllerNode = Hidden,
          CharacterStartPointNode = Hidden,
          SelfAvoidanceNode = Hidden,
          PreviewPhysicsVolumeNode = Shown,
          PreviewPhysicsJointNode = Shown,
          PreviewGhostPhysicsVolumeNode = Hidden,
          PreviewGhostPhysicsJointNode = Hidden,
          TriggerVolumeNode = Hidden,
        },
      }
    )
  end

  addPreset(
    "Environment Editing",
    {
      camera = "|LogicalRoot|Cameras|PerspectiveCameraTransform|PerspectiveCamera",
      displayMode = "Normal", colourMode = "Object",
      filters = {
        JointNode = Hidden,
        MorphemeMainSkeletonNode = Hidden,
        JointLimitNode = Hidden,
        BlendFrameTransformLocatorNode = Hidden,
        MorphemePreviewSkeletonNode = Shown,
        MeshNode = Shown,
        OffsetFrameNode = Hidden,
        EnvironmentPhysicsConstraintNode = Editable,
        PhysicsVolumeNode = Editable,
        PhysicsBodyNode = Editable,
        PhysicsMassPropertiesNode = Hidden,
        PhysicsJointLimitNode = Hidden,
        PhysicsSoftTwistSwingNode = Hidden,
        PhysicsJointNode = Hidden,
        ReachLimitNode = Hidden,
        LocatorNode = Hidden,
        InteractionProxyNode = Hidden,
        CharacterControllerNode = Hidden,
        CharacterStartPointNode = Editable,
        SelfAvoidanceNode = Hidden,
        PreviewPhysicsVolumeNode = Hidden,
        PreviewPhysicsJointNode = Hidden,
        PreviewGhostPhysicsVolumeNode = Hidden,
        PreviewGhostPhysicsJointNode = Hidden,
        TriggerVolumeNode = Hidden,
      },
    }
  )

  if not mcn.isPhysicsDisabled() then
    addPreset(
      "Collision Editing",
      {
        camera = "|LogicalRoot|Cameras|PerspectiveCameraTransform|PerspectiveCamera",
        displayMode = "XRay", colourMode = "Collision",
        filters = {
          JointNode = Hidden,
          MorphemeMainSkeletonNode = Hidden,
          JointLimitNode = Hidden,
          BlendFrameTransformLocatorNode = Hidden,
          MorphemePreviewSkeletonNode = Hidden,
          MeshNode = Shown,
          OffsetFrameNode = Hidden,
          EnvironmentPhysicsConstraintNode = Editable,
          PhysicsBodyNode = Editable,
          PhysicsMassPropertiesNode = Shown,
          PhysicsVolumeNode = Editable,
          PhysicsJointLimitNode = Hidden,
          PhysicsSoftTwistSwingNode = Hidden,
          PhysicsJointNode = Hidden,
          ReachLimitNode = Hidden,
          LocatorNode = Hidden,
          InteractionProxyNode = Hidden,
          CharacterControllerNode = Hidden,
          CharacterStartPointNode = Hidden,
          SelfAvoidanceNode = Hidden,
          PreviewPhysicsVolumeNode = Hidden,
          PreviewPhysicsJointNode = Hidden,
          PreviewGhostPhysicsVolumeNode = Hidden,
          PreviewGhostPhysicsJointNode = Hidden,
          TriggerVolumeNode = Hidden,
        },
      }
    )

    addPreset(
      "Physics Joint Editing",
      {
        camera = "|LogicalRoot|Cameras|PerspectiveCameraTransform|PerspectiveCamera",
        displayMode = "XRay", colourMode = "Object",
        filters = {
          JointNode = Hidden,
          MorphemeMainSkeletonNode = Hidden,
          JointLimitNode = Hidden,
          BlendFrameTransformLocatorNode = Hidden,
          MorphemePreviewSkeletonNode = Hidden,
          MeshNode = Shown,
          OffsetFrameNode = Hidden,
          EnvironmentPhysicsConstraintNode = Hidden,
          PhysicsBodyNode = Hidden,
          PhysicsMassPropertiesNode = Shown,
          PhysicsVolumeNode = Hidden,
          PhysicsJointLimitNode = Editable,
          PhysicsSoftTwistSwingNode = euphoriaTypesEditable,
          PhysicsJointNode = Editable,
          ReachLimitNode = Hidden,
          LocatorNode = Hidden,
          InteractionProxyNode = Hidden,
          CharacterControllerNode = Hidden,
          CharacterStartPointNode = Hidden,
          SelfAvoidanceNode = Hidden,
          PreviewPhysicsVolumeNode = Hidden,
          PreviewPhysicsJointNode = Hidden,
          PreviewGhostPhysicsVolumeNode = Hidden,
          PreviewGhostPhysicsJointNode = Hidden,
          TriggerVolumeNode = Hidden,
        },
      }
    )
  end

  addPreset(
    "Controller Editing",
    {
      camera = "|LogicalRoot|Cameras|PerspectiveCameraTransform|PerspectiveCamera",
      displayMode = "Normal", colourMode = "Object",
      filters = {
        JointNode = Hidden,
        MorphemeMainSkeletonNode = Hidden,
        JointLimitNode = Hidden,
        BlendFrameTransformLocatorNode = Hidden,
        MorphemePreviewSkeletonNode = Hidden,
        MeshNode = Shown,
        OffsetFrameNode = Hidden,
        EnvironmentPhysicsConstraintNode = Hidden,
        PhysicsBodyNode = Hidden,
        PhysicsMassPropertiesNode = Hidden,
        PhysicsVolumeNode = Hidden,
        PhysicsJointLimitNode = Hidden,
        PhysicsSoftTwistSwingNode = Hidden,
        PhysicsJointNode = Hidden,
        ReachLimitNode = Hidden,
        LocatorNode = Hidden,
        InteractionProxyNode = Hidden,
        CharacterControllerNode = Editable,
        CharacterStartPointNode = Hidden,
        SelfAvoidanceNode = Hidden,
        PreviewPhysicsVolumeNode = Hidden,
        PreviewPhysicsJointNode = Hidden,
        PreviewGhostPhysicsVolumeNode = Hidden,
        PreviewGhostPhysicsJointNode = Hidden,
        TriggerVolumeNode = Hidden,
      }
    }
  )

  addPreset(
    "Offset Editing",
    {
      camera = "|LogicalRoot|Cameras|PerspectiveCameraTransform|PerspectiveCamera",
      displayMode = "XRay", colourMode = "Object",
      filters = {
        JointNode = Shown,
        MorphemeMainSkeletonNode = Shown,
        JointLimitNode = Hidden,
        BlendFrameTransformLocatorNode = Hidden,
        MorphemePreviewSkeletonNode = Hidden,
        MeshNode = Shown,
        OffsetFrameNode = Editable,
        EnvironmentPhysicsConstraintNode = Hidden,
        PhysicsBodyNode = Hidden,
        PhysicsMassPropertiesNode = Hidden,
        PhysicsVolumeNode = Hidden,
        PhysicsJointLimitNode = Hidden,
        PhysicsSoftTwistSwingNode = Hidden,
        PhysicsJointNode = Hidden,
        ReachLimitNode = Hidden,
        LocatorNode = Hidden,
        InteractionProxyNode = Hidden,
        CharacterControllerNode = Hidden,
        CharacterStartPointNode = Hidden,
        SelfAvoidanceNode = Hidden,
        PreviewPhysicsVolumeNode = Hidden,
        PreviewPhysicsJointNode = Hidden,
        PreviewGhostPhysicsVolumeNode = Hidden,
        PreviewGhostPhysicsJointNode = Hidden,
      }
    }
  )

  if not mcn.isEuphoriaDisabled() then
    addPreset(
      "Limb Editing",
      {
        camera = "|LogicalRoot|Cameras|PerspectiveCameraTransform|PerspectiveCamera",
        displayMode = "XRay", colourMode = "Limb",
        filters = {
          JointNode = Hidden,
          MorphemeMainSkeletonNode = Hidden,
          JointLimitNode = Hidden,
          BlendFrameTransformLocatorNode = Hidden,
          MorphemePreviewSkeletonNode = Hidden,
          MeshNode = Shown,
          OffsetFrameNode = Hidden,
          EnvironmentPhysicsConstraintNode = Shown,
          PhysicsBodyNode = Shown,
          PhysicsMassPropertiesNode = Hidden,
          PhysicsVolumeNode = Shown,
          PhysicsJointLimitNode = Hidden,
          PhysicsSoftTwistSwingNode = Hidden,
          PhysicsJointNode = Shown,
          ReachLimitNode = euphoriaTypesEditable,
          LocatorNode = euphoriaTypesEditable,
          CharacterControllerNode = Hidden,
          CharacterStartPointNode = Hidden,
          SelfAvoidanceNode = Hidden,
          PreviewPhysicsVolumeNode = Hidden,
          PreviewPhysicsJointNode = Hidden,
          PreviewGhostPhysicsVolumeNode = Hidden,
          PreviewGhostPhysicsJointNode = Hidden,
          TriggerVolumeNode = Hidden,
        }
      }
    )
  end

  roamingUserSettings:endChangeBlock(cbRef, changeBlockInfo("End Preset Change"))
end

local addUnitSet = function(label, settings)
  -- label must be a string of greater than zero length
  if type(label) ~= "string" or string.len(label) == 0 then
    return false
  end

  -- settings must be a string containing valid lua code
  if type(settings) ~= "table" then
    return false
  end

  local nmxApp = nmx.Application.new()
  local roamingUserSettings = nmxApp:getRoamingUserSettings()

  local unitSets = roamingUserSettings:getSetting("|MorphemeAttributeDefaultsPresets")
  local nodeTypeId = nmxApp:lookupTypeId("MorphemeAttributeDefaultsNode")
  local unitDefaults = roamingUserSettings:createNode(nodeTypeId, label, unitSets)

  local distanceUnit = units.findByName(settings.distanceUnit)
  if distanceUnit then
    unitDefaults:findAttribute("DistanceScaleFactor"):setFloat(distanceUnit.scaleFactor)
  end

  if settings.upAxis ~= nil then
    local axis = nmx.MorphemeProjectSettingsNode.WorldUpAxes[settings.upAxis]
    if axis ~= nil then
      unitDefaults:findAttribute("UpAxis"):setInt(axis)
    end
  end

  -- utility function
  local setDefault = function(attrName, val)
    if val ~= nil then
      local attr = unitDefaults:findAttribute(attrName)
      if attr:isValid() then
        attr:setFloat(val)
      else
        app.error("init_settings : `" .. attrName .. "` unkown unit default")
      end
    end
  end

  setDefault("Density", settings.density)
  setDefault("SleepThreshold", settings.sleepThreshold)
  setDefault("JointStrength", settings.jointStrength)
  setDefault("JointDamping", settings.jointDamping)
  setDefault("SkinWidth", settings.skinWidth)
end

local addDefaultUnitSets = function()
  local nmxApp = nmx.Application.new()
  local roamingUserSettings = nmxApp:getRoamingUserSettings()

  local status, cbRef = roamingUserSettings:beginChangeBlock(getCurrentFileAndLine())

  addUnitSet(
    "cm Y-up",
    {
      distanceUnit = "cm",
      upAxis = "kWorldUpYAxis",
      density = 0.0002,
      sleepThreshold = 0.005,
      jointStrength = 10000,
      jointDamping = 50,
      skinWidth = 1,
    }
  )

  addUnitSet(
    "cm Z-up",
    {
      distanceUnit = "cm",
      upAxis = "kWorldUpZAxis",
      density = 0.0002,
      sleepThreshold = 0.005,
      jointStrength = 10000,
      jointDamping = 50,
      skinWidth = 1,
    }
  )

  addUnitSet(
    "m Y-up",
    {
      distanceUnit = "m",
      upAxis = "kWorldUpYAxis",
      density = 200,
      sleepThreshold = 0.005,
      jointStrength = 10000,
      jointDamping = 50,
      skinWidth = 0.01,
    }
  )

  addUnitSet(
    "m Z-up",
    {
      distanceUnit = "m",
      upAxis = "kWorldUpZAxis",
      density = 200,
      sleepThreshold = 0.005,
      jointStrength = 10000,
      jointDamping = 50,
      skinWidth = 0.01,
    }
  )

  addUnitSet(
    "Unreal",
    {
      distanceUnit = "UE3",
      upAxis = "kWorldUpZAxis",
      density = 0.0016,
      sleepThreshold = 0.005,
      jointStrength = 10000,
      jointDamping = 50,
      skinWidth = 0.5,
    }
  )

  addUnitSet( -- copy of "cm Z-up"
    "Gamebryo",
    {
      distanceUnit = "cm",
      upAxis = "kWorldUpZAxis",
      density = 0.0002,
      sleepThreshold = 0.005,
      jointStrength = 10000,
      jointDamping = 50,
      skinWidth = 1,
    }
  )

  addUnitSet( -- copy of "cm Z-up"
    "Vision",
    {
      distanceUnit = "cm",
      upAxis = "kWorldUpZAxis",
      density = 0.0002,
      sleepThreshold = 0.005,
      jointStrength = 10000,
      jointDamping = 50,
      skinWidth = 1,
    }
  )

  roamingUserSettings:endChangeBlock(cbRef, changeBlockInfo("End Preset Change"))
end

local roamingUserSettings = nmx.Application.new():getRoamingUserSettings()
local presets = roamingUserSettings:getSetting("|Presets")

-- if there are no existing presets then add some defaults
if presets and presets:getFirstChild() == nil then
  addDefaultPresets()
end

local unitSets = roamingUserSettings:getSetting("|MorphemeAttributeDefaultsPresets")
if unitSets and unitSets:getFirstChild() == nil then
  addDefaultUnitSets()
end
