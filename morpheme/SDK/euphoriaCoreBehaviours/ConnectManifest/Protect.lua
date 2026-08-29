--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "Protect",
  version = 8,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Protects a character's head and chest from injury due to impact by coordinating the use of other behaviours such as HeadAvoid, HeadDodge, Shield and Look. Protect chooses when and how to apply these behaviours, so is an autonomous reaction to the objects that it is aware of. ",
  group = "Behaviours",
  image = "ProtectBehaviour.png",
  id = generateNamespacedId(idNamespaces.NaturalMotion, 128),

  functionPins = 
  {
    ["Result"] = 
    {
      input = false,
      interfaces = 
      {
        "Transforms",
        "Time",
      },
    },

  },

  dataPins = 
  {
    ["OffsetFromChest"] = 
    {
      input = true,
      helptext = "Offset from the chest that is used as the primary part to protect. X is forward, Y is up and Z is right, relative to the chest. In metres (standard character).",
      type = "vector3",
      mode = "HiddenInNetworkEditor",
      displayName = "OffsetFromChest",
    },

  },

  pinOrder = 
  {
    "OffsetFromChest",
    "Result",
  },

  attributes = 
  {
    {
      name = "HeadLookWeight",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Look towards potential hazards that approach the character",
    },
    {
      name = "HazardLookTime",
      type = "float",
      value = 0.500000,
      min = -2.000000,
      max = 2.000000,
      helptext = "How long the character will look at an object that is no longer on collision course. Positive will look where the hazard was going to. Negative will look back to where the hazard came from. Both are extrapolations. In seconds (standard character).",
    },
    {
      name = "HazardLevelThreshold",
      type = "float",
      value = 0.750000,
      min = 0.000000,
      max = 4.000000,
      helptext = "Used only in hazardFreeTime to determine the time since the last hazard, as feedback info",
    },
    {
      name = "ObjectTrackingRadius",
      type = "float",
      value = 1.000000,
      min = 0.100000,
      max = 3.000000,
      helptext = "Radius for non-hazards, for example to use with headAvoid and armPlacement. In metres (standard character).",
    },
    {
      name = "HeadAvoidWeight",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Keep the head away from nearby hazards to reduce chance of head impact",
    },
    {
      name = "HeadDodgeWeight",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Move head laterally or duck out the way of an incoming projectile",
    },
    {
      name = "BraceWeight",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Move arms to intercept the incoming hazard, and cushion the impact",
    },
    {
      name = "ArmsPlacementWeight",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Place arms near or on nearby hazards that could be a danger in future",
    },
    {
      name = "SensitivityToCloseMovements",
      type = "float",
      value = 0.500000,
      min = 0.000000,
      max = 3.000000,
      helptext = "Determines the amount of danger given for a certain level of relative acceleration. Higher will do placement/headAvoid more eagerly. In seconds (standard character).",
    },
    {
      name = "CrouchDownAmount",
      type = "float",
      value = 0.250000,
      min = 0.000000,
      max = 0.500000,
      helptext = "How much the character should crouch downwards as the impact approaches",
    },
    {
      name = "CrouchPitchAmount",
      type = "float",
      value = 0.750000,
      min = 0.000000,
      max = 2.000000,
      helptext = "How much the character should lean forwards as the impact approaches",
    },
    {
      name = "SupportIgnoreRadius",
      type = "float",
      value = 0.500000,
      min = 0.000000,
      max = 2.000000,
      perAnimSet = true,
      helptext = "Radius of a sphere around the support point (i.e. the feet when balancing) where hazards will be ignored. This prevents bracing against the ground when crouching etc. In metres (standard character).",
    },
    {
      name = "ArmsSwingWeight",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Help rotate or stabilise the chest",
    },
    {
      name = "SwivelAmount",
      type = "float",
      value = 0.300000,
      min = -1.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Controls the arms' swivel, Positive is elbows out and up, 0 matches the guide pose, negative is elbows in and down",
    },
    {
      name = "MaxArmExtensionScale",
      type = "float",
      value = 1.000000,
      min = 0.100000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Limit the maximum extension of the arm. Expressed as a proportion of the total arm length. For use in bracen and placement",
    },
    {
      name = "LegsSwingWeight",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Help rotate or stabilise the pelvis. ",
    },
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(22, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(0, "numBehaviourInts")

    stream:writeUInt(16, "numBehaviourFloats")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "HeadLookWeight", asVal), string.format("Float_0_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "HazardLookTime"), string.format("Float_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "HazardLevelThreshold"), string.format("Float_2_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ObjectTrackingRadius"), string.format("Float_3_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "HeadAvoidWeight", asVal), string.format("Float_4_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "HeadDodgeWeight", asVal), string.format("Float_5_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "BraceWeight", asVal), string.format("Float_6_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ArmsPlacementWeight", asVal), string.format("Float_7_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "SensitivityToCloseMovements"), string.format("Float_8_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "CrouchDownAmount"), string.format("Float_9_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "CrouchPitchAmount"), string.format("Float_10_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "SupportIgnoreRadius", asVal), string.format("Float_11_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ArmsSwingWeight", asVal), string.format("Float_12_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "SwivelAmount", asVal), string.format("Float_13_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MaxArmExtensionScale", asVal), string.format("Float_14_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "LegsSwingWeight", asVal), string.format("Float_15_%d", asIdx-1))
    end

    stream:writeUInt(0, "numBehaviourVector3s")

    stream:writeUInt(0, "numBehaviourUInt64s")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numBehaviourNodeAnimationInputs")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numInputCPInts")
    stream:writeUInt(0, "numInputCPFloats")
    stream:writeUInt(0, "numInputCPUInt64s")
    stream:writeUInt(1, "numInputCPVector3s")
    stream:writeUInt(1, "numBehaviourControlParameterInputs")

    writeDataPinAltName(stream, node, "OffsetFromChest", "BehaviourControlParameterInputID_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_0")

    stream:writeUInt(0, "numBehaviourControlParameterOutputs")

    -------------------------------------------------------------------

    local numMessageSlots = 0
    local numOutputMessages = 0
    if (numOutputMessages > 0) then
      stream:writeUInt(numMessageSlots, "NumMessageSlots")
      stream:writeBool(true, "NodeEmitsMessages")
    else
      stream:writeUInt(0, "NumMessageSlots")
      stream:writeBool(false, "NodeEmitsMessages")
    end

  end, -- Serialize function

  -------------------------------------------------------------------

  validate = function(node) 
    return defaultPinValidation(node)
  end, -- Validate function

  -------------------------------------------------------------------

  getTransformChannels = function(node, set) 
    local transformChannels = { }
    local rigSize = anim.getRigSize(set)
    for i=1, rigSize do
      transformChannels[i - 1] = true
    end
    return transformChannels
  end,

}


-------------------------------------------------------------------

local attributeGroups =
{
  {
    title = "Hazard Awareness",
    isExpanded = true,
    details = 
    {
      {title = "HeadLookWeight", perAnimSet = true, type = "float", attributes = "HeadLookWeight"},
      {title = "HazardLookTime", perAnimSet = false, type = "float", attributes = "HazardLookTime"},
      {title = "HazardLevelThreshold", perAnimSet = false, type = "float", attributes = "HazardLevelThreshold"},
      {title = "ObjectTrackingRadius", perAnimSet = false, type = "float", attributes = "ObjectTrackingRadius"},
    },
  },
  {
    title = "Protect",
    isExpanded = false,
    details = 
    {
      {title = "HeadAvoidWeight", perAnimSet = true, type = "float", attributes = "HeadAvoidWeight"},
      {title = "HeadDodgeWeight", perAnimSet = true, type = "float", attributes = "HeadDodgeWeight"},
      {title = "BraceWeight", perAnimSet = true, type = "float", attributes = "BraceWeight"},
      {title = "ArmsPlacementWeight", perAnimSet = true, type = "float", attributes = "ArmsPlacementWeight"},
      {title = "Close movement sensitivity", perAnimSet = false, type = "float", attributes = "SensitivityToCloseMovements"},
      {title = "CrouchDownAmount", perAnimSet = false, type = "float", attributes = "CrouchDownAmount"},
      {title = "CrouchPitchAmount", perAnimSet = false, type = "float", attributes = "CrouchPitchAmount"},
      {title = "SupportIgnoreRadius", perAnimSet = true, type = "float", attributes = "SupportIgnoreRadius"},
    },
  },
  {
    title = "Arm Swing",
    helptext = "Configure the use of arm swing to avoid hazards by altering the chest's velocity.",
    isExpanded = false,
    details = 
    {
      {title = "Swing weight", perAnimSet = true, type = "float", attributes = "ArmsSwingWeight"},
      {title = "SwivelAmount", perAnimSet = true, type = "float", attributes = "SwivelAmount"},
      {title = "ArmExtension", perAnimSet = true, type = "float", attributes = "MaxArmExtensionScale"},
    },
  },
  {
    title = "Leg Swing",
    helptext = "Configure the use of leg swing to avoid hazards by altering the pelvis' velocity.",
    isExpanded = false,
    details = 
    {
      {title = "Swing weight", perAnimSet = true, type = "float", attributes = "LegsSwingWeight"},
    },
  },
  {
    title = "Input Defaults",
    isExpanded = false,
    details = 
    {
      {title = "OffsetFromChest", perAnimSet = false, type = "vector3", attributes = "OffsetFromChest"},
    },
  },
}

return {
  data = behaviourNode,
  attributeGroups = attributeGroups,
}
