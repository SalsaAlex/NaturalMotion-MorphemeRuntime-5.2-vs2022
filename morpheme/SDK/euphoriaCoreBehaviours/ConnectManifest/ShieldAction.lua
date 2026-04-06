--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "Shield Action",
  version = 2,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Makes the character form a shield pose against a specified hazard position.",
  group = "Behaviours",
  image = "ShieldActionBehaviour.png",
  id = generateNamespacedId(idNamespaces.NaturalMotion, 128),

  functionPins = 
  {
    ["ShieldPose"] = 
    {
      input = true,
      helptext = "The basic shield pose itself. If the character is supported,the shield pose will only be used for his heads and arms. The character will rotate with his spine to turn away from the hazard. If he is not supported, the poses are applied for the all body.",
      interfaces = 
      {
        "Transforms",
        "Time",
      },
    },

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
    ["Importance"] = 
    {
      input = true,
      helptext = "This determines whether shield happens or not. A value of 1 will force the character to shield.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Weight",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["HazardPos"] = 
    {
      input = true,
      helptext = "Position of the hazard to shield against in world space.",
      type = "vector3",
      displayName = "HazardPosition",
    },

    ["TurnAwayScale"] = 
    {
      input = true,
      helptext = "Scale used to set the amount of rotation to turn away from the hazard (0 - 1).",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "TurnAwayFromHazard",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["hazardAngle"] = 
    {
      input = false,
      helptext = "Angle in degrees towards the hazard. Zero means straight ahead, and +ve is to the right.",
      type = "float",
    },

  },

  pinOrder = 
  {
    "ShieldPose",
    "Importance",
    "HazardPos",
    "TurnAwayScale",
    "Result",
    "hazardAngle",
  },

  attributes = 
  {
    {
      name = "DefaultPitchAngle",
      type = "float",
      value = 15.000000,
      min = -180.000000,
      max = 180.000000,
      perAnimSet = true,
      helptext = "Pitch of the spine when the character stands up (in degrees).",
    },
    {
      name = "CrouchAmount",
      type = "float",
      value = 0.200000,
      min = 0.000000,
      max = 100.000000,
      perAnimSet = true,
      helptext = "Level of crouching, larger values will crouch more when shielding",
    },
    {
      name = "UseSingleFrameForShieldPose",
      type = "bool",
      value = false,
      perAnimSet = true,
      helptext = "If set then pose will be cached on the first update. This means that the pose that is used will not change if the input changes.",
    },
    {
      name = "SmoothReturnTimePeriod",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 100.000000,
      perAnimSet = true,
      helptext = "Minimum time period the character will continue to shield after shielding against a hazard. In seconds (standard character).",
    },
    {
      name = "ShieldPose_ApplyToArm_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Arm.",
    },
    {
      name = "ShieldPose_ApplyToArm_2",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Arm.",
    },
    {
      name = "ShieldPose_ApplyToHead_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Head.",
    },
    {
      name = "ShieldPose_ApplyToLeg_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Leg.",
    },
    {
      name = "ShieldPose_ApplyToLeg_2",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Leg.",
    },
    {
      name = "ShieldPose_ApplyToSpine_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Spine.",
    },
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(26, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(1, "numBehaviourInts")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "UseSingleFrameForShieldPose", asVal) and 1 ) or 0 ) , string.format("Int_0_%d", asIdx-1))
    end

    stream:writeUInt(9, "numBehaviourFloats")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "DefaultPitchAngle", asVal), string.format("Float_0_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "CrouchAmount", asVal), string.format("Float_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "SmoothReturnTimePeriod", asVal), string.format("Float_2_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ShieldPose_ApplyToArm_1"), string.format("Float_3_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ShieldPose_ApplyToArm_2"), string.format("Float_4_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ShieldPose_ApplyToHead_1"), string.format("Float_5_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ShieldPose_ApplyToLeg_1"), string.format("Float_6_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ShieldPose_ApplyToLeg_2"), string.format("Float_7_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ShieldPose_ApplyToSpine_1"), string.format("Float_8_%d", asIdx-1))
    end

    stream:writeUInt(0, "numBehaviourVector3s")

    stream:writeUInt(0, "numBehaviourUInt64s")

    -------------------------------------------------------------------

    local inputNodeID_ShieldPose = -1
    if isConnected{SourcePin = node .. ".ShieldPose", ResolveReferences = true} then
      inputNodeID_ShieldPose = getConnectedNodeID(node, "ShieldPose")
    end

    stream:writeUInt(1, "numBehaviourNodeAnimationInputs")

    stream:writeNetworkNodeId(inputNodeID_ShieldPose, "BehaviourNodeAnimationInput_0")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numInputCPInts")
    stream:writeUInt(2, "numInputCPFloats")
    stream:writeUInt(0, "numInputCPUInt64s")
    stream:writeUInt(1, "numInputCPVector3s")
    stream:writeUInt(3, "numBehaviourControlParameterInputs")

    writeDataPinAltName(stream, node, "Importance", "BehaviourControlParameterInputID_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_0")

    writeDataPinAltName(stream, node, "HazardPos", "BehaviourControlParameterInputID_1")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_1")

    writeDataPinAltName(stream, node, "TurnAwayScale", "BehaviourControlParameterInputID_2")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_2")

    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterOutputType_0")
    stream:writeUInt(1, "numBehaviourControlParameterOutputs")

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
    title = "Balance",
    isExpanded = true,
    details = 
    {
      {title = "SpinePitch", perAnimSet = true, type = "float", attributes = "DefaultPitchAngle"},
      {title = "CrouchAmount", perAnimSet = true, type = "float", attributes = "CrouchAmount"},
    },
  },
  {
    title = "Cache Pose",
    isExpanded = false,
    details = 
    {
      {title = "CacheFirstFrame", perAnimSet = true, type = "bool", attributes = "UseSingleFrameForShieldPose"},
    },
  },
  {
    title = "Shield",
    isExpanded = false,
    details = 
    {
      {title = "BlendOutDuration", perAnimSet = true, type = "float", attributes = "SmoothReturnTimePeriod"},
      {title = "TurnAwayFromHazard", perAnimSet = false, type = "float", attributes = "TurnAwayScale"},
    },
  },
  {
    title = "Input Defaults",
    isExpanded = false,
    details = 
    {
      {title = "Weight", perAnimSet = false, type = "float", attributes = "Importance"},
      {title = "HazardPosition", perAnimSet = false, type = "vector3", attributes = "HazardPos"},
    },
  },
  {
    title = "Pose",
    helptext = "The basic shield pose itself. If the character is supported,the shield pose will only be used for his heads and arms. The character will rotate with his spine to turn away from the hazard. If he is not supported, the poses are applied for the all body.",
    isExpanded = false,
    details = 
    {
      {title = "Arm {Arm0}", perAnimSet = false, type = "float", attributes = "ShieldPose_ApplyToArm_1"},
      {title = "Arm {Arm1}", perAnimSet = false, type = "float", attributes = "ShieldPose_ApplyToArm_2"},
      {title = "Head {Head0}", perAnimSet = false, type = "float", attributes = "ShieldPose_ApplyToHead_1"},
      {title = "Leg {Leg0}", perAnimSet = false, type = "float", attributes = "ShieldPose_ApplyToLeg_1"},
      {title = "Leg {Leg1}", perAnimSet = false, type = "float", attributes = "ShieldPose_ApplyToLeg_2"},
      {title = "Spine {Spine0}", perAnimSet = false, type = "float", attributes = "ShieldPose_ApplyToSpine_1"},
    },
  },
}

return {
  data = behaviourNode,
  attributeGroups = attributeGroups,
}
