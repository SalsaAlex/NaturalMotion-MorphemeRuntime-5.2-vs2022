--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "Shield",
  version = 4,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Allows a character to try to protect its main head and chest from extreme danger by taking a shield pose, just like a boxer being hit. ",
  group = "Behaviours",
  image = "ShieldBehaviour.png",
  id = generateNamespacedId(idNamespaces.NaturalMotion, 128),

  functionPins = 
  {
    ["ShieldPose"] = 
    {
      input = true,
      helptext = "The basic shield pose itself.",
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
    ["TurnAwayScale"] = 
    {
      input = true,
      helptext = "Sets the amount the character can turn away from a hazard. Accepts values in the range 0 to 1.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "TurnAwayFromHazard",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["isShielding"] = 
    {
      input = false,
      helptext = "1 if we are shielding, 0 otherwise",
      type = "float",
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
    "TurnAwayScale",
    "Result",
    "isShielding",
    "hazardAngle",
  },

  attributes = 
  {
    {
      name = "ImpactResponseForShield",
      type = "float",
      value = 7.000000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "Shield will be triggered if the character will change speed by more than this on impact, based on the relative masses. In m/s (standard character).",
    },
    {
      name = "AngularSpeedForShield",
      type = "float",
      value = 1.500000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "The character will shield if he predicts an impact with a hazard and he is rotating faster than this. In revolutions/second (standard character).",
    },
    {
      name = "TangentialSpeedForShield",
      type = "float",
      value = 6.000000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "The character will shield if his predicted tangential velocity at the time of impact with a hazard is greater than this. In m/s (standard character).",
    },
    {
      name = "SmoothReturnTimePeriod",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "Time period the character will need to completely stop shield. In seconds (standard character).",
    },
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
      name = "crouchAmount",
      type = "float",
      value = 0.200000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Level of crouching, larger values will crouch more when shielding",
    },
    {
      name = "UseSingleFrameForShieldPoses",
      type = "bool",
      value = false,
      perAnimSet = true,
      helptext = "If set then pose will be cached on the first update. This means that the pose that is used will not change if the input changes.",
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
    stream:writeInt(25, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(1, "numBehaviourInts")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "UseSingleFrameForShieldPoses", asVal) and 1 ) or 0 ) , string.format("Int_0_%d", asIdx-1))
    end

    stream:writeUInt(12, "numBehaviourFloats")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ImpactResponseForShield", asVal), string.format("Float_0_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "AngularSpeedForShield", asVal), string.format("Float_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "TangentialSpeedForShield", asVal), string.format("Float_2_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "SmoothReturnTimePeriod", asVal), string.format("Float_3_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "DefaultPitchAngle", asVal), string.format("Float_4_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "crouchAmount", asVal), string.format("Float_5_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ShieldPose_ApplyToArm_1"), string.format("Float_6_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ShieldPose_ApplyToArm_2"), string.format("Float_7_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ShieldPose_ApplyToHead_1"), string.format("Float_8_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ShieldPose_ApplyToLeg_1"), string.format("Float_9_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ShieldPose_ApplyToLeg_2"), string.format("Float_10_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ShieldPose_ApplyToSpine_1"), string.format("Float_11_%d", asIdx-1))
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
    stream:writeUInt(1, "numInputCPFloats")
    stream:writeUInt(0, "numInputCPUInt64s")
    stream:writeUInt(0, "numInputCPVector3s")
    stream:writeUInt(1, "numBehaviourControlParameterInputs")

    writeDataPinAltName(stream, node, "TurnAwayScale", "BehaviourControlParameterInputID_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_0")

    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterOutputType_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterOutputType_1")
    stream:writeUInt(2, "numBehaviourControlParameterOutputs")

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
    title = "Shield",
    isExpanded = true,
    details = 
    {
      {title = "ImpactResponseIsGreaterThan", perAnimSet = true, type = "float", attributes = "ImpactResponseForShield"},
      {title = "SpinSpeedIsGreaterThan", perAnimSet = true, type = "float", attributes = "AngularSpeedForShield"},
      {title = "TangentialSpeedIsGreaterThan", perAnimSet = true, type = "float", attributes = "TangentialSpeedForShield"},
      {title = "BlendOutDuration", perAnimSet = true, type = "float", attributes = "SmoothReturnTimePeriod"},
    },
  },
  {
    title = "Balance",
    isExpanded = false,
    details = 
    {
      {title = "SpinePitch", perAnimSet = true, type = "float", attributes = "DefaultPitchAngle"},
      {title = "CrouchAmount", perAnimSet = true, type = "float", attributes = "crouchAmount"},
    },
  },
  {
    title = "Cache Pose",
    isExpanded = false,
    details = 
    {
      {title = "CacheFirstFrame", perAnimSet = true, type = "bool", attributes = "UseSingleFrameForShieldPoses"},
    },
  },
  {
    title = "Input Defaults",
    isExpanded = false,
    details = 
    {
      {title = "TurnAwayFromHazard", perAnimSet = false, type = "float", attributes = "TurnAwayScale"},
    },
  },
  {
    title = "Pose",
    helptext = "The basic shield pose itself.",
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
