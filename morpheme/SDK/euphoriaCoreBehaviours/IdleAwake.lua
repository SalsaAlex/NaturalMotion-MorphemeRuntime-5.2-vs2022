--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "Idle Awake",
  version = 3,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Gives a character an awareness of the scene and the appearance of an awake state, as though they were waiting.  This is an \"always on\"/background behaviour that should quietly provide low-importance outputs to make  the character look as if he is awake, even if he has nothing to actually do. This might involve getting  into a comfy pose where possible, and idly looking around. It will also include modules (if they exist)  that simply provide \"alive\" behaviour - such as breathing or reflexes.",
  group = "Behaviours",
  image = "IdleAwakeBehaviour.png",
  id = generateNamespacedId(idNamespaces.NaturalMotion, 128),

  functionPins = 
  {
    ["FallenPose"] = 
    {
      input = true,
      helptext = "Sets the animation pose that should be adopted by the character when unsupported and lying on its back.",
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
    ["WholeBodyLook"] = 
    {
      input = true,
      helptext = "Sets how much of the body to move when looking. The pin accepts a value in the range 0 to 1.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "MoveBody",
      value = 0.250000,
      min = 0.000000,
      max = 1.000000,
    },

  },

  pinOrder = 
  {
    "FallenPose",
    "WholeBodyLook",
    "Result",
  },

  attributes = 
  {
    {
      name = "FwdRange",
      type = "float",
      value = 0.000000,
      perAnimSet = true,
      helptext = "Amount the Center of Mass may drift forwards, distance in metres (standard character).",
    },
    {
      name = "BackRange",
      type = "float",
      value = 0.000000,
      perAnimSet = true,
      helptext = "Amount the Center of Mass may drift backwards, distance in metres (standard character).",
    },
    {
      name = "LeftRange",
      type = "float",
      value = 0.000000,
      perAnimSet = true,
      helptext = "Amount the Center of Mass may drift left, distance in metres (standard character).",
    },
    {
      name = "RightRange",
      type = "float",
      value = 0.000000,
      perAnimSet = true,
      helptext = "Amount the Center of Mass may drift right, distance in metres (standard character).",
    },
    {
      name = "StanceChangeTime",
      type = "float",
      value = 0.000000,
      perAnimSet = true,
      helptext = "How often to choose a new stance, in seconds (standard character).",
    },
    {
      name = "PoseAdjustTime",
      type = "float",
      value = 0.000000,
      perAnimSet = true,
      helptext = "Time period over which to adjust to the new stance, in seconds (standard character).",
    },
    {
      name = "LookTimescale",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 10.000000,
      perAnimSet = true,
      helptext = "Scales how long to look at each target, so smaller values will result in the character changing his look target more frequently.",
    },
    {
      name = "LookRangeUpDown",
      type = "float",
      value = 30.000000,
      min = 0.000000,
      max = 90.000000,
      perAnimSet = true,
      helptext = "Range of angles to look up/down (degrees)",
    },
    {
      name = "LookRangeSideways",
      type = "float",
      value = 90.000000,
      min = 0.000000,
      max = 180.000000,
      perAnimSet = true,
      helptext = "Range of positions to look at sideways (degrees)",
    },
    {
      name = "LookStrengthWhenSupported",
      type = "float",
      value = 0.500000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Look importance when balancing",
    },
    {
      name = "LookStrengthWhenUnsupported",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Look importance when fallen",
    },
    {
      name = "LookTransitionTime",
      type = "float",
      value = 0.500000,
      min = 0.000000,
      max = 2.000000,
      perAnimSet = true,
      helptext = "Controls how slowly/smoothly the character switches between look targets. In seconds (standard character).",
    },
    {
      name = "LookFocusDistance",
      type = "float",
      value = 5.000000,
      min = 0.300000,
      max = 20.000000,
      perAnimSet = true,
      helptext = "Distance at which to focus. Affects what is noticed when running autonomous behaviours in parallel. In metres (standard character).",
    },
    {
      name = "LookVerticalOffset",
      type = "float",
      value = -0.800000,
      min = -2.000000,
      max = 2.000000,
      perAnimSet = true,
      helptext = "Positive will look vertically higher, negative will look down. In metres (standard character).",
    },
    {
      name = "FallenPose_ApplyToArm_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Arm.",
    },
    {
      name = "FallenPose_ApplyToArm_2",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Arm.",
    },
    {
      name = "FallenPose_ApplyToHead_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Head.",
    },
    {
      name = "FallenPose_ApplyToLeg_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Leg.",
    },
    {
      name = "FallenPose_ApplyToLeg_2",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Leg.",
    },
    {
      name = "FallenPose_ApplyToSpine_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Spine.",
    },
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(17, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(0, "numBehaviourInts")

    stream:writeUInt(20, "numBehaviourFloats")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "FwdRange", asVal), string.format("Float_0_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "BackRange", asVal), string.format("Float_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "LeftRange", asVal), string.format("Float_2_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "RightRange", asVal), string.format("Float_3_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "StanceChangeTime", asVal), string.format("Float_4_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "PoseAdjustTime", asVal), string.format("Float_5_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "LookTimescale", asVal), string.format("Float_6_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "LookRangeUpDown", asVal), string.format("Float_7_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "LookRangeSideways", asVal), string.format("Float_8_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "LookStrengthWhenSupported", asVal), string.format("Float_9_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "LookStrengthWhenUnsupported", asVal), string.format("Float_10_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "LookTransitionTime", asVal), string.format("Float_11_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "LookFocusDistance", asVal), string.format("Float_12_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "LookVerticalOffset", asVal), string.format("Float_13_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "FallenPose_ApplyToArm_1"), string.format("Float_14_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "FallenPose_ApplyToArm_2"), string.format("Float_15_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "FallenPose_ApplyToHead_1"), string.format("Float_16_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "FallenPose_ApplyToLeg_1"), string.format("Float_17_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "FallenPose_ApplyToLeg_2"), string.format("Float_18_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "FallenPose_ApplyToSpine_1"), string.format("Float_19_%d", asIdx-1))
    end

    stream:writeUInt(0, "numBehaviourVector3s")

    stream:writeUInt(0, "numBehaviourUInt64s")

    -------------------------------------------------------------------

    local inputNodeID_FallenPose = -1
    if isConnected{SourcePin = node .. ".FallenPose", ResolveReferences = true} then
      inputNodeID_FallenPose = getConnectedNodeID(node, "FallenPose")
    end

    stream:writeUInt(1, "numBehaviourNodeAnimationInputs")

    stream:writeNetworkNodeId(inputNodeID_FallenPose, "BehaviourNodeAnimationInput_0")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numInputCPInts")
    stream:writeUInt(1, "numInputCPFloats")
    stream:writeUInt(0, "numInputCPUInt64s")
    stream:writeUInt(0, "numInputCPVector3s")
    stream:writeUInt(1, "numBehaviourControlParameterInputs")

    writeDataPinAltName(stream, node, "WholeBodyLook", "BehaviourControlParameterInputID_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_0")

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
    title = "Balance",
    helptext = "Settings that define how the balance pose varies over time.",
    isExpanded = true,
    details = 
    {
      {title = "Front", perAnimSet = true, type = "float", attributes = "FwdRange"},
      {title = "Back", perAnimSet = true, type = "float", attributes = "BackRange"},
      {title = "Left", perAnimSet = true, type = "float", attributes = "LeftRange"},
      {title = "Right", perAnimSet = true, type = "float", attributes = "RightRange"},
      {title = "ChangeStanceEvery", perAnimSet = true, type = "float", attributes = "StanceChangeTime"},
      {title = "ChangeLasts", perAnimSet = true, type = "float", attributes = "PoseAdjustTime"},
    },
  },
  {
    title = "Look",
    helptext = "Settings that define how the character changes its gaze over time.",
    isExpanded = false,
    details = 
    {
      {title = "TargetDurationScale", perAnimSet = true, type = "float", attributes = "LookTimescale"},
      {title = "TargetRange(Up/Down)", perAnimSet = true, type = "float", attributes = "LookRangeUpDown"},
      {title = "TargetRange(Left/Right)", perAnimSet = true, type = "float", attributes = "LookRangeSideways"},
      {title = "Weight(WhenBalancing)", perAnimSet = true, type = "float", attributes = "LookStrengthWhenSupported"},
      {title = "Weight(WhenFallen)", perAnimSet = true, type = "float", attributes = "LookStrengthWhenUnsupported"},
      {title = "LookTransitionTime", perAnimSet = true, type = "float", attributes = "LookTransitionTime"},
      {title = "LookFocusDistance", perAnimSet = true, type = "float", attributes = "LookFocusDistance"},
      {title = "LookVerticalOffset", perAnimSet = true, type = "float", attributes = "LookVerticalOffset"},
    },
  },
  {
    title = "Input Defaults",
    isExpanded = false,
    details = 
    {
      {title = "MoveBody", perAnimSet = false, type = "float", attributes = "WholeBodyLook"},
    },
  },
  {
    title = "Fallen Pose Weight",
    helptext = "Sets the animation pose that should be adopted by the character when unsupported and lying on its back.",
    isExpanded = false,
    details = 
    {
      {title = "Arm {Arm0}", perAnimSet = false, type = "float", attributes = "FallenPose_ApplyToArm_1"},
      {title = "Arm {Arm1}", perAnimSet = false, type = "float", attributes = "FallenPose_ApplyToArm_2"},
      {title = "Head {Head0}", perAnimSet = false, type = "float", attributes = "FallenPose_ApplyToHead_1"},
      {title = "Leg {Leg0}", perAnimSet = false, type = "float", attributes = "FallenPose_ApplyToLeg_1"},
      {title = "Leg {Leg1}", perAnimSet = false, type = "float", attributes = "FallenPose_ApplyToLeg_2"},
      {title = "Spine {Spine0}", perAnimSet = false, type = "float", attributes = "FallenPose_ApplyToSpine_1"},
    },
  },
}

return {
  data = behaviourNode,
  attributeGroups = attributeGroups,
}
