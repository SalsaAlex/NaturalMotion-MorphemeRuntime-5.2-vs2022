--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "Sit",
  version = 2,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Drives the character toward a specifed sitting pose by using the feet, pelvis and arms to hold the spine in the pose.",
  group = "Behaviours",
  image = "SitBehaviour.png",
  id = generateNamespacedId(idNamespaces.NaturalMotion, 128),

  functionPins = 
  {
    ["Pose"] = 
    {
      input = true,
      helptext = "Target pose when sitting",
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
    ["SitAmount"] = 
    {
      input = false,
      helptext = "An indicator of how well the character is sitting from 0 to 1.",
      type = "float",
    },

  },

  pinOrder = 
  {
    "Pose",
    "Result",
    "SitAmount",
  },

  attributes = 
  {
    {
      name = "SupportWithArms",
      type = "bool",
      value = true,
      perAnimSet = true,
      helptext = "The arms will be used to help maintain a sitting position if this attribute is true.",
    },
    {
      name = "MinStandingBalanceAmount",
      type = "float",
      value = 0.500000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Only attempt to sit when the standing balance amount is below this threshold.",
    },
    {
      name = "MinSitAmount",
      type = "float",
      value = 0.100000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Don't try to maintain sitting balance when sitting amount is below this threshold.",
    },
    {
      name = "ArmStepHeight",
      type = "float",
      value = 0.300000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "The maximum height the hands will be raised to when stepping. Increase this to lift the hands further from the ground when stepping. In metres (standard character).",
    },
    {
      name = "ArmStepTargetExtrapolationTime",
      type = "float",
      value = 0.200000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "How far into the future should the chest's position be extrapolated to determine the hand placement target position. Increase this to move the hands further from the character when stepping. In seconds (standard character).",
    },
    {
      name = "ArmStepTargetSeparationFromBody",
      type = "float",
      value = 0.200000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Arm target is placed at least this far away from the characters body, as defined by the self avoidance radius. Increase this to move the hands further from the body when the character is stepping. In metres (standard character).",
    },
    {
      name = "UseSingleFrameForPose",
      type = "bool",
      value = false,
      perAnimSet = true,
    },
    {
      name = "Pose_ApplyToArm_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Arm.",
    },
    {
      name = "Pose_ApplyToArm_2",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Arm.",
    },
    {
      name = "Pose_ApplyToHead_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Head.",
    },
    {
      name = "Pose_ApplyToLeg_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Leg.",
    },
    {
      name = "Pose_ApplyToLeg_2",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Leg.",
    },
    {
      name = "Pose_ApplyToSpine_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Spine.",
    },
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(27, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(2, "numBehaviourInts")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "SupportWithArms", asVal) and 1 ) or 0 ) , string.format("Int_0_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "UseSingleFrameForPose", asVal) and 1 ) or 0 ) , string.format("Int_1_%d", asIdx-1))
    end

    stream:writeUInt(11, "numBehaviourFloats")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MinStandingBalanceAmount", asVal), string.format("Float_0_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MinSitAmount", asVal), string.format("Float_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ArmStepHeight", asVal), string.format("Float_2_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ArmStepTargetExtrapolationTime", asVal), string.format("Float_3_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ArmStepTargetSeparationFromBody", asVal), string.format("Float_4_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "Pose_ApplyToArm_1"), string.format("Float_5_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "Pose_ApplyToArm_2"), string.format("Float_6_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "Pose_ApplyToHead_1"), string.format("Float_7_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "Pose_ApplyToLeg_1"), string.format("Float_8_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "Pose_ApplyToLeg_2"), string.format("Float_9_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "Pose_ApplyToSpine_1"), string.format("Float_10_%d", asIdx-1))
    end

    stream:writeUInt(0, "numBehaviourVector3s")

    stream:writeUInt(0, "numBehaviourUInt64s")

    -------------------------------------------------------------------

    local inputNodeID_Pose = -1
    if isConnected{SourcePin = node .. ".Pose", ResolveReferences = true} then
      inputNodeID_Pose = getConnectedNodeID(node, "Pose")
    end

    stream:writeUInt(1, "numBehaviourNodeAnimationInputs")

    stream:writeNetworkNodeId(inputNodeID_Pose, "BehaviourNodeAnimationInput_0")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numInputCPInts")
    stream:writeUInt(0, "numInputCPFloats")
    stream:writeUInt(0, "numInputCPUInt64s")
    stream:writeUInt(0, "numInputCPVector3s")
    stream:writeUInt(0, "numBehaviourControlParameterInputs")

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
    title = "Supporting Limbs",
    isExpanded = true,
    details = 
    {
      {title = "SupportWithArms", perAnimSet = true, type = "bool", attributes = "SupportWithArms"},
    },
  },
  {
    title = "Activation",
    isExpanded = false,
    details = 
    {
      {title = "StartWhenBalanceAmountLessThan", perAnimSet = true, type = "float", attributes = "MinStandingBalanceAmount"},
      {title = "StopWhenSitAmountLessThan", perAnimSet = true, type = "float", attributes = "MinSitAmount"},
    },
  },
  {
    title = "Hand Stepping",
    helptext = "Configure how the behaviour repositions the hands when attempting to use them for support whilst sitting.",
    isExpanded = false,
    details = 
    {
      {title = "Height", perAnimSet = true, type = "float", attributes = "ArmStepHeight"},
      {title = "ExtrapolationTime", perAnimSet = true, type = "float", attributes = "ArmStepTargetExtrapolationTime"},
      {title = "SeparationFromBody", perAnimSet = true, type = "float", attributes = "ArmStepTargetSeparationFromBody"},
    },
  },
  {
    title = "Pose",
    helptext = "Target pose when sitting",
    isExpanded = false,
    details = 
    {
      {title = "CacheFirstFrame", perAnimSet = true, type = "bool", attributes = "UseSingleFrameForPose"},
      {title = "Arm {Arm0}", perAnimSet = false, type = "float", attributes = "Pose_ApplyToArm_1"},
      {title = "Arm {Arm1}", perAnimSet = false, type = "float", attributes = "Pose_ApplyToArm_2"},
      {title = "Head {Head0}", perAnimSet = false, type = "float", attributes = "Pose_ApplyToHead_1"},
      {title = "Leg {Leg0}", perAnimSet = false, type = "float", attributes = "Pose_ApplyToLeg_1"},
      {title = "Leg {Leg1}", perAnimSet = false, type = "float", attributes = "Pose_ApplyToLeg_2"},
      {title = "Spine {Spine0}", perAnimSet = false, type = "float", attributes = "Pose_ApplyToSpine_1"},
    },
  },
}

return {
  data = behaviourNode,
  attributeGroups = attributeGroups,
}
