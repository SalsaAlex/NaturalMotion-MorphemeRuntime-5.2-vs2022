--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "Animate",
  version = 3,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Animates specified limbs with the provided animation.",
  group = "Behaviours",
  image = "AnimateBehaviour.png",
  id = generateNamespacedId(idNamespaces.NaturalMotion, 128),

  functionPins = 
  {
    ["BodyPose"] = 
    {
      input = true,
      helptext = "Note that the body pose weight attributes are ignored and should not be displayed",
      mode = "Required",
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
    ["ArmStrength_0"] = 
    {
      input = true,
      helptext = "Scales the strength and weights the arm animation against other arm behaviours.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "ArmWeight {Arm0}",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

    ["ArmStrength_1"] = 
    {
      input = true,
      helptext = "Scales the strength and weights the arm animation against other arm behaviours.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "ArmWeight {Arm1}",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

    ["HeadStrength_0"] = 
    {
      input = true,
      helptext = "Scales the strength and weights the head animation against other head behaviours.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "HeadWeight {Head0}",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

    ["LegStrength_0"] = 
    {
      input = true,
      helptext = "Scales the strength and weights the leg animation against other leg behaviours.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "LegWeight {Leg0}",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

    ["LegStrength_1"] = 
    {
      input = true,
      helptext = "Scales the strength and weights the leg animation against other leg behaviours.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "LegWeight {Leg1}",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

    ["SpineStrength_0"] = 
    {
      input = true,
      helptext = "Scales the strength and weights the spine animation against other spine behaviours.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "SpineWeight",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

  },

  pinOrder = 
  {
    "BodyPose",
    "ArmStrength_0",
    "ArmStrength_1",
    "HeadStrength_0",
    "LegStrength_0",
    "LegStrength_1",
    "SpineStrength_0",
    "Result",
  },

  attributes = 
  {
    {
      name = "UseSingleFrameForPose",
      type = "bool",
      value = false,
      perAnimSet = true,
      helptext = "If set then pose will be cached on the first update. This means that the pose that is used will not change if the input changes.",
    },
    {
      name = "ArmsPriority",
      type = "bool",
      value = true,
      helptext = "If set then the animate behaviour will have priority over autonomous behaviours for all the arms",
    },
    {
      name = "HeadsPriority",
      type = "bool",
      value = true,
      helptext = "If set then the animate behaviour will have priority over autonomous behaviours for all the heads",
    },
    {
      name = "LegsPriority",
      type = "bool",
      value = true,
      helptext = "If set then the animate behaviour will have priority over autonomous behaviours for all the legs",
    },
    {
      name = "SpinesPriority",
      type = "bool",
      value = true,
      helptext = "If set then the animate behaviour will have priority over autonomous behaviours for the spine",
    },
    {
      name = "DisableSleeping",
      type = "bool",
      value = true,
      helptext = "False allows character to sleep when velocity is below a threshold, for performance. Only set to true if the animation won't change after becoming stationary, since a sleeping character will not auto-awaken when animating",
    },
    {
      name = "ArmGravityCompensation_0",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 2.000000,
      helptext = "Gravity compentstation scale for the arm.",
    },
    {
      name = "ArmGravityCompensation_1",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 2.000000,
      helptext = "Gravity compentstation scale for the arm.",
    },
    {
      name = "HeadGravityCompensation_0",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 2.000000,
      helptext = "Gravity compentstation scale for the head.",
    },
    {
      name = "LegGravityCompensation_0",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 2.000000,
      helptext = "Gravity compentstation scale for the leg.",
    },
    {
      name = "LegGravityCompensation_1",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 2.000000,
      helptext = "Gravity compentstation scale for the leg.",
    },
    {
      name = "SpineGravityCompensation_0",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 2.000000,
      helptext = "Gravity compentstation scale for the spine.",
    },
    {
      name = "BodyPose_ApplyToArm_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Arm.",
    },
    {
      name = "BodyPose_ApplyToArm_2",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Arm.",
    },
    {
      name = "BodyPose_ApplyToHead_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Head.",
    },
    {
      name = "BodyPose_ApplyToLeg_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Leg.",
    },
    {
      name = "BodyPose_ApplyToLeg_2",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Leg.",
    },
    {
      name = "BodyPose_ApplyToSpine_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Spine.",
    },
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(2, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(6, "numBehaviourInts")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "UseSingleFrameForPose", asVal) and 1 ) or 0 ) , string.format("Int_0_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "ArmsPriority") and 1 ) or 0 ) , string.format("Int_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "HeadsPriority") and 1 ) or 0 ) , string.format("Int_2_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "LegsPriority") and 1 ) or 0 ) , string.format("Int_3_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "SpinesPriority") and 1 ) or 0 ) , string.format("Int_4_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "DisableSleeping") and 1 ) or 0 ) , string.format("Int_5_%d", asIdx-1))
    end

    stream:writeUInt(12, "numBehaviourFloats")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ArmGravityCompensation_0"), string.format("Float_0_%d", asIdx-1))
      stream:writeFloat(getValue(node, "ArmGravityCompensation_1"), string.format("Float_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "HeadGravityCompensation_0"), string.format("Float_2_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "LegGravityCompensation_0"), string.format("Float_3_%d", asIdx-1))
      stream:writeFloat(getValue(node, "LegGravityCompensation_1"), string.format("Float_4_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "SpineGravityCompensation_0"), string.format("Float_5_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "BodyPose_ApplyToArm_1"), string.format("Float_6_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "BodyPose_ApplyToArm_2"), string.format("Float_7_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "BodyPose_ApplyToHead_1"), string.format("Float_8_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "BodyPose_ApplyToLeg_1"), string.format("Float_9_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "BodyPose_ApplyToLeg_2"), string.format("Float_10_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "BodyPose_ApplyToSpine_1"), string.format("Float_11_%d", asIdx-1))
    end

    stream:writeUInt(0, "numBehaviourVector3s")

    stream:writeUInt(0, "numBehaviourUInt64s")

    -------------------------------------------------------------------

    local inputNodeID_BodyPose = -1
    if isConnected{SourcePin = node .. ".BodyPose", ResolveReferences = true} then
      inputNodeID_BodyPose = getConnectedNodeID(node, "BodyPose")
    end

    stream:writeUInt(1, "numBehaviourNodeAnimationInputs")

    stream:writeNetworkNodeId(inputNodeID_BodyPose, "BehaviourNodeAnimationInput_0")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numInputCPInts")
    stream:writeUInt(6, "numInputCPFloats")
    stream:writeUInt(0, "numInputCPUInt64s")
    stream:writeUInt(0, "numInputCPVector3s")
    stream:writeUInt(6, "numBehaviourControlParameterInputs")

    writeDataPinAltName(stream, node, "ArmStrength_0", "BehaviourControlParameterInputID_0" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_0")
    writeDataPinAltName(stream, node, "ArmStrength_1", "BehaviourControlParameterInputID_1" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_1")

    writeDataPinAltName(stream, node, "HeadStrength_0", "BehaviourControlParameterInputID_2" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_2")

    writeDataPinAltName(stream, node, "LegStrength_0", "BehaviourControlParameterInputID_3" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_3")
    writeDataPinAltName(stream, node, "LegStrength_1", "BehaviourControlParameterInputID_4" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_4")

    writeDataPinAltName(stream, node, "SpineStrength_0", "BehaviourControlParameterInputID_5" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_5")

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
    title = "Animation",
    isExpanded = true,
    details = 
    {
      {title = "CacheFirstFrame", perAnimSet = true, type = "bool", attributes = "UseSingleFrameForPose"},
    },
  },
  {
    title = "Animation Priority",
    helptext = "These determine whether the animate behaviour will have priority over autonomous behaviours, or if it will operate in the background and be easily over-ridden by other behaviours when they need to be active.",
    isExpanded = false,
    details = 
    {
      {title = "Arms", perAnimSet = false, type = "bool", attributes = "ArmsPriority"},
      {title = "Heads", perAnimSet = false, type = "bool", attributes = "HeadsPriority"},
      {title = "Legs", perAnimSet = false, type = "bool", attributes = "LegsPriority"},
      {title = "Spine", perAnimSet = false, type = "bool", attributes = "SpinesPriority"},
    },
  },
  {
    title = "Default",
    isExpanded = false,
    details = 
    {
      {title = "DisableSleeping", perAnimSet = false, type = "bool", attributes = "DisableSleeping"},
    },
  },
  {
    title = "Gravity Compensation",
    helptext = "Gravity compensation is used to compensate for the sag of limbs due to gravity. In order for this to operate the limb needs to be supported at its root or end. This can happen if, for example, balance or hard keyframing is used on the lower body, and the animate behaviour is used on the upper body. Gravity compensation values above one should only normally be used if you want full gravity compensation with a limb that has got strength less than one.",
    isExpanded = false,
    details = 
    {
      {title = "Arm {Arm0}", perAnimSet = false, type = "float", attributes = "ArmGravityCompensation_0"},
      {title = "Arm {Arm1}", perAnimSet = false, type = "float", attributes = "ArmGravityCompensation_1"},
      {title = "Head {Head0}", perAnimSet = false, type = "float", attributes = "HeadGravityCompensation_0"},
      {title = "Leg {Leg0}", perAnimSet = false, type = "float", attributes = "LegGravityCompensation_0"},
      {title = "Leg {Leg1}", perAnimSet = false, type = "float", attributes = "LegGravityCompensation_1"},
      {title = "Spine ", perAnimSet = false, type = "float", attributes = "SpineGravityCompensation_0"},
    },
  },
  {
    title = "Input Strength Defaults",
    helptext = "These values act to scale both the physical strength, and the importance/weight of the animate behaviour against other behaviours that may be trying to use the limb. For example, a small (non zero) value on the arms will allow the arms to follow the input animation loosely, with influences form other behaviours.",
    isExpanded = false,
    details = 
    {
      {title = "ArmWeight {Arm0}", perAnimSet = false, type = "float", attributes = "ArmStrength_0"},
      {title = "ArmWeight {Arm1}", perAnimSet = false, type = "float", attributes = "ArmStrength_1"},
      {title = "HeadWeight {Head0}", perAnimSet = false, type = "float", attributes = "HeadStrength_0"},
      {title = "LegWeight {Leg0}", perAnimSet = false, type = "float", attributes = "LegStrength_0"},
      {title = "LegWeight {Leg1}", perAnimSet = false, type = "float", attributes = "LegStrength_1"},
      {title = "SpineWeight ", perAnimSet = false, type = "float", attributes = "SpineStrength_0"},
    },
  },
  {
    title = "BodyPose",
    helptext = "Note that the body pose weight attributes are ignored and should not be displayed",
    isExpanded = false,
    details = 
    {
      {title = "Arm {Arm0}", perAnimSet = false, type = "float", attributes = "BodyPose_ApplyToArm_1"},
      {title = "Arm {Arm1}", perAnimSet = false, type = "float", attributes = "BodyPose_ApplyToArm_2"},
      {title = "Head {Head0}", perAnimSet = false, type = "float", attributes = "BodyPose_ApplyToHead_1"},
      {title = "Leg {Leg0}", perAnimSet = false, type = "float", attributes = "BodyPose_ApplyToLeg_1"},
      {title = "Leg {Leg1}", perAnimSet = false, type = "float", attributes = "BodyPose_ApplyToLeg_2"},
      {title = "Spine {Spine0}", perAnimSet = false, type = "float", attributes = "BodyPose_ApplyToSpine_1"},
    },
  },
}

return {
  data = behaviourNode,
  attributeGroups = attributeGroups,
}
