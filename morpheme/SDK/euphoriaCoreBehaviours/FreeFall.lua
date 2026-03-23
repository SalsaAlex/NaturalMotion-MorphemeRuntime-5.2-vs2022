--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "Free Fall",
  version = 9,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Allows a character to try to keep a desired orientation when it is in the air by spinning its arms and legs. ",
  group = "Behaviours",
  image = "FreeFallBehaviour.png",
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
    ["orientation"] = 
    {
      input = true,
      helptext = "A rotation vector specifiying an offset to the target orientation for the character's pelvis relative to the orientation defined in \"Orientation Configuration\".",
      type = "vector3",
      mode = "HiddenInNetworkEditor",
      displayName = "Pelvis orientation",
      value = {x=0.000000, y=0.000000,z=0.000000},
    },

    ["weight"] = 
    {
      input = true,
      helptext = "Multiplier applied to the strength with which the character's orientation is driven.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "weight",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["disableWhenInContact"] = 
    {
      input = true,
      helptext = "If true, then free fall will not attempt to orientate the character when it's in contact.",
      type = "bool",
      mode = "HiddenInNetworkEditor",
      displayName = "disableWhenInContact",
      value = true,
    },

    ["orientationError"] = 
    {
      input = false,
      helptext = "Angle between the current and target orientation of the character, in degrees.",
      type = "float",
    },

  },

  pinOrder = 
  {
    "orientation",
    "weight",
    "disableWhenInContact",
    "Result",
    "orientationError",
  },

  attributes = 
  {
    {
      name = "StartOrientationTime",
      type = "float",
      value = 0.200000,
      min = 0.000000,
      helptext = "The time to start driving the character's orientation, in seconds (standard character).",
    },
    {
      name = "StopOrientationTime",
      type = "float",
      value = 0.400000,
      min = 0.000000,
      helptext = "The time to stop driving the character's orientation, in seconds (standard character).",
    },
    {
      name = "StartOrientationAtTimeBeforeImpact",
      type = "bool",
      value = false,
      helptext = "The StartOrientationTime will be interpreted as time before impact if this is true, else it is interpreted as time after entering free fall.",
    },
    {
      name = "StopOrientationAtTimeBeforeImpact",
      type = "bool",
      value = true,
      helptext = "The StopOrientationTime will be interpreted as time before impact if this is true, else it is interpreted as time after entering free fall.",
    },
    {
      name = "AssistanceAmount",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 10.000000,
      perAnimSet = true,
      helptext = "Amount of externally applied torque used to achieve the desired orientation. Unitless. No need to alter for larger or heavier characters",
    },
    {
      name = "DampingAmount",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 2.000000,
      perAnimSet = true,
      helptext = "Amount of externally applied torque used to damp existing rotation (tumbling). Unitless. No need to alter for larger or heavier characters",
    },
    {
      name = "RotationTime",
      type = "float",
      value = 0.500000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "Sets the amount of time in which to achieve the rotation, in seconds (standard character). Smaller values lead to faster movements. For times under about 0.5s the arms will swing rather than spin.",
    },
    {
      name = "ArmsAmount",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
      perAnimSet = true,
      helptext = "The amount to use the arms - when spinning or swinging",
    },
    {
      name = "LegsAmount",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
      perAnimSet = true,
      helptext = "The amount to use the legs - when spinning or swinging",
    },
    {
      name = "ArmsSpinAmount_0",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
      helptext = "Affects how strongly a request to spin the arms is applied. 1 is normal or average movement, 0 has no effect, the spin is turned off.",
    },
    {
      name = "ArmsSpinAmount_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
      helptext = "Affects how strongly a request to spin the arms is applied. 1 is normal or average movement, 0 has no effect, the spin is turned off.",
    },
    {
      name = "ArmsInPhase",
      type = "bool",
      value = true,
      helptext = "If true the the arms will move in phase.",
    },
    {
      name = "LegsSpinAmount_0",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
      helptext = "Affects how strongly a request to spin the legs is applied. 1 is normal or average movement, 0 has no effect, the swing is turned off.",
    },
    {
      name = "LegsSpinAmount_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
      helptext = "Affects how strongly a request to spin the legs is applied. 1 is normal or average movement, 0 has no effect, the swing is turned off.",
    },
    {
      name = "characterAxis0",
      type = "int",
      value = 1,
      min = 0,
      max = 6,
      helptext = "The primary axis local to the character that should be alligned to the primary target axis.",
    },
    {
      name = "characterAxis1",
      type = "int",
      value = 0,
      min = 0,
      max = 6,
      helptext = "The secondary axis local to the character that should be alligned to the secondary target axis.",
    },
    {
      name = "targetAxis0",
      type = "int",
      value = 2,
      min = 0,
      max = 3,
      helptext = "The primary world space axis with which to align the primary character axis.\n1. Velocity: The direction of movement of the Character's hips in world space.\n2. Landing Up: The best alignment for the character's up direction for a successful landing.\n3. World Up: The \"up\" direction of the scene, in world space.",
    },
    {
      name = "targetAxis1",
      type = "int",
      value = 0,
      min = 0,
      max = 3,
      helptext = "The secondary world space axis with which to align the secondary character axis. \n1. Velocity: The direction of movement of the Character's hips in world space. \n2. Landing Up: The best alignment for the character's up direction for a successful landing. \n3. World Up: The \"up\" direction of the scene, in world space.",
    },
    {
      name = "secondaryDirectionThreshold",
      type = "float",
      value = 0.900000,
      min = 0.000000,
      max = 1.000000,
      helptext = "The secondary axis is ignored if the dot product between the primary and secondary axis is greater than this value. This is useful for preventing undesired rotation around the primary axis in order to face the tiny perpendicular component of an almost parallel secondary axis.",
    },
    {
      name = "AngleLandingAmount",
      type = "float",
      value = 0.500000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "How much to angle the landing to match the lateral velocity of the character. 0 will land vertically, 1 will land oriented in the direction of velocity, like an arrow when it sticks into the ground where it lands. This is only relevant if the \"Landing Up\" target axis is used in the Base Orientation configuration.",
    },
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(11, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(7, "numBehaviourInts")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "StartOrientationAtTimeBeforeImpact") and 1 ) or 0 ) , string.format("Int_0_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "StopOrientationAtTimeBeforeImpact") and 1 ) or 0 ) , string.format("Int_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "ArmsInPhase") and 1 ) or 0 ) , string.format("Int_2_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt(getValue(node, "characterAxis0"), string.format("Int_3_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt(getValue(node, "characterAxis1"), string.format("Int_4_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt(getValue(node, "targetAxis0"), string.format("Int_5_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt(getValue(node, "targetAxis1"), string.format("Int_6_%d", asIdx-1))
    end

    stream:writeUInt(13, "numBehaviourFloats")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "StartOrientationTime"), string.format("Float_0_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "StopOrientationTime"), string.format("Float_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "AssistanceAmount", asVal), string.format("Float_2_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "DampingAmount", asVal), string.format("Float_3_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "RotationTime", asVal), string.format("Float_4_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ArmsAmount", asVal), string.format("Float_5_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "LegsAmount", asVal), string.format("Float_6_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ArmsSpinAmount_0"), string.format("Float_7_%d", asIdx-1))
      stream:writeFloat(getValue(node, "ArmsSpinAmount_1"), string.format("Float_8_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "LegsSpinAmount_0"), string.format("Float_9_%d", asIdx-1))
      stream:writeFloat(getValue(node, "LegsSpinAmount_1"), string.format("Float_10_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "secondaryDirectionThreshold"), string.format("Float_11_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "AngleLandingAmount", asVal), string.format("Float_12_%d", asIdx-1))
    end

    stream:writeUInt(0, "numBehaviourVector3s")

    stream:writeUInt(0, "numBehaviourUInt64s")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numBehaviourNodeAnimationInputs")

    -------------------------------------------------------------------

    stream:writeUInt(1, "numInputCPInts")
    stream:writeUInt(1, "numInputCPFloats")
    stream:writeUInt(0, "numInputCPUInt64s")
    stream:writeUInt(1, "numInputCPVector3s")
    stream:writeUInt(3, "numBehaviourControlParameterInputs")

    writeDataPinAltName(stream, node, "orientation", "BehaviourControlParameterInputID_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_0")

    writeDataPinAltName(stream, node, "weight", "BehaviourControlParameterInputID_1")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_1")

    writeDataPinAltName(stream, node, "disableWhenInContact", "BehaviourControlParameterInputID_2")
    stream:writeString("ATTRIB_SEMANTIC_CP_BOOL", "BehaviourControlParameterInputType_2")

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
    title = "When To Rotate",
    helptext = "Settings that control when, during free fall, the orientation of the character should be controlled.",
    isExpanded = true,
    details = 
    {
      {title = "StartOrientationTime", perAnimSet = false, type = "float", attributes = "StartOrientationTime"},
      {title = "StopOrientationTime", perAnimSet = false, type = "float", attributes = "StopOrientationTime"},
      {title = "StartOrientationAtTimeBeforeImpact", perAnimSet = false, type = "bool", attributes = "StartOrientationAtTimeBeforeImpact"},
      {title = "StopOrientationAtTimeBeforeImpact", perAnimSet = false, type = "bool", attributes = "StopOrientationAtTimeBeforeImpact"},
    },
  },
  {
    title = "Orientation Control",
    helptext = "Settings that control when position of the body during free fall",
    isExpanded = false,
    details = 
    {
      {title = "Assistance", perAnimSet = true, type = "float", attributes = "AssistanceAmount"},
      {title = "Rotation damping", perAnimSet = true, type = "float", attributes = "DampingAmount"},
      {title = "RotationTime", perAnimSet = true, type = "float", attributes = "RotationTime"},
      {title = "Arm Amount", perAnimSet = true, type = "float", attributes = "ArmsAmount"},
      {title = "Leg Amount", perAnimSet = true, type = "float", attributes = "LegsAmount"},
    },
  },
  {
    title = "Arm Windmill",
    helptext = "Settings that control how the arms windmill in free fall",
    isExpanded = false,
    details = 
    {
      {title = "Spin Amount {Arm0}", perAnimSet = false, type = "float", attributes = "ArmsSpinAmount_0"},
      {title = "Spin Amount {Arm1}", perAnimSet = false, type = "float", attributes = "ArmsSpinAmount_1"},
      {title = "In phase", perAnimSet = false, type = "bool", attributes = "ArmsInPhase"},
    },
  },
  {
    title = "Leg Spin",
    helptext = "Settings that control how the legs spin in free fall",
    isExpanded = false,
    details = 
    {
      {title = "Spin Amount {Leg0}", perAnimSet = false, type = "float", attributes = "LegsSpinAmount_0"},
      {title = "Spin Amount {Leg1}", perAnimSet = false, type = "float", attributes = "LegsSpinAmount_1"},
    },
  },
  {
    title = "Orientation Configuration",
    helptext = "Define the target orientation for the character's pelvis by choosing to align one or two axis in character space with axis in world space.",
    isExpanded = false,
    details = 
    {
      {title = "characterAxis0", perAnimSet = false, type = "integer", attributes = "characterAxis0"},
      {title = "characterAxis1", perAnimSet = false, type = "integer", attributes = "characterAxis1"},
      {title = "targetAxis0", perAnimSet = false, type = "integer", attributes = "targetAxis0"},
      {title = "targetAxis1", perAnimSet = false, type = "integer", attributes = "targetAxis1"},
      {title = "secondaryDirectionThreshold", perAnimSet = false, type = "float", attributes = "secondaryDirectionThreshold"},
    },
  },
  {
    title = "Landing Up Axis",
    helptext = "Define how the \"Landing Up Axis\" orientation target axis is calculated.",
    isExpanded = false,
    details = 
    {
      {title = "Orientation", perAnimSet = true, type = "float", attributes = "AngleLandingAmount"},
    },
  },
  {
    title = "Input Defaults",
    isExpanded = false,
    details = 
    {
      {title = "Pelvis orientation", perAnimSet = false, type = "vector3", attributes = "orientation"},
      {title = "weight", perAnimSet = false, type = "float", attributes = "weight"},
      {title = "disableWhenInContact", perAnimSet = false, type = "bool", attributes = "disableWhenInContact"},
    },
  },
}

return {
  data = behaviourNode,
  attributeGroups = attributeGroups,
}
