--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "Legs Pedal",
  version = 5,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Spins a character's legs in circles in order to achieve a rotation of the pelvis within a specified time period. ",
  group = "Behaviours",
  image = "LegsPedalBehaviour.png",
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
    ["TargetRotationDelta"] = 
    {
      input = true,
      helptext = "Sets a world or local space rotation vector whose direction determines the axis and magnitude determines the angle of rotation of the pelvis, in degrees.",
      type = "vector3",
      mode = "HiddenInNetworkEditor",
      displayName = "Rotation",
      value = {x=0.000000, y=0.000000,z=90.000000},
    },

    ["RotationTime"] = 
    {
      input = true,
      helptext = "Sets the amount of time in which to achieve the rotation, in seconds (standard character). Smaller values lead to faster movements. For times under about 0.5s the legs will swing rather than spin.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "RotationTime",
      value = 0.800000,
      min = 0.000000,
      max = 5.000000,
    },

    ["ImportanceForLeg_0"] = 
    {
      input = true,
      helptext = "Sets the strength of control this behaviour has over each leg. Accepts values in the range 0 to 1.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Weight {Leg0}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["ImportanceForLeg_1"] = 
    {
      input = true,
      helptext = "Sets the strength of control this behaviour has over each leg. Accepts values in the range 0 to 1.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Weight {Leg1}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

  },

  pinOrder = 
  {
    "TargetRotationDelta",
    "RotationTime",
    "ImportanceForLeg_0",
    "ImportanceForLeg_1",
    "Result",
  },

  attributes = 
  {
    {
      name = "SpinWeightForward",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Weight of rotation control around the forwards / x axis.",
    },
    {
      name = "SpinWeightUp",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Weight of rotation control around the upwards / y axis.",
    },
    {
      name = "SpinWeightLateral",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Weight of rotation control around the sideways / z axis.",
    },
    {
      name = "SpinAmounts_0",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
      helptext = "Affects how strongly a request to spin the legs is applied. 1 is normal or average stiffness, 0 has no effect, the spin is turned off.",
    },
    {
      name = "SpinAmounts_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
      helptext = "Affects how strongly a request to spin the legs is applied. 1 is normal or average stiffness, 0 has no effect, the spin is turned off.",
    },
    {
      name = "MaxAngSpeed",
      type = "float",
      value = 3.000000,
      min = 0.000000,
      max = 5.000000,
      perAnimSet = true,
      helptext = "Maximum angular speed of the leg, in revolutions/second (standard character).",
    },
    {
      name = "MaxRadius",
      type = "float",
      value = 0.400000,
      min = 0.000000,
      max = 2.000000,
      perAnimSet = true,
      helptext = "Maximum radius of spin around the spin centre, this is a length, in metres (standard character).",
    },
    {
      name = "Synchronised",
      type = "bool",
      value = true,
      perAnimSet = true,
      helptext = "If true then both feet will have opposite phases in their respective circles, if false then the feet will circle independently from their initial positions",
    },
    {
      name = "SwingAmounts_0",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 3.000000,
      helptext = "Affects how strongly a request to swing the legs is applied, 0 will not swing, 1 is normal stiffness.",
    },
    {
      name = "SwingAmounts_1",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 3.000000,
      helptext = "Affects how strongly a request to swing the legs is applied, 0 will not swing, 1 is normal stiffness.",
    },
    {
      name = "FallenFor",
      type = "float",
      value = 0.800000,
      min = 0.000000,
      max = 5.000000,
      helptext = "The swing will abort if the character is fallen over for longer than this amount of time (seconds). In seconds (standard character).",
    },
    {
      name = "SpinInLocalSpace",
      type = "bool",
      value = true,
      perAnimSet = true,
      helptext = "If true then the target rotation delta of the character will be interpreted in the local space of the pelvis. Otherwise it is interpreted in world space.",
    },
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(18, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(2, "numBehaviourInts")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "Synchronised", asVal) and 1 ) or 0 ) , string.format("Int_0_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "SpinInLocalSpace", asVal) and 1 ) or 0 ) , string.format("Int_1_%d", asIdx-1))
    end

    stream:writeUInt(10, "numBehaviourFloats")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "SpinWeightForward", asVal), string.format("Float_0_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "SpinWeightUp", asVal), string.format("Float_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "SpinWeightLateral", asVal), string.format("Float_2_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "SpinAmounts_0"), string.format("Float_3_%d", asIdx-1))
      stream:writeFloat(getValue(node, "SpinAmounts_1"), string.format("Float_4_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MaxAngSpeed", asVal), string.format("Float_5_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MaxRadius", asVal), string.format("Float_6_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "SwingAmounts_0"), string.format("Float_7_%d", asIdx-1))
      stream:writeFloat(getValue(node, "SwingAmounts_1"), string.format("Float_8_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "FallenFor"), string.format("Float_9_%d", asIdx-1))
    end

    stream:writeUInt(0, "numBehaviourVector3s")

    stream:writeUInt(0, "numBehaviourUInt64s")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numBehaviourNodeAnimationInputs")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numInputCPInts")
    stream:writeUInt(3, "numInputCPFloats")
    stream:writeUInt(0, "numInputCPUInt64s")
    stream:writeUInt(1, "numInputCPVector3s")
    stream:writeUInt(4, "numBehaviourControlParameterInputs")

    writeDataPinAltName(stream, node, "TargetRotationDelta", "BehaviourControlParameterInputID_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_0")

    writeDataPinAltName(stream, node, "RotationTime", "BehaviourControlParameterInputID_1")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_1")

    writeDataPinAltName(stream, node, "ImportanceForLeg_0", "BehaviourControlParameterInputID_2" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_2")
    writeDataPinAltName(stream, node, "ImportanceForLeg_1", "BehaviourControlParameterInputID_3" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_3")

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
    title = "Leg Spin Weight",
    helptext = "Affects how strongly the character will spin the legs to drive rotation about each axis. Axis are defined in character or world space depending on the value of the \"Operate in local space\" attribute.",
    isExpanded = true,
    details = 
    {
      {title = "AroundForwardAxis", perAnimSet = true, type = "float", attributes = "SpinWeightForward"},
      {title = "AroundUpAxis", perAnimSet = true, type = "float", attributes = "SpinWeightUp"},
      {title = "AroundRightAxis", perAnimSet = true, type = "float", attributes = "SpinWeightLateral"},
    },
  },
  {
    title = "Spin",
    helptext = "Affects how strongly a request to spin the legs is applied. 1 is normal or average stiffness, 0 has no effect, the spin is turned off.",
    isExpanded = false,
    details = 
    {
      {title = "SpinAmount {Leg0}", perAnimSet = false, type = "float", attributes = "SpinAmounts_0"},
      {title = "SpinAmount {Leg1}", perAnimSet = false, type = "float", attributes = "SpinAmounts_1"},
      {title = "MaximumAngularSpeed", perAnimSet = true, type = "float", attributes = "MaxAngSpeed"},
      {title = "MaximumSpinRadius", perAnimSet = true, type = "float", attributes = "MaxRadius"},
      {title = "Synchronised", perAnimSet = true, type = "bool", attributes = "Synchronised"},
    },
  },
  {
    title = "Swing",
    helptext = "Affects how strongly a request to swing the legs is applied.",
    isExpanded = false,
    details = 
    {
      {title = "SwingAmount {Leg0}", perAnimSet = false, type = "float", attributes = "SwingAmounts_0"},
      {title = "SwingAmount {Leg1}", perAnimSet = false, type = "float", attributes = "SwingAmounts_1"},
    },
  },
  {
    title = "Stop Swinging When",
    isExpanded = false,
    details = 
    {
      {title = "FallenFor", perAnimSet = false, type = "float", attributes = "FallenFor"},
    },
  },
  {
    title = "Input Defaults",
    isExpanded = false,
    details = 
    {
      {title = "OperateInLocalSpace", perAnimSet = true, type = "bool", attributes = "SpinInLocalSpace"},
      {title = "Rotation", perAnimSet = false, type = "vector3", attributes = "TargetRotationDelta"},
      {title = "RotationTime", perAnimSet = false, type = "float", attributes = "RotationTime"},
      {title = "Weight {Leg0}", perAnimSet = false, type = "float", attributes = "ImportanceForLeg_0"},
      {title = "Weight {Leg1}", perAnimSet = false, type = "float", attributes = "ImportanceForLeg_1"},
    },
  },
}

return {
  data = behaviourNode,
  attributeGroups = attributeGroups,
}
