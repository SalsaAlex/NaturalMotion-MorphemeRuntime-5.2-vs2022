--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "Arms Placement",
  version = 2,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Places hands palm down against an object so that they are ready to brace if the object becomes a hazard. ",
  group = "Behaviours",
  image = "ArmsPlacementBehaviour.png",
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
    ["TargetPosition"] = 
    {
      input = true,
      helptext = "Position of the target in world space and absolute units.",
      type = "vector3",
      displayName = "Position",
    },

    ["TargetNormal"] = 
    {
      input = true,
      helptext = "Normal of target in world space - does not need to be normalised.",
      type = "vector3",
      displayName = "Normal",
    },

    ["TargetVelocity"] = 
    {
      input = true,
      helptext = "Velocity of the target in world space and absolute units.",
      type = "vector3",
      mode = "HiddenInNetworkEditor",
      displayName = "Velocity",
    },

    ["Weight"] = 
    {
      input = true,
      helptext = "Sets the importance of this behaviour relative to others.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Weight",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

  },

  pinOrder = 
  {
    "TargetPosition",
    "TargetNormal",
    "TargetVelocity",
    "Weight",
    "Result",
  },

  attributes = 
  {
    {
      name = "SwivelAmount",
      type = "float",
      value = 0.300000,
      min = -1.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Controls the arms' swivel. A positive value for elbows out and up, negative for elbows in and down, at zero the swivel will match the guide pose.",
    },
    {
      name = "MaxArmExtensionScale",
      type = "float",
      value = 1.000000,
      min = 0.100000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Limit the maximum extension of the arm. Expressed as a proportion of the total arm length.",
    },
    {
      name = "handOffsetMultiplier",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
      helptext = "Controls how close the hands are together when placing. 0 places hands in the same position, 1 is shoulder width apart so arms are effectively parallel, 2 is two shoulder widths apart.",
    },
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(4, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(0, "numBehaviourInts")

    stream:writeUInt(3, "numBehaviourFloats")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "SwivelAmount", asVal), string.format("Float_0_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MaxArmExtensionScale", asVal), string.format("Float_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "handOffsetMultiplier"), string.format("Float_2_%d", asIdx-1))
    end

    stream:writeUInt(0, "numBehaviourVector3s")

    stream:writeUInt(0, "numBehaviourUInt64s")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numBehaviourNodeAnimationInputs")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numInputCPInts")
    stream:writeUInt(1, "numInputCPFloats")
    stream:writeUInt(0, "numInputCPUInt64s")
    stream:writeUInt(3, "numInputCPVector3s")
    stream:writeUInt(4, "numBehaviourControlParameterInputs")

    writeDataPinAltName(stream, node, "TargetPosition", "BehaviourControlParameterInputID_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_0")

    writeDataPinAltName(stream, node, "TargetNormal", "BehaviourControlParameterInputID_1")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_1")

    writeDataPinAltName(stream, node, "TargetVelocity", "BehaviourControlParameterInputID_2")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_2")

    writeDataPinAltName(stream, node, "Weight", "BehaviourControlParameterInputID_3")
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
    title = "Positioning",
    isExpanded = true,
    details = 
    {
      {title = "SwivelAmount", perAnimSet = true, type = "float", attributes = "SwivelAmount"},
      {title = "ArmExtension", perAnimSet = true, type = "float", attributes = "MaxArmExtensionScale"},
      {title = "HandOffsetScale", perAnimSet = false, type = "float", attributes = "handOffsetMultiplier"},
    },
  },
  {
    title = "Target Inputs",
    helptext = "Describes a surface, in terms of its position, normal and velocity, on which the palms of the hands should be placed. All in absolute, world space coordinates.",
    isExpanded = false,
    details = 
    {
      {title = "Position", perAnimSet = false, type = "vector3", attributes = "TargetPosition"},
      {title = "Normal", perAnimSet = false, type = "vector3", attributes = "TargetNormal"},
      {title = "Velocity", perAnimSet = false, type = "vector3", attributes = "TargetVelocity"},
    },
  },
  {
    title = "Importance",
    isExpanded = false,
    details = 
    {
      {title = "Weight", perAnimSet = false, type = "float", attributes = "Weight"},
    },
  },
}

return {
  data = behaviourNode,
  attributeGroups = attributeGroups,
}
