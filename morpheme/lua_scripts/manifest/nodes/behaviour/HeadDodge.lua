--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "Head Dodge",
  version = 2,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Moves a character's head to the side of an incoming hazard to avoid hitting it. ",
  group = "Behaviours",
  image = "HeadDodgeBehaviour.png",
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
    ["HazardPosition"] = 
    {
      input = true,
      helptext = "Position in absolute units and in world space",
      type = "vector3",
      mode = "HiddenInNetworkEditor",
      displayName = "Position",
    },

    ["HazardVelocity"] = 
    {
      input = true,
      helptext = "Velocity in absolute units.",
      type = "vector3",
      mode = "HiddenInNetworkEditor",
      displayName = "Velocity",
    },

    ["HazardRadius"] = 
    {
      input = true,
      helptext = "Radius of the hazard, this affects how far the character lean when dodging. In absolute units.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Radius",
      value = 0.000000,
      min = 0.000000,
      max = 100.000000,
    },

    ["Weight"] = 
    {
      input = true,
      helptext = "Sets the importance of this behaviour relative to others. The pin accepts values in the range 0 to 1.",
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
    "HazardPosition",
    "HazardVelocity",
    "HazardRadius",
    "Weight",
    "Result",
  },

  attributes = 
  {
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(14, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(0, "numBehaviourInts")

    stream:writeUInt(0, "numBehaviourFloats")

    stream:writeUInt(0, "numBehaviourVector3s")

    stream:writeUInt(0, "numBehaviourUInt64s")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numBehaviourNodeAnimationInputs")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numInputCPInts")
    stream:writeUInt(2, "numInputCPFloats")
    stream:writeUInt(0, "numInputCPUInt64s")
    stream:writeUInt(2, "numInputCPVector3s")
    stream:writeUInt(4, "numBehaviourControlParameterInputs")

    writeDataPinAltName(stream, node, "HazardPosition", "BehaviourControlParameterInputID_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_0")

    writeDataPinAltName(stream, node, "HazardVelocity", "BehaviourControlParameterInputID_1")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_1")

    writeDataPinAltName(stream, node, "HazardRadius", "BehaviourControlParameterInputID_2")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_2")

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
    title = "Hazard",
    helptext = "Describes a hazard that the character should dodge in terms of its position, velolcity and radius. In absolute world space units.",
    isExpanded = true,
    details = 
    {
      {title = "Position", perAnimSet = false, type = "vector3", attributes = "HazardPosition"},
      {title = "Velocity", perAnimSet = false, type = "vector3", attributes = "HazardVelocity"},
      {title = "Radius", perAnimSet = false, type = "float", attributes = "HazardRadius"},
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
