--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "Arms Brace",
  version = 2,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Reaches for and absorbs the impact of an approaching object to protect the head and chest from damage. ",
  group = "Behaviours",
  image = "ArmsBraceBehaviour.png",
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
    ["HazardVelocity"] = 
    {
      input = true,
      helptext = "Velocity (m/s) of the hazard",
      type = "vector3",
      mode = "HiddenInNetworkEditor",
      displayName = "Velocity",
    },

    ["HazardPosition"] = 
    {
      input = true,
      helptext = "Position (m) of the hazard surface",
      type = "vector3",
      displayName = "Position",
    },

    ["HazardNormal"] = 
    {
      input = true,
      helptext = "Normal of the hazard surface (doesn't have to be normalised, but should be non-zero)",
      type = "vector3",
      displayName = "Normal",
    },

    ["HazardMass"] = 
    {
      input = true,
      helptext = "Mass (kg) of the hazard",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Mass",
      value = 1.000000,
      min = 0.000000,
      max = 10000.000000,
    },

    ["HazardRadius"] = 
    {
      input = true,
      helptext = "Extent and curvature (1/radius) of the hazard surface",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Radius",
      value = 0.250000,
      min = 0.000000,
      max = 100.000000,
    },

    ["Weight"] = 
    {
      input = true,
      helptext = "Sets the importance of this behaviour relative to others. Accepts values in the range 0 to 1.",
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
    "HazardVelocity",
    "HazardPosition",
    "HazardNormal",
    "HazardMass",
    "HazardRadius",
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
      helptext = "Controls the arms' swivel. A positive value encourages the elbows to be placed out and up, negative for elbows in and down. When zero the swivel will match the guide pose.",
    },
    {
      name = "MaxArmExtensionScale",
      type = "float",
      value = 1.000000,
      min = 0.100000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Limit the maximum extension of the arm. Expressed as a proportion of the total arm length",
    },
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(3, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(0, "numBehaviourInts")

    stream:writeUInt(2, "numBehaviourFloats")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "SwivelAmount", asVal), string.format("Float_0_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MaxArmExtensionScale", asVal), string.format("Float_1_%d", asIdx-1))
    end

    stream:writeUInt(0, "numBehaviourVector3s")

    stream:writeUInt(0, "numBehaviourUInt64s")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numBehaviourNodeAnimationInputs")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numInputCPInts")
    stream:writeUInt(3, "numInputCPFloats")
    stream:writeUInt(0, "numInputCPUInt64s")
    stream:writeUInt(3, "numInputCPVector3s")
    stream:writeUInt(6, "numBehaviourControlParameterInputs")

    writeDataPinAltName(stream, node, "HazardVelocity", "BehaviourControlParameterInputID_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_0")

    writeDataPinAltName(stream, node, "HazardPosition", "BehaviourControlParameterInputID_1")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_1")

    writeDataPinAltName(stream, node, "HazardNormal", "BehaviourControlParameterInputID_2")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_2")

    writeDataPinAltName(stream, node, "HazardMass", "BehaviourControlParameterInputID_3")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_3")

    writeDataPinAltName(stream, node, "HazardRadius", "BehaviourControlParameterInputID_4")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_4")

    writeDataPinAltName(stream, node, "Weight", "BehaviourControlParameterInputID_5")
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
    title = "Positioning",
    isExpanded = true,
    details = 
    {
      {title = "SwivelAmount", perAnimSet = true, type = "float", attributes = "SwivelAmount"},
      {title = "ArmExtension", perAnimSet = true, type = "float", attributes = "MaxArmExtensionScale"},
    },
  },
  {
    title = "Hazard Inputs",
    helptext = "Describes an approaching object that the arms should brace against. Hazard position and hazard normal represent the hazard surface whilst hazard radius defines the extent and curvature (1/radius) of that surface. In absolute, world space units",
    isExpanded = false,
    details = 
    {
      {title = "Velocity", perAnimSet = false, type = "vector3", attributes = "HazardVelocity"},
      {title = "Position", perAnimSet = false, type = "vector3", attributes = "HazardPosition"},
      {title = "Normal", perAnimSet = false, type = "vector3", attributes = "HazardNormal"},
      {title = "Mass", perAnimSet = false, type = "float", attributes = "HazardMass"},
      {title = "Radius", perAnimSet = false, type = "float", attributes = "HazardRadius"},
    },
  },
  {
    title = "Input Defaults",
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
