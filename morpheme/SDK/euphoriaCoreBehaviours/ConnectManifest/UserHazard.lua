--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "User Hazard",
  version = 2,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "To allow user-generated hazards to be passed in.",
  group = "Behaviours",
  image = "UserHazardBehaviour.png",
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
    ["UserHazard"] = 
    {
      input = true,
      helptext = "If true, a user defined hazard will be used alongside hazards identified by environment awareness.",
      type = "bool",
      mode = "HiddenInNetworkEditor",
      displayName = "User defined hazard",
      value = false,
    },

    ["IgnoreOtherHazards"] = 
    {
      input = true,
      helptext = "Force the behaviour to use only this hazard, rather than using the environment awareness.",
      type = "bool",
      mode = "HiddenInNetworkEditor",
      displayName = "Ignore other hazards",
      value = false,
    },

    ["HazardPosition"] = 
    {
      input = true,
      helptext = "Sets the position of the user defined hazard. In absolute world space units.",
      type = "vector3",
      displayName = "Position",
      value = {x=0.000000, y=0.000000,z=0.000000},
    },

    ["HazardVelocity"] = 
    {
      input = true,
      helptext = "Sets the velocity of the user defined hazard. In absolute world space units.",
      type = "vector3",
      displayName = "Velocity",
      value = {x=0.000000, y=0.000000,z=0.000000},
    },

    ["HazardMass"] = 
    {
      input = true,
      helptext = "Sets the mass of the user defined hazard, which affects the strength of bracing. In absolute units.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Mass",
      value = 0.000000,
      min = 0.000000,
      max = 10000.000000,
    },

    ["HazardRadius"] = 
    {
      input = true,
      helptext = "Sets the radius of the user defined hazard (the object is approximated as a sphere). In absolute units.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Radius",
      value = 0.000000,
      min = 0.000000,
      max = 100.000000,
    },

  },

  pinOrder = 
  {
    "UserHazard",
    "IgnoreOtherHazards",
    "HazardPosition",
    "HazardVelocity",
    "HazardMass",
    "HazardRadius",
    "Result",
  },

  attributes = 
  {
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(28, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(0, "numBehaviourInts")

    stream:writeUInt(0, "numBehaviourFloats")

    stream:writeUInt(0, "numBehaviourVector3s")

    stream:writeUInt(0, "numBehaviourUInt64s")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numBehaviourNodeAnimationInputs")

    -------------------------------------------------------------------

    stream:writeUInt(2, "numInputCPInts")
    stream:writeUInt(2, "numInputCPFloats")
    stream:writeUInt(0, "numInputCPUInt64s")
    stream:writeUInt(2, "numInputCPVector3s")
    stream:writeUInt(6, "numBehaviourControlParameterInputs")

    writeDataPinAltName(stream, node, "UserHazard", "BehaviourControlParameterInputID_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_BOOL", "BehaviourControlParameterInputType_0")

    writeDataPinAltName(stream, node, "IgnoreOtherHazards", "BehaviourControlParameterInputID_1")
    stream:writeString("ATTRIB_SEMANTIC_CP_BOOL", "BehaviourControlParameterInputType_1")

    writeDataPinAltName(stream, node, "HazardPosition", "BehaviourControlParameterInputID_2")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_2")

    writeDataPinAltName(stream, node, "HazardVelocity", "BehaviourControlParameterInputID_3")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_3")

    writeDataPinAltName(stream, node, "HazardMass", "BehaviourControlParameterInputID_4")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_4")

    writeDataPinAltName(stream, node, "HazardRadius", "BehaviourControlParameterInputID_5")
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
    title = "Input Defaults",
    isExpanded = true,
    details = 
    {
      {title = "User defined hazard", perAnimSet = false, type = "bool", attributes = "UserHazard"},
      {title = "Ignore other hazards", perAnimSet = false, type = "bool", attributes = "IgnoreOtherHazards"},
    },
  },
  {
    title = "Hazard Inputs",
    isExpanded = false,
    details = 
    {
      {title = "Position", perAnimSet = false, type = "vector3", attributes = "HazardPosition"},
      {title = "Velocity", perAnimSet = false, type = "vector3", attributes = "HazardVelocity"},
      {title = "Mass", perAnimSet = false, type = "float", attributes = "HazardMass"},
      {title = "Radius", perAnimSet = false, type = "float", attributes = "HazardRadius"},
    },
  },
}

return {
  data = behaviourNode,
  attributeGroups = attributeGroups,
}
