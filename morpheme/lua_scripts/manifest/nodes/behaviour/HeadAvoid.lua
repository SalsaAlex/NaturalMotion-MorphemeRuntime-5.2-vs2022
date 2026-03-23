--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "Head Avoid",
  version = 4,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Keeps the head a safe distance from a plane, usually to prevent collision, just like humans tend to keep their head from touching objects. ",
  group = "Behaviours",
  image = "HeadAvoidBehaviour.png",
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
    ["AvoidPlanePosition"] = 
    {
      input = true,
      helptext = "Position of the hazard plane to avoid. In absolute world space units.",
      type = "vector3",
      mode = "HiddenInNetworkEditor",
      displayName = "Position",
    },

    ["AvoidPlaneNormal"] = 
    {
      input = true,
      helptext = "Surface normal of the hazard plane to avoid.",
      type = "vector3",
      mode = "HiddenInNetworkEditor",
      displayName = "Normal",
    },

    ["AvoidSafeDistance"] = 
    {
      input = true,
      helptext = "Sets the maximum distance a hazard plane can be from the character before it is ignored by the behaviour. In metres (standard character).",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "SafeDistance",
      value = 0.500000,
      min = 0.000000,
      max = 3.000000,
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
    "AvoidPlanePosition",
    "AvoidPlaneNormal",
    "AvoidSafeDistance",
    "Weight",
    "Result",
  },

  attributes = 
  {
    {
      name = "twoSidedPlane",
      type = "bool",
      value = false,
      helptext = "Determines if character always avoids the plane from both sides, or a just one side.",
    },
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(13, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(1, "numBehaviourInts")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "twoSidedPlane") and 1 ) or 0 ) , string.format("Int_0_%d", asIdx-1))
    end

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

    writeDataPinAltName(stream, node, "AvoidPlanePosition", "BehaviourControlParameterInputID_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_0")

    writeDataPinAltName(stream, node, "AvoidPlaneNormal", "BehaviourControlParameterInputID_1")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_1")

    writeDataPinAltName(stream, node, "AvoidSafeDistance", "BehaviourControlParameterInputID_2")
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
    title = "Hazard Plane",
    helptext = "The plane to avoid",
    isExpanded = true,
    details = 
    {
      {title = "PlaneType", perAnimSet = false, type = "bool", attributes = "twoSidedPlane"},
      {title = "Position", perAnimSet = false, type = "vector3", attributes = "AvoidPlanePosition"},
      {title = "Normal", perAnimSet = false, type = "vector3", attributes = "AvoidPlaneNormal"},
      {title = "SafeDistance", perAnimSet = false, type = "float", attributes = "AvoidSafeDistance"},
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
