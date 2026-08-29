--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "Observe",
  version = 4,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Automatically looks at nearby objects based on parameters that define the relevence of each object",
  group = "Behaviours",
  image = "ObserveBehaviour.png",
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
    ["ObserveWeight"] = 
    {
      input = true,
      helptext = "The weight (importance) of look relative to other looking. 0 can be used to get output control params without actually looking",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Weight",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["observedWeight"] = 
    {
      input = false,
      helptext = "Indicates how strongly the object selected by this behaviour is being observed. The value is 0 when no objects are observed, 1 when the character is only looking at the selected object.",
      type = "float",
    },

  },

  pinOrder = 
  {
    "ObserveWeight",
    "Result",
    "observedWeight",
  },

  attributes = 
  {
    {
      name = "MinSpeed",
      type = "float",
      value = 1.500000,
      min = 0.000000,
      max = 5.000000,
      helptext = "The minimum relative speed at which the character will observe a moving object, in m/s (standard character).",
    },
    {
      name = "MinAcceleration",
      type = "float",
      value = 8.000000,
      min = 0.000000,
      max = 20.000000,
      helptext = "The minimum relative acceleration at which the character will observe a moving object, in m/s^2 (standard character).",
    },
    {
      name = "MaxDistance",
      type = "float",
      value = 4.000000,
      min = 0.000000,
      max = 30.000000,
      helptext = "The maximum distance for the character at which the object will be observed, in metres (standard character).",
    },
    {
      name = "MaxSize",
      type = "float",
      value = 4.000000,
      min = 0.000000,
      max = 30.000000,
      helptext = "The maximum size of an object, at which it will be observed. Larger may look odd as the character looks at the object centre. In metres (standard character).",
    },
    {
      name = "MinMass",
      type = "float",
      value = 0.100000,
      min = 0.000000,
      max = 10.000000,
      helptext = "The smallest mass at which to observe the object. In kg (standard character).",
    },
    {
      name = "winnersAdvantage",
      type = "float",
      value = 0.500000,
      min = 0.000000,
      max = 1.000000,
      helptext = "An extra boost in value for the highest rated object, to reduce flipping between many objects",
    },
    {
      name = "interestReductionRate",
      type = "float",
      value = 0.500000,
      min = 0.000000,
      max = 10.000000,
      helptext = "Rate of exponential decrease of the output observe weight, so an object will still be observed for a while even if it stops moving. Units 1/seconds (standard character).",
    },
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(20, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(0, "numBehaviourInts")

    stream:writeUInt(7, "numBehaviourFloats")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MinSpeed"), string.format("Float_0_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MinAcceleration"), string.format("Float_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MaxDistance"), string.format("Float_2_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MaxSize"), string.format("Float_3_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MinMass"), string.format("Float_4_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "winnersAdvantage"), string.format("Float_5_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "interestReductionRate"), string.format("Float_6_%d", asIdx-1))
    end

    stream:writeUInt(0, "numBehaviourVector3s")

    stream:writeUInt(0, "numBehaviourUInt64s")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numBehaviourNodeAnimationInputs")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numInputCPInts")
    stream:writeUInt(1, "numInputCPFloats")
    stream:writeUInt(0, "numInputCPUInt64s")
    stream:writeUInt(0, "numInputCPVector3s")
    stream:writeUInt(1, "numBehaviourControlParameterInputs")

    writeDataPinAltName(stream, node, "ObserveWeight", "BehaviourControlParameterInputID_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_0")

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
    title = "Observed Object",
    isExpanded = true,
    details = 
    {
      {title = "MinSpeed", perAnimSet = false, type = "float", attributes = "MinSpeed"},
      {title = "MinAcceleration", perAnimSet = false, type = "float", attributes = "MinAcceleration"},
      {title = "MaxDistance", perAnimSet = false, type = "float", attributes = "MaxDistance"},
      {title = "MaxSize", perAnimSet = false, type = "float", attributes = "MaxSize"},
      {title = "MinMass", perAnimSet = false, type = "float", attributes = "MinMass"},
      {title = "winnersAdvantage", perAnimSet = false, type = "float", attributes = "winnersAdvantage"},
      {title = "WeightReductionRate", perAnimSet = false, type = "float", attributes = "interestReductionRate"},
    },
  },
  {
    title = "Importance",
    isExpanded = false,
    details = 
    {
      {title = "Weight", perAnimSet = false, type = "float", attributes = "ObserveWeight"},
    },
  },
}

return {
  data = behaviourNode,
  attributeGroups = attributeGroups,
}
