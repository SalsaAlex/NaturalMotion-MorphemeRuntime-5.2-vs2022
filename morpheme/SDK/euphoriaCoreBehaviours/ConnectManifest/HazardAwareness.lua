--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "Hazard Awareness",
  version = 3,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "To modify how hazards are predicted.",
  group = "Behaviours",
  image = "HazardAwarenessBehaviour.png",
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
    ["HazardFreeTime"] = 
    {
      input = false,
      helptext = "The amount of time that has passed since a hazard has existed at a hazard level higher than hazardLevelThreshold above. In absolute units.",
      type = "float",
    },

    ["HazardLevel"] = 
    {
      input = false,
      helptext = "A unitless value in the range 0-infinity indicating the threat posed by the current hazard. Calculated as 1/time to impact, scaled by a 0-1 measure of the mass of the hazard.",
      type = "float",
    },

    ["TimeToImpact"] = 
    {
      input = false,
      helptext = "The time left until the hazard collides with the character or 0 if there is no hazard. In absolute units.",
      type = "float",
    },

    ["ImpactSpeed"] = 
    {
      input = false,
      helptext = "The speed with which the hazard will impact the character or 0 if there is no hazard. In absolute units.",
      type = "float",
    },

  },

  pinOrder = 
  {
    "Result",
    "HazardFreeTime",
    "HazardLevel",
    "TimeToImpact",
    "ImpactSpeed",
  },

  attributes = 
  {
    {
      name = "UseControlledVelocity",
      type = "bool",
      value = false,
      helptext = "When set then the chest's velocity is not considered hazardous, hazards are only moving objects",
    },
    {
      name = "IgnoreVerticalPredictionAmount",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "How much the vertical component of the character's velocity is ignored when predicting impacts. 0 is the default, 1 means that the vertical velocity of the chest is completely ignored so the ground will not present a hazard when the character is falling.",
    },
    {
      name = "HazardLevelThreshold",
      type = "float",
      value = 0.430000,
      min = 0.000000,
      max = 4.000000,
      helptext = "Used, as feedback info, to determine the time since the last hazard, as feedback info. Is 1/(1+time to impact), in 1/seconds (standard character).",
    },
    {
      name = "ProbeRadius",
      type = "float",
      value = 0.500000,
      min = 0.100000,
      max = 2.000000,
      helptext = "Radius around the chest that the character has \"hazard awarenesss\" for, in metres (standard character).",
    },
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(12, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(1, "numBehaviourInts")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "UseControlledVelocity") and 1 ) or 0 ) , string.format("Int_0_%d", asIdx-1))
    end

    stream:writeUInt(3, "numBehaviourFloats")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "IgnoreVerticalPredictionAmount", asVal), string.format("Float_0_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "HazardLevelThreshold"), string.format("Float_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ProbeRadius"), string.format("Float_2_%d", asIdx-1))
    end

    stream:writeUInt(0, "numBehaviourVector3s")

    stream:writeUInt(0, "numBehaviourUInt64s")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numBehaviourNodeAnimationInputs")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numInputCPInts")
    stream:writeUInt(0, "numInputCPFloats")
    stream:writeUInt(0, "numInputCPUInt64s")
    stream:writeUInt(0, "numInputCPVector3s")
    stream:writeUInt(0, "numBehaviourControlParameterInputs")

    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterOutputType_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterOutputType_1")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterOutputType_2")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterOutputType_3")
    stream:writeUInt(4, "numBehaviourControlParameterOutputs")

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
    title = "Default",
    isExpanded = true,
    details = 
    {
      {title = "MovingHazardsOnly", perAnimSet = false, type = "bool", attributes = "UseControlledVelocity"},
      {title = "IgnoreVerticalPredictionAmount", perAnimSet = true, type = "float", attributes = "IgnoreVerticalPredictionAmount"},
      {title = "HazardLevelThreshold", perAnimSet = false, type = "float", attributes = "HazardLevelThreshold"},
      {title = "ProbeRadius", perAnimSet = false, type = "float", attributes = "ProbeRadius"},
    },
  },
}

return {
  data = behaviourNode,
  attributeGroups = attributeGroups,
}
