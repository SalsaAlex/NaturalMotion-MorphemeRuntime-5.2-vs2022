--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "Euphoria Ragdoll",
  version = 2,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Makes the character appear unconscious by only activating the physical characteristics, such as joint limits, soft limits and hamstrings, but not the animation joints. ",
  group = "Behaviours",
  image = "EuphoriaRagdollBehaviour.png",
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

  pinOrder = 
  {
    "Result",
  },

  attributes = 
  {
    {
      name = "DisableSleeping",
      type = "bool",
      value = false,
      helptext = "False allows character to sleep when velocity is below a threshold. Used as an optimisation when there are many stationary characters",
    },
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(9, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(1, "numBehaviourInts")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "DisableSleeping") and 1 ) or 0 ) , string.format("Int_0_%d", asIdx-1))
    end

    stream:writeUInt(0, "numBehaviourFloats")

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
    title = "Default",
    isExpanded = true,
    details = 
    {
      {title = "DisableSleeping", perAnimSet = false, type = "bool", attributes = "DisableSleeping"},
    },
  },
}

return {
  data = behaviourNode,
  attributeGroups = attributeGroups,
}
