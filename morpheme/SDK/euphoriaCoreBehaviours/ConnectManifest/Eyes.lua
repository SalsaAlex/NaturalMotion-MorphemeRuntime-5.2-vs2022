--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "Eyes",
  version = 1,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Provides a game with the target position and direction of a character's eyes. ",
  group = "Behaviours",
  image = "EyesBehaviour.png",
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
    ["LookDirection"] = 
    {
      input = false,
      helptext = "A normalised vector in the direction that the character is looking.",
      type = "vector3",
    },

    ["FocalCentre"] = 
    {
      input = false,
      helptext = "The position in world space that the character is focussing on. In absolute world space units.",
      type = "vector3",
    },

    ["FocalRadius"] = 
    {
      input = false,
      helptext = "The radius of the field around the focalCentre in which the character will notice objects. In absolute units.",
      type = "float",
    },

  },

  pinOrder = 
  {
    "Result",
    "LookDirection",
    "FocalCentre",
    "FocalRadius",
  },

  attributes = 
  {
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(10, "BehaviourID")


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
    stream:writeUInt(0, "numInputCPFloats")
    stream:writeUInt(0, "numInputCPUInt64s")
    stream:writeUInt(0, "numInputCPVector3s")
    stream:writeUInt(0, "numBehaviourControlParameterInputs")

    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterOutputType_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterOutputType_1")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterOutputType_2")
    stream:writeUInt(3, "numBehaviourControlParameterOutputs")

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
}

return {
  data = behaviourNode,
  attributeGroups = attributeGroups,
}
