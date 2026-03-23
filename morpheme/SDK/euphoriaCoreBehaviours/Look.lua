--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "Look",
  version = 4,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Makes a character look at a target point.",
  group = "Behaviours",
  image = "LookBehaviour.png",
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
    ["LookPosition"] = 
    {
      input = true,
      helptext = "Sets the position in world space to look at.",
      type = "vector3",
      displayName = "Target",
    },

    ["WholeBodyLook"] = 
    {
      input = true,
      helptext = "Sets how much of the body to move when looking. Accepts values in the range 0 to 1.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "MoveBody",
      value = 0.200000,
      min = 0.000000,
      max = 1.000000,
    },

    ["LookWeight"] = 
    {
      input = true,
      helptext = "The weight (importance) of the look behaviour relative to others.",
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
    "LookPosition",
    "WholeBodyLook",
    "LookWeight",
    "Result",
  },

  attributes = 
  {
    {
      name = "IgnoreDirectionWhenOutOfRange",
      type = "bool",
      value = false,
    },
    {
      name = "TargetYawRight",
      type = "float",
      value = -90.000000,
      min = -180.000000,
      max = 180.000000,
      helptext = "Ignore look target if yaw is outside of this range. The centre of the range points in the pelvis forward direction and the yaw value increases from the character's right to its left. In degrees.",
    },
    {
      name = "TargetYawLeft",
      type = "float",
      value = 90.000000,
      min = -180.000000,
      max = 180.000000,
      helptext = "Ignore look target if yaw is outside of this range. The centre of the range points in the pelvis forward direction and the yaw value increases from the character's right to its left. In degrees.",
    },
    {
      name = "TargetPitchDown",
      type = "float",
      value = -90.000000,
      min = -180.000000,
      max = 180.000000,
      helptext = "Ignore look target if pitch is outside of this range. The centre of the range points in the pelvis forward direction and the pitch value increases from character down to up. In degrees.",
    },
    {
      name = "TargetPitchUp",
      type = "float",
      value = 90.000000,
      min = -180.000000,
      max = 180.000000,
      helptext = "Ignore look target if pitch is outside of this range. The centre of the range points in the pelvis forward direction and the pitch value increases from character down to up. In degrees.",
    },
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(19, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(1, "numBehaviourInts")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "IgnoreDirectionWhenOutOfRange") and 1 ) or 0 ) , string.format("Int_0_%d", asIdx-1))
    end

    stream:writeUInt(4, "numBehaviourFloats")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "TargetYawRight"), string.format("Float_0_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "TargetYawLeft"), string.format("Float_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "TargetPitchDown"), string.format("Float_2_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "TargetPitchUp"), string.format("Float_3_%d", asIdx-1))
    end

    stream:writeUInt(0, "numBehaviourVector3s")

    stream:writeUInt(0, "numBehaviourUInt64s")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numBehaviourNodeAnimationInputs")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numInputCPInts")
    stream:writeUInt(2, "numInputCPFloats")
    stream:writeUInt(0, "numInputCPUInt64s")
    stream:writeUInt(1, "numInputCPVector3s")
    stream:writeUInt(3, "numBehaviourControlParameterInputs")

    writeDataPinAltName(stream, node, "LookPosition", "BehaviourControlParameterInputID_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_0")

    writeDataPinAltName(stream, node, "WholeBodyLook", "BehaviourControlParameterInputID_1")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_1")

    writeDataPinAltName(stream, node, "LookWeight", "BehaviourControlParameterInputID_2")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_2")

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
    title = "Ignore out of range targets",
    helptext = "Don't Look at a target if the direction from the pelvis to the target is outside of this range. The center of the range points in the pelvis forward direction. Pitch and Yaw are in degrees.",
    isExpanded = true,
    details = 
    {
      {title = "IgnoreDirectionWhenOutOfRange", perAnimSet = false, type = "bool", attributes = "IgnoreDirectionWhenOutOfRange"},
      {title = "Yaw right (min)", perAnimSet = false, type = "float", attributes = "TargetYawRight"},
      {title = "Yaw left (max)", perAnimSet = false, type = "float", attributes = "TargetYawLeft"},
      {title = "Pitch down (min)", perAnimSet = false, type = "float", attributes = "TargetPitchDown"},
      {title = "Pitch up (max)", perAnimSet = false, type = "float", attributes = "TargetPitchUp"},
    },
  },
  {
    title = "Input Defaults",
    isExpanded = false,
    details = 
    {
      {title = "Target", perAnimSet = false, type = "vector3", attributes = "LookPosition"},
      {title = "MoveBody", perAnimSet = false, type = "float", attributes = "WholeBodyLook"},
      {title = "Weight", perAnimSet = false, type = "float", attributes = "LookWeight"},
    },
  },
}

return {
  data = behaviourNode,
  attributeGroups = attributeGroups,
}
