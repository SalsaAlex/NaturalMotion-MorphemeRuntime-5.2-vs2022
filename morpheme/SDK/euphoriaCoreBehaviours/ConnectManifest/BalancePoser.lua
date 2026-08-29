--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "Balance Poser",
  version = 2,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Modifies a character's balance behaviour to change the balancing posture, such as leaning, twisting and crouching. ",
  group = "Behaviours",
  image = "BalancePoserBehaviour.png",
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
    ["Yaw"] = 
    {
      input = true,
      helptext = "Sets the rotation around the spine axis. Positive values cause a rotation to the left. In degrees.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Yaw",
    },

    ["Pitch"] = 
    {
      input = true,
      helptext = "Sets the angle with which the character leans forwards or backwards. Positive values pitch the character forwards and down. In degrees.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Pitch",
    },

    ["Roll"] = 
    {
      input = true,
      helptext = "Sets the angle with which the character leans to the left or right. Positive values tip the character to its left. In degrees.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Roll",
    },

    ["CrouchAmount"] = 
    {
      input = true,
      helptext = "Interpolate the character's target height (as a fraction of its full height) between 1 (when crouch amount is 0) and the value of the behaviour's \"Crouch height fraction\" attribute (when crouch amount is 1).",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "CrouchAmount",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["ChestTranslation"] = 
    {
      input = true,
      helptext = "Sets the requested translation of the chest from the current position. This is interpreted in absolute units if the 'translation in' attribute is set to 'World Space'. Otherwise, if the 'translation in' attribute is set to 'Character Space' then +ve x is forwards, +ve y is upwards, +ve z is rightwards.",
      type = "vector3",
      mode = "HiddenInNetworkEditor",
      displayName = "ChestTranslation",
    },

  },

  pinOrder = 
  {
    "Yaw",
    "Pitch",
    "Roll",
    "CrouchAmount",
    "ChestTranslation",
    "Result",
  },

  attributes = 
  {
    {
      name = "CrouchHeightFraction",
      type = "float",
      value = 0.600000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Fraction of the balance pose pelvis height when fully crouching",
    },
    {
      name = "PelvisWeight",
      type = "float",
      value = 0.200000,
      min = -1.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Fraction of the pose angle change applied to the pelvis",
    },
    {
      name = "SpineWeight",
      type = "float",
      value = 0.800000,
      min = -1.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Fraction of the pose angle change applied to the chest",
    },
    {
      name = "HeadWeight",
      type = "float",
      value = 0.500000,
      min = -1.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Fraction of the pose angle change applied to the head",
    },
    {
      name = "PitchPerCrouchAmount",
      type = "float",
      value = 30.000000,
      perAnimSet = true,
      helptext = "Pitch forward (degrees) when CrouchAmount = 1",
    },
    {
      name = "ChestTranslationInCharacterSpace",
      type = "bool",
      value = false,
      helptext = "If true then the chest translation is done in character space so that x = forward, y = up and z = right",
    },
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(7, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(1, "numBehaviourInts")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "ChestTranslationInCharacterSpace") and 1 ) or 0 ) , string.format("Int_0_%d", asIdx-1))
    end

    stream:writeUInt(5, "numBehaviourFloats")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "CrouchHeightFraction", asVal), string.format("Float_0_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "PelvisWeight", asVal), string.format("Float_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "SpineWeight", asVal), string.format("Float_2_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "HeadWeight", asVal), string.format("Float_3_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "PitchPerCrouchAmount", asVal), string.format("Float_4_%d", asIdx-1))
    end

    stream:writeUInt(0, "numBehaviourVector3s")

    stream:writeUInt(0, "numBehaviourUInt64s")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numBehaviourNodeAnimationInputs")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numInputCPInts")
    stream:writeUInt(4, "numInputCPFloats")
    stream:writeUInt(0, "numInputCPUInt64s")
    stream:writeUInt(1, "numInputCPVector3s")
    stream:writeUInt(5, "numBehaviourControlParameterInputs")

    writeDataPinAltName(stream, node, "Yaw", "BehaviourControlParameterInputID_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_0")

    writeDataPinAltName(stream, node, "Pitch", "BehaviourControlParameterInputID_1")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_1")

    writeDataPinAltName(stream, node, "Roll", "BehaviourControlParameterInputID_2")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_2")

    writeDataPinAltName(stream, node, "CrouchAmount", "BehaviourControlParameterInputID_3")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_3")

    writeDataPinAltName(stream, node, "ChestTranslation", "BehaviourControlParameterInputID_4")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_4")

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
      {title = "CrouchHeightFraction", perAnimSet = true, type = "float", attributes = "CrouchHeightFraction"},
      {title = "PelvisWeight", perAnimSet = true, type = "float", attributes = "PelvisWeight"},
      {title = "SpineWeight", perAnimSet = true, type = "float", attributes = "SpineWeight"},
      {title = "HeadWeight", perAnimSet = true, type = "float", attributes = "HeadWeight"},
      {title = "PitchPerCrouchAmount", perAnimSet = true, type = "float", attributes = "PitchPerCrouchAmount"},
      {title = "ChestTranslationInCharacterSpace", perAnimSet = false, type = "bool", attributes = "ChestTranslationInCharacterSpace"},
    },
  },
  {
    title = "Input Defaults",
    isExpanded = false,
    details = 
    {
      {title = "Yaw", perAnimSet = false, type = "float", attributes = "Yaw"},
      {title = "Pitch", perAnimSet = false, type = "float", attributes = "Pitch"},
      {title = "Roll", perAnimSet = false, type = "float", attributes = "Roll"},
      {title = "CrouchAmount", perAnimSet = false, type = "float", attributes = "CrouchAmount"},
      {title = "ChestTranslation", perAnimSet = false, type = "vector3", attributes = "ChestTranslation"},
    },
  },
}

return {
  data = behaviourNode,
  attributeGroups = attributeGroups,
}
