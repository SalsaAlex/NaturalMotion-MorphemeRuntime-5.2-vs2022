--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "Writhe",
  version = 2,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Allows a character to writhe and flail, based on a default pose or on the current limb position.",
  group = "Behaviours",
  image = "WritheBehaviour.png",
  id = generateNamespacedId(idNamespaces.NaturalMotion, 128),

  functionPins = 
  {
    ["DefaultPose"] = 
    {
      input = true,
      helptext = "Sets an average pose for the character's body during the behaviour. All random movement of body parts will be interpreted as offsets from this pose.",
      interfaces = 
      {
        "Transforms",
        "Time",
      },
    },

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
    ["ImportanceArm_0"] = 
    {
      input = true,
      helptext = "Sets the strength of the control this behaviour has over the arms, scaling the generated movement.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Arm {Arm0}",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

    ["ImportanceArm_1"] = 
    {
      input = true,
      helptext = "Sets the strength of the control this behaviour has over the arms, scaling the generated movement.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Arm {Arm1}",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

    ["ImportanceHead_0"] = 
    {
      input = true,
      helptext = "Sets the strength of the control this behaviour has over the heads, scaling the generated movement.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Head {Head0}",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

    ["ImportanceLeg_0"] = 
    {
      input = true,
      helptext = "Sets the strength of the control this behaviour has over the legs, scaling the generated movement.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Leg {Leg0}",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

    ["ImportanceLeg_1"] = 
    {
      input = true,
      helptext = "Sets the strength of the control this behaviour has over the legs, scaling the generated movement.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Leg {Leg1}",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

    ["ImportanceSpine_0"] = 
    {
      input = true,
      helptext = "Sets the strength of the control this behaviour has over the spines, scaling the generated movement.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Spine",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

  },

  pinOrder = 
  {
    "DefaultPose",
    "ImportanceArm_0",
    "ImportanceArm_1",
    "ImportanceHead_0",
    "ImportanceLeg_0",
    "ImportanceLeg_1",
    "ImportanceSpine_0",
    "Result",
  },

  attributes = 
  {
    {
      name = "ArmsDriveCompensationScale_0",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
      perAnimSet = true,
      helptext = "A scale on the drive compensation, larger values give more controlled motion, smaller give more springy motion with more oscillations.",
    },
    {
      name = "ArmsDriveCompensationScale_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
      perAnimSet = true,
      helptext = "A scale on the drive compensation, larger values give more controlled motion, smaller give more springy motion with more oscillations.",
    },
    {
      name = "ArmsAmplitudeScale_0",
      type = "float",
      value = 0.750000,
      min = 0.000000,
      max = 100.000000,
      perAnimSet = true,
      helptext = "Scales the length of the randomly generated displacement of the arm position.",
    },
    {
      name = "ArmsAmplitudeScale_1",
      type = "float",
      value = 0.750000,
      min = 0.000000,
      max = 100.000000,
      perAnimSet = true,
      helptext = "Scales the length of the randomly generated displacement of the arm position.",
    },
    {
      name = "ArmsFrequency_0",
      type = "float",
      value = 15.000000,
      min = 0.100000,
      max = 100.000000,
      perAnimSet = true,
      helptext = "How often the randomly generated arm position changes. In Hertz on the standard character.",
    },
    {
      name = "ArmsFrequency_1",
      type = "float",
      value = 15.000000,
      min = 0.100000,
      max = 100.000000,
      perAnimSet = true,
      helptext = "How often the randomly generated arm position changes. In Hertz on the standard character.",
    },
    {
      name = "ArmsBasedOnDefaultPose_0",
      type = "bool",
      value = true,
      perAnimSet = true,
      helptext = "When enabled, the arm is randomly displaced from the position in the pose. If disabled, the random displacement is added to the current position.",
    },
    {
      name = "ArmsBasedOnDefaultPose_1",
      type = "bool",
      value = true,
      perAnimSet = true,
      helptext = "When enabled, the arm is randomly displaced from the position in the pose. If disabled, the random displacement is added to the current position.",
    },
    {
      name = "HeadsDriveCompensationScale_0",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
      perAnimSet = true,
      helptext = "A scale on the drive compensation, larger values give more controlled motion, smaller give more springy motion with more oscillations.",
    },
    {
      name = "HeadsAmplitudeScale_0",
      type = "float",
      value = 0.750000,
      min = 0.000000,
      max = 100.000000,
      perAnimSet = true,
      helptext = "Scales the length of the randomly generated displacement of the head position.",
    },
    {
      name = "HeadsFrequency_0",
      type = "float",
      value = 15.000000,
      min = 0.100000,
      max = 100.000000,
      perAnimSet = true,
      helptext = "How often the randomly generated head position changes. In Hertz on the standard character.",
    },
    {
      name = "HeadsBasedOnDefaultPose_0",
      type = "bool",
      value = true,
      perAnimSet = true,
      helptext = "When enabled, the head is randomly displaced from the position in the pose. If disabled, the random displacement is added to the current position.",
    },
    {
      name = "LegsDriveCompensationScale_0",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
      perAnimSet = true,
      helptext = "A scale on the drive compensation, larger values give more controlled motion, smaller give more springy motion with more oscillations.",
    },
    {
      name = "LegsDriveCompensationScale_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
      perAnimSet = true,
      helptext = "A scale on the drive compensation, larger values give more controlled motion, smaller give more springy motion with more oscillations.",
    },
    {
      name = "LegsAmplitudeScale_0",
      type = "float",
      value = 0.750000,
      min = 0.000000,
      max = 100.000000,
      perAnimSet = true,
      helptext = "Scales the length of the randomly generated displacement of the leg position.",
    },
    {
      name = "LegsAmplitudeScale_1",
      type = "float",
      value = 0.750000,
      min = 0.000000,
      max = 100.000000,
      perAnimSet = true,
      helptext = "Scales the length of the randomly generated displacement of the leg position.",
    },
    {
      name = "LegsFrequency_0",
      type = "float",
      value = 10.000000,
      min = 0.100000,
      max = 100.000000,
      perAnimSet = true,
      helptext = "How often the randomly generated leg position changes. In Hertz on the standard character.",
    },
    {
      name = "LegsFrequency_1",
      type = "float",
      value = 10.000000,
      min = 0.100000,
      max = 100.000000,
      perAnimSet = true,
      helptext = "How often the randomly generated leg position changes. In Hertz on the standard character.",
    },
    {
      name = "LegsBasedOnDefaultPose_0",
      type = "bool",
      value = true,
      perAnimSet = true,
      helptext = "When enabled, the leg is randomly displaced from the position in the pose. If disabled, the random displacement is added to the current position.",
    },
    {
      name = "LegsBasedOnDefaultPose_1",
      type = "bool",
      value = true,
      perAnimSet = true,
      helptext = "When enabled, the leg is randomly displaced from the position in the pose. If disabled, the random displacement is added to the current position.",
    },
    {
      name = "SpinesDriveCompensationScale_0",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
      perAnimSet = true,
      helptext = "A scale on the drive compensation, larger values give more controlled motion, smaller give more springy motion with more oscillations.",
    },
    {
      name = "SpinesAmplitudeScale_0",
      type = "float",
      value = 0.750000,
      min = 0.000000,
      max = 100.000000,
      perAnimSet = true,
      helptext = "Scales the length of the randomly generated displacement of the spine position.",
    },
    {
      name = "SpinesFrequency_0",
      type = "float",
      value = 15.000000,
      min = 0.100000,
      max = 100.000000,
      perAnimSet = true,
      helptext = "How often the randomly generated spine position changes. In Hertz on the standard character.",
    },
    {
      name = "SpinesBasedOnDefaultPose_0",
      type = "bool",
      value = true,
      perAnimSet = true,
      helptext = "When enabled, the spine is randomly displaced from the position in the pose. If disabled, the random displacement is added to the current position.",
    },
    {
      name = "DefaultPose_ApplyToArm_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Arm.",
    },
    {
      name = "DefaultPose_ApplyToArm_2",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Arm.",
    },
    {
      name = "DefaultPose_ApplyToHead_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Head.",
    },
    {
      name = "DefaultPose_ApplyToLeg_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Leg.",
    },
    {
      name = "DefaultPose_ApplyToLeg_2",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Leg.",
    },
    {
      name = "DefaultPose_ApplyToSpine_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Spine.",
    },
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(29, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(6, "numBehaviourInts")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "ArmsBasedOnDefaultPose_0", asVal) and 1 ) or 0 ) , string.format("Int_0_%d", asIdx-1))
      stream:writeInt( ( ( getValue(node, "ArmsBasedOnDefaultPose_1", asVal) and 1 ) or 0 ) , string.format("Int_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "HeadsBasedOnDefaultPose_0", asVal) and 1 ) or 0 ) , string.format("Int_2_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "LegsBasedOnDefaultPose_0", asVal) and 1 ) or 0 ) , string.format("Int_3_%d", asIdx-1))
      stream:writeInt( ( ( getValue(node, "LegsBasedOnDefaultPose_1", asVal) and 1 ) or 0 ) , string.format("Int_4_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "SpinesBasedOnDefaultPose_0", asVal) and 1 ) or 0 ) , string.format("Int_5_%d", asIdx-1))
    end

    stream:writeUInt(24, "numBehaviourFloats")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ArmsDriveCompensationScale_0", asVal), string.format("Float_0_%d", asIdx-1))
      stream:writeFloat(getValue(node, "ArmsDriveCompensationScale_1", asVal), string.format("Float_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ArmsAmplitudeScale_0", asVal), string.format("Float_2_%d", asIdx-1))
      stream:writeFloat(getValue(node, "ArmsAmplitudeScale_1", asVal), string.format("Float_3_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ArmsFrequency_0", asVal), string.format("Float_4_%d", asIdx-1))
      stream:writeFloat(getValue(node, "ArmsFrequency_1", asVal), string.format("Float_5_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "HeadsDriveCompensationScale_0", asVal), string.format("Float_6_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "HeadsAmplitudeScale_0", asVal), string.format("Float_7_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "HeadsFrequency_0", asVal), string.format("Float_8_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "LegsDriveCompensationScale_0", asVal), string.format("Float_9_%d", asIdx-1))
      stream:writeFloat(getValue(node, "LegsDriveCompensationScale_1", asVal), string.format("Float_10_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "LegsAmplitudeScale_0", asVal), string.format("Float_11_%d", asIdx-1))
      stream:writeFloat(getValue(node, "LegsAmplitudeScale_1", asVal), string.format("Float_12_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "LegsFrequency_0", asVal), string.format("Float_13_%d", asIdx-1))
      stream:writeFloat(getValue(node, "LegsFrequency_1", asVal), string.format("Float_14_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "SpinesDriveCompensationScale_0", asVal), string.format("Float_15_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "SpinesAmplitudeScale_0", asVal), string.format("Float_16_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "SpinesFrequency_0", asVal), string.format("Float_17_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "DefaultPose_ApplyToArm_1"), string.format("Float_18_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "DefaultPose_ApplyToArm_2"), string.format("Float_19_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "DefaultPose_ApplyToHead_1"), string.format("Float_20_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "DefaultPose_ApplyToLeg_1"), string.format("Float_21_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "DefaultPose_ApplyToLeg_2"), string.format("Float_22_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "DefaultPose_ApplyToSpine_1"), string.format("Float_23_%d", asIdx-1))
    end

    stream:writeUInt(0, "numBehaviourVector3s")

    stream:writeUInt(0, "numBehaviourUInt64s")

    -------------------------------------------------------------------

    local inputNodeID_DefaultPose = -1
    if isConnected{SourcePin = node .. ".DefaultPose", ResolveReferences = true} then
      inputNodeID_DefaultPose = getConnectedNodeID(node, "DefaultPose")
    end

    stream:writeUInt(1, "numBehaviourNodeAnimationInputs")

    stream:writeNetworkNodeId(inputNodeID_DefaultPose, "BehaviourNodeAnimationInput_0")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numInputCPInts")
    stream:writeUInt(6, "numInputCPFloats")
    stream:writeUInt(0, "numInputCPUInt64s")
    stream:writeUInt(0, "numInputCPVector3s")
    stream:writeUInt(6, "numBehaviourControlParameterInputs")

    writeDataPinAltName(stream, node, "ImportanceArm_0", "BehaviourControlParameterInputID_0" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_0")
    writeDataPinAltName(stream, node, "ImportanceArm_1", "BehaviourControlParameterInputID_1" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_1")

    writeDataPinAltName(stream, node, "ImportanceHead_0", "BehaviourControlParameterInputID_2" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_2")

    writeDataPinAltName(stream, node, "ImportanceLeg_0", "BehaviourControlParameterInputID_3" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_3")
    writeDataPinAltName(stream, node, "ImportanceLeg_1", "BehaviourControlParameterInputID_4" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_4")

    writeDataPinAltName(stream, node, "ImportanceSpine_0", "BehaviourControlParameterInputID_5" )
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
    title = "Arms",
    isExpanded = true,
    details = 
    {
      {title = "DriveCompensationScale {Arm0}", perAnimSet = true, type = "float", attributes = "ArmsDriveCompensationScale_0"},
      {title = "DriveCompensationScale {Arm1}", perAnimSet = true, type = "float", attributes = "ArmsDriveCompensationScale_1"},
      {title = "AmplitudeScale {Arm0}", perAnimSet = true, type = "float", attributes = "ArmsAmplitudeScale_0"},
      {title = "AmplitudeScale {Arm1}", perAnimSet = true, type = "float", attributes = "ArmsAmplitudeScale_1"},
      {title = "Frequency {Arm0}", perAnimSet = true, type = "float", attributes = "ArmsFrequency_0"},
      {title = "Frequency {Arm1}", perAnimSet = true, type = "float", attributes = "ArmsFrequency_1"},
      {title = "UsePose {Arm0}", perAnimSet = true, type = "bool", attributes = "ArmsBasedOnDefaultPose_0"},
      {title = "UsePose {Arm1}", perAnimSet = true, type = "bool", attributes = "ArmsBasedOnDefaultPose_1"},
    },
  },
  {
    title = "Heads",
    isExpanded = false,
    details = 
    {
      {title = "DriveCompensationScale {Head0}", perAnimSet = true, type = "float", attributes = "HeadsDriveCompensationScale_0"},
      {title = "AmplitudeScale {Head0}", perAnimSet = true, type = "float", attributes = "HeadsAmplitudeScale_0"},
      {title = "Frequency {Head0}", perAnimSet = true, type = "float", attributes = "HeadsFrequency_0"},
      {title = "UsePose {Head0}", perAnimSet = true, type = "bool", attributes = "HeadsBasedOnDefaultPose_0"},
    },
  },
  {
    title = "Legs",
    isExpanded = false,
    details = 
    {
      {title = "DriveCompensationScale {Leg0}", perAnimSet = true, type = "float", attributes = "LegsDriveCompensationScale_0"},
      {title = "DriveCompensationScale {Leg1}", perAnimSet = true, type = "float", attributes = "LegsDriveCompensationScale_1"},
      {title = "AmplitudeScale {Leg0}", perAnimSet = true, type = "float", attributes = "LegsAmplitudeScale_0"},
      {title = "AmplitudeScale {Leg1}", perAnimSet = true, type = "float", attributes = "LegsAmplitudeScale_1"},
      {title = "Frequency {Leg0}", perAnimSet = true, type = "float", attributes = "LegsFrequency_0"},
      {title = "Frequency {Leg1}", perAnimSet = true, type = "float", attributes = "LegsFrequency_1"},
      {title = "UsePose {Leg0}", perAnimSet = true, type = "bool", attributes = "LegsBasedOnDefaultPose_0"},
      {title = "UsePose {Leg1}", perAnimSet = true, type = "bool", attributes = "LegsBasedOnDefaultPose_1"},
    },
  },
  {
    title = "Spines",
    isExpanded = false,
    details = 
    {
      {title = "DriveCompensationScale ", perAnimSet = true, type = "float", attributes = "SpinesDriveCompensationScale_0"},
      {title = "AmplitudeScale ", perAnimSet = true, type = "float", attributes = "SpinesAmplitudeScale_0"},
      {title = "Frequency ", perAnimSet = true, type = "float", attributes = "SpinesFrequency_0"},
      {title = "UsePose ", perAnimSet = true, type = "bool", attributes = "SpinesBasedOnDefaultPose_0"},
    },
  },
  {
    title = "Strength Scale Defaults",
    helptext = "The strength this behaviour has to control each limb type",
    isExpanded = false,
    details = 
    {
      {title = "Arm {Arm0}", perAnimSet = false, type = "float", attributes = "ImportanceArm_0"},
      {title = "Arm {Arm1}", perAnimSet = false, type = "float", attributes = "ImportanceArm_1"},
      {title = "Head {Head0}", perAnimSet = false, type = "float", attributes = "ImportanceHead_0"},
      {title = "Leg {Leg0}", perAnimSet = false, type = "float", attributes = "ImportanceLeg_0"},
      {title = "Leg {Leg1}", perAnimSet = false, type = "float", attributes = "ImportanceLeg_1"},
      {title = "Spine ", perAnimSet = false, type = "float", attributes = "ImportanceSpine_0"},
    },
  },
  {
    title = "DefaultPose",
    helptext = "Sets an average pose for the character's body during the behaviour. All random movement of body parts will be interpreted as offsets from this pose.",
    isExpanded = false,
    details = 
    {
      {title = "Arm {Arm0}", perAnimSet = false, type = "float", attributes = "DefaultPose_ApplyToArm_1"},
      {title = "Arm {Arm1}", perAnimSet = false, type = "float", attributes = "DefaultPose_ApplyToArm_2"},
      {title = "Head {Head0}", perAnimSet = false, type = "float", attributes = "DefaultPose_ApplyToHead_1"},
      {title = "Leg {Leg0}", perAnimSet = false, type = "float", attributes = "DefaultPose_ApplyToLeg_1"},
      {title = "Leg {Leg1}", perAnimSet = false, type = "float", attributes = "DefaultPose_ApplyToLeg_2"},
      {title = "Spine {Spine0}", perAnimSet = false, type = "float", attributes = "DefaultPose_ApplyToSpine_1"},
    },
  },
}

return {
  data = behaviourNode,
  attributeGroups = attributeGroups,
}
