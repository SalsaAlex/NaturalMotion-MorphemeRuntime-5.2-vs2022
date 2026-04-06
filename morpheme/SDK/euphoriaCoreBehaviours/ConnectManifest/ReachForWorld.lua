--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "Reach World",
  version = 4,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Makes a character reach to the specified target with their hands, using their whole body. ",
  group = "Behaviours",
  image = "ReachForWorldBehaviour.png",
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
    ["PositionForArm_0"] = 
    {
      input = true,
      helptext = "Point to reach for in world space.",
      type = "vector3",
      displayName = "Position {Arm0}",
    },

    ["PositionForArm_1"] = 
    {
      input = true,
      helptext = "Point to reach for in world space.",
      type = "vector3",
      displayName = "Position {Arm1}",
    },

    ["NormalForArm_0"] = 
    {
      input = true,
      helptext = "Normal of the point to be reached to, in world space. The back of the hand will be aligned with this direction.",
      type = "vector3",
      displayName = "Normal {Arm0}",
    },

    ["NormalForArm_1"] = 
    {
      input = true,
      helptext = "Normal of the point to be reached to, in world space. The back of the hand will be aligned with this direction.",
      type = "vector3",
      displayName = "Normal {Arm1}",
    },

    ["ReachImportanceForArm_0"] = 
    {
      input = true,
      helptext = "Weight for reaching with each arm, 0 will not reach with that arm.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Amount {Arm0}",
      value = 0.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["ReachImportanceForArm_1"] = 
    {
      input = true,
      helptext = "Weight for reaching with each arm, 0 will not reach with that arm.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Amount {Arm1}",
      value = 0.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["ReachImminenceForArm_0"] = 
    {
      input = true,
      helptext = "Reciprocal of the time period in which the reach request needs to achieve its reach target position. Larger values will cause the hands to move with more urgency. In 1/s (standard character).",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Imminence {Arm0}",
      value = 1.000000,
      min = 0.000000,
      max = 10.000000,
    },

    ["ReachImminenceForArm_1"] = 
    {
      input = true,
      helptext = "Reciprocal of the time period in which the reach request needs to achieve its reach target position. Larger values will cause the hands to move with more urgency. In 1/s (standard character).",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Imminence {Arm1}",
      value = 1.000000,
      min = 0.000000,
      max = 10.000000,
    },

    ["StiffnessScaleForArm_0"] = 
    {
      input = true,
      helptext = "A scale on the stiffness of each arm, 1 is normal stiffness.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Stiffness scale {Arm0}",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

    ["StiffnessScaleForArm_1"] = 
    {
      input = true,
      helptext = "A scale on the stiffness of each arm, 1 is normal stiffness.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Stiffness scale {Arm1}",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

    ["DampingScaleForArm_0"] = 
    {
      input = true,
      helptext = "A scale on the damping ratio, larger values will give slower movement without oscillation, smaller will give faster movement with more oscillations.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Damping ratio scale {Arm0}",
      value = 1.000000,
      min = 0.000000,
      max = 5.000000,
    },

    ["DampingScaleForArm_1"] = 
    {
      input = true,
      helptext = "A scale on the damping ratio, larger values will give slower movement without oscillation, smaller will give faster movement with more oscillations.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Damping ratio scale {Arm1}",
      value = 1.000000,
      min = 0.000000,
      max = 5.000000,
    },

    ["DriveCompensationScaleForArm_0"] = 
    {
      input = true,
      helptext = "A scale on the drive compensation, larger values give more controlled motion, smaller give more springy motion with more oscillations",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Drive compensation scale {Arm0}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["DriveCompensationScaleForArm_1"] = 
    {
      input = true,
      helptext = "A scale on the drive compensation, larger values give more controlled motion, smaller give more springy motion with more oscillations",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Drive compensation scale {Arm1}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["LookImminenceForArm_0"] = 
    {
      input = true,
      helptext = "Reciprocal of the time period in which the look request needs to achieve its look at goal. Larger values will cause the head to move with more urgency. In 1/s (standard character).",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Look imminence {Arm0}",
      value = 1.000000,
      min = 0.000000,
      max = 10.000000,
    },

    ["LookImminenceForArm_1"] = 
    {
      input = true,
      helptext = "Reciprocal of the time period in which the look request needs to achieve its look at goal. Larger values will cause the head to move with more urgency. In 1/s (standard character).",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Look imminence {Arm1}",
      value = 1.000000,
      min = 0.000000,
      max = 10.000000,
    },

    ["UnreachableTargetImportanceScale"] = 
    {
      input = true,
      helptext = "How much should the character reach for targets that out of range. 0 = not at all, 1 = try as hard as possible. ",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Unreachable target importance",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

  },

  pinOrder = 
  {
    "PositionForArm_0",
    "PositionForArm_1",
    "NormalForArm_0",
    "NormalForArm_1",
    "ReachImportanceForArm_0",
    "ReachImportanceForArm_1",
    "ReachImminenceForArm_0",
    "ReachImminenceForArm_1",
    "StiffnessScaleForArm_0",
    "StiffnessScaleForArm_1",
    "DampingScaleForArm_0",
    "DampingScaleForArm_1",
    "DriveCompensationScaleForArm_0",
    "DriveCompensationScaleForArm_1",
    "LookImminenceForArm_0",
    "LookImminenceForArm_1",
    "UnreachableTargetImportanceScale",
    "Result",
  },

  attributes = 
  {
    {
      name = "PositionWeightForArm_0",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "The weight associated with the target position in the IK solver. A larger value means the hand will get closer to the target position but may be further away from the target orientation.",
    },
    {
      name = "PositionWeightForArm_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "The weight associated with the target position in the IK solver. A larger value means the hand will get closer to the target position but may be further away from the target orientation.",
    },
    {
      name = "NormalWeightForArm_0",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "The weight associated with the target normal in the IK solver. A larger value means the hand will get closer to the target orientation but may be further away from the target position.",
    },
    {
      name = "NormalWeightForArm_1",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "The weight associated with the target normal in the IK solver. A larger value means the hand will get closer to the target orientation but may be further away from the target position.",
    },
    {
      name = "SlideAlongNormalForArm_0",
      type = "bool",
      value = false,
      helptext = "To force the hand to slide along the normal when approaching the target.",
    },
    {
      name = "SlideAlongNormalForArm_1",
      type = "bool",
      value = false,
      helptext = "To force the hand to slide along the normal when approaching the target.",
    },
    {
      name = "IKSubstepSize",
      type = "float",
      value = 1.000000,
      min = 0.010000,
      max = 1.000000,
      helptext = "Modify IK targets by interpolating between the hands current transform and the original target transform with this weight. A step size of 1 means no substepping.",
    },
    {
      name = "SwivelMode_0",
      type = "int",
      value = 0,
      min = -1,
      max = 1,
      perAnimSet = true,
      helptext = "Defines how or whether arm swivel should be applied to the reach: \"None\" means swivel is not required; \"Automatic\" means swivel will be determined procedurally by the behaviour based on the reach target, \"Specified\" means the behaviour will use the fixed swivel amount specified below.",
    },
    {
      name = "SwivelMode_1",
      type = "int",
      value = 0,
      min = -1,
      max = 1,
      perAnimSet = true,
      helptext = "Defines how or whether arm swivel should be applied to the reach: \"None\" means swivel is not required; \"Automatic\" means swivel will be determined procedurally by the behaviour based on the reach target, \"Specified\" means the behaviour will use the fixed swivel amount specified below.",
    },
    {
      name = "SwivelAmountForArm_0",
      type = "float",
      value = -1.000000,
      min = -1.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Swivel value to use when Swivel Mode is set to \"Specified\". Value in range [-1,1],  positive values drive elbows out and up, negative drive elbows in and down.",
    },
    {
      name = "SwivelAmountForArm_1",
      type = "float",
      value = -1.000000,
      min = -1.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Swivel value to use when Swivel Mode is set to \"Specified\". Value in range [-1,1],  positive values drive elbows out and up, negative drive elbows in and down.",
    },
    {
      name = "MaxReachScale_0",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Maximum reach as a scale of the full arm length",
    },
    {
      name = "MaxReachScale_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Maximum reach as a scale of the full arm length",
    },
    {
      name = "SelfAvoidanceEnable_0",
      type = "bool",
      value = true,
      perAnimSet = true,
      helptext = "Avoid collisions between the arm and torso when reaching.",
    },
    {
      name = "SelfAvoidanceEnable_1",
      type = "bool",
      value = true,
      perAnimSet = true,
      helptext = "Avoid collisions between the arm and torso when reaching.",
    },
    {
      name = "TorsoAvoidanceRadiusMultiplier_0",
      type = "float",
      value = 1.000000,
      min = 0.001000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Scales the self avoidance radius set on the spine limb.",
    },
    {
      name = "TorsoAvoidanceRadiusMultiplier_1",
      type = "float",
      value = 1.000000,
      min = 0.001000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Scales the self avoidance radius set on the spine limb.",
    },
    {
      name = "ChestRotationScaleForArm_0",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Multiplier applied to any rotation that this arm might request from the chest.",
    },
    {
      name = "ChestRotationScaleForArm_1",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Multiplier applied to any rotation that this arm might request from the chest.",
    },
    {
      name = "PelvisRotationScaleForArm_0",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Multiplier applied to any rotation that this arm might request from the pelvis",
    },
    {
      name = "PelvisRotationScaleForArm_1",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Multiplier applied to any rotation that this arm might request from the pelvis",
    },
    {
      name = "MaxChestTranslationForArm_0",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Maximum chest displacement (relative to spineLength) that this arm will request.",
    },
    {
      name = "MaxChestTranslationForArm_1",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Maximum chest displacement (relative to spineLength) that this arm will request.",
    },
    {
      name = "MaxPelvisTranslationForArm_0",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Maximum pelvis displacement (relative to legLength) that this arm will request.",
    },
    {
      name = "MaxPelvisTranslationForArm_1",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Maximum pelvis displacement (relative to legLength) that this arm will request.",
    },
    {
      name = "LookWeightToPositionForArm_0",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Specifies the weight to look at the reach target for each arm's reach target.",
    },
    {
      name = "LookWeightToPositionForArm_1",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Specifies the weight to look at the reach target for each arm's reach target.",
    },
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(24, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(6, "numBehaviourInts")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "SlideAlongNormalForArm_0") and 1 ) or 0 ) , string.format("Int_0_%d", asIdx-1))
      stream:writeInt( ( ( getValue(node, "SlideAlongNormalForArm_1") and 1 ) or 0 ) , string.format("Int_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt(getValue(node, "SwivelMode_0", asVal), string.format("Int_2_%d", asIdx-1))
      stream:writeInt(getValue(node, "SwivelMode_1", asVal), string.format("Int_3_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "SelfAvoidanceEnable_0", asVal) and 1 ) or 0 ) , string.format("Int_4_%d", asIdx-1))
      stream:writeInt( ( ( getValue(node, "SelfAvoidanceEnable_1", asVal) and 1 ) or 0 ) , string.format("Int_5_%d", asIdx-1))
    end

    stream:writeUInt(21, "numBehaviourFloats")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "PositionWeightForArm_0"), string.format("Float_0_%d", asIdx-1))
      stream:writeFloat(getValue(node, "PositionWeightForArm_1"), string.format("Float_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "NormalWeightForArm_0"), string.format("Float_2_%d", asIdx-1))
      stream:writeFloat(getValue(node, "NormalWeightForArm_1"), string.format("Float_3_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "IKSubstepSize"), string.format("Float_4_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "SwivelAmountForArm_0", asVal), string.format("Float_5_%d", asIdx-1))
      stream:writeFloat(getValue(node, "SwivelAmountForArm_1", asVal), string.format("Float_6_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MaxReachScale_0", asVal), string.format("Float_7_%d", asIdx-1))
      stream:writeFloat(getValue(node, "MaxReachScale_1", asVal), string.format("Float_8_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "TorsoAvoidanceRadiusMultiplier_0", asVal), string.format("Float_9_%d", asIdx-1))
      stream:writeFloat(getValue(node, "TorsoAvoidanceRadiusMultiplier_1", asVal), string.format("Float_10_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ChestRotationScaleForArm_0", asVal), string.format("Float_11_%d", asIdx-1))
      stream:writeFloat(getValue(node, "ChestRotationScaleForArm_1", asVal), string.format("Float_12_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "PelvisRotationScaleForArm_0", asVal), string.format("Float_13_%d", asIdx-1))
      stream:writeFloat(getValue(node, "PelvisRotationScaleForArm_1", asVal), string.format("Float_14_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MaxChestTranslationForArm_0", asVal), string.format("Float_15_%d", asIdx-1))
      stream:writeFloat(getValue(node, "MaxChestTranslationForArm_1", asVal), string.format("Float_16_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MaxPelvisTranslationForArm_0", asVal), string.format("Float_17_%d", asIdx-1))
      stream:writeFloat(getValue(node, "MaxPelvisTranslationForArm_1", asVal), string.format("Float_18_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "LookWeightToPositionForArm_0"), string.format("Float_19_%d", asIdx-1))
      stream:writeFloat(getValue(node, "LookWeightToPositionForArm_1"), string.format("Float_20_%d", asIdx-1))
    end

    stream:writeUInt(0, "numBehaviourVector3s")

    stream:writeUInt(0, "numBehaviourUInt64s")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numBehaviourNodeAnimationInputs")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numInputCPInts")
    stream:writeUInt(13, "numInputCPFloats")
    stream:writeUInt(0, "numInputCPUInt64s")
    stream:writeUInt(4, "numInputCPVector3s")
    stream:writeUInt(17, "numBehaviourControlParameterInputs")

    writeDataPinAltName(stream, node, "PositionForArm_0", "BehaviourControlParameterInputID_0" )
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_0")
    writeDataPinAltName(stream, node, "PositionForArm_1", "BehaviourControlParameterInputID_1" )
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_1")

    writeDataPinAltName(stream, node, "NormalForArm_0", "BehaviourControlParameterInputID_2" )
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_2")
    writeDataPinAltName(stream, node, "NormalForArm_1", "BehaviourControlParameterInputID_3" )
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_3")

    writeDataPinAltName(stream, node, "ReachImportanceForArm_0", "BehaviourControlParameterInputID_4" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_4")
    writeDataPinAltName(stream, node, "ReachImportanceForArm_1", "BehaviourControlParameterInputID_5" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_5")

    writeDataPinAltName(stream, node, "ReachImminenceForArm_0", "BehaviourControlParameterInputID_6" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_6")
    writeDataPinAltName(stream, node, "ReachImminenceForArm_1", "BehaviourControlParameterInputID_7" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_7")

    writeDataPinAltName(stream, node, "StiffnessScaleForArm_0", "BehaviourControlParameterInputID_8" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_8")
    writeDataPinAltName(stream, node, "StiffnessScaleForArm_1", "BehaviourControlParameterInputID_9" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_9")

    writeDataPinAltName(stream, node, "DampingScaleForArm_0", "BehaviourControlParameterInputID_10" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_10")
    writeDataPinAltName(stream, node, "DampingScaleForArm_1", "BehaviourControlParameterInputID_11" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_11")

    writeDataPinAltName(stream, node, "DriveCompensationScaleForArm_0", "BehaviourControlParameterInputID_12" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_12")
    writeDataPinAltName(stream, node, "DriveCompensationScaleForArm_1", "BehaviourControlParameterInputID_13" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_13")

    writeDataPinAltName(stream, node, "LookImminenceForArm_0", "BehaviourControlParameterInputID_14" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_14")
    writeDataPinAltName(stream, node, "LookImminenceForArm_1", "BehaviourControlParameterInputID_15" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_15")

    writeDataPinAltName(stream, node, "UnreachableTargetImportanceScale", "BehaviourControlParameterInputID_16")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_16")

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
    title = "Target Weight",
    helptext = "Set the relative importance of the target position and target orientation (normal) for each arm. ",
    isExpanded = true,
    details = 
    {
      {title = "PositionWeight {Arm0}", perAnimSet = false, type = "float", attributes = "PositionWeightForArm_0"},
      {title = "PositionWeight {Arm1}", perAnimSet = false, type = "float", attributes = "PositionWeightForArm_1"},
      {title = "NormalWeight {Arm0}", perAnimSet = false, type = "float", attributes = "NormalWeightForArm_0"},
      {title = "NormalWeight {Arm1}", perAnimSet = false, type = "float", attributes = "NormalWeightForArm_1"},
    },
  },
  {
    title = "Arm Movement",
    helptext = "Determine how each hand approaches the target.",
    isExpanded = false,
    details = 
    {
      {title = "Slide hand along target normal {Arm0}", perAnimSet = false, type = "bool", attributes = "SlideAlongNormalForArm_0"},
      {title = "Slide hand along target normal {Arm1}", perAnimSet = false, type = "bool", attributes = "SlideAlongNormalForArm_1"},
      {title = "IKSubstepSize", perAnimSet = false, type = "float", attributes = "IKSubstepSize"},
      {title = "Swivel Mode {Arm0}", perAnimSet = true, type = "integer", attributes = "SwivelMode_0"},
      {title = "Swivel Mode {Arm1}", perAnimSet = true, type = "integer", attributes = "SwivelMode_1"},
      {title = "Swivel {Arm0}", perAnimSet = true, type = "float", attributes = "SwivelAmountForArm_0"},
      {title = "Swivel {Arm1}", perAnimSet = true, type = "float", attributes = "SwivelAmountForArm_1"},
      {title = "Max reach {Arm0}", perAnimSet = true, type = "float", attributes = "MaxReachScale_0"},
      {title = "Max reach {Arm1}", perAnimSet = true, type = "float", attributes = "MaxReachScale_1"},
    },
  },
  {
    title = "Self Avoidance",
    helptext = "Configure he behaviour to avoid positioning the hands within a specified distance from the spine.",
    isExpanded = false,
    details = 
    {
      {title = "Enable {Arm0}", perAnimSet = true, type = "bool", attributes = "SelfAvoidanceEnable_0"},
      {title = "Enable {Arm1}", perAnimSet = true, type = "bool", attributes = "SelfAvoidanceEnable_1"},
      {title = "Scale spine radius {Arm0}", perAnimSet = true, type = "float", attributes = "TorsoAvoidanceRadiusMultiplier_0"},
      {title = "Scale spine radius {Arm1}", perAnimSet = true, type = "float", attributes = "TorsoAvoidanceRadiusMultiplier_1"},
    },
  },
  {
    title = "Full Body Motion",
    isExpanded = false,
    details = 
    {
      {title = "RotationScale {Arm0}", perAnimSet = true, type = "float", attributes = "ChestRotationScaleForArm_0"},
      {title = "RotationScale {Arm1}", perAnimSet = true, type = "float", attributes = "ChestRotationScaleForArm_1"},
      {title = "RotationScale {Arm0}", perAnimSet = true, type = "float", attributes = "PelvisRotationScaleForArm_0"},
      {title = "RotationScale {Arm1}", perAnimSet = true, type = "float", attributes = "PelvisRotationScaleForArm_1"},
      {title = "MaxTranslation {Arm0}", perAnimSet = true, type = "float", attributes = "MaxChestTranslationForArm_0"},
      {title = "MaxTranslation {Arm1}", perAnimSet = true, type = "float", attributes = "MaxChestTranslationForArm_1"},
      {title = "MaxTranslation {Arm0}", perAnimSet = true, type = "float", attributes = "MaxPelvisTranslationForArm_0"},
      {title = "MaxTranslation {Arm1}", perAnimSet = true, type = "float", attributes = "MaxPelvisTranslationForArm_1"},
    },
  },
  {
    title = "Look at Target",
    isExpanded = false,
    details = 
    {
      {title = "Weight {Arm0}", perAnimSet = false, type = "float", attributes = "LookWeightToPositionForArm_0"},
      {title = "Weight {Arm1}", perAnimSet = false, type = "float", attributes = "LookWeightToPositionForArm_1"},
    },
  },
  {
    title = "Target Input Defaults",
    isExpanded = false,
    details = 
    {
      {title = "Position {Arm0}", perAnimSet = false, type = "vector3", attributes = "PositionForArm_0"},
      {title = "Position {Arm1}", perAnimSet = false, type = "vector3", attributes = "PositionForArm_1"},
      {title = "Normal {Arm0}", perAnimSet = false, type = "vector3", attributes = "NormalForArm_0"},
      {title = "Normal {Arm1}", perAnimSet = false, type = "vector3", attributes = "NormalForArm_1"},
    },
  },
  {
    title = "Input Defaults",
    isExpanded = false,
    details = 
    {
      {title = "Amount {Arm0}", perAnimSet = false, type = "float", attributes = "ReachImportanceForArm_0"},
      {title = "Amount {Arm1}", perAnimSet = false, type = "float", attributes = "ReachImportanceForArm_1"},
      {title = "Imminence {Arm0}", perAnimSet = false, type = "float", attributes = "ReachImminenceForArm_0"},
      {title = "Imminence {Arm1}", perAnimSet = false, type = "float", attributes = "ReachImminenceForArm_1"},
      {title = "Stiffness scale {Arm0}", perAnimSet = false, type = "float", attributes = "StiffnessScaleForArm_0"},
      {title = "Stiffness scale {Arm1}", perAnimSet = false, type = "float", attributes = "StiffnessScaleForArm_1"},
      {title = "Damping ratio scale {Arm0}", perAnimSet = false, type = "float", attributes = "DampingScaleForArm_0"},
      {title = "Damping ratio scale {Arm1}", perAnimSet = false, type = "float", attributes = "DampingScaleForArm_1"},
      {title = "Drive compensation scale {Arm0}", perAnimSet = false, type = "float", attributes = "DriveCompensationScaleForArm_0"},
      {title = "Drive compensation scale {Arm1}", perAnimSet = false, type = "float", attributes = "DriveCompensationScaleForArm_1"},
      {title = "Look imminence {Arm0}", perAnimSet = false, type = "float", attributes = "LookImminenceForArm_0"},
      {title = "Look imminence {Arm1}", perAnimSet = false, type = "float", attributes = "LookImminenceForArm_1"},
      {title = "Unreachable target importance", perAnimSet = false, type = "float", attributes = "UnreachableTargetImportanceScale"},
    },
  },
}

return {
  data = behaviourNode,
  attributeGroups = attributeGroups,
}
