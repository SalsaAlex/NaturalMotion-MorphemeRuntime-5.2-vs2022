--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "Reach Body",
  version = 5,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Allows a character to reach a target with its hands using its whole body. ",
  group = "Behaviours",
  image = "ReachForBodyBehaviour.png",
  id = generateNamespacedId(idNamespaces.NaturalMotion, 128),

  functionPins = 
  {
    ["ArmReachForBodyPose"] = 
    {
      input = true,
      helptext = "Sets a suitable pose for an arm that has become a reach target for another arm. Typically this pose will place the target arm in a position where it can be reached by another arm.",
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
    ["newHit_0"] = 
    {
      input = true,
      helptext = "Indicates that a new target has been supplied. Should be set to true on the frame in which the target is changed.",
      type = "bool",
      mode = "HiddenInNetworkEditor",
      displayName = "Use latest hit info {Arm0}",
      value = false,
    },

    ["newHit_1"] = 
    {
      input = true,
      helptext = "Indicates that a new target has been supplied. Should be set to true on the frame in which the target is changed.",
      type = "bool",
      mode = "HiddenInNetworkEditor",
      displayName = "Use latest hit info {Arm1}",
      value = false,
    },

    ["Strength_0"] = 
    {
      input = true,
      helptext = "Sets the emphasis placed on the reach behaviour for this limb. A reach toward a target can be effectively faded out by ramping this value from 1 to 0.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Amount {Arm0}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["Strength_1"] = 
    {
      input = true,
      helptext = "Sets the emphasis placed on the reach behaviour for this limb. A reach toward a target can be effectively faded out by ramping this value from 1 to 0.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Amount {Arm1}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["EffectorSpeedLimit_0"] = 
    {
      input = true,
      helptext = "Sets the maximum speed of the hand when moving towards the target. In m/s (standard character).",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "EffectorSpeedLimit {Arm0}",
      value = 20.000000,
      min = 0.000000,
      max = 20.000000,
    },

    ["EffectorSpeedLimit_1"] = 
    {
      input = true,
      helptext = "Sets the maximum speed of the hand when moving towards the target. In m/s (standard character).",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "EffectorSpeedLimit {Arm1}",
      value = 20.000000,
      min = 0.000000,
      max = 20.000000,
    },

    ["LimbIndex_0"] = 
    {
      input = true,
      helptext = "Index of the limb to reach for",
      type = "int",
      subtype = "LimbIndex",
      mode = "HiddenInNetworkEditor",
      displayName = "Limb {Arm0}",
      value = -1,
      min = -1,
      max = 10,
perAnimSet = true,     },

    ["LimbIndex_1"] = 
    {
      input = true,
      helptext = "Index of the limb to reach for",
      type = "int",
      subtype = "LimbIndex",
      mode = "HiddenInNetworkEditor",
      displayName = "Limb {Arm1}",
      value = -1,
      min = -1,
      max = 10,
perAnimSet = true,     },

    ["PartIndex_0"] = 
    {
      input = true,
      helptext = "Index of the part to reach for, e.g. the upper arm, or lower leg.",
      type = "int",
      subtype = "LimbPartIndex",
      mode = "HiddenInNetworkEditor",
      displayName = "Part {Arm0}",
      value = -1,
      min = -1,
      max = 10,
perAnimSet = true,     },

    ["PartIndex_1"] = 
    {
      input = true,
      helptext = "Index of the part to reach for, e.g. the upper arm, or lower leg.",
      type = "int",
      subtype = "LimbPartIndex",
      mode = "HiddenInNetworkEditor",
      displayName = "Part {Arm1}",
      value = -1,
      min = -1,
      max = 10,
perAnimSet = true,     },

    ["Position_0"] = 
    {
      input = true,
      helptext = "Local surface point to reach to on the specified limb part (in frame of a physics joint).",
      type = "vector3",
      displayName = "Position {Arm0}",
    },

    ["Position_1"] = 
    {
      input = true,
      helptext = "Local surface point to reach to on the specified limb part (in frame of a physics joint).",
      type = "vector3",
      displayName = "Position {Arm1}",
    },

    ["Normal_0"] = 
    {
      input = true,
      helptext = "Local surface normal of point to be reached to (in frame of a physics joint).",
      type = "vector3",
      displayName = "Normal {Arm0}",
    },

    ["Normal_1"] = 
    {
      input = true,
      helptext = "Local surface normal of point to be reached to (in frame of a physics joint).",
      type = "vector3",
      displayName = "Normal {Arm1}",
    },

  },

  pinOrder = 
  {
    "ArmReachForBodyPose",
    "newHit_0",
    "newHit_1",
    "Strength_0",
    "Strength_1",
    "EffectorSpeedLimit_0",
    "EffectorSpeedLimit_1",
    "LimbIndex_0",
    "LimbIndex_1",
    "PartIndex_0",
    "PartIndex_1",
    "Position_0",
    "Position_1",
    "Normal_0",
    "Normal_1",
    "Result",
  },

  attributes = 
  {
    {
      name = "OutOfReachTimeout_0",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "Give up after this time out of reach, in seconds (standard character).",
    },
    {
      name = "OutOfReachTimeout_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "Give up after this time out of reach, in seconds (standard character).",
    },
    {
      name = "RampDownFailedDuration_0",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "Back off ramp (after failed reach) duration, in seconds (standard character).",
    },
    {
      name = "RampDownFailedDuration_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "Back off ramp (after failed reach) duration, in seconds (standard character).",
    },
    {
      name = "WithinReachTimeout_0",
      type = "float",
      value = 5.000000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "Give up after this time within reach, in seconds (standard character)..",
    },
    {
      name = "WithinReachTimeout_1",
      type = "float",
      value = 5.000000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "Give up after this time within reach, in seconds (standard character)..",
    },
    {
      name = "RampDownCompletedDuration_0",
      type = "float",
      value = 2.000000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "Back off ramp (after successful reach) duration, in seconds (standard character)..",
    },
    {
      name = "RampDownCompletedDuration_1",
      type = "float",
      value = 2.000000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "Back off ramp (after successful reach) duration, in seconds (standard character)..",
    },
    {
      name = "OutOfReachDistance_0",
      type = "float",
      value = 0.150000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "On target when within this distance, in metres (standard character).",
    },
    {
      name = "OutOfReachDistance_1",
      type = "float",
      value = 0.150000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "On target when within this distance, in metres (standard character).",
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
      name = "SwivelAmount_0",
      type = "float",
      value = 0.000000,
      min = -1.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Swivel value to use when Swivel Mode is \"Specified\". Value in range [-1,1],  positive values drive elbows out and up, negative drive elbows in and down.",
    },
    {
      name = "SwivelAmount_1",
      type = "float",
      value = 0.000000,
      min = -1.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Swivel value to use when Swivel Mode is \"Specified\". Value in range [-1,1],  positive values drive elbows out and up, negative drive elbows in and down.",
    },
    {
      name = "MaxArmExtensionScale_0",
      type = "float",
      value = 1.000000,
      min = 0.100000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Limit the maximum extension of the arm. Expressed as a proportion of the total arm length.",
    },
    {
      name = "MaxArmExtensionScale_1",
      type = "float",
      value = 1.000000,
      min = 0.100000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Limit the maximum extension of the arm. Expressed as a proportion of the total arm length.",
    },
    {
      name = "SurfacePenetration_0",
      type = "float",
      value = 0.010000,
      min = 0.000000,
      max = 0.100000,
      perAnimSet = true,
      helptext = "Small offset, to drive hand against surface, in metres (standard character).",
    },
    {
      name = "SurfacePenetration_1",
      type = "float",
      value = 0.010000,
      min = 0.000000,
      max = 0.100000,
      perAnimSet = true,
      helptext = "Small offset, to drive hand against surface, in metres (standard character).",
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
      name = "CollisionGroupIndex",
      type = "int",
      value = -1,
      helptext = "Collision group used to disable hand-hand collision when reaching. This is the collision group within the collision groups editor.",
    },
    {
      name = "ArmReachForBodyPose_ApplyToArm_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Arm.",
    },
    {
      name = "ArmReachForBodyPose_ApplyToArm_2",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Arm.",
    },
    {
      name = "ArmReachForBodyPose_ApplyToHead_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Head.",
    },
    {
      name = "ArmReachForBodyPose_ApplyToLeg_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Leg.",
    },
    {
      name = "ArmReachForBodyPose_ApplyToLeg_2",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Leg.",
    },
    {
      name = "ArmReachForBodyPose_ApplyToSpine_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Spine.",
    },
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(23, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(3, "numBehaviourInts")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt(getValue(node, "SwivelMode_0", asVal), string.format("Int_0_%d", asIdx-1))
      stream:writeInt(getValue(node, "SwivelMode_1", asVal), string.format("Int_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt(getValue(node, "CollisionGroupIndex"), string.format("Int_2_%d", asIdx-1))
    end

    stream:writeUInt(24, "numBehaviourFloats")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "OutOfReachTimeout_0", asVal), string.format("Float_0_%d", asIdx-1))
      stream:writeFloat(getValue(node, "OutOfReachTimeout_1", asVal), string.format("Float_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "RampDownFailedDuration_0", asVal), string.format("Float_2_%d", asIdx-1))
      stream:writeFloat(getValue(node, "RampDownFailedDuration_1", asVal), string.format("Float_3_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "WithinReachTimeout_0", asVal), string.format("Float_4_%d", asIdx-1))
      stream:writeFloat(getValue(node, "WithinReachTimeout_1", asVal), string.format("Float_5_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "RampDownCompletedDuration_0", asVal), string.format("Float_6_%d", asIdx-1))
      stream:writeFloat(getValue(node, "RampDownCompletedDuration_1", asVal), string.format("Float_7_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "OutOfReachDistance_0", asVal), string.format("Float_8_%d", asIdx-1))
      stream:writeFloat(getValue(node, "OutOfReachDistance_1", asVal), string.format("Float_9_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "SwivelAmount_0", asVal), string.format("Float_10_%d", asIdx-1))
      stream:writeFloat(getValue(node, "SwivelAmount_1", asVal), string.format("Float_11_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MaxArmExtensionScale_0", asVal), string.format("Float_12_%d", asIdx-1))
      stream:writeFloat(getValue(node, "MaxArmExtensionScale_1", asVal), string.format("Float_13_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "SurfacePenetration_0", asVal), string.format("Float_14_%d", asIdx-1))
      stream:writeFloat(getValue(node, "SurfacePenetration_1", asVal), string.format("Float_15_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "TorsoAvoidanceRadiusMultiplier_0", asVal), string.format("Float_16_%d", asIdx-1))
      stream:writeFloat(getValue(node, "TorsoAvoidanceRadiusMultiplier_1", asVal), string.format("Float_17_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ArmReachForBodyPose_ApplyToArm_1"), string.format("Float_18_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ArmReachForBodyPose_ApplyToArm_2"), string.format("Float_19_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ArmReachForBodyPose_ApplyToHead_1"), string.format("Float_20_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ArmReachForBodyPose_ApplyToLeg_1"), string.format("Float_21_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ArmReachForBodyPose_ApplyToLeg_2"), string.format("Float_22_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ArmReachForBodyPose_ApplyToSpine_1"), string.format("Float_23_%d", asIdx-1))
    end

    stream:writeUInt(0, "numBehaviourVector3s")

    stream:writeUInt(0, "numBehaviourUInt64s")

    -------------------------------------------------------------------

    local inputNodeID_ArmReachForBodyPose = -1
    if isConnected{SourcePin = node .. ".ArmReachForBodyPose", ResolveReferences = true} then
      inputNodeID_ArmReachForBodyPose = getConnectedNodeID(node, "ArmReachForBodyPose")
    end

    stream:writeUInt(1, "numBehaviourNodeAnimationInputs")

    stream:writeNetworkNodeId(inputNodeID_ArmReachForBodyPose, "BehaviourNodeAnimationInput_0")

    -------------------------------------------------------------------

    stream:writeUInt(6, "numInputCPInts")
    stream:writeUInt(4, "numInputCPFloats")
    stream:writeUInt(0, "numInputCPUInt64s")
    stream:writeUInt(4, "numInputCPVector3s")
    stream:writeUInt(14, "numBehaviourControlParameterInputs")

    writeDataPinAltName(stream, node, "newHit_0", "BehaviourControlParameterInputID_0" )
    stream:writeString("ATTRIB_SEMANTIC_CP_BOOL", "BehaviourControlParameterInputType_0")
    writeDataPinAltName(stream, node, "newHit_1", "BehaviourControlParameterInputID_1" )
    stream:writeString("ATTRIB_SEMANTIC_CP_BOOL", "BehaviourControlParameterInputType_1")

    writeDataPinAltName(stream, node, "Strength_0", "BehaviourControlParameterInputID_2" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_2")
    writeDataPinAltName(stream, node, "Strength_1", "BehaviourControlParameterInputID_3" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_3")

    writeDataPinAltName(stream, node, "EffectorSpeedLimit_0", "BehaviourControlParameterInputID_4" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_4")
    writeDataPinAltName(stream, node, "EffectorSpeedLimit_1", "BehaviourControlParameterInputID_5" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_5")

    writePerAnimSetDataPinAltName(stream, node, "LimbIndex_0", "BehaviourControlParameterInputID_6" )
    stream:writeString("ATTRIB_SEMANTIC_CP_INT", "BehaviourControlParameterInputType_6")
    writePerAnimSetDataPinAltName(stream, node, "LimbIndex_1", "BehaviourControlParameterInputID_7" )
    stream:writeString("ATTRIB_SEMANTIC_CP_INT", "BehaviourControlParameterInputType_7")

    writePerAnimSetDataPinAltName(stream, node, "PartIndex_0", "BehaviourControlParameterInputID_8" )
    stream:writeString("ATTRIB_SEMANTIC_CP_INT", "BehaviourControlParameterInputType_8")
    writePerAnimSetDataPinAltName(stream, node, "PartIndex_1", "BehaviourControlParameterInputID_9" )
    stream:writeString("ATTRIB_SEMANTIC_CP_INT", "BehaviourControlParameterInputType_9")

    writeDataPinAltName(stream, node, "Position_0", "BehaviourControlParameterInputID_10" )
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_10")
    writeDataPinAltName(stream, node, "Position_1", "BehaviourControlParameterInputID_11" )
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_11")

    writeDataPinAltName(stream, node, "Normal_0", "BehaviourControlParameterInputID_12" )
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_12")
    writeDataPinAltName(stream, node, "Normal_1", "BehaviourControlParameterInputID_13" )
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_13")

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
    title = "Out of Reach",
    isExpanded = true,
    details = 
    {
      {title = "Timeout {Arm0}", perAnimSet = true, type = "float", attributes = "OutOfReachTimeout_0"},
      {title = "Timeout {Arm1}", perAnimSet = true, type = "float", attributes = "OutOfReachTimeout_1"},
      {title = "ExitDuration {Arm0}", perAnimSet = true, type = "float", attributes = "RampDownFailedDuration_0"},
      {title = "ExitDuration {Arm1}", perAnimSet = true, type = "float", attributes = "RampDownFailedDuration_1"},
    },
  },
  {
    title = "Within Reach",
    isExpanded = false,
    details = 
    {
      {title = "Timeout {Arm0}", perAnimSet = true, type = "float", attributes = "WithinReachTimeout_0"},
      {title = "Timeout {Arm1}", perAnimSet = true, type = "float", attributes = "WithinReachTimeout_1"},
      {title = "Exit duration {Arm0}", perAnimSet = true, type = "float", attributes = "RampDownCompletedDuration_0"},
      {title = "Exit duration {Arm1}", perAnimSet = true, type = "float", attributes = "RampDownCompletedDuration_1"},
    },
  },
  {
    title = "Target",
    helptext = "Set the target position an orientation for each arm.",
    isExpanded = false,
    details = 
    {
      {title = "Radius {Arm0}", perAnimSet = true, type = "float", attributes = "OutOfReachDistance_0"},
      {title = "Radius {Arm1}", perAnimSet = true, type = "float", attributes = "OutOfReachDistance_1"},
      {title = "Limb {Arm0}", perAnimSet = true, type = "integer", attributes = "LimbIndex_0"},
      {title = "Limb {Arm1}", perAnimSet = true, type = "integer", attributes = "LimbIndex_1"},
      {title = "Part {Arm0}", perAnimSet = true, type = "integer", attributes = "PartIndex_0"},
      {title = "Part {Arm1}", perAnimSet = true, type = "integer", attributes = "PartIndex_1"},
      {title = "Position {Arm0}", perAnimSet = false, type = "vector3", attributes = "Position_0"},
      {title = "Position {Arm1}", perAnimSet = false, type = "vector3", attributes = "Position_1"},
      {title = "Normal {Arm0}", perAnimSet = false, type = "vector3", attributes = "Normal_0"},
      {title = "Normal {Arm1}", perAnimSet = false, type = "vector3", attributes = "Normal_1"},
    },
  },
  {
    title = "Arm Movement",
    isExpanded = false,
    details = 
    {
      {title = "Swivel Mode {Arm0}", perAnimSet = true, type = "integer", attributes = "SwivelMode_0"},
      {title = "Swivel Mode {Arm1}", perAnimSet = true, type = "integer", attributes = "SwivelMode_1"},
      {title = "Swivel Amount {Arm0}", perAnimSet = true, type = "float", attributes = "SwivelAmount_0"},
      {title = "Swivel Amount {Arm1}", perAnimSet = true, type = "float", attributes = "SwivelAmount_1"},
      {title = "ArmExtension {Arm0}", perAnimSet = true, type = "float", attributes = "MaxArmExtensionScale_0"},
      {title = "ArmExtension {Arm1}", perAnimSet = true, type = "float", attributes = "MaxArmExtensionScale_1"},
      {title = "SurfacePenetration {Arm0}", perAnimSet = true, type = "float", attributes = "SurfacePenetration_0"},
      {title = "SurfacePenetration {Arm1}", perAnimSet = true, type = "float", attributes = "SurfacePenetration_1"},
    },
  },
  {
    title = "Self Avoidance",
    helptext = "Configure he behaviour to avoid positioning the hands within a specified distance from the spine.",
    isExpanded = false,
    details = 
    {
      {title = "Scale torso radius {Arm0}", perAnimSet = true, type = "float", attributes = "TorsoAvoidanceRadiusMultiplier_0"},
      {title = "Scale torso radius {Arm1}", perAnimSet = true, type = "float", attributes = "TorsoAvoidanceRadiusMultiplier_1"},
      {title = "CollisionGroup", perAnimSet = false, type = "integer", attributes = "CollisionGroupIndex"},
    },
  },
  {
    title = "Input Defaults",
    isExpanded = false,
    details = 
    {
      {title = "Use latest hit info {Arm0}", perAnimSet = false, type = "bool", attributes = "newHit_0"},
      {title = "Use latest hit info {Arm1}", perAnimSet = false, type = "bool", attributes = "newHit_1"},
      {title = "Amount {Arm0}", perAnimSet = false, type = "float", attributes = "Strength_0"},
      {title = "Amount {Arm1}", perAnimSet = false, type = "float", attributes = "Strength_1"},
      {title = "EffectorSpeedLimit {Arm0}", perAnimSet = false, type = "float", attributes = "EffectorSpeedLimit_0"},
      {title = "EffectorSpeedLimit {Arm1}", perAnimSet = false, type = "float", attributes = "EffectorSpeedLimit_1"},
    },
  },
  {
    title = "Target Arm Pose",
    helptext = "Sets a suitable pose for an arm that has become a reach target for another arm. Typically this pose will place the target arm in a position where it can be reached by another arm.",
    isExpanded = false,
    details = 
    {
      {title = "Arm {Arm0}", perAnimSet = false, type = "float", attributes = "ArmReachForBodyPose_ApplyToArm_1"},
      {title = "Arm {Arm1}", perAnimSet = false, type = "float", attributes = "ArmReachForBodyPose_ApplyToArm_2"},
      {title = "Head {Head0}", perAnimSet = false, type = "float", attributes = "ArmReachForBodyPose_ApplyToHead_1"},
      {title = "Leg {Leg0}", perAnimSet = false, type = "float", attributes = "ArmReachForBodyPose_ApplyToLeg_1"},
      {title = "Leg {Leg1}", perAnimSet = false, type = "float", attributes = "ArmReachForBodyPose_ApplyToLeg_2"},
      {title = "Spine {Spine0}", perAnimSet = false, type = "float", attributes = "ArmReachForBodyPose_ApplyToSpine_1"},
    },
  },
}

return {
  data = behaviourNode,
  attributeGroups = attributeGroups,
}
