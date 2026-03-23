------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


registerBehaviourNode("Arms Windmill",
{
  version = 3,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Spins a character's arms in circles in order to achieve a rotation of the chest within a specified time period. ",
  group = "Behaviours",
  image = "ArmsWindmillBehaviour.png",
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
    ["TargetRotationDelta"] = 
    {
      input = true,
      helptext = "Sets a world space rotation vector whose direction determines the axis and magnitude determines the angle of rotation of the chest, in degrees. When operating in local space, the x axis points forwards, z points right and y points up (along the spine).",
      type = "vector3",
      mode = "HiddenInNetworkEditor",
      displayName = "Rotation",
      value = {x=0.000000, y=0.000000,z=90.000000},
    },

    ["RotationTime"] = 
    {
      input = true,
      helptext = "Sets the amount of time in which to achieve the rotation, in seconds (standard character). Smaller values lead to faster movements. For times under about 0.5s the arms will swing rather than spin.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "RotationTime",
      value = 0.500000,
      min = 0.000000,
      max = 5.000000,
    },

    ["ImportanceForArm_0"] = 
    {
      input = true,
      helptext = "Sets the strength of control over each arm. Accepts values in the range 0 to 1.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Weight {Arm0}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["ImportanceForArm_1"] = 
    {
      input = true,
      helptext = "Sets the strength of control over each arm. Accepts values in the range 0 to 1.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Weight {Arm1}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

  },

  pinOrder = 
  {
    "TargetRotationDelta",
    "RotationTime",
    "ImportanceForArm_0",
    "ImportanceForArm_1",
    "Result",
  },

  attributes = 
  {
    {
      name = "MaxAngSpeed",
      type = "float",
      value = 3.000000,
      min = 0.000000,
      max = 5.000000,
      perAnimSet = true,
      helptext = "Maximum angular speed of the arm, in revolutions/second (standard character).",
    },
    {
      name = "MaxRadius",
      type = "float",
      value = 0.600000,
      min = 0.000000,
      max = 2.000000,
      perAnimSet = true,
      helptext = "Maximum radius of spin around the spin centre, in metres (standard character).. ",
    },
    {
      name = "Synchronised",
      type = "bool",
      value = true,
      perAnimSet = true,
      helptext = "If true then both hands will have the same phase in their respective circles, if false then the hands will circle independently from their initial positions",
    },
    {
      name = "SpinAmounts_0",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
      helptext = "Affects how strongly a request to spin the arms is applied. 1 is normal or average stiffness, 0 has no effect, the spin is turned off.",
    },
    {
      name = "SpinAmounts_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
      helptext = "Affects how strongly a request to spin the arms is applied. 1 is normal or average stiffness, 0 has no effect, the spin is turned off.",
    },
    {
      name = "ArmsInPhase",
      type = "bool",
      value = true,
      helptext = "If true the the arms will move in phase.",
    },
    {
      name = "SpinCentreLateral",
      type = "float",
      value = 0.250000,
      min = -0.500000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Centre of spin circle sideways away from the shoulder. In metres (standard character).",
    },
    {
      name = "SpinCentreUp",
      type = "float",
      value = 0.000000,
      min = -1.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Centre of spin circle locally upwards from the shoulder. In metres (standard character).",
    },
    {
      name = "SpinCentreForward",
      type = "float",
      value = 0.100000,
      min = -1.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Centre of spin circle forwards from the shoulder. In metres (standard character).",
    },
    {
      name = "SpinWeightLateral",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Weight of control in the local sideways axis from the chest",
    },
    {
      name = "SpinWeightUp",
      type = "float",
      value = 0.500000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Weight of control in the local upwards  axis from the chest",
    },
    {
      name = "SpinWeightForward",
      type = "float",
      value = 0.500000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Weight of control in the local forwards axis from the chest",
    },
    {
      name = "SpinOutwardsDistanceWhenBehind",
      type = "float",
      value = 0.200000,
      min = 0.000000,
      max = 2.000000,
      perAnimSet = true,
      helptext = "Distance to push the hands out when they are at the back of the spin cycle. Increase this if the character tends to get his arms caught on the shoulder joint limits. In metres (standard character).",
    },
    {
      name = "SpinArmControlCompensationScale",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
      perAnimSet = true,
      helptext = "Small values make the arm appear more loose, larger values make the arm appear more controlled",
    },
    {
      name = "StrengthReductionTowardsHand",
      type = "float",
      value = 0.800000,
      min = 0.000000,
      max = 10.000000,
      perAnimSet = true,
      helptext = "Small values result in the wrist being as strong as the shoulder. 1.0 will result in the wrist being completely loose. Larger values confine the strength to the joints in the upper arm.",
    },
    {
      name = "SwingAmounts_0",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 3.000000,
      helptext = "Affects how strongly a request to swing the arms is applied. 1 is normal or average stiffness, 0 has no effect, the swing is turned off.",
    },
    {
      name = "SwingAmounts_1",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 3.000000,
      helptext = "Affects how strongly a request to swing the arms is applied. 1 is normal or average stiffness, 0 has no effect, the swing is turned off.",
    },
    {
      name = "SwingOutwardsOnly",
      type = "bool",
      value = false,
      helptext = "If true, this prevent arms from swinging across the body laterally, they will only swing away from the body.",
    },
    {
      name = "SpinInLocalSpace",
      type = "bool",
      value = true,
      perAnimSet = true,
      helptext = "The target rotation delta will be interpretted in local space of the chest, .z value is the rightwards axis used for spinning forwards or backwards, .x value is for spinning around the forwards axis, and .y for spinning around the local vertical or spine axis ",
    },
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(5, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(4, "numBehaviourInts")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getAttribute(node, "Synchronised", asVal) and 1 ) or 0 ) , string.format("Int_0_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getAttribute(node, "ArmsInPhase") and 1 ) or 0 ) , string.format("Int_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getAttribute(node, "SwingOutwardsOnly") and 1 ) or 0 ) , string.format("Int_2_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getAttribute(node, "SpinInLocalSpace", asVal) and 1 ) or 0 ) , string.format("Int_3_%d", asIdx-1))
    end

    stream:writeUInt(15, "numBehaviourFloats")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "MaxAngSpeed", asVal), string.format("Float_0_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "MaxRadius", asVal), string.format("Float_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "SpinAmounts_0"), string.format("Float_2_%d", asIdx-1))
      stream:writeFloat(getAttribute(node, "SpinAmounts_1"), string.format("Float_3_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "SpinCentreLateral", asVal), string.format("Float_4_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "SpinCentreUp", asVal), string.format("Float_5_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "SpinCentreForward", asVal), string.format("Float_6_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "SpinWeightLateral", asVal), string.format("Float_7_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "SpinWeightUp", asVal), string.format("Float_8_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "SpinWeightForward", asVal), string.format("Float_9_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "SpinOutwardsDistanceWhenBehind", asVal), string.format("Float_10_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "SpinArmControlCompensationScale", asVal), string.format("Float_11_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "StrengthReductionTowardsHand", asVal), string.format("Float_12_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "SwingAmounts_0"), string.format("Float_13_%d", asIdx-1))
      stream:writeFloat(getAttribute(node, "SwingAmounts_1"), string.format("Float_14_%d", asIdx-1))
    end

    stream:writeUInt(0, "numBehaviourVector3s")

    stream:writeUInt(0, "numBehaviourUInt64s")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numBehaviourNodeAnimationInputs")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numInputCPInts")
    stream:writeUInt(3, "numInputCPFloats")
    stream:writeUInt(0, "numInputCPUInt64s")
    stream:writeUInt(1, "numInputCPVector3s")
    stream:writeUInt(4, "numBehaviourControlParameterInputs")
	
	local writeDataPinAltName = function(stream, node, pin, field)
		local inNodeInfo = getConnectedNodeInfo(node, pin)
		if inNodeInfo ~= nil then
			stream:writeNetworkNodeId(inNodeInfo.id, field, inNodeInfo.pinIndex)
		else
			stream:writeNetworkNodeId(-1, field)
		end
	end

    writeDataPinAltName(stream, node, "TargetRotationDelta", "BehaviourControlParameterInputID_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_0")

    writeDataPinAltName(stream, node, "RotationTime", "BehaviourControlParameterInputID_1")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_1")

    writeDataPinAltName(stream, node, "ImportanceForArm_0", "BehaviourControlParameterInputID_2" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_2")
    writeDataPinAltName(stream, node, "ImportanceForArm_1", "BehaviourControlParameterInputID_3" )
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
    --return defaultPinValidation(node)

      return true
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
)

-------------------------------------------------------------------

local attributeGroups =
{
  {
    title = "Spin",
    helptext = "Settings that control how the arms spin. This the motion to change the torsos orientation and will last for a couple of seconds.",
    isExpanded = true,
    details = 
    {
      {title = "MaximumAngularSpeed", perAnimSet = true, type = "float", attributes = "MaxAngSpeed"},
      {title = "MaximumRadius", perAnimSet = true, type = "float", attributes = "MaxRadius"},
      {title = "SynchroniseArms", perAnimSet = true, type = "bool", attributes = "Synchronised"},
      {title = "SpinAmount {Arm0}", perAnimSet = false, type = "float", attributes = "SpinAmounts_0"},
      {title = "SpinAmount {Arm1}", perAnimSet = false, type = "float", attributes = "SpinAmounts_1"},
      {title = "ArmsInPhase", perAnimSet = false, type = "bool", attributes = "ArmsInPhase"},
      {title = "SpinCentreLateral", perAnimSet = true, type = "float", attributes = "SpinCentreLateral"},
      {title = "SpinCentreUp", perAnimSet = true, type = "float", attributes = "SpinCentreUp"},
      {title = "SpinCentreForward", perAnimSet = true, type = "float", attributes = "SpinCentreForward"},
      {title = "SpinWeightLateral", perAnimSet = true, type = "float", attributes = "SpinWeightLateral"},
      {title = "SpinWeightUp", perAnimSet = true, type = "float", attributes = "SpinWeightUp"},
      {title = "SpinWeightForward", perAnimSet = true, type = "float", attributes = "SpinWeightForward"},
      {title = "Hand offset when behind", perAnimSet = true, type = "float", attributes = "SpinOutwardsDistanceWhenBehind"},
      {title = "Arm control scale", perAnimSet = true, type = "float", attributes = "SpinArmControlCompensationScale"},
      {title = "Wrist looseness", perAnimSet = true, type = "float", attributes = "StrengthReductionTowardsHand"},
    },
  },
  {
    title = "Swing",
    helptext = "Settings that control how the arms swing. This is a one off motion to apply an impulse to the torso to change it's velocity. ",
    isExpanded = false,
    details = 
    {
      {title = "SwingAmount {Arm0}", perAnimSet = false, type = "float", attributes = "SwingAmounts_0"},
      {title = "SwingAmount {Arm1}", perAnimSet = false, type = "float", attributes = "SwingAmounts_1"},
      {title = "SwingOutwardsOnly", perAnimSet = false, type = "bool", attributes = "SwingOutwardsOnly"},
    },
  },
  {
    title = "Input Defaults",
    isExpanded = false,
    details = 
    {
      {title = "OperateInLocalSpace", perAnimSet = true, type = "bool", attributes = "SpinInLocalSpace"},
      {title = "Rotation", perAnimSet = false, type = "vector3", attributes = "TargetRotationDelta"},
      {title = "RotationTime", perAnimSet = false, type = "float", attributes = "RotationTime"},
      {title = "Weight {Arm0}", perAnimSet = false, type = "float", attributes = "ImportanceForArm_0"},
      {title = "Weight {Arm1}", perAnimSet = false, type = "float", attributes = "ImportanceForArm_1"},
    },
  },
}

return {
  data = behaviourNode,
  attributeGroups = attributeGroups,
}
