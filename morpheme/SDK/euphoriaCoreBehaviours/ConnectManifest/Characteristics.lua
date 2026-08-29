--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "Characteristics",
  version = 3,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Defines the common characteristics of the character such as whole/partial body strength.",
  group = "Behaviours",
  image = "CharacteristicsBehaviour.png",
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
    ["WholeBodyStrengthScale"] = 
    {
      input = true,
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Whole body scale",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

    ["ArmStrengthScale_0"] = 
    {
      input = true,
      helptext = "Strength multiplier for this arm",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Arm scale {Arm0}",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

    ["ArmStrengthScale_1"] = 
    {
      input = true,
      helptext = "Strength multiplier for this arm",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Arm scale {Arm1}",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

    ["HeadStrengthScale_0"] = 
    {
      input = true,
      helptext = "Strength multiplier for this head",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Head scale {Head0}",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

    ["LegStrengthScale_0"] = 
    {
      input = true,
      helptext = "Strength multiplier for this leg",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Leg scale {Leg0}",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

    ["LegStrengthScale_1"] = 
    {
      input = true,
      helptext = "Strength multiplier for this leg",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Leg scale {Leg1}",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

    ["SpineStrengthScale_0"] = 
    {
      input = true,
      helptext = "Strength multiplier for the spine",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Spine scale",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

    ["WholeBodyControlCompensationScale"] = 
    {
      input = true,
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Whole body scale",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["ArmControlCompensationScale_0"] = 
    {
      input = true,
      helptext = "Control compensation multiplier for this arm",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Arm scale {Arm0}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["ArmControlCompensationScale_1"] = 
    {
      input = true,
      helptext = "Control compensation multiplier for this arm",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Arm scale {Arm1}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["HeadControlCompensationScale_0"] = 
    {
      input = true,
      helptext = "Control compensation multiplier for this head",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Head scale {Head0}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["LegControlCompensationScale_0"] = 
    {
      input = true,
      helptext = "Control compensation multiplier for this leg",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Leg scale {Leg0}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["LegControlCompensationScale_1"] = 
    {
      input = true,
      helptext = "Control compensation multiplier for this leg",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Leg scale {Leg1}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["SpineControlCompensationScale_0"] = 
    {
      input = true,
      helptext = "Control compensation multiplier for the spine",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Spine scale",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["WholeBodyExternalComplianceScale"] = 
    {
      input = true,
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Whole body scale",
      value = 1.000000,
      min = 0.000000,
      max = 10.000000,
    },

    ["ArmExternalComplianceScale_0"] = 
    {
      input = true,
      helptext = "External Compliance multiplier for this arm",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Arm scale {Arm0}",
      value = 1.000000,
      min = 0.000000,
      max = 10.000000,
    },

    ["ArmExternalComplianceScale_1"] = 
    {
      input = true,
      helptext = "External Compliance multiplier for this arm",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Arm scale {Arm1}",
      value = 1.000000,
      min = 0.000000,
      max = 10.000000,
    },

    ["HeadExternalComplianceScale_0"] = 
    {
      input = true,
      helptext = "External Compliance multiplier for this head",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Head scale {Head0}",
      value = 1.000000,
      min = 0.000000,
      max = 10.000000,
    },

    ["LegExternalComplianceScale_0"] = 
    {
      input = true,
      helptext = "External Compliance multiplier for this leg",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Leg scale {Leg0}",
      value = 1.000000,
      min = 0.000000,
      max = 10.000000,
    },

    ["LegExternalComplianceScale_1"] = 
    {
      input = true,
      helptext = "External Compliance multiplier for this leg",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Leg scale {Leg1}",
      value = 1.000000,
      min = 0.000000,
      max = 10.000000,
    },

    ["SpineExternalComplianceScale_0"] = 
    {
      input = true,
      helptext = "External Compliance multiplier for the spine",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Spine scale",
      value = 1.000000,
      min = 0.000000,
      max = 10.000000,
    },

    ["WholeBodyDampingRatioScale"] = 
    {
      input = true,
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Whole body scale",
      value = 1.000000,
      min = 0.000000,
      max = 5.000000,
    },

    ["ArmDampingRatioScale_0"] = 
    {
      input = true,
      helptext = "Damping ratio multiplier for this arm",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Arm scale {Arm0}",
      value = 1.000000,
      min = 0.000000,
      max = 5.000000,
    },

    ["ArmDampingRatioScale_1"] = 
    {
      input = true,
      helptext = "Damping ratio multiplier for this arm",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Arm scale {Arm1}",
      value = 1.000000,
      min = 0.000000,
      max = 5.000000,
    },

    ["HeadDampingRatioScale_0"] = 
    {
      input = true,
      helptext = "Damping ratio multiplier for this head",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Head scale {Head0}",
      value = 1.000000,
      min = 0.000000,
      max = 5.000000,
    },

    ["LegDampingRatioScale_0"] = 
    {
      input = true,
      helptext = "Damping ratio multiplier for this leg",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Leg scale {Leg0}",
      value = 1.000000,
      min = 0.000000,
      max = 5.000000,
    },

    ["LegDampingRatioScale_1"] = 
    {
      input = true,
      helptext = "Damping ratio multiplier for this leg",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Leg scale {Leg1}",
      value = 1.000000,
      min = 0.000000,
      max = 5.000000,
    },

    ["SpineDampingRatioScale_0"] = 
    {
      input = true,
      helptext = "Damping ratio multiplier for the spine",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Spine scale",
      value = 1.000000,
      min = 0.000000,
      max = 5.000000,
    },

    ["WholeBodySoftLimitStiffnessScale"] = 
    {
      input = true,
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Whole body soft limit stiffness scale",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["ArmSoftLimitStiffnessScale_0"] = 
    {
      input = true,
      helptext = "Control soft limit stiffness multiplier for this arm",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Arm scale {Arm0}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["ArmSoftLimitStiffnessScale_1"] = 
    {
      input = true,
      helptext = "Control soft limit stiffness multiplier for this arm",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Arm scale {Arm1}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["HeadSoftLimitStiffnessScale_0"] = 
    {
      input = true,
      helptext = "Control soft limit stiffness multiplier for this head",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Head scale {Head0}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["LegSoftLimitStiffnessScale_0"] = 
    {
      input = true,
      helptext = "Control soft limit stiffness multiplier for this leg",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Leg scale {Leg0}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["LegSoftLimitStiffnessScale_1"] = 
    {
      input = true,
      helptext = "Control soft limit stiffness multiplier for this leg",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Leg scale {Leg1}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["SpineSoftLimitStiffnessScale_0"] = 
    {
      input = true,
      helptext = "Control soft limit stiffness multiplier for the spine",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "Spine scale",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

  },

  pinOrder = 
  {
    "WholeBodyStrengthScale",
    "ArmStrengthScale_0",
    "ArmStrengthScale_1",
    "HeadStrengthScale_0",
    "LegStrengthScale_0",
    "LegStrengthScale_1",
    "SpineStrengthScale_0",
    "WholeBodyControlCompensationScale",
    "ArmControlCompensationScale_0",
    "ArmControlCompensationScale_1",
    "HeadControlCompensationScale_0",
    "LegControlCompensationScale_0",
    "LegControlCompensationScale_1",
    "SpineControlCompensationScale_0",
    "WholeBodyExternalComplianceScale",
    "ArmExternalComplianceScale_0",
    "ArmExternalComplianceScale_1",
    "HeadExternalComplianceScale_0",
    "LegExternalComplianceScale_0",
    "LegExternalComplianceScale_1",
    "SpineExternalComplianceScale_0",
    "WholeBodyDampingRatioScale",
    "ArmDampingRatioScale_0",
    "ArmDampingRatioScale_1",
    "HeadDampingRatioScale_0",
    "LegDampingRatioScale_0",
    "LegDampingRatioScale_1",
    "SpineDampingRatioScale_0",
    "WholeBodySoftLimitStiffnessScale",
    "ArmSoftLimitStiffnessScale_0",
    "ArmSoftLimitStiffnessScale_1",
    "HeadSoftLimitStiffnessScale_0",
    "LegSoftLimitStiffnessScale_0",
    "LegSoftLimitStiffnessScale_1",
    "SpineSoftLimitStiffnessScale_0",
    "Result",
  },

  attributes = 
  {
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(8, "BehaviourID")


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
    stream:writeUInt(35, "numInputCPFloats")
    stream:writeUInt(0, "numInputCPUInt64s")
    stream:writeUInt(0, "numInputCPVector3s")
    stream:writeUInt(35, "numBehaviourControlParameterInputs")

    writeDataPinAltName(stream, node, "WholeBodyStrengthScale", "BehaviourControlParameterInputID_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_0")

    writeDataPinAltName(stream, node, "ArmStrengthScale_0", "BehaviourControlParameterInputID_1" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_1")
    writeDataPinAltName(stream, node, "ArmStrengthScale_1", "BehaviourControlParameterInputID_2" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_2")

    writeDataPinAltName(stream, node, "HeadStrengthScale_0", "BehaviourControlParameterInputID_3" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_3")

    writeDataPinAltName(stream, node, "LegStrengthScale_0", "BehaviourControlParameterInputID_4" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_4")
    writeDataPinAltName(stream, node, "LegStrengthScale_1", "BehaviourControlParameterInputID_5" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_5")

    writeDataPinAltName(stream, node, "SpineStrengthScale_0", "BehaviourControlParameterInputID_6" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_6")

    writeDataPinAltName(stream, node, "WholeBodyControlCompensationScale", "BehaviourControlParameterInputID_7")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_7")

    writeDataPinAltName(stream, node, "ArmControlCompensationScale_0", "BehaviourControlParameterInputID_8" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_8")
    writeDataPinAltName(stream, node, "ArmControlCompensationScale_1", "BehaviourControlParameterInputID_9" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_9")

    writeDataPinAltName(stream, node, "HeadControlCompensationScale_0", "BehaviourControlParameterInputID_10" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_10")

    writeDataPinAltName(stream, node, "LegControlCompensationScale_0", "BehaviourControlParameterInputID_11" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_11")
    writeDataPinAltName(stream, node, "LegControlCompensationScale_1", "BehaviourControlParameterInputID_12" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_12")

    writeDataPinAltName(stream, node, "SpineControlCompensationScale_0", "BehaviourControlParameterInputID_13" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_13")

    writeDataPinAltName(stream, node, "WholeBodyExternalComplianceScale", "BehaviourControlParameterInputID_14")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_14")

    writeDataPinAltName(stream, node, "ArmExternalComplianceScale_0", "BehaviourControlParameterInputID_15" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_15")
    writeDataPinAltName(stream, node, "ArmExternalComplianceScale_1", "BehaviourControlParameterInputID_16" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_16")

    writeDataPinAltName(stream, node, "HeadExternalComplianceScale_0", "BehaviourControlParameterInputID_17" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_17")

    writeDataPinAltName(stream, node, "LegExternalComplianceScale_0", "BehaviourControlParameterInputID_18" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_18")
    writeDataPinAltName(stream, node, "LegExternalComplianceScale_1", "BehaviourControlParameterInputID_19" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_19")

    writeDataPinAltName(stream, node, "SpineExternalComplianceScale_0", "BehaviourControlParameterInputID_20" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_20")

    writeDataPinAltName(stream, node, "WholeBodyDampingRatioScale", "BehaviourControlParameterInputID_21")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_21")

    writeDataPinAltName(stream, node, "ArmDampingRatioScale_0", "BehaviourControlParameterInputID_22" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_22")
    writeDataPinAltName(stream, node, "ArmDampingRatioScale_1", "BehaviourControlParameterInputID_23" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_23")

    writeDataPinAltName(stream, node, "HeadDampingRatioScale_0", "BehaviourControlParameterInputID_24" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_24")

    writeDataPinAltName(stream, node, "LegDampingRatioScale_0", "BehaviourControlParameterInputID_25" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_25")
    writeDataPinAltName(stream, node, "LegDampingRatioScale_1", "BehaviourControlParameterInputID_26" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_26")

    writeDataPinAltName(stream, node, "SpineDampingRatioScale_0", "BehaviourControlParameterInputID_27" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_27")

    writeDataPinAltName(stream, node, "WholeBodySoftLimitStiffnessScale", "BehaviourControlParameterInputID_28")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_28")

    writeDataPinAltName(stream, node, "ArmSoftLimitStiffnessScale_0", "BehaviourControlParameterInputID_29" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_29")
    writeDataPinAltName(stream, node, "ArmSoftLimitStiffnessScale_1", "BehaviourControlParameterInputID_30" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_30")

    writeDataPinAltName(stream, node, "HeadSoftLimitStiffnessScale_0", "BehaviourControlParameterInputID_31" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_31")

    writeDataPinAltName(stream, node, "LegSoftLimitStiffnessScale_0", "BehaviourControlParameterInputID_32" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_32")
    writeDataPinAltName(stream, node, "LegSoftLimitStiffnessScale_1", "BehaviourControlParameterInputID_33" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_33")

    writeDataPinAltName(stream, node, "SpineSoftLimitStiffnessScale_0", "BehaviourControlParameterInputID_34" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_34")

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
    title = "Input strength defaults",
    helptext = "Sets the strength scale for a character's limbs. Stronger limbs will be move faster and be better able to reach and maintain target poses.",
    isExpanded = true,
    details = 
    {
      {title = "Whole body scale", perAnimSet = false, type = "float", attributes = "WholeBodyStrengthScale"},
      {title = "Arm scale {Arm0}", perAnimSet = false, type = "float", attributes = "ArmStrengthScale_0"},
      {title = "Arm scale {Arm1}", perAnimSet = false, type = "float", attributes = "ArmStrengthScale_1"},
      {title = "Head scale {Head0}", perAnimSet = false, type = "float", attributes = "HeadStrengthScale_0"},
      {title = "Leg scale {Leg0}", perAnimSet = false, type = "float", attributes = "LegStrengthScale_0"},
      {title = "Leg scale {Leg1}", perAnimSet = false, type = "float", attributes = "LegStrengthScale_1"},
      {title = "Spine scale ", perAnimSet = false, type = "float", attributes = "SpineStrengthScale_0"},
    },
  },
  {
    title = "Input compensation defaults",
    helptext = "The control compensation is used to make joints compensate for controlled motion in other parts of the rig. A high value will cause a character to appear coordinated whilst a low value will lead to loose, uncontrolled motion.",
    isExpanded = false,
    details = 
    {
      {title = "Whole body scale", perAnimSet = false, type = "float", attributes = "WholeBodyControlCompensationScale"},
      {title = "Arm scale {Arm0}", perAnimSet = false, type = "float", attributes = "ArmControlCompensationScale_0"},
      {title = "Arm scale {Arm1}", perAnimSet = false, type = "float", attributes = "ArmControlCompensationScale_1"},
      {title = "Head scale {Head0}", perAnimSet = false, type = "float", attributes = "HeadControlCompensationScale_0"},
      {title = "Leg scale {Leg0}", perAnimSet = false, type = "float", attributes = "LegControlCompensationScale_0"},
      {title = "Leg scale {Leg1}", perAnimSet = false, type = "float", attributes = "LegControlCompensationScale_1"},
      {title = "Spine scale ", perAnimSet = false, type = "float", attributes = "SpineControlCompensationScale_0"},
    },
  },
  {
    title = "External compliance defaults",
    helptext = "The external compliance is used to make joints compensate for external forces on other parts of the rig (e.g. contact forces). A high value will allow motion caused by external forces to propagate through the rig (making it seem quite loose) whilst a low value will limit movement to the affected part, making the rest of the rig appear stiff.",
    isExpanded = false,
    details = 
    {
      {title = "Whole body scale", perAnimSet = false, type = "float", attributes = "WholeBodyExternalComplianceScale"},
      {title = "Arm scale {Arm0}", perAnimSet = false, type = "float", attributes = "ArmExternalComplianceScale_0"},
      {title = "Arm scale {Arm1}", perAnimSet = false, type = "float", attributes = "ArmExternalComplianceScale_1"},
      {title = "Head scale {Head0}", perAnimSet = false, type = "float", attributes = "HeadExternalComplianceScale_0"},
      {title = "Leg scale {Leg0}", perAnimSet = false, type = "float", attributes = "LegExternalComplianceScale_0"},
      {title = "Leg scale {Leg1}", perAnimSet = false, type = "float", attributes = "LegExternalComplianceScale_1"},
      {title = "Spine scale ", perAnimSet = false, type = "float", attributes = "SpineExternalComplianceScale_0"},
    },
  },
  {
    title = "Input damping defaults",
    helptext = "Sets the damping scale for a character's limbs. High values (above 1) will result in the charater moving his joints more slowly in a controlled way, and they will deform less as a result of external influences. Small values (below 1) will result in the joints being looser andless well controlled. ",
    isExpanded = false,
    details = 
    {
      {title = "Whole body scale", perAnimSet = false, type = "float", attributes = "WholeBodyDampingRatioScale"},
      {title = "Arm scale {Arm0}", perAnimSet = false, type = "float", attributes = "ArmDampingRatioScale_0"},
      {title = "Arm scale {Arm1}", perAnimSet = false, type = "float", attributes = "ArmDampingRatioScale_1"},
      {title = "Head scale {Head0}", perAnimSet = false, type = "float", attributes = "HeadDampingRatioScale_0"},
      {title = "Leg scale {Leg0}", perAnimSet = false, type = "float", attributes = "LegDampingRatioScale_0"},
      {title = "Leg scale {Leg1}", perAnimSet = false, type = "float", attributes = "LegDampingRatioScale_1"},
      {title = "Spine scale ", perAnimSet = false, type = "float", attributes = "SpineDampingRatioScale_0"},
    },
  },
  {
    title = "Soft limits stiffness scale defaults",
    helptext = "Sets the stiffness scale for the soft limits. Lowering the value to less than 1 will reduce the effect of the soft limits when the limb is being weakly controlled.",
    isExpanded = false,
    details = 
    {
      {title = "Whole body soft limit stiffness scale", perAnimSet = false, type = "float", attributes = "WholeBodySoftLimitStiffnessScale"},
      {title = "Arm scale {Arm0}", perAnimSet = false, type = "float", attributes = "ArmSoftLimitStiffnessScale_0"},
      {title = "Arm scale {Arm1}", perAnimSet = false, type = "float", attributes = "ArmSoftLimitStiffnessScale_1"},
      {title = "Head scale {Head0}", perAnimSet = false, type = "float", attributes = "HeadSoftLimitStiffnessScale_0"},
      {title = "Leg scale {Leg0}", perAnimSet = false, type = "float", attributes = "LegSoftLimitStiffnessScale_0"},
      {title = "Leg scale {Leg1}", perAnimSet = false, type = "float", attributes = "LegSoftLimitStiffnessScale_1"},
      {title = "Spine scale ", perAnimSet = false, type = "float", attributes = "SpineSoftLimitStiffnessScale_0"},
    },
  },
}

return {
  data = behaviourNode,
  attributeGroups = attributeGroups,
}
