--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "Properties",
  version = 1,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Defines the common properties of the character that are not limb dependent",
  group = "Behaviours",
  image = "PropertiesBehaviour.png",
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
    ["CollidingSupportTime"] = 
    {
      input = true,
      helptext = "When a supporting contact is lost, the character still considers itself partially supported during this time period. This is so that it does not suddenly \"panic\" if it hops off the ground. In seconds (standard character).",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "CollidingSupportTime",
      value = 0.500000,
    },

    ["MaxSlopeForGround"] = 
    {
      input = true,
      helptext = "If a surface slopes more than this angle, in degrees, then it will not be considered \"ground\" for the purposes of balance etc.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "MaxSlopeForGround",
      value = 60.000000,
    },

    ["AwarenessPredictionTime"] = 
    {
      input = true,
      helptext = "Only respond objects in the environment (hazards or edges to grab etc) when they are predicted to interact with the character within this time. In seconds (standard character).",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "AwarenessPredictionTime",
      value = 1.000000,
    },

    ["EnableJointProjection"] = 
    {
      input = true,
      helptext = "Use joint projection to reduce joint separation",
      type = "bool",
      mode = "HiddenInNetworkEditor",
      displayName = "Enable",
      value = false,
    },

    ["JointProjectionLinearTolerance"] = 
    {
      input = true,
      helptext = "Tolerance above which joint separation will be corrected.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "LinearTolerance",
      value = 0.100000,
      min = 0.000000,
    },

    ["JointProjectionAngularTolerance"] = 
    {
      input = true,
      helptext = "Angular tolerance in degrees above which joint limit errors will be corrected.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "AngularTolerance",
      value = 10.000000,
      min = 0.000000,
      max = 180.000000,
    },

    ["JointProjectionIterations"] = 
    {
      input = true,
      helptext = "Number of iterations used for resolving joint separation.",
      type = "int",
      mode = "HiddenInNetworkEditor",
      displayName = "Iterations",
      value = 1,
      min = 1,
    },

  },

  pinOrder = 
  {
    "CollidingSupportTime",
    "MaxSlopeForGround",
    "AwarenessPredictionTime",
    "EnableJointProjection",
    "JointProjectionLinearTolerance",
    "JointProjectionAngularTolerance",
    "JointProjectionIterations",
    "Result",
  },

  attributes = 
  {
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(21, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(0, "numBehaviourInts")

    stream:writeUInt(0, "numBehaviourFloats")

    stream:writeUInt(0, "numBehaviourVector3s")

    stream:writeUInt(0, "numBehaviourUInt64s")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numBehaviourNodeAnimationInputs")

    -------------------------------------------------------------------

    stream:writeUInt(2, "numInputCPInts")
    stream:writeUInt(5, "numInputCPFloats")
    stream:writeUInt(0, "numInputCPUInt64s")
    stream:writeUInt(0, "numInputCPVector3s")
    stream:writeUInt(7, "numBehaviourControlParameterInputs")

    writeDataPinAltName(stream, node, "CollidingSupportTime", "BehaviourControlParameterInputID_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_0")

    writeDataPinAltName(stream, node, "MaxSlopeForGround", "BehaviourControlParameterInputID_1")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_1")

    writeDataPinAltName(stream, node, "AwarenessPredictionTime", "BehaviourControlParameterInputID_2")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_2")

    writeDataPinAltName(stream, node, "EnableJointProjection", "BehaviourControlParameterInputID_3")
    stream:writeString("ATTRIB_SEMANTIC_CP_BOOL", "BehaviourControlParameterInputType_3")

    writeDataPinAltName(stream, node, "JointProjectionLinearTolerance", "BehaviourControlParameterInputID_4")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_4")

    writeDataPinAltName(stream, node, "JointProjectionAngularTolerance", "BehaviourControlParameterInputID_5")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_5")

    writeDataPinAltName(stream, node, "JointProjectionIterations", "BehaviourControlParameterInputID_6")
    stream:writeString("ATTRIB_SEMANTIC_CP_INT", "BehaviourControlParameterInputType_6")

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
    title = "Input Defaults",
    isExpanded = true,
    details = 
    {
      {title = "CollidingSupportTime", perAnimSet = false, type = "float", attributes = "CollidingSupportTime"},
      {title = "MaxSlopeForGround", perAnimSet = false, type = "float", attributes = "MaxSlopeForGround"},
      {title = "AwarenessPredictionTime", perAnimSet = false, type = "float", attributes = "AwarenessPredictionTime"},
    },
  },
  {
    title = "Joint Projection Inputs",
    helptext = "Enable and configure joint projection, an additional solver step for correcting joint separation in the physics rig.",
    isExpanded = false,
    details = 
    {
      {title = "Enable", perAnimSet = false, type = "bool", attributes = "EnableJointProjection"},
      {title = "LinearTolerance", perAnimSet = false, type = "float", attributes = "JointProjectionLinearTolerance"},
      {title = "AngularTolerance", perAnimSet = false, type = "float", attributes = "JointProjectionAngularTolerance"},
      {title = "Iterations", perAnimSet = false, type = "integer", attributes = "JointProjectionIterations"},
    },
  },
}

return {
  data = behaviourNode,
  attributeGroups = attributeGroups,
}
