--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "Hold",
  version = 6,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Makes the character grab and hold on to any edge that is within their reach. ",
  group = "Behaviours",
  image = "HoldBehaviour.png",
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
    ["DoHold"] = 
    {
      input = true,
      helptext = "Dynamically toggle Hold. Set it to false to make character release grabbed edge or to prevent character from grabbing any edges.",
      type = "bool",
      mode = "HiddenInNetworkEditor",
      displayName = "DoHold",
      value = true,
    },

    ["IgnoreOvershotEdges"] = 
    {
      input = true,
      helptext = "Prevents the character from grabbing an edge when its chest is likly to land on the horizontal surface above that edge.",
      type = "bool",
      mode = "HiddenInNetworkEditor",
      displayName = "IgnoreOvershotEdges",
      value = true,
    },

    ["HoldImportance_0"] = 
    {
      input = true,
      helptext = "Sets the importance of Hold relative to other behaviours for each arm.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "HoldWeight {Arm0}",
      value = 1.000000,
    },

    ["HoldImportance_1"] = 
    {
      input = true,
      helptext = "Sets the importance of Hold relative to other behaviours for each arm.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "HoldWeight {Arm1}",
      value = 1.000000,
    },

    ["PullUpAmount"] = 
    {
      input = true,
      helptext = "How high the character tries to pull himself up. 1: to chest level, 0: arms fully extended (but note that the arms may still have strength).",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "PullUpAmount",
      value = 0.250000,
      min = 0.000000,
      max = 1.000000,
    },

    ["PullUpStrengthScale"] = 
    {
      input = true,
      helptext = "How strong the arms will try to pull up. This scales the normal strength is scaled by this factor. 0: no strength, 1: normal strength, 2: twice normal strength (super strong)",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "PullUpStrengthScale",
      value = 0.500000,
      min = 0.000000,
      max = 2.000000,
    },

    ["NumConstrainedHands"] = 
    {
      input = false,
      helptext = "Number of hands which have successfully grabbed and held onto an edge.",
      type = "float",
    },

    ["HandHolding_0"] = 
    {
      input = false,
      helptext = "Is the nth hand constrained.",
      type = "bool",
    },

    ["HandHolding_1"] = 
    {
      input = false,
      helptext = "Is the nth hand constrained.",
      type = "bool",
    },

    ["HoldAttemptImportance"] = 
    {
      input = false,
      helptext = "Indicates how much the behaviour is attempting to grab the edge (between 0 and 1).",
      type = "float",
    },

    ["HoldDuration"] = 
    {
      input = false,
      helptext = "Indicates how long the character has been constrained to an edge.",
      type = "float",
    },

    ["EdgeForwardNormal"] = 
    {
      input = false,
      helptext = "The normal to the front surface of the edge (as opposed to the upper surface).",
      type = "vector3",
    },

  },

  pinOrder = 
  {
    "DoHold",
    "IgnoreOvershotEdges",
    "HoldImportance_0",
    "HoldImportance_1",
    "PullUpAmount",
    "PullUpStrengthScale",
    "Result",
    "NumConstrainedHands",
    "HandHolding_0",
    "HandHolding_1",
    "HoldAttemptImportance",
    "HoldDuration",
    "EdgeForwardNormal",
  },

  attributes = 
  {
    {
      name = "MinSupportSlope",
      type = "float",
      value = 25.000000,
      min = 0.000000,
      max = 90.000000,
      perAnimSet = true,
      helptext = "If ground is detected below the character or below the edge (if edge raycast probing is enabled) it is not considered supportive (i.e. grabbing is allowed) if it is steeper than this (angle in degrees between \"up\" and ground normal). This ensures the grab is only disabled if the surface below the character or below the edge can support the character.",
    },
    {
      name = "VerticalSpeedToStart",
      type = "float",
      value = 0.800000,
      perAnimSet = true,
      helptext = "Grabbing is allowed to start when the character is moving downwards faster than this, in m/s (standard character).",
    },
    {
      name = "UnbalancedAmount",
      type = "float",
      value = 0.650000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Balance amounts greater than this for a period of time are considered supporting (no grab)",
    },
    {
      name = "MinUnbalancedPeriod",
      type = "float",
      value = 0.500000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "When balanced more than this time, the character is considered supported so grab is disabled. In seconds (standard character).",
    },
    {
      name = "VerticalSpeedToStop",
      type = "float",
      value = 0.060000,
      perAnimSet = true,
      helptext = "The reach/grab is stopped when the character is moving downwards slower than this, in m/s (standard character). The default value disables grabbing when he is moving upwards. Set to a large -ve number to disable this check.",
    },
    {
      name = "enableRaycast",
      type = "bool",
      value = true,
      perAnimSet = true,
      helptext = "Enables raycast probing to see if there is enough space to hang without unwanted obstructions.",
    },
    {
      name = "raycastEdgeOffset",
      type = "float",
      value = 0.300000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "Horizontal offset from the edge for the raycast origin. In metres (standard character).",
    },
    {
      name = "raycastLength",
      type = "float",
      value = 2.300000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "The length of the raycast. In metres (standard character).",
    },
    {
      name = "MinHoldPeriod",
      type = "float",
      value = 0.100000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "If a hold is successful it will always be active for at least this long. In seconds (standard character).",
    },
    {
      name = "MaxHoldPeriod",
      type = "float",
      value = 10.000000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "A hold will never last longer than this before giving up. In seconds (standard character). A value less than or equal to zero means to never let go due to the hold duration.",
    },
    {
      name = "NoHoldPeriod",
      type = "float",
      value = 0.500000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "At least this much time is enforced between successive holds. In seconds (standard character).",
    },
    {
      name = "MaxHandsBehindBackPeriod",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "If the hands are constrained behind the back for longer than this, the hold will be aborted. In seconds (standard character).",
    },
    {
      name = "MaxReachAttemptPeriod",
      type = "float",
      value = 5.000000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "If a character has tried but failed to reach an edge for this long, the reach attempt is aborted. In seconds (standard character).",
    },
    {
      name = "MinReachRecoveryPeriod",
      type = "float",
      value = 3.000000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "After a failed reach attempt, a new attempt isn't started until this period has elapsed. In seconds (standard character).",
    },
    {
      name = "CreateAtDistance",
      type = "float",
      value = 0.100000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "Hands needs to be at least this close to their target for constraint creation. In metres (standard character).",
    },
    {
      name = "EnableOrientationAtAngle",
      type = "float",
      value = 120.000000,
      min = 0.000000,
      max = 180.000000,
      perAnimSet = true,
      helptext = "Hand orientation is enforced when the angle between it and the target is less than this angle, in degrees",
    },
    {
      name = "DestroyAtDistance",
      type = "float",
      value = 0.250000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "When constrained, hands need to be this far from their target for the constraint to be destroyed (the hold to be broken). In metres (standard character).",
    },
    {
      name = "DisableOrientationAtAngle",
      type = "float",
      value = 160.000000,
      min = 0.000000,
      max = 180.000000,
      perAnimSet = true,
      helptext = "Hand orientation will stop being enforced when the angle between it and the target is greater than this angle, in degrees",
    },
    {
      name = "LockedLinearDofs",
      type = "int",
      value = 7,
      min = 0,
      max = 7,
      perAnimSet = true,
      helptext = "Which linear constraint axes are locked.",
    },
    {
      name = "LockedAngularDofs",
      type = "int",
      value = 7,
      min = 0,
      max = 7,
      perAnimSet = true,
      helptext = "Which angular constraint axes are locked.",
    },
    {
      name = "HoldOnContact",
      type = "bool",
      value = false,
      perAnimSet = true,
      helptext = "Hold on to whatever the hand touches. This makes the hands to be magically sticky.",
    },
    {
      name = "DisableCollisions",
      type = "bool",
      value = true,
      perAnimSet = true,
      helptext = "Disable collisions between hands and object",
    },
    {
      name = "DisableAngularDofsUntilHanging",
      type = "bool",
      value = true,
      perAnimSet = true,
      helptext = "If true then the orientations will only get locked (if requested) after the character has come to hang vertically.",
    },
    {
      name = "timeBeforeLookingDown",
      type = "float",
      value = 0.800000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "The proportion of maximum hold time at which the character will switch from looking at the edge to looking down. This gives an impression of fatigue.",
    },
    {
      name = "ChestControlImminence",
      type = "float",
      value = 2.000000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "Used to control the chest (make it upright) by swinging/spinning the legs and arms. In units of 1/seconds (standard character)",
    },
    {
      name = "ChestControlStiffnessScale",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
      perAnimSet = true,
      helptext = "Strength multiplier for swinging/spinning the legs and arms when controlling the chest",
    },
    {
      name = "ChestControlPassOnAmount",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "When controlling the chest, a small value confines the motions to the arms, a larger value results in leg motions",
    },
    {
      name = "MaxReachDistance",
      type = "float",
      value = 2.000000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "An edge will be considered only when closer to the character's chest than this distance. In metres (standard character).",
    },
    {
      name = "MinEdgeLength",
      type = "float",
      value = 0.250000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "An edge will be considered only when longer than this. In metres (standard character).",
    },
    {
      name = "MinAngleBetweenNormals",
      type = "float",
      value = 10.000000,
      min = 0.000000,
      max = 180.000000,
      perAnimSet = true,
      helptext = "An edge will be considered only if the angle between its two faces (normals) is larger than this",
    },
    {
      name = "MaxSlope",
      type = "float",
      value = 50.000000,
      min = 0.000000,
      max = 90.000000,
      perAnimSet = true,
      helptext = "An edge will be considered only if the angle between the binormal (average of the surface normals either side of the edge) and up is less than this. In degrees.",
    },
    {
      name = "MinMass",
      type = "float",
      value = 30.000000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "If an edge belongs to a dynamic or static object, it will only be considered if it is heavier than this. In kilograms (standard character).",
    },
    {
      name = "MinVolume",
      type = "float",
      value = 0.500000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "If an edge belongs to a dynamic object, it will only be considered if it is bigger than this. In cubic metres (standard character).",
    },
    {
      name = "MinEdgeQuality",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "An edge will be considered only if overall \"grabbability\" is greater than this",
    },
    {
      name = "ProjectEdgeNormalOntoGroundPlane",
      type = "bool",
      value = true,
      helptext = "Project edge front surface normal (as opposed to the upper surface normal) into the ground plane.",
    },
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(15, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(7, "numBehaviourInts")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "enableRaycast", asVal) and 1 ) or 0 ) , string.format("Int_0_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt(getValue(node, "LockedLinearDofs", asVal), string.format("Int_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt(getValue(node, "LockedAngularDofs", asVal), string.format("Int_2_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "HoldOnContact", asVal) and 1 ) or 0 ) , string.format("Int_3_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "DisableCollisions", asVal) and 1 ) or 0 ) , string.format("Int_4_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "DisableAngularDofsUntilHanging", asVal) and 1 ) or 0 ) , string.format("Int_5_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "ProjectEdgeNormalOntoGroundPlane") and 1 ) or 0 ) , string.format("Int_6_%d", asIdx-1))
    end

    stream:writeUInt(28, "numBehaviourFloats")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MinSupportSlope", asVal), string.format("Float_0_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "VerticalSpeedToStart", asVal), string.format("Float_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "UnbalancedAmount", asVal), string.format("Float_2_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MinUnbalancedPeriod", asVal), string.format("Float_3_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "VerticalSpeedToStop", asVal), string.format("Float_4_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "raycastEdgeOffset", asVal), string.format("Float_5_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "raycastLength", asVal), string.format("Float_6_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MinHoldPeriod", asVal), string.format("Float_7_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MaxHoldPeriod", asVal), string.format("Float_8_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "NoHoldPeriod", asVal), string.format("Float_9_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MaxHandsBehindBackPeriod", asVal), string.format("Float_10_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MaxReachAttemptPeriod", asVal), string.format("Float_11_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MinReachRecoveryPeriod", asVal), string.format("Float_12_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "CreateAtDistance", asVal), string.format("Float_13_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "EnableOrientationAtAngle", asVal), string.format("Float_14_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "DestroyAtDistance", asVal), string.format("Float_15_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "DisableOrientationAtAngle", asVal), string.format("Float_16_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "timeBeforeLookingDown", asVal), string.format("Float_17_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ChestControlImminence", asVal), string.format("Float_18_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ChestControlStiffnessScale", asVal), string.format("Float_19_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ChestControlPassOnAmount", asVal), string.format("Float_20_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MaxReachDistance", asVal), string.format("Float_21_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MinEdgeLength", asVal), string.format("Float_22_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MinAngleBetweenNormals", asVal), string.format("Float_23_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MaxSlope", asVal), string.format("Float_24_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MinMass", asVal), string.format("Float_25_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MinVolume", asVal), string.format("Float_26_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MinEdgeQuality", asVal), string.format("Float_27_%d", asIdx-1))
    end

    stream:writeUInt(0, "numBehaviourVector3s")

    stream:writeUInt(0, "numBehaviourUInt64s")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numBehaviourNodeAnimationInputs")

    -------------------------------------------------------------------

    stream:writeUInt(2, "numInputCPInts")
    stream:writeUInt(4, "numInputCPFloats")
    stream:writeUInt(0, "numInputCPUInt64s")
    stream:writeUInt(0, "numInputCPVector3s")
    stream:writeUInt(6, "numBehaviourControlParameterInputs")

    writeDataPinAltName(stream, node, "DoHold", "BehaviourControlParameterInputID_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_BOOL", "BehaviourControlParameterInputType_0")

    writeDataPinAltName(stream, node, "IgnoreOvershotEdges", "BehaviourControlParameterInputID_1")
    stream:writeString("ATTRIB_SEMANTIC_CP_BOOL", "BehaviourControlParameterInputType_1")

    writeDataPinAltName(stream, node, "HoldImportance_0", "BehaviourControlParameterInputID_2" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_2")
    writeDataPinAltName(stream, node, "HoldImportance_1", "BehaviourControlParameterInputID_3" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_3")

    writeDataPinAltName(stream, node, "PullUpAmount", "BehaviourControlParameterInputID_4")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_4")

    writeDataPinAltName(stream, node, "PullUpStrengthScale", "BehaviourControlParameterInputID_5")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_5")

    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterOutputType_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_BOOL", "BehaviourControlParameterOutputType_1")
    stream:writeString("ATTRIB_SEMANTIC_CP_BOOL", "BehaviourControlParameterOutputType_2")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterOutputType_3")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterOutputType_4")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterOutputType_5")
    stream:writeUInt(6, "numBehaviourControlParameterOutputs")

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
    title = "Reach Condition",
    helptext = "Settings that govern when a reach happens",
    isExpanded = true,
    details = 
    {
      {title = "GroundSlopeMoreThan", perAnimSet = true, type = "float", attributes = "MinSupportSlope"},
      {title = "FallingFasterThan", perAnimSet = true, type = "float", attributes = "VerticalSpeedToStart"},
      {title = "BalanceBelow", perAnimSet = true, type = "float", attributes = "UnbalancedAmount"},
      {title = "MinUnbalancedPeriod", perAnimSet = true, type = "float", attributes = "MinUnbalancedPeriod"},
    },
  },
  {
    title = "Stop Condition",
    helptext = "Settings that govern when a reach ends",
    isExpanded = false,
    details = 
    {
      {title = "MovingDownSlowerThan", perAnimSet = true, type = "float", attributes = "VerticalSpeedToStop"},
    },
  },
  {
    title = "Ignore Obstructed Edges",
    helptext = "The area underneath the edge to be grabbed can be probed to see if there is enough space to hang without unwanted obstructions. This is done using a raycast against the environment from a horizontal position offset from the edge (Edge offset) directed downwards.",
    isExpanded = false,
    details = 
    {
      {title = "Enable edge probing", perAnimSet = true, type = "bool", attributes = "enableRaycast"},
      {title = "Offset from edge", perAnimSet = true, type = "float", attributes = "raycastEdgeOffset"},
      {title = "Length", perAnimSet = true, type = "float", attributes = "raycastLength"},
    },
  },
  {
    title = "Hold Durations",
    helptext = "Settings that govern the time the character spends performing certain actions.",
    isExpanded = false,
    details = 
    {
      {title = "Min hold duration", perAnimSet = true, type = "float", attributes = "MinHoldPeriod"},
      {title = "Max hold duration", perAnimSet = true, type = "float", attributes = "MaxHoldPeriod"},
      {title = "Min time between holds", perAnimSet = true, type = "float", attributes = "NoHoldPeriod"},
      {title = "Max hold behind back time", perAnimSet = true, type = "float", attributes = "MaxHandsBehindBackPeriod"},
      {title = "Stop reach attempt after", perAnimSet = true, type = "float", attributes = "MaxReachAttemptPeriod"},
      {title = "Min time between attempts", perAnimSet = true, type = "float", attributes = "MinReachRecoveryPeriod"},
    },
  },
  {
    title = "Hold Constraint",
    helptext = "A hold is achieved by establishing a constraint, the hold is broken (the edge released) when the constraint is destroyed.",
    isExpanded = false,
    details = 
    {
      {title = "Closer than", perAnimSet = true, type = "float", attributes = "CreateAtDistance"},
      {title = "Angle less than", perAnimSet = true, type = "float", attributes = "EnableOrientationAtAngle"},
      {title = "Further than", perAnimSet = true, type = "float", attributes = "DestroyAtDistance"},
      {title = "Angle more than", perAnimSet = true, type = "float", attributes = "DisableOrientationAtAngle"},
      {title = "LinearDOF", perAnimSet = true, type = "integer", attributes = "LockedLinearDofs"},
      {title = "AngularDOF", perAnimSet = true, type = "integer", attributes = "LockedAngularDofs"},
      {title = "In contact", perAnimSet = true, type = "bool", attributes = "HoldOnContact"},
      {title = "DisableHandCollisions", perAnimSet = true, type = "bool", attributes = "DisableCollisions"},
      {title = "Constrain orientation only when hanging", perAnimSet = true, type = "bool", attributes = "DisableAngularDofsUntilHanging"},
    },
  },
  {
    title = "During Hold",
    helptext = "When holding a character can attempt to pull himself up rather than simply hanging. This can add to the realism.",
    isExpanded = false,
    details = 
    {
      {title = "timeBeforeLookingDown", perAnimSet = true, type = "float", attributes = "timeBeforeLookingDown"},
      {title = "Imminence", perAnimSet = true, type = "float", attributes = "ChestControlImminence"},
      {title = "Stiffness scale", perAnimSet = true, type = "float", attributes = "ChestControlStiffnessScale"},
      {title = "Move legs", perAnimSet = true, type = "float", attributes = "ChestControlPassOnAmount"},
    },
  },
  {
    title = "Edge Evaluation",
    helptext = "Settings that govern what the character will consider as an edge that can be grabbed.",
    isExpanded = false,
    details = 
    {
      {title = "MaxReachDistance", perAnimSet = true, type = "float", attributes = "MaxReachDistance"},
      {title = "MinEdgeLength", perAnimSet = true, type = "float", attributes = "MinEdgeLength"},
      {title = "MinAngleBetweenFaces", perAnimSet = true, type = "float", attributes = "MinAngleBetweenNormals"},
      {title = "MaxSlope", perAnimSet = true, type = "float", attributes = "MaxSlope"},
      {title = "MinMass", perAnimSet = true, type = "float", attributes = "MinMass"},
      {title = "MinVolume", perAnimSet = true, type = "float", attributes = "MinVolume"},
      {title = "MinEdgeQuality", perAnimSet = true, type = "float", attributes = "MinEdgeQuality"},
    },
  },
  {
    title = "Edge Normal",
    helptext = "Settings that govern edge normal.",
    isExpanded = false,
    details = 
    {
      {title = "projectIntoGroundPlane", perAnimSet = false, type = "bool", attributes = "ProjectEdgeNormalOntoGroundPlane"},
    },
  },
  {
    title = "Input Defaults",
    isExpanded = false,
    details = 
    {
      {title = "DoHold", perAnimSet = false, type = "bool", attributes = "DoHold"},
      {title = "IgnoreOvershotEdges", perAnimSet = false, type = "bool", attributes = "IgnoreOvershotEdges"},
      {title = "HoldWeight {Arm0}", perAnimSet = false, type = "float", attributes = "HoldImportance_0"},
      {title = "HoldWeight {Arm1}", perAnimSet = false, type = "float", attributes = "HoldImportance_1"},
      {title = "PullUpAmount", perAnimSet = false, type = "float", attributes = "PullUpAmount"},
      {title = "PullUpStrengthScale", perAnimSet = false, type = "float", attributes = "PullUpStrengthScale"},
    },
  },
}

return {
  data = behaviourNode,
  attributeGroups = attributeGroups,
}
