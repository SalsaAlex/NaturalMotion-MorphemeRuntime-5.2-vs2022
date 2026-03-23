--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "Hold Action",
  version = 5,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Makes the character grab and hold onto specific, marked up, edges when they are within reach",
  group = "Behaviours",
  image = "HoldActionBehaviour.png",
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
      helptext = "Dynamically toggle Hold. Set it to false to make character release the grabbed edge or to prevent the character from grabbing any edges.",
      type = "bool",
      mode = "HiddenInNetworkEditor",
      displayName = "DoHold",
      value = true,
    },

    ["IsWall"] = 
    {
      input = true,
      helptext = "True if the object being held is a wall, as opposed to a pole or ledge. This information is used to determine whether the steepness of the surface matters for grabbing, and whether there is a preferred side from which to hang.",
      type = "bool",
      mode = "HiddenInNetworkEditor",
      displayName = "IsWall",
    },

    ["EdgeImportance"] = 
    {
      input = true,
      helptext = "Edge importance that is used to arbitrate with other behaviours when reaching. The user specified endge will always be used if weight is > 1. Otherwise the edge will be evaluated alongside other, detected edges.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "EdgeWeight",
      value = 1.010000,
      min = 0.000000,
      max = 1.010000,
    },

    ["EdgeStart"] = 
    {
      input = true,
      helptext = "Position in world space of start of edge segment",
      type = "vector3",
      displayName = "EdgeStart",
    },

    ["EdgeEnd"] = 
    {
      input = true,
      helptext = "Position in world space of end of edge segment",
      type = "vector3",
      displayName = "EdgeEnd",
    },

    ["EdgeNormal"] = 
    {
      input = true,
      helptext = "Normal of (perpendicular to) edge. The back-of-hand normal will try to match this",
      type = "vector3",
      displayName = "EdgeNormal",
    },

    ["PhysicsObjectID"] = 
    {
      input = true,
      helptext = "The object to try to hold. If set, then the edge is specified relative to this object, in the local space of the object.",
      type = "physicsObjectID",
      displayName = "PhysicsObjectID",
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

    ["HoldImportance_0"] = 
    {
      input = true,
      helptext = "The amount each arm should hold onto the edge, in terms of competing with other behaviours for use of the arm. A value of zero will prevent the character from holding with the arm.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "HoldWeight {Arm0}",
      value = 1.000000,
    },

    ["HoldImportance_1"] = 
    {
      input = true,
      helptext = "The amount each arm should hold onto the edge, in terms of competing with other behaviours for use of the arm. A value of zero will prevent the character from holding with the arm.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "HoldWeight {Arm1}",
      value = 1.000000,
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
      helptext = "Indicates how much the behaviour is attempting to grab the edge (between 0 and 1). When the character has successfully grabbed and held onto an edge the value is 1.",
      type = "float",
    },

    ["HoldDuration"] = 
    {
      input = false,
      helptext = "Indicates how long the character has been constrained to an edge. In absolute units.",
      type = "float",
    },

  },

  pinOrder = 
  {
    "DoHold",
    "IsWall",
    "EdgeImportance",
    "EdgeStart",
    "EdgeEnd",
    "EdgeNormal",
    "PhysicsObjectID",
    "PullUpAmount",
    "PullUpStrengthScale",
    "HoldImportance_0",
    "HoldImportance_1",
    "Result",
    "NumConstrainedHands",
    "HandHolding_0",
    "HandHolding_1",
    "HoldAttemptImportance",
    "HoldDuration",
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
      helptext = "If ground is detected below the character it is not considered supportive (i.e. grabbing is allowed) if it is steeper than this (angle in degrees between \"up\" and ground normal)",
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
      max = 360.000000,
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
      max = 360.000000,
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
      name = "TimeBeforeLookingDown",
      type = "float",
      value = 0.800000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "The proportion of maximum hold duration at which the character will switch from looking at the edge to looking down. This gives an impression of fatigue.",
    },
    {
      name = "ChestControlImminence",
      type = "float",
      value = 2.000000,
      min = 0.000000,
      perAnimSet = true,
      helptext = "Used to control the chest (make it upright) by swinging/spinning the legs and arms. Smaller values will result in more vigorous movements. In units of 1/seconds (standard character)",
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
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(16, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(5, "numBehaviourInts")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt(getValue(node, "LockedLinearDofs", asVal), string.format("Int_0_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt(getValue(node, "LockedAngularDofs", asVal), string.format("Int_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "HoldOnContact", asVal) and 1 ) or 0 ) , string.format("Int_2_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "DisableCollisions", asVal) and 1 ) or 0 ) , string.format("Int_3_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "DisableAngularDofsUntilHanging", asVal) and 1 ) or 0 ) , string.format("Int_4_%d", asIdx-1))
    end

    stream:writeUInt(20, "numBehaviourFloats")
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
      stream:writeFloat(getValue(node, "MinHoldPeriod", asVal), string.format("Float_5_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MaxHoldPeriod", asVal), string.format("Float_6_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "NoHoldPeriod", asVal), string.format("Float_7_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MaxHandsBehindBackPeriod", asVal), string.format("Float_8_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MaxReachAttemptPeriod", asVal), string.format("Float_9_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MinReachRecoveryPeriod", asVal), string.format("Float_10_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "CreateAtDistance", asVal), string.format("Float_11_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "EnableOrientationAtAngle", asVal), string.format("Float_12_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "DestroyAtDistance", asVal), string.format("Float_13_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "DisableOrientationAtAngle", asVal), string.format("Float_14_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "TimeBeforeLookingDown", asVal), string.format("Float_15_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ChestControlImminence", asVal), string.format("Float_16_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ChestControlStiffnessScale", asVal), string.format("Float_17_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ChestControlPassOnAmount", asVal), string.format("Float_18_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "MaxReachDistance", asVal), string.format("Float_19_%d", asIdx-1))
    end

    stream:writeUInt(0, "numBehaviourVector3s")

    stream:writeUInt(0, "numBehaviourUInt64s")

    -------------------------------------------------------------------

    stream:writeUInt(0, "numBehaviourNodeAnimationInputs")

    -------------------------------------------------------------------

    stream:writeUInt(2, "numInputCPInts")
    stream:writeUInt(5, "numInputCPFloats")
    stream:writeUInt(1, "numInputCPUInt64s")
    stream:writeUInt(3, "numInputCPVector3s")
    stream:writeUInt(11, "numBehaviourControlParameterInputs")

    writeDataPinAltName(stream, node, "DoHold", "BehaviourControlParameterInputID_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_BOOL", "BehaviourControlParameterInputType_0")

    writeDataPinAltName(stream, node, "IsWall", "BehaviourControlParameterInputID_1")
    stream:writeString("ATTRIB_SEMANTIC_CP_BOOL", "BehaviourControlParameterInputType_1")

    writeDataPinAltName(stream, node, "EdgeImportance", "BehaviourControlParameterInputID_2")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_2")

    writeDataPinAltName(stream, node, "EdgeStart", "BehaviourControlParameterInputID_3")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_3")

    writeDataPinAltName(stream, node, "EdgeEnd", "BehaviourControlParameterInputID_4")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_4")

    writeDataPinAltName(stream, node, "EdgeNormal", "BehaviourControlParameterInputID_5")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_5")

    writeDataPinAltName(stream, node, "PhysicsObjectID", "BehaviourControlParameterInputID_6")
    stream:writeString("ATTRIB_SEMANTIC_CP_PHYSICS_OBJECT_POINTER", "BehaviourControlParameterInputType_6")

    writeDataPinAltName(stream, node, "PullUpAmount", "BehaviourControlParameterInputID_7")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_7")

    writeDataPinAltName(stream, node, "PullUpStrengthScale", "BehaviourControlParameterInputID_8")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_8")

    writeDataPinAltName(stream, node, "HoldImportance_0", "BehaviourControlParameterInputID_9" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_9")
    writeDataPinAltName(stream, node, "HoldImportance_1", "BehaviourControlParameterInputID_10" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_10")

    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterOutputType_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_BOOL", "BehaviourControlParameterOutputType_1")
    stream:writeString("ATTRIB_SEMANTIC_CP_BOOL", "BehaviourControlParameterOutputType_2")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterOutputType_3")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterOutputType_4")
    stream:writeUInt(5, "numBehaviourControlParameterOutputs")

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
    title = "Hold Durations",
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
      {title = "TimeBeforeLookingDown", perAnimSet = true, type = "float", attributes = "TimeBeforeLookingDown"},
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
    },
  },
  {
    title = "Input Defaults",
    isExpanded = false,
    details = 
    {
      {title = "DoHold", perAnimSet = false, type = "bool", attributes = "DoHold"},
      {title = "IsWall", perAnimSet = false, type = "bool", attributes = "IsWall"},
      {title = "EdgeWeight", perAnimSet = false, type = "float", attributes = "EdgeImportance"},
      {title = "EdgeStart", perAnimSet = false, type = "vector3", attributes = "EdgeStart"},
      {title = "EdgeEnd", perAnimSet = false, type = "vector3", attributes = "EdgeEnd"},
      {title = "EdgeNormal", perAnimSet = false, type = "vector3", attributes = "EdgeNormal"},
      {title = "PhysicsObjectID", perAnimSet = false, type = "int", attributes = "PhysicsObjectID"},
      {title = "PullUpAmount", perAnimSet = false, type = "float", attributes = "PullUpAmount"},
      {title = "PullUpStrengthScale", perAnimSet = false, type = "float", attributes = "PullUpStrengthScale"},
      {title = "HoldWeight {Arm0}", perAnimSet = false, type = "float", attributes = "HoldImportance_0"},
      {title = "HoldWeight {Arm1}", perAnimSet = false, type = "float", attributes = "HoldImportance_1"},
    },
  },
}

return {
  data = behaviourNode,
  attributeGroups = attributeGroups,
}
