------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

registerBehaviourNode("Balance",
{
  displayName = "Balance",
  version = 3,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Keeps the character in a specified standing pose or returns the character to that pose by stepping, if the character is off balance. ",
  group = "Behaviours",
  image = "BalanceBehaviour.png",
  id = generateNamespacedId(idNamespaces.NaturalMotion, 128),

  functionPins = 
  {
    ["BalancePose"] = 
    {
      input = true,
      helptext = "Target pose when balancing",
      interfaces = 
      {
        "Transforms",
        "Time",
      },
    },

    ["ReadyPose"] = 
    {
      input = true,
      helptext = "Pose used for the arms when stepping",
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
    ["TargetVelocity"] = 
    {
      input = true,
      helptext = "Creates a tendency to lean/stagger in a certain direction (will provoke stepping if enabled). Requires a non-zero velocity assistance. Specified in absolute units.",
      type = "vector3",
      mode = "HiddenInNetworkEditor",
      displayName = "Desired velocity",
    },

    ["TargetVelocityInCharacterSpace"] = 
    {
      input = true,
      helptext = "If true then the target velocity is specified in character space with x forward, y up and z right. If false then it is specified in world space.",
      type = "bool",
      mode = "HiddenInNetworkEditor",
      displayName = "TargetVelocityInCharacterSpace",
      value = false,
    },

    ["BalanceFwdOffset"] = 
    {
      input = true,
      helptext = "Sets a forwards offset for the pelvis position when balancing, in metres (standard character).",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "BalanceFwdOffset",
      value = 0.000000,
    },

    ["BalanceRightOffset"] = 
    {
      input = true,
      helptext = "Sets a rightwards offset for the pelvis position when balancing, in metres (standard character).",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "BalanceRightOffset",
      value = 0.000000,
    },

    ["LegStepStrength_0"] = 
    {
      input = true,
      helptext = "Sets the stepping strength scale on the legs. Accepts values in the range 0 to 1.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "LegStepStrength {Leg0}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["LegStepStrength_1"] = 
    {
      input = true,
      helptext = "Sets the stepping strength scale on the legs. Accepts values in the range 0 to 1.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "LegStepStrength {Leg1}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["ArmStepStrength_0"] = 
    {
      input = true,
      helptext = "Sets the stepping strength scale on the arms. Accepts values in the range 0 to 1.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "ArmStepStrength {Arm0}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["ArmStepStrength_1"] = 
    {
      input = true,
      helptext = "Sets the stepping strength scale on the arms. Accepts values in the range 0 to 1.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "ArmStepStrength {Arm1}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["LookInStepDirection"] = 
    {
      input = true,
      helptext = "Activates the tendency for the character to look in the direction it is stepping.",
      type = "bool",
      mode = "HiddenInNetworkEditor",
      displayName = "LookInStepDirection",
      value = true,
    },

    ["TargetPelvisDirection"] = 
    {
      input = true,
      helptext = "Target direction in world space for the pelvis. The character will try to align the pelvis forwards direction with this. The magnitude determines how much this is used, so 0 means it is ignored, and 1 means it overrides the normal direction, based on the character's current orientation.",
      type = "vector3",
      mode = "HiddenInNetworkEditor",
      displayName = "TargetPelvisDirection",
    },

    ["BalanceWithAnimationPose"] = 
    {
      input = true,
      helptext = "If zero, then the balancer tries to place the centre of mass over the support area. If one then it tries to match the animation pose (which may not be intrinsically balanced). Intermediate values blend between these extremes.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "MatchPoseWhenBalancing",
      value = 0.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["LegStandStrength_0"] = 
    {
      input = true,
      helptext = "Sets the stand strength scale on the legs. Accepts values in the range 0 to 1.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "LegStandStrength {Leg0}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["LegStandStrength_1"] = 
    {
      input = true,
      helptext = "Sets the stand strength scale on the legs. Accepts values in the range 0 to 1.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "LegStandStrength {Leg1}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["ArmStandStrength_0"] = 
    {
      input = true,
      helptext = "Sets the stand strength scale on the arms. Accepts values in the range 0 to 1.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "ArmStandStrength {Arm0}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["ArmStandStrength_1"] = 
    {
      input = true,
      helptext = "Sets the stand strength scale on the arms. Accepts values in the range 0 to 1.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "ArmStandStrength {Arm1}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["ExclusionZonePoint"] = 
    {
      input = true,
      helptext = "Point on the plane defining the stepping exclusion zone, in absolute units",
      type = "vector3",
      mode = "HiddenInNetworkEditor",
      displayName = "Point",
    },

    ["ExclusionZoneDirection"] = 
    {
      input = true,
      helptext = "Points into the exclusion zone. If the direction is zero then the exclusion zone is disabled.",
      type = "vector3",
      mode = "HiddenInNetworkEditor",
      displayName = "Direction",
    },

    ["OrientationAssistanceAmount"] = 
    {
      input = true,
      helptext = "Sets the scale of cheat forces which can be used in keeping the pelvis orientated correctly for balance. A value of 1 results in a moderate amount of assistance.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "OrientationAssistanceAmount",
      value = 0.000000,
      min = 0.000000,
      max = 4.000000,
    },

    ["PositionAssistanceAmount"] = 
    {
      input = true,
      helptext = "Sets the scale of cheat forces which can be used in keeping the pelvis positioned correctly for balance. A value of 1 results in a moderate amount of assistance.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "PositionAssistanceAmount",
      value = 0.000000,
      min = 0.000000,
      max = 4.000000,
    },

    ["VelocityAssistanceAmount"] = 
    {
      input = true,
      helptext = "Sets the scale of cheat forces which can be used in damping the pelvis/chest velocity towards the target/desired velocity. A value of 1 results in a moderate amount of assistance.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "VelocityAssistanceAmount",
      value = 0.000000,
      min = 0.000000,
      max = 4.000000,
    },

    ["UseCounterForceOnFeet"] = 
    {
      input = true,
      helptext = "Determines whether the stabilisation forces applied to the pelvis should be compensated for by an equal an opposite force on the feet. This helps preserve momentum when stepping on dynamic objects.",
      type = "bool",
      mode = "HiddenInNetworkEditor",
      displayName = "UseCounterForceOnFeet",
      value = false,
    },

    ["StandingStillTime"] = 
    {
      input = false,
      helptext = "The time that the character has been balancing without stepping (will be zero if stepping or fallen). In absolute units.",
      type = "float",
    },

    ["FallenTime"] = 
    {
      input = false,
      helptext = "The time that the character has been fallen, defined as when BalanceAmount is 0. In absolute units.",
      type = "float",
    },

    ["OnGroundTime"] = 
    {
      input = false,
      helptext = "The time that the character has been touching something that appears to be ground with his legs/feet. In absolute units.",
      type = "float",
    },

    ["BalanceAmount"] = 
    {
      input = false,
      helptext = "An indicator of how balanced the character is from 0 to 1. Will be close to 1 when balanced, and decrease to 0 if he is falling/lifted off the ground.",
      type = "float",
    },

    ["ForwardsDirection"] = 
    {
      input = false,
      helptext = "Direction forwards relative to the character, projected onto the horizontal plane",
      type = "vector3",
    },

    ["RightDirection"] = 
    {
      input = false,
      helptext = "Direction sideways to the right of the character, projected onto the horizontal plane",
      type = "vector3",
    },

    ["SupportVelocity"] = 
    {
      input = false,
      helptext = "Velocity of the balance support region",
      type = "vector3",
    },

    ["SteppingTime"] = 
    {
      input = false,
      helptext = "The time that stepping has been active (will be reset after StepCountResetTime). In absolute units.",
      type = "float",
    },

    ["StepCount"] = 
    {
      input = false,
      helptext = "The total number of steps (will be reset after StepCountResetTime)",
      type = "int",
    },

    ["LegStepFractions_0"] = 
    {
      input = false,
      helptext = "The approximate fraction through the step so 0 means the leg is just about to leave the ground, 1 when it reaches its target and the step finishes.",
      type = "float",
    },

    ["LegStepFractions_1"] = 
    {
      input = false,
      helptext = "The approximate fraction through the step so 0 means the leg is just about to leave the ground, 1 when it reaches its target and the step finishes.",
      type = "float",
    },

  },

  pinOrder = 
  {
    "BalancePose",
    "ReadyPose",
    "TargetVelocity",
    "TargetVelocityInCharacterSpace",
    "BalanceFwdOffset",
    "BalanceRightOffset",
    "LegStepStrength_0",
    "LegStepStrength_1",
    "ArmStepStrength_0",
    "ArmStepStrength_1",
    "LookInStepDirection",
    "TargetPelvisDirection",
    "BalanceWithAnimationPose",
    "LegStandStrength_0",
    "LegStandStrength_1",
    "ArmStandStrength_0",
    "ArmStandStrength_1",
    "ExclusionZonePoint",
    "ExclusionZoneDirection",
    "OrientationAssistanceAmount",
    "PositionAssistanceAmount",
    "VelocityAssistanceAmount",
    "UseCounterForceOnFeet",
    "Result",
    "StandingStillTime",
    "FallenTime",
    "OnGroundTime",
    "BalanceAmount",
    "ForwardsDirection",
    "RightDirection",
    "SupportVelocity",
    "SteppingTime",
    "StepCount",
    "LegStepFractions_0",
    "LegStepFractions_1",
  },

  attributes = 
  {
    {
      name = "EnableStand",
      type = "bool",
      value = true,
      helptext = "Unused",
    },
    {
      name = "SupportWithArms",
      type = "bool",
      value = false,
      perAnimSet = true,
      helptext = "Use the arms as well as the legs to stand and balance on. This is a possibility for balancing for quadrapeds whose forlegs are marked as arms.",
    },
    {
      name = "FootBalanceAmount",
      type = "float",
      value = 481.191376,
      min = 0.000000,
      max = 5000.000000,
      perAnimSet = true,
      helptext = "Amount to angle the feet to balance (this is the foot target angle in degrees when at the edge of balancing). Generally make it as large as possible, but if it is too large it will cause the feet to jitter.",
    },
    {
      name = "DecelerationAmount",
      type = "float",
      value = 0.549265,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Amount that the character should lean away from movement in order to decelerate",
    },
    {
      name = "MaxAngle",
      type = "float",
      value = 60.000000,
      min = 0.000000,
      max = 90.000000,
      perAnimSet = true,
      helptext = "Maximum angle for the feet, in degrees",
    },
    {
      name = "FootLength",
      type = "float",
      value = 0.392197,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Foot length in metres (standard character). Smaller values result in the character stepping earlier, which can prevent him teetering on the edge of static balance.",
    },
    {
      name = "LowerPelvisDistanceWhenFootLifts",
      type = "float",
      value = 0.456781,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Distance to lower the pelvis (i.e. crouch) when a supporting foot lifts off the floor. In metres (standard character).",
    },
    {
      name = "AllowArmSpin",
      type = "bool",
      value = false,
      perAnimSet = true,
      helptext = "Allow the character to or windmill its arms (when needed) in order to maintain balance. If a character is unable to step then windmilling its arms can be the only way it can maintain balance.",
    },
    {
      name = "SpinAmount",
      type = "float",
      value = 1.000000,
      min = 0.100000,
      max = 5.000000,
      perAnimSet = true,
      helptext = "Scales the amount and speed of arm spinning",
    },
    {
      name = "SpinThreshold",
      type = "float",
      value = 2.000000,
      min = 0.000000,
      max = 5.000000,
      perAnimSet = true,
      helptext = "How off-balance the character needs to be to start spinning his arms",
    },
    {
      name = "EnableCollisionGroup",
      type = "bool",
      value = true,
      helptext = "Use a collision group to avoid leg/ arm collision.  If enabled the collision group index must be provided. ",
    },
    {
      name = "CollisionGroupIndex",
      type = "int",
      value = -1,
      helptext = "Collision group used to disable leg/arm collision when stepping. This is the name of the collision group within the collision groups editor. This name is stored as an index into the group as appears in the collision group index.",
    },
    {
      name = "StepWithLegs",
      type = "bool",
      value = true,
      perAnimSet = true,
      helptext = "Use the legs to step with",
    },
    {
      name = "StepWithArms",
      type = "bool",
      value = false,
      perAnimSet = true,
      helptext = "Use the arms to step with. This is a possibility for quadrapeds whose forlegs are marked as arms",
    },
    {
      name = "SuppressSteppingTime",
      type = "float",
      value = 0.000000,
      perAnimSet = true,
      helptext = "Prevent stepping for this time after enabling, in seconds (standard character).",
    },
    {
      name = "ArmSwingStrengthScaleWhenStepping",
      type = "float",
      value = 0.500000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Scales the strength with which the character can swing its arms when stepping. ",
    },
    {
      name = "StepCountResetTime",
      type = "float",
      value = 1.000000,
      perAnimSet = true,
      helptext = "The StepCount that is emitted as an output parameter is reset if this amount of time has passed from the last step. In seconds (standard character).",
    },
    {
      name = "SuppressFootCrossing",
      type = "bool",
      value = false,
      perAnimSet = true,
      helptext = "Attempt to prevent/reduce stepping through such that the step would cross the other stance feet",
    },
    {
      name = "FootCrossingOffset",
      type = "float",
      value = 0.100000,
      min = -0.400000,
      max = 0.400000,
      perAnimSet = true,
      helptext = "Positive values will cut the step short before it would cross the other foot. Negative values will allow for more foot crossing. In metres (standard character).",
    },
    {
      name = "StepPredictionTimeForward",
      type = "float",
      value = 0.127242,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Time used to calculate the step length when stepping forwards, in seconds (standard character).",
    },
    {
      name = "StepPredictionTimeBackward",
      type = "float",
      value = 0.408668,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Time used to calculate the step length when stepping backwards, in seconds (standard character).",
    },
    {
      name = "StepUpDistance",
      type = "float",
      value = 0.702910,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "How high the steps should be, in metres (standard character). Higher values will tend to make the character stagger with high steps, so the foot will be unlikely to catch on the ground. Lower values may look more natural, at the risk of catching the foot on the ground.",
    },
    {
      name = "StepDownDistance",
      type = "float",
      value = 0.945113,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "When the balancer wishes to end a step he aims this distance below the ground. In metres (standard character).",
    },
    {
      name = "MaxStepLength",
      type = "float",
      value = 2.000000,
      min = 0.000000,
      max = 2.000000,
      perAnimSet = true,
      helptext = "Max step length, in metres (standard character).",
    },
    {
      name = "StepDownSpeed",
      type = "float",
      value = 0.942423,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Speed to aim the foot down as the step time progresses, in m/s (standard character). This can help prevent the foot from hanging if for some reason it does not progress.",
    },
    {
      name = "FootSeparationFraction",
      type = "float",
      value = 0.800000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Target foot separation as a fraction of the balance pose during the step.",
    },
    {
      name = "AlignFootToFloorWeight",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 5.000000,
      perAnimSet = true,
      helptext = "Amount to align the foot to the floor (rather than making it follow the natural swing of the leg) during the step.",
    },
    {
      name = "LowerPelvisAmount",
      type = "float",
      value = 0.354872,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Amount to lower the pelvis when stepping as a fraction of the step length.",
    },
    {
      name = "StrengthScale",
      type = "float",
      value = 1.600000,
      min = 0.000000,
      max = 2.000000,
      perAnimSet = true,
      helptext = "Step strength multiplier, so larger values make the stepping leg stronger.",
    },
    {
      name = "DampingRatioScale",
      type = "float",
      value = 1.134721,
      min = 0.000000,
      max = 2.000000,
      perAnimSet = true,
      helptext = "Step damping ratio multiplier, so larger values make the stepping leg move slower but more stiffly.",
    },
    {
      name = "SteppingOrientationWeight",
      type = "float",
      value = 0.475309,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "IK orientation weight.",
    },
    {
      name = "IKSubStep",
      type = "float",
      value = 0.500000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "IK sub step (values < 1 make the stepping foot follow a straighter line rather than an arc).",
    },
    {
      name = "SteppingGravityCompensation",
      type = "float",
      value = 0.042387,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Amount of gravity compensation on the stepping leg",
    },
    {
      name = "SteppingLimbLengthToAbort",
      type = "float",
      value = 0.700000,
      min = 0.000000,
      max = 2.000000,
      perAnimSet = true,
      helptext = "First condition for preventing or aborting a step - if the distance between the foot and leg root is less than this (e.g. if the character has fallen and the legs are compressed), in metres (standard character).",
    },
    {
      name = "SteppingRootDownSpeedToAbort",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 10.000000,
      perAnimSet = true,
      helptext = "Second condition for preventing or aborting a step - if the leg root is moving downwards with a speed greater than this, in m/s (standard character).",
    },
    {
      name = "SteppingDirectionThreshold",
      type = "float",
      value = -0.502405,
      min = -1.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Threshold for when the character should step toe or heel first. If 0 then it happens when stepping directly forwards. -1 results in stepping toe first even when stepping backwards.",
    },
    {
      name = "SteppingImplicitStiffness",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "How much the character anticipates the ground when stepping. Smaller values result in the leg being weaker when it hits the ground (which can appear more natural)",
    },
    {
      name = "EnableLook",
      type = "bool",
      value = true,
      perAnimSet = true,
      helptext = "Allow the character to Look in the step direction.",
    },
    {
      name = "StepsBeforeLooking",
      type = "int",
      value = 3,
      min = 0,
      max = 10,
      perAnimSet = true,
      helptext = "Look in step direction after this many consecutive steps",
    },
    {
      name = "StopLookingTime",
      type = "float",
      value = 1.000000,
      min = 0.100000,
      max = 10.000000,
      perAnimSet = true,
      helptext = "Time over which to stop looking in the step direction, in seconds (standard character).",
    },
    {
      name = "MinSpeedForLooking",
      type = "float",
      value = 0.300000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Speed below which the character shouldn't look where they're stepping. In m/s (standard character).",
    },
    {
      name = "WholeBodyLook",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Set to 0 to only look with the head, or 1 to use as much of the body as possible",
    },
    {
      name = "LookDownAngle",
      type = "float",
      value = 20.000000,
      min = -45.000000,
      max = 60.000000,
      perAnimSet = true,
      helptext = "Angle (in degrees) to look down when stepping",
    },
    {
      name = "LookWeight",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "0- don't look where you step, 1- look with normal strength in the direction you are stepping",
    },
    {
      name = "StepToRecoverPose",
      type = "bool",
      value = true,
      perAnimSet = true,
      helptext = "Enable/disable stepping to recover the stand pose",
    },
    {
      name = "FwdDistanceToTriggerStep",
      type = "float",
      value = 0.150000,
      perAnimSet = true,
      helptext = "Step if a foot is this much ahead of where it should be, relative to the balance pose, in metres (standard character).",
    },
    {
      name = "SidewaysDistanceToTriggerStep",
      type = "float",
      value = 0.150000,
      perAnimSet = true,
      helptext = "Step if a foot is this much sideways relative to the balance pose, in metres (standard character).",
    },
    {
      name = "TimeBeforeShiftingWeight",
      type = "float",
      value = 0.200000,
      min = 0.000000,
      max = 10.000000,
      perAnimSet = true,
      helptext = "Time to wait before starting to shift the balance weight prior to a recovery step, in seconds (standard character).",
    },
    {
      name = "WeightShiftingTime",
      type = "float",
      value = 0.500000,
      min = 0.000000,
      max = 10.000000,
      perAnimSet = true,
      helptext = "Time to spend shifting the balance weight before stepping, in seconds (standard character).",
    },
    {
      name = "EnablePoseClamping",
      type = "bool",
      value = true,
      helptext = "Enable clamping of the pose - there is a small performance cost to doing so",
    },
    {
      name = "DynamicClamping",
      type = "bool",
      value = false,
      perAnimSet = true,
      helptext = "If set then the clamping is made more restrictive depending on the state of the character. This can help him keep his balance, but in some situations may cause him to oscillate to some degree.",
    },
    {
      name = "MinPelvisPitch",
      type = "float",
      value = -45.000000,
      min = -90.000000,
      max = 0.000000,
      perAnimSet = true,
      helptext = "Minimum pelvis pitch in degrees - positive values pitch up",
    },
    {
      name = "MaxPelvisPitch",
      type = "float",
      value = 25.000000,
      min = 0.000000,
      max = 90.000000,
      perAnimSet = true,
      helptext = "Maximum pelvis pitch in degrees - positive values pitch up",
    },
    {
      name = "MinPelvisRoll",
      type = "float",
      value = -30.000000,
      min = -90.000000,
      max = 0.000000,
      perAnimSet = true,
      helptext = "Minimum pelvis roll in degrees - positive values roll to the right",
    },
    {
      name = "MaxPelvisRoll",
      type = "float",
      value = 30.000000,
      min = 0.000000,
      max = 90.000000,
      perAnimSet = true,
      helptext = "Maximum pelvis roll in degrees - positive values roll to the right",
    },
    {
      name = "MinPelvisYaw",
      type = "float",
      value = -45.000000,
      min = -90.000000,
      max = 0.000000,
      perAnimSet = true,
      helptext = "Minimum pelvis yaw in degrees - positive values yaw to the left",
    },
    {
      name = "MaxPelvisYaw",
      type = "float",
      value = 45.000000,
      min = 0.000000,
      max = 90.000000,
      perAnimSet = true,
      helptext = "Maximum pelvis yaw in degrees - positive values yaw to the left",
    },
    {
      name = "MinPelvisHeight",
      type = "float",
      value = 0.400000,
      perAnimSet = true,
      helptext = "Minimum pelvis height above the feet, in metres (standard character).",
    },
    {
      name = "MaxPelvisHeight",
      type = "float",
      value = 1.000000,
      perAnimSet = true,
      helptext = "Maximum pelvis height above the feet, in metres (standard character).",
    },
    {
      name = "PelvisPositionChangeTimescale",
      type = "float",
      value = 0.300000,
      perAnimSet = true,
      helptext = "Timescale over which to smooth the pelvis target position, in seconds (standard character).",
    },
    {
      name = "PelvisOrientationChangeTimescale",
      type = "float",
      value = 0.400000,
      perAnimSet = true,
      helptext = "Timescale over which to smooth the pelvis target orientation, in seconds (standard character).",
    },
    {
      name = "UseSingleFrameForBalancePose",
      type = "bool",
      value = false,
      perAnimSet = true,
      helptext = "If set then pose will be cached on the first update. This means that the pose that is used will not change if the input changes.",
    },
    {
      name = "NonSupportingGravityCompensationScale",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 4.000000,
      perAnimSet = true,
      helptext = "Gravity compensation on the non-supporting arm pose. Can be greater than 1 if you want full compensation on otherwise weak arms - in which case set it to one divided by the balance pose weight.",
    },
    {
      name = "NonSupportingDampingScale",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 100.000000,
      perAnimSet = true,
      helptext = "Damping scale on the non-supporting arm pose (1 means use default damping). Note that when using low strengths (in the BalancePose section) the damping will automatically be reduced, so this value may need to be rather high (e.g. 1/strength) in order to see significant damping.",
    },
    {
      name = "NonSupportingDriveCompensationScale",
      type = "float",
      value = 0.200000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "Drive compensation on the non-supporting arm pose. A value of 1 should be used if you want these limbs to be strong and coordinated. However, if you want the unsupported limbs to be quite loose, then a value of zero is better, as that will prevent 'floaty' artefacts.",
    },
    {
      name = "UseSingleFrameForReadyPose",
      type = "bool",
      value = false,
      perAnimSet = true,
      helptext = "If set then pose will be cached on the first update. This means that the pose that is used will not change if the input changes.",
    },
    {
      name = "ArmDisplacementTime",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
      perAnimSet = true,
      helptext = "How much to move the hands in the direction of the step when stepping, calculated by multiplying velocity of the character by this value. In seconds (standard character).",
    },
    {
      name = "EnableAssistance",
      type = "bool",
      value = true,
      perAnimSet = true,
      helptext = "Enable assistance forces.",
    },
    {
      name = "ReduceAssistanceWhenFalling",
      type = "bool",
      value = true,
      perAnimSet = true,
      helptext = "If true then the assistance amount will be reduced if the character looses balance, which means less assistance when more horizontal in angle. This prevents the character from appearing to magically stay up while leaning at an angle that should cause it to fall.",
    },
    {
      name = "MaxLinearAccelerationAssistance",
      type = "float",
      value = 0.000000,
      perAnimSet = true,
      helptext = "Maximum linear acceleration provided by the assistance (disabled if zero) in m/s^2 (standard character).",
    },
    {
      name = "MaxAngularAccelerationAssistance",
      type = "float",
      value = 0.000000,
      perAnimSet = true,
      helptext = "Maximum linear acceleration provided by the assistance (disabled if zero) in deg/s^2 (standard character).",
    },
    {
      name = "AssistanceChestToPelvisRatio",
      type = "float",
      value = 0.500000,
      min = 0.000000,
      max = 1.000000,
      perAnimSet = true,
      helptext = "How to distribute the balance cheat forces. A value of 1 results in them all being applied to the chest. A value of 0 results in them all being applied to the pelvis.",
    },
    {
      name = "FallenTimeToEmitRequests",
      type = "float",
      value = 1.000000,
      perAnimSet = true,
      helptext = "Messages for fallen/recovered get triggered when the fallen time exceeds this, in seconds (standard character).",
    },
    {
      name = "IgnoreDirectionWhenOutOfRange",
      type = "bool",
      value = false,
      helptext = "Ignore desired pelvis direction input if it is outside of this range. The centre of the range points in the pelvis forward direction. Pitch and Yaw are in degrees.",
    },
    {
      name = "TargetYawRight",
      type = "float",
      value = -90.000000,
      min = -180.000000,
      max = 180.000000,
      helptext = "Ignore Desired Pelvis Direction input if yaw is outside of this range. The centre of the range points in the pelvis forward direction and the yaw value increases from the character's right to its left. In degrees.",
    },
    {
      name = "TargetYawLeft",
      type = "float",
      value = 90.000000,
      min = -180.000000,
      max = 180.000000,
      helptext = "Ignore Desired Pelvis Direction input if yaw is outside of this range. The centre of the range points in the pelvis forward direction and the yaw value increases from the character's right to its left. In degrees",
    },
    {
      name = "TargetPitchDown",
      type = "float",
      value = -90.000000,
      min = -180.000000,
      max = 180.000000,
      helptext = "Ignore Desired Pelvis Direction input if pitch is outside of this range. The centre of the range points in the pelvis forward direction and the pitch value increases from character down to up. In degrees.",
    },
    {
      name = "TargetPitchUp",
      type = "float",
      value = 90.000000,
      min = -180.000000,
      max = 180.000000,
      helptext = "Ignore Desired Pelvis Direction input if pitch is outside of this range. The centre of the range points in the pelvis forward direction and the pitch value increases from character down to up. In degrees.",
    },
    {
      name = "BalancePose_ApplyToArm_1",
      type = "float",
      value = 0.300000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Arm.",
    },
    {
      name = "BalancePose_ApplyToArm_2",
      type = "float",
      value = 0.300000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Arm.",
    },
    {
      name = "BalancePose_ApplyToHead_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Head.",
    },
    {
      name = "BalancePose_ApplyToLeg_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Leg.",
    },
    {
      name = "BalancePose_ApplyToLeg_2",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Leg.",
    },
    {
      name = "BalancePose_ApplyToSpine_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Spine.",
    },
    {
      name = "ReadyPose_ApplyToArm_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Arm.",
    },
    {
      name = "ReadyPose_ApplyToArm_2",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Arm.",
    },
    {
      name = "ReadyPose_ApplyToHead_1",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Head.",
    },
    {
      name = "ReadyPose_ApplyToLeg_1",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Leg.",
    },
    {
      name = "ReadyPose_ApplyToLeg_2",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Leg.",
    },
    {
      name = "ReadyPose_ApplyToSpine_1",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Spine.",
    },
    -- Request 0, (HasFallen)
    { name = "EmittedRequest0", type = "request", helptext = "The message that should be sent or send a Clear All to remove any unhandled messages from the target state machine"},
    { name = "Action0", type = "string", value = "Set", helptext = "The action that should be performed on the destination statemachine. Set marks this message as sent, Clear removes this message from the destination if it hasn't been processed."},
    { name = "Target0", type = "ref", kind = "allStateMachines", helptext = "The state machine to send this message to. It is only possible to send messages to state machines below this node in the network."},

    -- Request 1, (HasRecoveredBalance)
    { name = "EmittedRequest1", type = "request", helptext = "The message that should be sent or send a Clear All to remove any unhandled messages from the target state machine"},
    { name = "Action1", type = "string", value = "Set", helptext = "The action that should be performed on the destination statemachine. Set marks this message as sent, Clear removes this message from the destination if it hasn't been processed."},
    { name = "Target1", type = "ref", kind = "allStateMachines", helptext = "The state machine to send this message to. It is only possible to send messages to state machines below this node in the network."},

  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(6, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(18, "numBehaviourInts")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getAttribute(node, "EnableStand") and 1 ) or 0 ) , string.format("Int_0_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getAttribute(node, "SupportWithArms", asVal) and 1 ) or 0 ) , string.format("Int_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getAttribute(node, "AllowArmSpin", asVal) and 1 ) or 0 ) , string.format("Int_2_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getAttribute(node, "EnableCollisionGroup") and 1 ) or 0 ) , string.format("Int_3_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt(getAttribute(node, "CollisionGroupIndex"), string.format("Int_4_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getAttribute(node, "StepWithLegs", asVal) and 1 ) or 0 ) , string.format("Int_5_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getAttribute(node, "StepWithArms", asVal) and 1 ) or 0 ) , string.format("Int_6_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getAttribute(node, "SuppressFootCrossing", asVal) and 1 ) or 0 ) , string.format("Int_7_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getAttribute(node, "EnableLook", asVal) and 1 ) or 0 ) , string.format("Int_8_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt(getAttribute(node, "StepsBeforeLooking", asVal), string.format("Int_9_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getAttribute(node, "StepToRecoverPose", asVal) and 1 ) or 0 ) , string.format("Int_10_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getAttribute(node, "EnablePoseClamping") and 1 ) or 0 ) , string.format("Int_11_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getAttribute(node, "DynamicClamping", asVal) and 1 ) or 0 ) , string.format("Int_12_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getAttribute(node, "UseSingleFrameForBalancePose", asVal) and 1 ) or 0 ) , string.format("Int_13_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getAttribute(node, "UseSingleFrameForReadyPose", asVal) and 1 ) or 0 ) , string.format("Int_14_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getAttribute(node, "EnableAssistance", asVal) and 1 ) or 0 ) , string.format("Int_15_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getAttribute(node, "ReduceAssistanceWhenFalling", asVal) and 1 ) or 0 ) , string.format("Int_16_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getAttribute(node, "IgnoreDirectionWhenOutOfRange") and 1 ) or 0 ) , string.format("Int_17_%d", asIdx-1))
    end

    stream:writeUInt(72, "numBehaviourFloats")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "FootBalanceAmount", asVal), string.format("Float_0_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "DecelerationAmount", asVal), string.format("Float_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "MaxAngle", asVal), string.format("Float_2_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "FootLength", asVal), string.format("Float_3_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "LowerPelvisDistanceWhenFootLifts", asVal), string.format("Float_4_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "SpinAmount", asVal), string.format("Float_5_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "SpinThreshold", asVal), string.format("Float_6_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "SuppressSteppingTime", asVal), string.format("Float_7_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "ArmSwingStrengthScaleWhenStepping", asVal), string.format("Float_8_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "StepCountResetTime", asVal), string.format("Float_9_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "FootCrossingOffset", asVal), string.format("Float_10_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "StepPredictionTimeForward", asVal), string.format("Float_11_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "StepPredictionTimeBackward", asVal), string.format("Float_12_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "StepUpDistance", asVal), string.format("Float_13_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "StepDownDistance", asVal), string.format("Float_14_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "MaxStepLength", asVal), string.format("Float_15_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "StepDownSpeed", asVal), string.format("Float_16_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "FootSeparationFraction", asVal), string.format("Float_17_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "AlignFootToFloorWeight", asVal), string.format("Float_18_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "LowerPelvisAmount", asVal), string.format("Float_19_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "StrengthScale", asVal), string.format("Float_20_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "DampingRatioScale", asVal), string.format("Float_21_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "SteppingOrientationWeight", asVal), string.format("Float_22_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "IKSubStep", asVal), string.format("Float_23_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "SteppingGravityCompensation", asVal), string.format("Float_24_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "SteppingLimbLengthToAbort", asVal), string.format("Float_25_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "SteppingRootDownSpeedToAbort", asVal), string.format("Float_26_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "SteppingDirectionThreshold", asVal), string.format("Float_27_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "SteppingImplicitStiffness", asVal), string.format("Float_28_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "StopLookingTime", asVal), string.format("Float_29_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "MinSpeedForLooking", asVal), string.format("Float_30_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "WholeBodyLook", asVal), string.format("Float_31_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "LookDownAngle", asVal), string.format("Float_32_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "LookWeight", asVal), string.format("Float_33_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "FwdDistanceToTriggerStep", asVal), string.format("Float_34_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "SidewaysDistanceToTriggerStep", asVal), string.format("Float_35_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "TimeBeforeShiftingWeight", asVal), string.format("Float_36_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "WeightShiftingTime", asVal), string.format("Float_37_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "MinPelvisPitch", asVal), string.format("Float_38_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "MaxPelvisPitch", asVal), string.format("Float_39_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "MinPelvisRoll", asVal), string.format("Float_40_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "MaxPelvisRoll", asVal), string.format("Float_41_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "MinPelvisYaw", asVal), string.format("Float_42_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "MaxPelvisYaw", asVal), string.format("Float_43_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "MinPelvisHeight", asVal), string.format("Float_44_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "MaxPelvisHeight", asVal), string.format("Float_45_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "PelvisPositionChangeTimescale", asVal), string.format("Float_46_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "PelvisOrientationChangeTimescale", asVal), string.format("Float_47_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "NonSupportingGravityCompensationScale", asVal), string.format("Float_48_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "NonSupportingDampingScale", asVal), string.format("Float_49_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "NonSupportingDriveCompensationScale", asVal), string.format("Float_50_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "ArmDisplacementTime", asVal), string.format("Float_51_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "MaxLinearAccelerationAssistance", asVal), string.format("Float_52_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "MaxAngularAccelerationAssistance", asVal), string.format("Float_53_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "AssistanceChestToPelvisRatio", asVal), string.format("Float_54_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "FallenTimeToEmitRequests", asVal), string.format("Float_55_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "TargetYawRight"), string.format("Float_56_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "TargetYawLeft"), string.format("Float_57_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "TargetPitchDown"), string.format("Float_58_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "TargetPitchUp"), string.format("Float_59_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "BalancePose_ApplyToArm_1"), string.format("Float_60_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "BalancePose_ApplyToArm_2"), string.format("Float_61_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "BalancePose_ApplyToHead_1"), string.format("Float_62_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "BalancePose_ApplyToLeg_1"), string.format("Float_63_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "BalancePose_ApplyToLeg_2"), string.format("Float_64_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "BalancePose_ApplyToSpine_1"), string.format("Float_65_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "ReadyPose_ApplyToArm_1"), string.format("Float_66_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "ReadyPose_ApplyToArm_2"), string.format("Float_67_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "ReadyPose_ApplyToHead_1"), string.format("Float_68_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "ReadyPose_ApplyToLeg_1"), string.format("Float_69_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "ReadyPose_ApplyToLeg_2"), string.format("Float_70_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getAttribute(node, "ReadyPose_ApplyToSpine_1"), string.format("Float_71_%d", asIdx-1))
    end

    stream:writeUInt(0, "numBehaviourVector3s")

    stream:writeUInt(0, "numBehaviourUInt64s")

    -------------------------------------------------------------------

    local inputNodeID_BalancePose = -1
    if isConnected{SourcePin = node .. ".BalancePose", ResolveReferences = true} then
      inputNodeID_BalancePose = getConnectedNodeID(node, "BalancePose")
    end

    local inputNodeID_ReadyPose = -1
    if isConnected{SourcePin = node .. ".ReadyPose", ResolveReferences = true} then
      inputNodeID_ReadyPose = getConnectedNodeID(node, "ReadyPose")
    end

    stream:writeUInt(2, "numBehaviourNodeAnimationInputs")

    stream:writeNetworkNodeId(inputNodeID_BalancePose, "BehaviourNodeAnimationInput_0")

    stream:writeNetworkNodeId(inputNodeID_ReadyPose, "BehaviourNodeAnimationInput_1")

    -------------------------------------------------------------------

    stream:writeUInt(3, "numInputCPInts")
    stream:writeUInt(14, "numInputCPFloats")
    stream:writeUInt(0, "numInputCPUInt64s")
    stream:writeUInt(4, "numInputCPVector3s")
    stream:writeUInt(21, "numBehaviourControlParameterInputs")
	
	local writeDataPinAltName = function(stream, node, pin, field)
		local inNodeInfo = getConnectedNodeInfo(node, pin)
		if inNodeInfo ~= nil then
			stream:writeNetworkNodeId(inNodeInfo.id, field, inNodeInfo.pinIndex)
		else
			stream:writeNetworkNodeId(-1, field)
		end
	end

    writeDataPinAltName(stream, node, "TargetVelocity", "BehaviourControlParameterInputID_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_0")

    writeDataPinAltName(stream, node, "TargetVelocityInCharacterSpace", "BehaviourControlParameterInputID_1")
    stream:writeString("ATTRIB_SEMANTIC_CP_BOOL", "BehaviourControlParameterInputType_1")

    writeDataPinAltName(stream, node, "BalanceFwdOffset", "BehaviourControlParameterInputID_2")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_2")

    writeDataPinAltName(stream, node, "BalanceRightOffset", "BehaviourControlParameterInputID_3")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_3")

    writeDataPinAltName(stream, node, "LegStepStrength_0", "BehaviourControlParameterInputID_4" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_4")
    writeDataPinAltName(stream, node, "LegStepStrength_1", "BehaviourControlParameterInputID_5" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_5")

    writeDataPinAltName(stream, node, "ArmStepStrength_0", "BehaviourControlParameterInputID_6" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_6")
    writeDataPinAltName(stream, node, "ArmStepStrength_1", "BehaviourControlParameterInputID_7" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_7")

    writeDataPinAltName(stream, node, "LookInStepDirection", "BehaviourControlParameterInputID_8")
    stream:writeString("ATTRIB_SEMANTIC_CP_BOOL", "BehaviourControlParameterInputType_8")

    writeDataPinAltName(stream, node, "TargetPelvisDirection", "BehaviourControlParameterInputID_9")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_9")

    writeDataPinAltName(stream, node, "BalanceWithAnimationPose", "BehaviourControlParameterInputID_10")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_10")

    writeDataPinAltName(stream, node, "LegStandStrength_0", "BehaviourControlParameterInputID_11" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_11")
    writeDataPinAltName(stream, node, "LegStandStrength_1", "BehaviourControlParameterInputID_12" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_12")

    writeDataPinAltName(stream, node, "ArmStandStrength_0", "BehaviourControlParameterInputID_13" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_13")
    writeDataPinAltName(stream, node, "ArmStandStrength_1", "BehaviourControlParameterInputID_14" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_14")

    writeDataPinAltName(stream, node, "ExclusionZonePoint", "BehaviourControlParameterInputID_15")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_15")

    writeDataPinAltName(stream, node, "ExclusionZoneDirection", "BehaviourControlParameterInputID_16")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_16")

    writeDataPinAltName(stream, node, "OrientationAssistanceAmount", "BehaviourControlParameterInputID_17")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_17")

    writeDataPinAltName(stream, node, "PositionAssistanceAmount", "BehaviourControlParameterInputID_18")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_18")

    writeDataPinAltName(stream, node, "VelocityAssistanceAmount", "BehaviourControlParameterInputID_19")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_19")

    writeDataPinAltName(stream, node, "UseCounterForceOnFeet", "BehaviourControlParameterInputID_20")
    stream:writeString("ATTRIB_SEMANTIC_CP_BOOL", "BehaviourControlParameterInputType_20")

    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterOutputType_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterOutputType_1")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterOutputType_2")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterOutputType_3")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterOutputType_4")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterOutputType_5")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterOutputType_6")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterOutputType_7")
    stream:writeString("ATTRIB_SEMANTIC_CP_INT", "BehaviourControlParameterOutputType_8")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterOutputType_9")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterOutputType_10")
    stream:writeUInt(11, "numBehaviourControlParameterOutputs")

    -------------------------------------------------------------------

    local numMessageSlots = 2
    local numOutputMessages = 0
    -- An array of emitted messages length numMessageSlots or 0, (use all slots or none).
    for i = 0, (numMessageSlots-1) do
      if (serializeRequest(node, stream, tostring(i), tostring(i), true)) then
        numOutputMessages = numOutputMessages + 1
      end
    end
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
    title = "Stand",
    helptext = "Settings that govern the characters behaviour when standing (stationary balance)",
    isExpanded = true,
    details = 
    {
      {title = "EnableStand", perAnimSet = false, type = "bool", attributes = "EnableStand"},
      {title = "SupportWithArms", perAnimSet = true, type = "bool", attributes = "SupportWithArms"},
      {title = "FootBalanceAngle", perAnimSet = true, type = "float", attributes = "FootBalanceAmount"},
      {title = "DecelerationAmount", perAnimSet = true, type = "float", attributes = "DecelerationAmount"},
      {title = "MaxFootAngle", perAnimSet = true, type = "float", attributes = "MaxAngle"},
      {title = "FootLength", perAnimSet = true, type = "float", attributes = "FootLength"},
      {title = "OnFootLiftLowerPelvisBy", perAnimSet = true, type = "float", attributes = "LowerPelvisDistanceWhenFootLifts"},
      {title = "LegStandStrength {Leg0}", perAnimSet = false, type = "float", attributes = "LegStandStrength_0"},
      {title = "LegStandStrength {Leg1}", perAnimSet = false, type = "float", attributes = "LegStandStrength_1"},
      {title = "ArmStandStrength {Arm0}", perAnimSet = false, type = "float", attributes = "ArmStandStrength_0"},
      {title = "ArmStandStrength {Arm1}", perAnimSet = false, type = "float", attributes = "ArmStandStrength_1"},
    },
  },
  {
    title = "Arm Spin",
    helptext = "Settings that govern the tendency to spin arms to regain balance",
    isExpanded = false,
    details = 
    {
      {title = "SpinArmsForBalance", perAnimSet = true, type = "bool", attributes = "AllowArmSpin"},
      {title = "SpinAmount", perAnimSet = true, type = "float", attributes = "SpinAmount"},
      {title = "SpinThreshold", perAnimSet = true, type = "float", attributes = "SpinThreshold"},
    },
  },
  {
    title = "Step",
    helptext = "Settings that govern the characters behaviour when stepping (to maintain balance)",
    isExpanded = false,
    details = 
    {
      {title = "EnableCollisionGroup", perAnimSet = false, type = "bool", attributes = "EnableCollisionGroup"},
      {title = "CollisionGroup", perAnimSet = false, type = "integer", attributes = "CollisionGroupIndex"},
      {title = "StepWithLegs", perAnimSet = true, type = "bool", attributes = "StepWithLegs"},
      {title = "StepWithArms", perAnimSet = true, type = "bool", attributes = "StepWithArms"},
      {title = "DelayBeforeStepping", perAnimSet = true, type = "float", attributes = "SuppressSteppingTime"},
      {title = "ArmSwingStrengthScale", perAnimSet = true, type = "float", attributes = "ArmSwingStrengthScaleWhenStepping"},
      {title = "ResetStepCountAfter", perAnimSet = true, type = "float", attributes = "StepCountResetTime"},
      {title = "SuppressFootCrossing", perAnimSet = true, type = "bool", attributes = "SuppressFootCrossing"},
      {title = "FootCrossingOffset", perAnimSet = true, type = "float", attributes = "FootCrossingOffset"},
      {title = "ForwardStepPredictionTime", perAnimSet = true, type = "float", attributes = "StepPredictionTimeForward"},
      {title = "BackwardStepPredictionTime", perAnimSet = true, type = "float", attributes = "StepPredictionTimeBackward"},
      {title = "StepLiftUpDistance", perAnimSet = true, type = "float", attributes = "StepUpDistance"},
      {title = "StepPlaceDownDistance", perAnimSet = true, type = "float", attributes = "StepDownDistance"},
      {title = "MaxStepLength", perAnimSet = true, type = "float", attributes = "MaxStepLength"},
      {title = "StepDownSpeed", perAnimSet = true, type = "float", attributes = "StepDownSpeed"},
      {title = "FootSeparationFraction", perAnimSet = true, type = "float", attributes = "FootSeparationFraction"},
      {title = "AlignFootToFloorWeight", perAnimSet = true, type = "float", attributes = "AlignFootToFloorWeight"},
      {title = "LowerPelvisAmount", perAnimSet = true, type = "float", attributes = "LowerPelvisAmount"},
      {title = "StrengthScale", perAnimSet = true, type = "float", attributes = "StrengthScale"},
      {title = "DampingRatioScale", perAnimSet = true, type = "float", attributes = "DampingRatioScale"},
      {title = "IKOrientationWeight", perAnimSet = true, type = "float", attributes = "SteppingOrientationWeight"},
      {title = "IKSubSteps", perAnimSet = true, type = "float", attributes = "IKSubStep"},
      {title = "GravityCompensation", perAnimSet = true, type = "float", attributes = "SteppingGravityCompensation"},
      {title = "LimbLengthToAbort", perAnimSet = true, type = "float", attributes = "SteppingLimbLengthToAbort"},
      {title = "DownwardSpeedToAbort", perAnimSet = true, type = "float", attributes = "SteppingRootDownSpeedToAbort"},
      {title = "Toe-firstSteppingThreshold", perAnimSet = true, type = "float", attributes = "SteppingDirectionThreshold"},
      {title = "ImplicitLegStiffness", perAnimSet = true, type = "float", attributes = "SteppingImplicitStiffness"},
    },
  },
  {
    title = "Look",
    helptext = "Settings to make the character look where it is stepping.",
    isExpanded = false,
    details = 
    {
      {title = "EnableLook", perAnimSet = true, type = "bool", attributes = "EnableLook"},
      {title = "StartLookingAfter", perAnimSet = true, type = "integer", attributes = "StepsBeforeLooking"},
      {title = "StopLookingAfter", perAnimSet = true, type = "float", attributes = "StopLookingTime"},
      {title = "MinimumSpeed", perAnimSet = true, type = "float", attributes = "MinSpeedForLooking"},
      {title = "Movement", perAnimSet = true, type = "float", attributes = "WholeBodyLook"},
      {title = "TargetAngle(Down)", perAnimSet = true, type = "float", attributes = "LookDownAngle"},
      {title = "Weight", perAnimSet = true, type = "float", attributes = "LookWeight"},
      {title = "LookInStepDirection", perAnimSet = false, type = "bool", attributes = "LookInStepDirection"},
    },
  },
  {
    title = "Step to Recover",
    helptext = "These settings can ensure that the character will attempt to regain the stand pose when sufficiently displaced from that pose",
    isExpanded = false,
    details = 
    {
      {title = "Enable", perAnimSet = true, type = "bool", attributes = "StepToRecoverPose"},
      {title = "Forward", perAnimSet = true, type = "float", attributes = "FwdDistanceToTriggerStep"},
      {title = "Sideways", perAnimSet = true, type = "float", attributes = "SidewaysDistanceToTriggerStep"},
      {title = "DelayBeforeShiftingWeight", perAnimSet = true, type = "float", attributes = "TimeBeforeShiftingWeight"},
      {title = "WeightShiftLastsFor", perAnimSet = true, type = "float", attributes = "WeightShiftingTime"},
    },
  },
  {
    title = "Pose Clamping",
    helptext = "These settings stabilise the character by restricting and smoothing the target balance pose, which may be modified by other behaviours (for example by BalancePoser).",
    isExpanded = false,
    details = 
    {
      {title = "EnablePoseClamping", perAnimSet = false, type = "bool", attributes = "EnablePoseClamping"},
      {title = "DynamicClamping", perAnimSet = true, type = "bool", attributes = "DynamicClamping"},
      {title = "MinPelvisPitch", perAnimSet = true, type = "float", attributes = "MinPelvisPitch"},
      {title = "MaxPelvisPitch", perAnimSet = true, type = "float", attributes = "MaxPelvisPitch"},
      {title = "MinPelvisRoll", perAnimSet = true, type = "float", attributes = "MinPelvisRoll"},
      {title = "MaxPelvisRoll", perAnimSet = true, type = "float", attributes = "MaxPelvisRoll"},
      {title = "MinPelvisYaw", perAnimSet = true, type = "float", attributes = "MinPelvisYaw"},
      {title = "MaxPelvisYaw", perAnimSet = true, type = "float", attributes = "MaxPelvisYaw"},
      {title = "MinPelvisHeight", perAnimSet = true, type = "float", attributes = "MinPelvisHeight"},
      {title = "MaxPelvisHeight", perAnimSet = true, type = "float", attributes = "MaxPelvisHeight"},
      {title = "PositionTimescale", perAnimSet = true, type = "float", attributes = "PelvisPositionChangeTimescale"},
      {title = "OrientationTimescale", perAnimSet = true, type = "float", attributes = "PelvisOrientationChangeTimescale"},
    },
  },
  {
    title = "Stand Pose",
    helptext = "The pose guide for the limbs when the character is in a stationary balance.",
    isExpanded = false,
    details = 
    {
      {title = "CacheFirstFrame", perAnimSet = true, type = "bool", attributes = "UseSingleFrameForBalancePose"},
      {title = "GravityCompensation", perAnimSet = true, type = "float", attributes = "NonSupportingGravityCompensationScale"},
      {title = "DampingScale", perAnimSet = true, type = "float", attributes = "NonSupportingDampingScale"},
      {title = "DriveCompensation", perAnimSet = true, type = "float", attributes = "NonSupportingDriveCompensationScale"},
      {title = "MatchPoseWhenBalancing", perAnimSet = false, type = "float", attributes = "BalanceWithAnimationPose"},
    },
  },
  {
    title = "Arm Pose During Step",
    helptext = "The pose used to place the arms when the character is stepping. All other parts of the pose are ignored.",
    isExpanded = false,
    details = 
    {
      {title = "CacheFirstFrame", perAnimSet = true, type = "bool", attributes = "UseSingleFrameForReadyPose"},
      {title = "ArmDisplacementTime", perAnimSet = true, type = "float", attributes = "ArmDisplacementTime"},
    },
  },
  {
    title = "Assistance",
    helptext = "Balance assistance provided through external forces/torques",
    isExpanded = false,
    details = 
    {
      {title = "EnableAssistance", perAnimSet = true, type = "bool", attributes = "EnableAssistance"},
      {title = "ReduceAssistanceWhenFalling", perAnimSet = true, type = "bool", attributes = "ReduceAssistanceWhenFalling"},
      {title = "MaxLinearAcceleration", perAnimSet = true, type = "float", attributes = "MaxLinearAccelerationAssistance"},
      {title = "MaxAngularAcceleration", perAnimSet = true, type = "float", attributes = "MaxAngularAccelerationAssistance"},
      {title = "ChestToPelvisRatio", perAnimSet = true, type = "float", attributes = "AssistanceChestToPelvisRatio"},
      {title = "OrientationAssistanceAmount", perAnimSet = false, type = "float", attributes = "OrientationAssistanceAmount"},
      {title = "PositionAssistanceAmount", perAnimSet = false, type = "float", attributes = "PositionAssistanceAmount"},
      {title = "VelocityAssistanceAmount", perAnimSet = false, type = "float", attributes = "VelocityAssistanceAmount"},
      {title = "UseCounterForceOnFeet", perAnimSet = false, type = "bool", attributes = "UseCounterForceOnFeet"},
    },
  },
  {
    title = "Desired Pelvis Direction",
    isExpanded = false,
    details = 
    {
      {title = "IgnoreDirectionWhenOutOfRange", perAnimSet = false, type = "bool", attributes = "IgnoreDirectionWhenOutOfRange"},
      {title = "Yaw right (min)", perAnimSet = false, type = "float", attributes = "TargetYawRight"},
      {title = "Yaw left (max)", perAnimSet = false, type = "float", attributes = "TargetYawLeft"},
      {title = "Pitch down (min)", perAnimSet = false, type = "float", attributes = "TargetPitchDown"},
      {title = "Pitch up (max)", perAnimSet = false, type = "float", attributes = "TargetPitchUp"},
      {title = "TargetPelvisDirection", perAnimSet = false, type = "vector3", attributes = "TargetPelvisDirection"},
    },
  },
  {
    title = "Input Defaults",
    isExpanded = false,
    details = 
    {
      {title = "Desired velocity", perAnimSet = false, type = "vector3", attributes = "TargetVelocity"},
      {title = "TargetVelocityInCharacterSpace", perAnimSet = false, type = "bool", attributes = "TargetVelocityInCharacterSpace"},
      {title = "BalanceFwdOffset", perAnimSet = false, type = "float", attributes = "BalanceFwdOffset"},
      {title = "BalanceRightOffset", perAnimSet = false, type = "float", attributes = "BalanceRightOffset"},
      {title = "LegStepStrength {Leg0}", perAnimSet = false, type = "float", attributes = "LegStepStrength_0"},
      {title = "LegStepStrength {Leg1}", perAnimSet = false, type = "float", attributes = "LegStepStrength_1"},
      {title = "ArmStepStrength {Arm0}", perAnimSet = false, type = "float", attributes = "ArmStepStrength_0"},
      {title = "ArmStepStrength {Arm1}", perAnimSet = false, type = "float", attributes = "ArmStepStrength_1"},
    },
  },
  {
    title = "Exclusion Zone",
    helptext = "The exclusion zone allows you to specify a region into which the charatcer will not step. This can be used to prevent him stepping off the edge of a cliff, for example.",
    isExpanded = false,
    details = 
    {
      {title = "Point", perAnimSet = false, type = "vector3", attributes = "ExclusionZonePoint"},
      {title = "Direction", perAnimSet = false, type = "vector3", attributes = "ExclusionZoneDirection"},
    },
  },
  {
    title = "Stand Pose Weights",
    helptext = "Target pose when balancing",
    isExpanded = false,
    details = 
    {
      {title = "Arm {Arm0}", perAnimSet = false, type = "float", attributes = "BalancePose_ApplyToArm_1"},
      {title = "Arm {Arm1}", perAnimSet = false, type = "float", attributes = "BalancePose_ApplyToArm_2"},
      {title = "Head {Head0}", perAnimSet = false, type = "float", attributes = "BalancePose_ApplyToHead_1"},
      {title = "Leg {Leg0}", perAnimSet = false, type = "float", attributes = "BalancePose_ApplyToLeg_1"},
      {title = "Leg {Leg1}", perAnimSet = false, type = "float", attributes = "BalancePose_ApplyToLeg_2"},
      {title = "Spine {Spine0}", perAnimSet = false, type = "float", attributes = "BalancePose_ApplyToSpine_1"},
    },
  },
  {
    title = "Step Pose Weights",
    helptext = "Pose used for the arms when stepping",
    isExpanded = false,
    details = 
    {
      {title = "Arm {Arm0}", perAnimSet = false, type = "float", attributes = "ReadyPose_ApplyToArm_1"},
      {title = "Arm {Arm1}", perAnimSet = false, type = "float", attributes = "ReadyPose_ApplyToArm_2"},
      {title = "Head {Head0}", perAnimSet = false, type = "float", attributes = "ReadyPose_ApplyToHead_1"},
      {title = "Leg {Leg0}", perAnimSet = false, type = "float", attributes = "ReadyPose_ApplyToLeg_1"},
      {title = "Leg {Leg1}", perAnimSet = false, type = "float", attributes = "ReadyPose_ApplyToLeg_2"},
      {title = "Spine {Spine0}", perAnimSet = false, type = "float", attributes = "ReadyPose_ApplyToSpine_1"},
    },
  },
  {
    title = "Emitted Messages",
    requests = 
    {
      {
        name = "HasFallen", 
        helptext = "Message emmited when the character has fallen",
      },
      {
        name = "HasRecoveredBalance", 
        helptext = "Message emmited when the character has recovered its balance",
      },
    },
    details = 
    {
      {title = "SettleTimeBeforeSending", perAnimSet = true, type = "float", attributes = "FallenTimeToEmitRequests"},
    },
  },
}

return {
  data = behaviourNode,
  attributeGroups = attributeGroups,
}
