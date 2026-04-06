--------------------------------------------------------------------------------------------------
--                                 This file is auto-generated                                  --
--------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local behaviourNode = 
{
  displayName = "Aim",
  version = 12,
  topology =
  {
    networkMaxNumArms = 2,
    networkMaxNumLegs = 2,
    networkMaxNumHeads = 1,
  },
  helptext = "Behaviour that drives character to aim a hand gun at a worldspace target.",
  group = "Behaviours",
  image = "AimBehaviour.png",
  id = generateNamespacedId(idNamespaces.NaturalMotion, 128),

  functionPins = 
  {
    ["Pose"] = 
    {
      input = true,
      mode = "Required",
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
    ["ArmStrength_0"] = 
    {
      input = true,
      helptext = "Weights this behaviour's influence over the arm against that of other behaviours. Lower values will be weaker if no other arm control active",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "ArmStrength {Arm0}",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

    ["ArmStrength_1"] = 
    {
      input = true,
      helptext = "Weights this behaviour's influence over the arm against that of other behaviours. Lower values will be weaker if no other arm control active",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "ArmStrength {Arm1}",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

    ["HeadStrength_0"] = 
    {
      input = true,
      helptext = "Weights this behaviour's influence over the head against that of other behaviours. Lower values will be weaker if no other head control active",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "HeadStrength {Head0}",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

    ["SpineStrength_0"] = 
    {
      input = true,
      helptext = "Weights this behaviour's influence over the spine against that of other behaviours. Lower values will be weaker if no other spine control active",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "SpineStrength",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

    ["ArmDampingScale_0"] = 
    {
      input = true,
      helptext = "Damping scale for the arm.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "ArmDampingScale {Arm0}",
      value = 1.200000,
      min = 0.000000,
      max = 2.000000,
    },

    ["ArmDampingScale_1"] = 
    {
      input = true,
      helptext = "Damping scale for the arm.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "ArmDampingScale {Arm1}",
      value = 1.200000,
      min = 0.000000,
      max = 2.000000,
    },

    ["HeadDampingScale_0"] = 
    {
      input = true,
      helptext = "Damping scale for the head.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "HeadDampingScale {Head0}",
      value = 1.200000,
      min = 0.000000,
      max = 2.000000,
    },

    ["SpineDampingScale_0"] = 
    {
      input = true,
      helptext = "Damping scale for the spine.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "SpineDampingScale",
      value = 1.200000,
      min = 0.000000,
      max = 2.000000,
    },

    ["ArmRootRotationCompensation_0"] = 
    {
      input = true,
      helptext = "How much to adjust the arm target position to compensate for rotation of the chest.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "ArmRootRotationCompensation {Arm0}",
      value = 0.500000,
      min = 0.000000,
      max = 2.000000,
    },

    ["ArmRootRotationCompensation_1"] = 
    {
      input = true,
      helptext = "How much to adjust the arm target position to compensate for rotation of the chest.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "ArmRootRotationCompensation {Arm1}",
      value = 0.500000,
      min = 0.000000,
      max = 2.000000,
    },

    ["HeadRootRotationCompensation_0"] = 
    {
      input = true,
      helptext = "How much to adjust the head target position to compensate for rotation of the chest.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "HeadRootRotationCompensation {Head0}",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

    ["SpineRootRotationCompensation_0"] = 
    {
      input = true,
      helptext = "How much to adjust the chest target position to compensate for rotation of the pelvis.",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "SpineRootRotationCompensation",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
    },

    ["ArmSwivel_0"] = 
    {
      input = true,
      helptext = "Positive is elbows out and up, negative is elbows in and down",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "ArmSwivel {Arm0}",
      value = 0.000000,
      min = -1.000000,
      max = 1.000000,
    },

    ["ArmSwivel_1"] = 
    {
      input = true,
      helptext = "Positive is elbows out and up, negative is elbows in and down",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "ArmSwivel {Arm1}",
      value = 0.000000,
      min = -1.000000,
      max = 1.000000,
    },

    ["ArmIKSubStep_0"] = 
    {
      input = true,
      helptext = "IK sub step (values < 1 make a moving arm follow a straighter line rather than an arc).",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "ArmIKSubStep {Arm0}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["ArmIKSubStep_1"] = 
    {
      input = true,
      helptext = "IK sub step (values < 1 make a moving arm follow a straighter line rather than an arc).",
      type = "float",
      mode = "HiddenInNetworkEditor",
      displayName = "ArmIKSubStep {Arm1}",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },

    ["BarrelDirection"] = 
    {
      input = true,
      helptext = "The direction of the gun barrel (or whatever is being pointed) in the space of the aiming hand.",
      type = "vector3",
      mode = "HiddenInNetworkEditor",
      displayName = "BarrelDirection",
      value = {x=0.000000, y=1.000000,z=0.000000},
    },

    ["TargetPositionInWorldSpace"] = 
    {
      input = true,
      helptext = "Current target position in world space.",
      type = "vector3",
      displayName = "Target Position",
    },

    ["AllowConstraint"] = 
    {
      input = true,
      helptext = "Enable configured constraint between a non aiming effector and the aiming effector. This can be used to dynamically enable/disable the supporting arm constraint.",
      type = "bool",
      mode = "HiddenInNetworkEditor",
      displayName = "AllowConstraint",
      value = true,
    },

    ["targetPitch"] = 
    {
      input = false,
      helptext = "Rotation from target position for pose to actual target position relative to chest perpendicular to spine direction.",
      type = "float",
    },

    ["targetYaw"] = 
    {
      input = false,
      helptext = "Rotation from target position for pose to actual target position relative to chest about spine direction.",
      type = "float",
    },

    ["handSeparation"] = 
    {
      input = false,
      helptext = "How far is the supporting hand from the aiming hand.",
      type = "float",
    },

  },

  pinOrder = 
  {
    "Pose",
    "ArmStrength_0",
    "ArmStrength_1",
    "HeadStrength_0",
    "SpineStrength_0",
    "ArmDampingScale_0",
    "ArmDampingScale_1",
    "HeadDampingScale_0",
    "SpineDampingScale_0",
    "ArmRootRotationCompensation_0",
    "ArmRootRotationCompensation_1",
    "HeadRootRotationCompensation_0",
    "SpineRootRotationCompensation_0",
    "ArmSwivel_0",
    "ArmSwivel_1",
    "ArmIKSubStep_0",
    "ArmIKSubStep_1",
    "BarrelDirection",
    "TargetPositionInWorldSpace",
    "AllowConstraint",
    "Result",
    "targetPitch",
    "targetYaw",
    "handSeparation",
  },

  attributes = 
  {
    {
      name = "UseSingleFrameForPose",
      type = "bool",
      value = false,
      perAnimSet = true,
      helptext = "If set then pose will be cached on the first update. This means that the pose that is used will not change if the input changes.",
    },
    {
      name = "Pose_ApplyToArm_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Arm.",
    },
    {
      name = "Pose_ApplyToArm_2",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Arm.",
    },
    {
      name = "Pose_ApplyToHead_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Head.",
    },
    {
      name = "Pose_ApplyToLeg_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Leg.",
    },
    {
      name = "Pose_ApplyToLeg_2",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Leg.",
    },
    {
      name = "Pose_ApplyToSpine_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
      helptext = "Set how much of the pose to use on this Spine.",
    },
    {
      name = "AimingLimbIndex",
      type = "int",
      value = 0,
      min = 0,
      max = 32,
      helptext = "Index of the arm that the character is aiming with (i.e. the arm holding the gun etc).",
    },
    {
      name = "EnableSupportingArm",
      type = "bool",
      value = true,
      helptext = "Turn on or off the ability to have an arm that supports the aiming arm",
    },
    {
      name = "SupportingLimbIndex",
      type = "int",
      value = 1,
      min = 0,
      max = 32,
      helptext = "Select an arm whose hand will always be positioned relative to the aiming hand. This is useful for placing a hand on a rifle stock or representing a two handed grip on a pistol etc.",
    },
    {
      name = "ShouldDisableHandsOnSeparationInPose",
      type = "bool",
      value = false,
      helptext = "Stop controlling the supporting hand if it is more than a threshold distance from the aiming hand in the input pose.",
    },
    {
      name = "DisableHandsSeparationInPoseMax",
      type = "float",
      value = 0.100000,
      min = 0.000000,
      max = 2.000000,
      helptext = "Stop controlling the supporting hand if it is more than this distance from the aiming hand in the input pose. In metres (standard character).",
    },
    {
      name = "SupportingArmTwist",
      type = "float",
      value = -115.000000,
      min = -180.000000,
      max = 180.000000,
      helptext = "The ideal twist in the supporting arm. This is used to determine when the arm is twisted in an uncomfortable way. The behaviour will atempt to unwind the arm in this case. In degrees.",
    },
    {
      name = "EnableConstraint",
      type = "bool",
      value = true,
      helptext = "Create a physical constraint between the supporting hand and the aiming hand. Constraint tries to maintain the relative transforms of the end parts in the input pose. ",
    },
    {
      name = "EnableOrientationConstraint",
      type = "bool",
      value = false,
      helptext = "Constrain the orientation of the supporting hand as well as the position.",
    },
    {
      name = "ConstraintAccelerationLimit",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 10000.000000,
      helptext = "Maximum acceleration that the constraint can apply, scaled by the one over the separation between the aiming and constrained hands. In metres per second per second (standard character).",
    },
    {
      name = "ShouldDisableConstraintOnSeparationInPose",
      type = "bool",
      value = true,
      helptext = "Deactivate the constraint if the constrained hand is more than a threshold distance from the aiming hand in the input pose.",
    },
    {
      name = "DisableConstraintOnSeparationInPoseMax",
      type = "float",
      value = 0.200000,
      min = 0.000000,
      max = 2.000000,
      helptext = "Deactivate the constraint if the constrained hand is more than this distance from the aiming hand in the input pose. In metres (standard character).",
    },
    {
      name = "ShouldDisableConstraintOnSeparationInRig",
      type = "bool",
      value = true,
      helptext = "Deactivate the constraint if the constrained hand is more than a threshold distance from the aiming hand in the current rig configuration.",
    },
    {
      name = "DisableConstraintOnSeparationInRigMax",
      type = "float",
      value = 0.200000,
      min = 0.000000,
      max = 2.000000,
      helptext = "Deactivate the constraint if the constrained hand is more than this distance from the aiming hand in the current rig configuration. In metres (standard character).",
    },
    {
      name = "AimingLimbInertiaScale",
      type = "float",
      value = 10.000000,
      min = 0.000000,
      max = 100.000000,
      helptext = "Scale the inertia of the aiming hand to reduce any movement caused by the constraint.",
    },
    {
      name = "DisableWhenLyingOnGround",
      type = "bool",
      value = false,
      helptext = "Don't Aim if the character is lying on the ground in any orientation.",
    },
    {
      name = "DisableWhenLyingOnFront",
      type = "bool",
      value = false,
      helptext = "Don't Aim if the character is lying with its front facing the ground.",
    },
    {
      name = "DisableWhenTargetOutsideRange",
      type = "bool",
      value = false,
      helptext = "Don't Aim if the direction from the pelvis to the target is outside of this range. The center of the range points in the pelvis forward direction. Pitch and Yaw are in degrees.",
    },
    {
      name = "TargetYawRight",
      type = "float",
      value = -90.000000,
      min = -180.000000,
      max = 180.000000,
      helptext = "Don't aim if target yaw is outside of this range. The centre of the range points in the pelvis forward direction and the yaw value increases from the character's right to its left. In degrees.",
    },
    {
      name = "TargetYawLeft",
      type = "float",
      value = 90.000000,
      min = -180.000000,
      max = 180.000000,
      helptext = "Don't aim if target yaw is outside of this range. The centre of the range points in the pelvis forward direction and the yaw value increases from the character's right to its left. In degrees.",
    },
    {
      name = "TargetPitchDown",
      type = "float",
      value = -90.000000,
      min = -180.000000,
      max = 180.000000,
      helptext = "Don't aim if target pitch is outside of this range. The centre of the range points in the pelvis forward direction and the pitch value increases from character down to up. In degrees.",
    },
    {
      name = "TargetPitchUp",
      type = "float",
      value = 90.000000,
      min = -180.000000,
      max = 180.000000,
      helptext = "Don't aim if target pitch is outside of this range. The centre of the range points in the pelvis forward direction and the pitch value increases from character down to up. In degrees.",
    },
    {
      name = "TwistBodyAmount",
      type = "float",
      value = 0.750000,
      min = 0.000000,
      max = 1.000000,
    },
    {
      name = "SwingBodyAmount",
      type = "float",
      value = 0.000000,
      min = 0.000000,
      max = 1.000000,
    },
    {
      name = "TwistHeadAmount",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },
    {
      name = "SwingHeadAmount",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 1.000000,
    },
    {
      name = "ArmGravityCompensation_0",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
      helptext = "Gravity compensation scale for the arm.",
    },
    {
      name = "ArmGravityCompensation_1",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
      helptext = "Gravity compensation scale for the arm.",
    },
    {
      name = "HeadGravityCompensation_0",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
      helptext = "Gravity compensation scale for the head.",
    },
    {
      name = "SpineGravityCompensation_0",
      type = "float",
      value = 1.000000,
      min = 0.000000,
      max = 2.000000,
      helptext = "Gravity compensation scale for the spine.",
    },
  },

  -------------------------------------------------------------------

  serialize = function(node, stream) 
    stream:writeInt(1, "BehaviourID")


    local animSets = listAnimSets()

    -------------------------------------------------------------------

    stream:writeUInt(12, "numBehaviourInts")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "UseSingleFrameForPose", asVal) and 1 ) or 0 ) , string.format("Int_0_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt(getValue(node, "AimingLimbIndex"), string.format("Int_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "EnableSupportingArm") and 1 ) or 0 ) , string.format("Int_2_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt(getValue(node, "SupportingLimbIndex"), string.format("Int_3_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "ShouldDisableHandsOnSeparationInPose") and 1 ) or 0 ) , string.format("Int_4_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "EnableConstraint") and 1 ) or 0 ) , string.format("Int_5_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "EnableOrientationConstraint") and 1 ) or 0 ) , string.format("Int_6_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "ShouldDisableConstraintOnSeparationInPose") and 1 ) or 0 ) , string.format("Int_7_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "ShouldDisableConstraintOnSeparationInRig") and 1 ) or 0 ) , string.format("Int_8_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "DisableWhenLyingOnGround") and 1 ) or 0 ) , string.format("Int_9_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "DisableWhenLyingOnFront") and 1 ) or 0 ) , string.format("Int_10_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeInt( ( ( getValue(node, "DisableWhenTargetOutsideRange") and 1 ) or 0 ) , string.format("Int_11_%d", asIdx-1))
    end

    stream:writeUInt(24, "numBehaviourFloats")
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "Pose_ApplyToArm_1"), string.format("Float_0_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "Pose_ApplyToArm_2"), string.format("Float_1_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "Pose_ApplyToHead_1"), string.format("Float_2_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "Pose_ApplyToLeg_1"), string.format("Float_3_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "Pose_ApplyToLeg_2"), string.format("Float_4_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "Pose_ApplyToSpine_1"), string.format("Float_5_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "DisableHandsSeparationInPoseMax"), string.format("Float_6_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "SupportingArmTwist"), string.format("Float_7_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ConstraintAccelerationLimit"), string.format("Float_8_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "DisableConstraintOnSeparationInPoseMax"), string.format("Float_9_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "DisableConstraintOnSeparationInRigMax"), string.format("Float_10_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "AimingLimbInertiaScale"), string.format("Float_11_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "TargetYawRight"), string.format("Float_12_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "TargetYawLeft"), string.format("Float_13_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "TargetPitchDown"), string.format("Float_14_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "TargetPitchUp"), string.format("Float_15_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "TwistBodyAmount"), string.format("Float_16_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "SwingBodyAmount"), string.format("Float_17_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "TwistHeadAmount"), string.format("Float_18_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "SwingHeadAmount"), string.format("Float_19_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "ArmGravityCompensation_0"), string.format("Float_20_%d", asIdx-1))
      stream:writeFloat(getValue(node, "ArmGravityCompensation_1"), string.format("Float_21_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "HeadGravityCompensation_0"), string.format("Float_22_%d", asIdx-1))
    end
    for asIdx, asVal in ipairs(animSets) do
      stream:writeFloat(getValue(node, "SpineGravityCompensation_0"), string.format("Float_23_%d", asIdx-1))
    end

    stream:writeUInt(0, "numBehaviourVector3s")

    stream:writeUInt(0, "numBehaviourUInt64s")

    -------------------------------------------------------------------

    local inputNodeID_Pose = -1
    if isConnected{SourcePin = node .. ".Pose", ResolveReferences = true} then
      inputNodeID_Pose = getConnectedNodeID(node, "Pose")
    end

    stream:writeUInt(1, "numBehaviourNodeAnimationInputs")

    stream:writeNetworkNodeId(inputNodeID_Pose, "BehaviourNodeAnimationInput_0")

    -------------------------------------------------------------------

    stream:writeUInt(1, "numInputCPInts")
    stream:writeUInt(16, "numInputCPFloats")
    stream:writeUInt(0, "numInputCPUInt64s")
    stream:writeUInt(2, "numInputCPVector3s")
    stream:writeUInt(19, "numBehaviourControlParameterInputs")

    writeDataPinAltName(stream, node, "ArmStrength_0", "BehaviourControlParameterInputID_0" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_0")
    writeDataPinAltName(stream, node, "ArmStrength_1", "BehaviourControlParameterInputID_1" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_1")

    writeDataPinAltName(stream, node, "HeadStrength_0", "BehaviourControlParameterInputID_2" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_2")

    writeDataPinAltName(stream, node, "SpineStrength_0", "BehaviourControlParameterInputID_3" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_3")

    writeDataPinAltName(stream, node, "ArmDampingScale_0", "BehaviourControlParameterInputID_4" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_4")
    writeDataPinAltName(stream, node, "ArmDampingScale_1", "BehaviourControlParameterInputID_5" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_5")

    writeDataPinAltName(stream, node, "HeadDampingScale_0", "BehaviourControlParameterInputID_6" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_6")

    writeDataPinAltName(stream, node, "SpineDampingScale_0", "BehaviourControlParameterInputID_7" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_7")

    writeDataPinAltName(stream, node, "ArmRootRotationCompensation_0", "BehaviourControlParameterInputID_8" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_8")
    writeDataPinAltName(stream, node, "ArmRootRotationCompensation_1", "BehaviourControlParameterInputID_9" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_9")

    writeDataPinAltName(stream, node, "HeadRootRotationCompensation_0", "BehaviourControlParameterInputID_10" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_10")

    writeDataPinAltName(stream, node, "SpineRootRotationCompensation_0", "BehaviourControlParameterInputID_11" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_11")

    writeDataPinAltName(stream, node, "ArmSwivel_0", "BehaviourControlParameterInputID_12" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_12")
    writeDataPinAltName(stream, node, "ArmSwivel_1", "BehaviourControlParameterInputID_13" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_13")

    writeDataPinAltName(stream, node, "ArmIKSubStep_0", "BehaviourControlParameterInputID_14" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_14")
    writeDataPinAltName(stream, node, "ArmIKSubStep_1", "BehaviourControlParameterInputID_15" )
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterInputType_15")

    writeDataPinAltName(stream, node, "BarrelDirection", "BehaviourControlParameterInputID_16")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_16")

    writeDataPinAltName(stream, node, "TargetPositionInWorldSpace", "BehaviourControlParameterInputID_17")
    stream:writeString("ATTRIB_SEMANTIC_CP_VECTOR3", "BehaviourControlParameterInputType_17")

    writeDataPinAltName(stream, node, "AllowConstraint", "BehaviourControlParameterInputID_18")
    stream:writeString("ATTRIB_SEMANTIC_CP_BOOL", "BehaviourControlParameterInputType_18")

    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterOutputType_0")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterOutputType_1")
    stream:writeString("ATTRIB_SEMANTIC_CP_FLOAT", "BehaviourControlParameterOutputType_2")
    stream:writeUInt(3, "numBehaviourControlParameterOutputs")

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
    title = "Pose",
    isExpanded = true,
    details = 
    {
      {title = "CacheFirstFrame", perAnimSet = true, type = "bool", attributes = "UseSingleFrameForPose"},
      {title = "Arm {Arm0}", perAnimSet = false, type = "float", attributes = "Pose_ApplyToArm_1"},
      {title = "Arm {Arm1}", perAnimSet = false, type = "float", attributes = "Pose_ApplyToArm_2"},
      {title = "Head {Head0}", perAnimSet = false, type = "float", attributes = "Pose_ApplyToHead_1"},
      {title = "Leg {Leg0}", perAnimSet = false, type = "float", attributes = "Pose_ApplyToLeg_1"},
      {title = "Leg {Leg1}", perAnimSet = false, type = "float", attributes = "Pose_ApplyToLeg_2"},
      {title = "Spine {Spine0}", perAnimSet = false, type = "float", attributes = "Pose_ApplyToSpine_1"},
    },
  },
  {
    title = "Aiming Limb",
    helptext = "Configure the arm that the character is aiming with.",
    isExpanded = false,
    details = 
    {
      {title = "AimingArm", perAnimSet = false, type = "integer", attributes = "AimingLimbIndex"},
      {title = "BarrelDirection", perAnimSet = false, type = "vector3", attributes = "BarrelDirection"},
    },
  },
  {
    title = "Supporting Arm Behaviour",
    helptext = "Enables special treatment of one of the character's arms to make it look more like it is supporting the aiming arm. This includes calculating its IK target relative to the aiming arm, potentially enabling a physical constraint, controlling the path taken by the hand as it moves toward the aiming hand etc.",
    isExpanded = false,
    details = 
    {
      {title = "EnableSupportingArm", perAnimSet = false, type = "bool", attributes = "EnableSupportingArm"},
      {title = "SupportingArm", perAnimSet = false, type = "integer", attributes = "SupportingLimbIndex"},
      {title = "Disable control on distance from aiming hand", perAnimSet = false, type = "bool", attributes = "ShouldDisableHandsOnSeparationInPose"},
      {title = "Disable when separation in pose >", perAnimSet = false, type = "float", attributes = "DisableHandsSeparationInPoseMax"},
      {title = "Desired Twist", perAnimSet = false, type = "float", attributes = "SupportingArmTwist"},
      {title = "EnableConstraint", perAnimSet = false, type = "bool", attributes = "EnableConstraint"},
      {title = "ConstrainOrientation", perAnimSet = false, type = "bool", attributes = "EnableOrientationConstraint"},
      {title = "AccelerationLimit", perAnimSet = false, type = "float", attributes = "ConstraintAccelerationLimit"},
      {title = "Disable constraint on distance from aiming hand in pose", perAnimSet = false, type = "bool", attributes = "ShouldDisableConstraintOnSeparationInPose"},
      {title = "Disable when separation in pose >", perAnimSet = false, type = "float", attributes = "DisableConstraintOnSeparationInPoseMax"},
      {title = "Disable constraint on distance from aiming hand on rig", perAnimSet = false, type = "bool", attributes = "ShouldDisableConstraintOnSeparationInRig"},
      {title = "Disable when separation on rig >", perAnimSet = false, type = "float", attributes = "DisableConstraintOnSeparationInRigMax"},
      {title = "InertiaScale", perAnimSet = false, type = "float", attributes = "AimingLimbInertiaScale"},
    },
  },
  {
    title = "Deactivation Conditions",
    helptext = "When to switch off the aiming behaviour.",
    isExpanded = false,
    details = 
    {
      {title = "LyingOnGround", perAnimSet = false, type = "bool", attributes = "DisableWhenLyingOnGround"},
      {title = "LyingOnFront", perAnimSet = false, type = "bool", attributes = "DisableWhenLyingOnFront"},
      {title = "Target Outside Range", perAnimSet = false, type = "bool", attributes = "DisableWhenTargetOutsideRange"},
      {title = "Yaw right (min)", perAnimSet = false, type = "float", attributes = "TargetYawRight"},
      {title = "Yaw left (max)", perAnimSet = false, type = "float", attributes = "TargetYawLeft"},
      {title = "Pitch down (min)", perAnimSet = false, type = "float", attributes = "TargetPitchDown"},
      {title = "Pitch up (max)", perAnimSet = false, type = "float", attributes = "TargetPitchUp"},
    },
  },
  {
    title = "Pose Modification Weights",
    helptext = "How much should the aim behaviour change the input pose for each limb.",
    isExpanded = false,
    details = 
    {
      {title = "TwistBodyAmount", perAnimSet = false, type = "float", attributes = "TwistBodyAmount"},
      {title = "SwingBodyAmount", perAnimSet = false, type = "float", attributes = "SwingBodyAmount"},
      {title = "TwistHeadAmount", perAnimSet = false, type = "float", attributes = "TwistHeadAmount"},
      {title = "SwingHeadAmount", perAnimSet = false, type = "float", attributes = "SwingHeadAmount"},
    },
  },
  {
    title = "Gravity Compensation",
    helptext = "Set the amount of gravity compensation applied to each limb. Low values will allow the limbs to drop under the influence of gravity, which may make the character look tierd or weak. Values higher than one may be needed to get the appearence of full gravity compensation on otherwise weak limbs.",
    isExpanded = false,
    details = 
    {
      {title = "Arm {Arm0}", perAnimSet = false, type = "float", attributes = "ArmGravityCompensation_0"},
      {title = "Arm {Arm1}", perAnimSet = false, type = "float", attributes = "ArmGravityCompensation_1"},
      {title = "Head {Head0}", perAnimSet = false, type = "float", attributes = "HeadGravityCompensation_0"},
      {title = "Spine ", perAnimSet = false, type = "float", attributes = "SpineGravityCompensation_0"},
    },
  },
  {
    title = "Control",
    helptext = "Set the strength and damping scale applied to each limb.",
    isExpanded = false,
    details = 
    {
      {title = "ArmStrength {Arm0}", perAnimSet = false, type = "float", attributes = "ArmStrength_0"},
      {title = "ArmStrength {Arm1}", perAnimSet = false, type = "float", attributes = "ArmStrength_1"},
      {title = "HeadStrength {Head0}", perAnimSet = false, type = "float", attributes = "HeadStrength_0"},
      {title = "SpineStrength ", perAnimSet = false, type = "float", attributes = "SpineStrength_0"},
      {title = "ArmDampingScale {Arm0}", perAnimSet = false, type = "float", attributes = "ArmDampingScale_0"},
      {title = "ArmDampingScale {Arm1}", perAnimSet = false, type = "float", attributes = "ArmDampingScale_1"},
      {title = "HeadDampingScale {Head0}", perAnimSet = false, type = "float", attributes = "HeadDampingScale_0"},
      {title = "SpineDampingScale ", perAnimSet = false, type = "float", attributes = "SpineDampingScale_0"},
      {title = "ArmRootRotationCompensation {Arm0}", perAnimSet = false, type = "float", attributes = "ArmRootRotationCompensation_0"},
      {title = "ArmRootRotationCompensation {Arm1}", perAnimSet = false, type = "float", attributes = "ArmRootRotationCompensation_1"},
      {title = "HeadRootRotationCompensation {Head0}", perAnimSet = false, type = "float", attributes = "HeadRootRotationCompensation_0"},
      {title = "SpineRootRotationCompensation ", perAnimSet = false, type = "float", attributes = "SpineRootRotationCompensation_0"},
    },
  },
  {
    title = "Arm IK",
    helptext = "Set the IK swivel parameters for each arm.",
    isExpanded = false,
    details = 
    {
      {title = "ArmSwivel {Arm0}", perAnimSet = false, type = "float", attributes = "ArmSwivel_0"},
      {title = "ArmSwivel {Arm1}", perAnimSet = false, type = "float", attributes = "ArmSwivel_1"},
      {title = "ArmIKSubStep {Arm0}", perAnimSet = false, type = "float", attributes = "ArmIKSubStep_0"},
      {title = "ArmIKSubStep {Arm1}", perAnimSet = false, type = "float", attributes = "ArmIKSubStep_1"},
    },
  },
  {
    title = "Input Defaults",
    isExpanded = false,
    details = 
    {
      {title = "Target Position", perAnimSet = false, type = "vector3", attributes = "TargetPositionInWorldSpace"},
      {title = "AllowConstraint", perAnimSet = false, type = "bool", attributes = "AllowConstraint"},
    },
  },
}

return {
  data = behaviourNode,
  attributeGroups = attributeGroups,
}
