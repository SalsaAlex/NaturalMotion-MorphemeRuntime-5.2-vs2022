// Not to be copied, adapted, modified, used, distributed, sold,
// licensed or commercially exploited in any manner without the
// written consent of NaturalMotion.  
//
// All non public elements of this software are the confidential
// information of NaturalMotion and may not be disclosed to any
// person nor used for any purpose not expressly approved by
// NaturalMotion in writing.

//----------------------------------------------------------------------------------------------------------------------
#define  _CRT_SECURE_NO_WARNINGS // for strncpy

#include "NMPlatform/NMPlatform.h"
#include "morpheme/mrRig.h"
#include "morpheme/mrBlendOps.h"
#include "mrJoltPhys.h"
#include "mrPhysicsDriverDataJoltPhys.h"
#include "mrPhysicsRigJoltPhysRagdoll.h"
#include "physics/mrPhysicsRigDef.h"
#include "physics/mrPhysicsAttribData.h"
#include "mrPhysicsSceneJoltPhys.h"
#include "physics/mrPhysicsSerialisationBuffer.h"
#include "morpheme/mrAttribData.h"

#include "NMPlatform/NMProfiler.h"

#include "NMPlatform/NMvpu.h"

#include "morpheme/mrDebugMacros.h"

//----------------------------------------------------------------------------------------------------------------------
#define MINIMUM_COMPLIANCE 0.001f

// Sanity checks on passing strength/damping to physx
#define MAX_STRENGTH 1e12f
#define MAX_DAMPING 1e25f

namespace MR 
{

// This limit isn't nice, but PhysX is very jittery with tiny ranges, and currently we don't
// have a hinge joint type in PhysX. See MORPH-11273
static const float s_minSwingLimit = NMP::degreesToRadians(3.0f);


RagdollExplosionHandler* PhysicsRigJoltPhysRagdoll::s_explosionHandler = 0;

//----------------------------------------------------------------------------------------------------------------------
NMP::Memory::Format PhysicsRigJoltPhysRagdoll::getMemoryRequirements(PhysicsRigDef *physicsRigDef)
{
  uint32_t numBones = physicsRigDef->getNumParts();
  uint32_t numJoints = physicsRigDef->getNumJoints();
  uint32_t numMaterials = physicsRigDef->getNumMaterials();

  NMP::Memory::Format result(sizeof(PhysicsRigJoltPhysRagdoll), NMP_VECTOR_ALIGNMENT);

  // Space for the part pointers
  result += NMP::Memory::Format(sizeof(PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll*) * numBones, NMP_NATURAL_TYPE_ALIGNMENT);

  // Space for the joint pointers
  result += NMP::Memory::Format(sizeof(PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll*) * numJoints, NMP_NATURAL_TYPE_ALIGNMENT);

  // Space for the parts
  result += NMP::Memory::Format(
    NMP::Memory::align(sizeof(PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll), NMP_NATURAL_TYPE_ALIGNMENT) * numBones,
    NMP_NATURAL_TYPE_ALIGNMENT);

  // Space for the joints
  result += NMP::Memory::Format(
    NMP::Memory::align(sizeof(PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll), NMP_NATURAL_TYPE_ALIGNMENT) * numJoints,
    NMP_NATURAL_TYPE_ALIGNMENT);

  return result;
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysRagdoll*PhysicsRigJoltPhysRagdoll::init(
  NMP::Memory::Resource &resource, 
  PhysicsRigDef         *physicsRigDef, 
  PhysicsScene          *physicsScene, 
  AnimRigDef            *animRigDef,
  AnimToPhysicsMap      *animToPhysicsMap,
  int32_t                collisionTypeMask,
  int32_t                collisionIgnoreMask)
{
  PhysicsSceneJoltPhys* joltphys_scene = (PhysicsSceneJoltPhys*)physicsScene;

  PhysicsRigJoltPhysRagdoll*result = (PhysicsRigJoltPhysRagdoll*)resource.ptr;
  resource.increment(sizeof(PhysicsRigJoltPhysRagdoll));

  new (result) PhysicsRigJoltPhysRagdoll((PhysicsSceneJoltPhys*) physicsScene);
  PhysicsRigJoltPhys::init(result);
  result->m_cachedSleepThreshold = 0.0f;

  uint32_t numParts = physicsRigDef->getNumParts();
  uint32_t numJoints = physicsRigDef->getNumJoints();

  // Part pointers
  resource.align(NMP::Memory::Format(sizeof(PhysicsRig::Part*) * numParts, NMP_NATURAL_TYPE_ALIGNMENT));
  result->m_parts = (PhysicsRig::Part **)resource.ptr;
  resource.increment(NMP::Memory::Format(sizeof(PhysicsRig::Part*) * numParts, NMP_NATURAL_TYPE_ALIGNMENT));

  // Joint pointers
  resource.align(NMP::Memory::Format(sizeof(PhysicsRig::Joint*) * numJoints, NMP_NATURAL_TYPE_ALIGNMENT));
  result->m_joints = (PhysicsRig::Joint **)resource.ptr;
  resource.increment(NMP::Memory::Format(sizeof(PhysicsRig::Joint*) * numJoints, NMP_NATURAL_TYPE_ALIGNMENT));

  // Parts
  for (uint32_t i = 0; i < numParts; i++)
  {
    resource.align(NMP::Memory::Format(sizeof(PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll), NMP_NATURAL_TYPE_ALIGNMENT));
    result->m_parts[i] = (PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll*)resource.ptr;
    resource.increment(NMP::Memory::Format(sizeof(PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll), NMP_NATURAL_TYPE_ALIGNMENT));
  }

  // Joints
  for (uint32_t i = 0; i < numJoints; i++)
  {
    resource.align(NMP::Memory::Format(sizeof(PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll), NMP_NATURAL_TYPE_ALIGNMENT));
    result->m_joints[i] = (PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll*)resource.ptr;
    resource.increment(NMP::Memory::Format(sizeof(PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll), NMP_NATURAL_TYPE_ALIGNMENT));
  }

  result->m_animRigDef = animRigDef;
  result->m_animToPhysicsMap = animToPhysicsMap;
  result->m_physicsRigDef = physicsRigDef;

  result->m_collisionTypeMask = collisionTypeMask;
  result->m_collisionIgnoreMask = collisionIgnoreMask;
  result->m_numCollisionGroupIndices = 0;

  //1 compound shape for every part
  JPH::Ref<JPH::Shape>* partshapes = new JPH::Ref<JPH::Shape>[numParts];
  JPH::Body** partbodies = (JPH::Body**)NMPMemoryAlloc(sizeof(JPH::Body*) * numParts);

  //make shapes first
  uint32_t totalNumShapes = 0;
  for (uint32_t iPart = 0 ; iPart < numParts ; ++iPart)
  {
    const PhysicsRigDef::Part &part = physicsRigDef->m_parts[iPart];
    const PhysicsRigDef::Part::Volume &volume = part.volume;

    JPH::StaticCompoundShapeSettings shapesettings;

    for (int i = 0; i < volume.numSpheres; i++)
    {
        const PhysicsRigDef::Part::Volume::Sphere& spheredef = volume.spheres[i];
        JPH::Mat44 localpose = nmMatrix34ToJPHMat44(spheredef.localPose);

        JPH::SphereShapeSettings spheresettings(spheredef.radius);
        NMP_ASSERT(spheredef.density > 0);
        spheresettings.SetDensity(spheredef.density);

        JPH::Ref<JPH::Shape> joltsphere = spheresettings.Create().Get();

        shapesettings.AddShape(localpose.GetTranslation(), localpose.GetQuaternion(), joltsphere);
    }
    for (int i = 0; i < volume.numBoxes; i++)
    {
        const PhysicsRigDef::Part::Volume::Box& boxdef = volume.boxes[i];
        JPH::Mat44 localpose = nmMatrix34ToJPHMat44(boxdef.localPose);

        JPH::BoxShapeSettings boxsettings(nmVector3ToJPHVec3(boxdef.dimensions));
        NMP_ASSERT(boxdef.density > 0);
        boxsettings.SetDensity(boxdef.density);

        JPH::Ref<JPH::Shape> joltbox = boxsettings.Create().Get();

        shapesettings.AddShape(localpose.GetTranslation(), localpose.GetQuaternion(), joltbox);
    }
    for (int i = 0; i < volume.numCapsules; i++)
    {
        const PhysicsRigDef::Part::Volume::Capsule& capsuledef = volume.capsules[i];
        JPH::Mat44 localpose = nmMatrix34ToJPHMat44(capsuledef.localPose);
        JPH::Mat44 capsule_correction = JPH::Mat44::sRotation(JPH::Quat::sRotation(JPH::Vec3(1, 0, 0), NM_PI_OVER_TWO));

        localpose = localpose * capsule_correction;

        JPH::CapsuleShapeSettings capsulesettings(capsuledef.height / 2.f, capsuledef.radius);
        NMP_ASSERT(capsuledef.density > 0);
        capsulesettings.SetDensity(capsuledef.density);

        JPH::Ref<JPH::Shape> joltcapsule = capsulesettings.Create().Get();

        shapesettings.AddShape(localpose.GetTranslation(), localpose.GetQuaternion(), joltcapsule);
    }

    JPH::ShapeSettings::ShapeResult result = shapesettings.Create();
    if (result.HasError())
    {
        JPH::String stringerror = result.GetError();
        NMP_ASSERT(0);
    }
    partshapes[iPart] = result.Get();

    totalNumShapes += (volume.numSpheres + volume.numBoxes + volume.numCapsules);
  }

  JPH::BodyInterface& interface = joltphys_scene->m_joltPhysScene->GetBodyInterface();
  const JPH::BodyLockInterfaceLocking& lockinterface = joltphys_scene->m_joltPhysScene->GetBodyLockInterface();
  JPH::Ref<JPH::RagdollSettings> ragdollsettings = new JPH::RagdollSettings;

  ragdollsettings->mParts.resize(numParts);

  for (uint32_t iPart = 0; iPart < numParts; ++iPart)
  {
      const PhysicsRigDef::Part& part = physicsRigDef->m_parts[iPart];
      const PhysicsRigDef::Part::Actor& actor = part.actor;
      JPH::Mat44 transform = nmMatrix34ToJPHMat44(actor.globalPose);

      JPH::RagdollSettings::Part bodysettings;
      bodysettings.SetShape(partshapes[iPart]);

      bodysettings.mPosition = transform.GetTranslation();
      bodysettings.mRotation = transform.GetQuaternion();
      bodysettings.mMotionType = JPH::EMotionType::Dynamic;
      bodysettings.mObjectLayer = NMPhysLayers::CHARACTER_PART;
      bodysettings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
      bodysettings.mMassPropertiesOverride.mMass = 1;
      bodysettings.mToParent = nullptr;
      bodysettings.mGravityFactor = 1;

      ragdollsettings->mParts[iPart] = bodysettings;
  }
  if (physicsRigDef->getNumJoints())
  {//create skeleton
    PhysicsRigJoltPhysRagdoll::createJoints(joltphys_scene, physicsRigDef, result, ragdollsettings);
  }

  ragdollsettings->DisableParentChildCollisions();
  ragdollsettings->Stabilize();

  result->m_ragdoll = ragdollsettings->CreateRagdoll(0, 0, joltphys_scene->m_joltPhysScene);
  for (int i = 0; i < result->m_ragdoll->GetConstraintCount(); i++)
  {
      JPH::SwingTwistConstraint* swingtwistconstraint = (JPH::SwingTwistConstraint*)result->m_ragdoll->GetConstraint(i);

      swingtwistconstraint->SetSwingMotorState(JPH::EMotorState::Position);
      swingtwistconstraint->SetTwistMotorState(JPH::EMotorState::Position);
  }
  for (int i = 0; i < physicsRigDef->getNumJoints(); i++)
  {
      new(result->m_joints[i]) MR::PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll(result->m_ragdoll->GetConstraint(i), (PhysicsSixDOFJointDef*)physicsRigDef->m_joints[i]);

      PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll* jointJolt = (PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll*)result->m_joints[i];
      const PhysicsSixDOFJointDef* jointDef = (const PhysicsSixDOFJointDef*)physicsRigDef->m_joints[i];
      const PhysicsJointDriverDataJoltPhys* driverData = (const PhysicsJointDriverDataJoltPhys*)jointDef->m_driverData;

      jointJolt->m_strength = 0.0f;
      jointJolt->m_damping = driverData->m_articulationDamping;

      jointJolt->m_maxStrength = driverData->m_articulationSpring;
      jointJolt->m_maxDamping = driverData->m_articulationDamping;

      jointJolt->m_driveStrengthScale = driverData->m_driveStrengthScale;
      jointJolt->m_driveDampingScale = driverData->m_driveDampingScale;
      jointJolt->m_driveCompensationScale = driverData->m_driveCompensationScale;
      jointJolt->m_driveMinDampingScale = driverData->m_driveMinDampingScale;

      jointJolt->m_lastTargetOrientation.setXYZW(0, 0, 0, 0); // So will always update the physX internal target orientation on the first frame.

      jointJolt->m_rotationDirty = true;
  }
  for (int i = 0; i < result->m_ragdoll->GetBodyCount(); i++)
  {
      new (result->m_parts[i]) PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll(0, 0);
      PartJoltPhysRagdoll* part = (PartJoltPhysRagdoll*)result->m_parts[i];
      part->m_physicsRig = result;
      part->m_rigidBody = part->m_kinematicBody = lockinterface.TryGetBody(result->m_ragdoll->GetBodyID(i));
  }

  // Note that the parent joint index of a bone is guaranteed (in morpheme export) to always the
  // bone index - 1, since every part has a parent joint and parent part, except for the root part.
  Part* part = result->getPart(0);
  part->setParentPartIndex(-1);
  for (uint32_t i = 1; i < physicsRigDef->getNumParts(); i++)
  {
    part = result->getPart(i);
    part->setParentPartIndex(physicsRigDef->m_joints[i-1]->m_parentPartIndex);
  }

  result->restoreAllJointDrivesToDefault();

  result->m_isRagdollAddedToScene = false;

  // Need to generate the cached values.
  result->generateCachedValues(1.0f); // timestep is irrelevant at this point

  return result;
}

void PhysicsRigJoltPhysRagdoll::createJoints(
    MR::PhysicsSceneJoltPhys* physscene,
    PhysicsRigDef* physicsRigDef,
    PhysicsRigJoltPhysRagdoll* ragdollrig,
    JPH::RagdollSettings* joltragdoll)
{
    //joint index == part index

    JPH::Skeleton* rag_skel = joltragdoll->mSkeleton = new JPH::Skeleton();

    rag_skel->AddJoint("root", -1);

    for (int i = 0; i < physicsRigDef->getNumJoints(); i++)
    {
        PhysicsJointDef* jointdef = physicsRigDef->m_joints[i];
        rag_skel->AddJoint(jointdef->m_name, jointdef->m_parentPartIndex);
    }

    for (int i = 1; i < rag_skel->GetJointCount(); i++) //skip 0 (root)
    {
        PhysicsJointDef* jointdef = physicsRigDef->m_joints[i - 1];
        int childindex = jointdef->m_childPartIndex;
        int parentindex = jointdef->m_parentPartIndex;

        JPH::RagdollSettings::Part& childpart = joltragdoll->mParts[childindex];
        JPH::RagdollSettings::Part& parentpart = joltragdoll->mParts[parentindex];

        JPH::Vec3 parentCOM = parentpart.GetShape()->GetCenterOfMass();
        JPH::Vec3 childCOM = childpart.GetShape()->GetCenterOfMass();

        JPH::Mat44 parentpartframe = MR::nmMatrix34ToJPHMat44(jointdef->m_parentPartFrame);
        JPH::Mat44 childpartframe = MR::nmMatrix34ToJPHMat44(jointdef->m_childPartFrame);
        parentpartframe = JPH::Mat44::sRotationTranslation(MR::nmQuatToJPHQuat(jointdef->m_parentFrameQuat), parentpartframe.GetTranslation());
        childpartframe = JPH::Mat44::sRotationTranslation(MR::nmQuatToJPHQuat(jointdef->m_childFrameQuat), childpartframe.GetTranslation());

        switch (jointdef->m_jointType)
        {
            case PhysicsJointDef::JOINT_TYPE_SIX_DOF:
            {
                PhysicsSixDOFJointDef* sixdofjointdef = (PhysicsSixDOFJointDef*)jointdef;
                JPH::SwingTwistConstraintSettings* csettings = new JPH::SwingTwistConstraintSettings;

                csettings->mSwingType = JPH::ESwingType::Cone;

                csettings->mEnabled = true;

                csettings->mSpace = JPH::EConstraintSpace::LocalToBodyCOM;

                csettings->mPosition2 = childpartframe.GetTranslation() - childCOM;
                csettings->mPosition1 = parentpartframe.GetTranslation() - parentCOM;

                csettings->mTwistAxis1 = parentpartframe.GetAxisX();
                csettings->mPlaneAxis1 = parentpartframe.GetAxisZ();
                csettings->mTwistAxis2 = childpartframe.GetAxisX();
                csettings->mPlaneAxis2 = childpartframe.GetAxisZ();

                float swing1_limit = sixdofjointdef->m_hardLimits.getSwing1Limit();
                float swing2_limit = sixdofjointdef->m_hardLimits.getSwing2Limit();
                float twist_lowlimit = sixdofjointdef->m_hardLimits.getTwistLimitLow();
                float twist_highlimit = sixdofjointdef->m_hardLimits.getTwistLimitHigh();


                csettings->mTwistMinAngle = twist_lowlimit;
                csettings->mTwistMaxAngle = twist_highlimit;

                csettings->mNormalHalfConeAngle = swing1_limit;
                csettings->mPlaneHalfConeAngle = swing2_limit;

                JPH::MotorSettings& swingmotor = csettings->mSwingMotorSettings;
                swingmotor.mMinTorqueLimit = -20000; 
                swingmotor.mMaxTorqueLimit = 20000;
                swingmotor.mSpringSettings.mDamping = 1;
                swingmotor.mSpringSettings.mStiffness = 100;
                swingmotor.mSpringSettings.mMode = JPH::ESpringMode::StiffnessAndDamping;
                JPH::MotorSettings& twistmotor = csettings->mTwistMotorSettings;
                twistmotor.mMinTorqueLimit = -20000;
                twistmotor.mMaxTorqueLimit = 20000;
                twistmotor.mSpringSettings.mDamping = 1;
                twistmotor.mSpringSettings.mStiffness = 100;
                twistmotor.mSpringSettings.mMode = JPH::ESpringMode::StiffnessAndDamping;

                //maybe this is too big ?
                csettings->mNumPositionStepsOverride = csettings->mNumVelocityStepsOverride = 32;
                
                childpart.mToParent = csettings;
            }
            break;
            default:
                NMP_ASSERT_FAIL();
                break;

        }
    }
    joltragdoll->CalculateBodyIndexToConstraintIndex();
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysRagdoll::term()
{
  JPH::PhysicsSystem *joltPhysScene = getPhysicsSceneJoltPhys()->m_joltPhysScene;
  if (joltPhysScene)
  {
    m_ragdoll->RemoveFromPhysicsSystem();
    delete m_ragdoll;
    m_ragdoll = nullptr;
  }

  return true;
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysRagdoll::PhysicsRigJoltPhysRagdoll(PhysicsSceneJoltPhys *physicsScene)
{
  m_physicsScene = physicsScene;
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::PartJoltPhysRagdoll(
  int32_t defaultCollisionGroupMask, int32_t allowedCollisionGroupMask)
{
  m_userData = NULL;
  m_recalcVels = false;
  m_parentPartIndex = -1;
  m_isKinematic = false;
  m_defaultCollisionGroupMask = m_currentCollisionGroupMask = defaultCollisionGroupMask;
  m_allowedCollisionGroupMask = allowedCollisionGroupMask;
  m_mass = 0.0f;
  m_extraMass = 0.0f;
  m_extraMassCOMPosition.setToZero();
  m_modifiedFlags = 0;
  m_kinematicBody = 0;
  m_constraintToKinematic = 0;
  m_massMultiplier = 1.0f;
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::~PartJoltPhysRagdoll() 
{
  if (m_constraintToKinematic)
    m_constraintToKinematic->Release();
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::PartJoltPhysRagdoll(const PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll &other) 
{
  *this = other;
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll &PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::operator=(const PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll &other) 
{
  if (this == &other)
    return *this;

  m_rigidBody = other.m_rigidBody;
  m_parentPartIndex = other.m_parentPartIndex;
  return *this;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::setCurrentCollisionGroupMask(uint32_t mask)
{
  m_currentCollisionGroupMask = mask;
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::generateCachedValues(float timeStep)
{
  // Get the pose from the kinematic part if the articulation wasn't added,
  // otherwise it is delayed by a frame in HK (because in HK we set the position
  // to the last transform and expect physx to move it according to the
  // velocity).
  const JPH::Mat44 currentGlobalPose = 
    ((PhysicsRigJoltPhysRagdoll*)m_physicsRig)->m_isRagdollAddedToScene 
    ? m_rigidBody->GetWorldTransform()
    : m_kinematicBody->GetWorldTransform();

  //if (!currentGlobalPose.isFinite())
  //{
  //  return false;
  //}

  const NMP::Matrix34 currentTM = nmJPHMat44ToNmMatrix34(currentGlobalPose);

  // Note that the local COM offset wouldn't normally change, but it might if the part shapes are modified.
  //NMP_ASSERT(m_rigidBody->getCMassLocalPose().isFinite());
  //m_cache.cachedCOMOffset = nmJPHMat44ToNmMatrix34(m_rigidBody->getCMassLocalPose());

  NMP::Vector3 currentComPos = currentTM.getTransformedVector(m_cache.cachedCOMOffset.translation());

  if (m_recalcVels && timeStep > 0.0f) 
  {
    // reset velocities after physics based on new transform relative to cached (previous) transform

    // calculate the motion at the CoM for the velocities
    NMP::Matrix34 invPrevTM(m_cache.cachedTransform); invPrevTM.invertFast();
    NMP::Matrix34 motionTM = invPrevTM * currentTM;
    m_cache.cachedVel = (motionTM.getTransformedVector(currentComPos) - currentComPos) / timeStep;
    m_cache.cachedAngVel = motionTM.toQuat().toRotationVector(false) / timeStep;
    m_recalcVels = false;
  }
  else
  {
    JPH::Vec3 angVel = m_rigidBody->GetAngularVelocity();
    JPH::Vec3 linVel = m_rigidBody->GetLinearVelocity();
    if (angVel.IsNaN() || linVel.IsNaN())
    {
      return false;
    }
    m_cache.cachedAngVel = nmJPHVec3ToVector3(angVel);
    m_cache.cachedVel = nmJPHVec3ToVector3(linVel);
  }

  m_cache.cachedTransform = currentTM;
  m_cache.cachedCOMPosition = currentComPos;

  // The transforms from PhysX can be pretty bad, so orthonormalise the result.
  m_cache.cachedTransform.orthonormalise();

  return true;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::handleExplosion(const NMP::Matrix34& worldRoot)
{
  if (m_isRagdollAddedToScene)
  {
    removeRagdollFromScene();
    addRagdollToScene();
  }

  for (uint32_t i = 0; i < getNumParts(); i++)
  {
    PartJoltPhysRagdoll* partJoltPhysRagdoll = (PartJoltPhysRagdoll*)m_parts[i];

    NMP::Matrix34 tm;
    calculateWorldSpacePartTM(
      tm, 
      i, 
      *m_animRigDef->getBindPose()->m_transformBuffer, 
      *m_animRigDef->getBindPose()->m_transformBuffer, 
      worldRoot, 
      false);

    partJoltPhysRagdoll->setTransform(tm);
    partJoltPhysRagdoll->setVel(NMP::Vector3::InitZero);
    partJoltPhysRagdoll->setAngVel(NMP::Vector3::InitZero);
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::generateCachedValues(float timeStep)
{
  bool OK = true;
  for (uint32_t i = 0; i < getNumParts(); i++)
  {
    if (!((PartJoltPhysRagdoll*)m_parts[i])->generateCachedValues(timeStep))
    {
      OK = false;
    }
  }
  if (!OK)
  {
    NMP_MSG("JoltPhys has exploded - reinitialising at the last known position\n");

    PartJoltPhysRagdoll* rootPartJoltPhysRagdoll = (PartJoltPhysRagdoll*)m_parts[0];
    NMP::Matrix34 worldRoot = rootPartJoltPhysRagdoll->getTransform();
    if (!worldRoot.isValidTM(0.001f))
    {
      worldRoot.identity();
    }

    if (s_explosionHandler)
    {
      (*s_explosionHandler)(this, worldRoot);
    }
    else
    {
      handleExplosion(worldRoot);
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::updatePrePhysics(float timeStep)
{
  const PhysicsRigDef* physicsRigDef = getPhysicsRigDef();

  // Activate requested collision sets
  for (uint32_t i = 0, numParts = getNumParts(); i < numParts; i++)
  {
    MR::PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll *partPhysX = getPartJoltPhysRagdoll(i);
    uint32_t desiredMask = partPhysX->getDefaultCollisionGroupMask();
    uint32_t allowedMask = partPhysX->getAllowedCollisionGroupMask();
    NMP_ASSERT(m_numCollisionGroupIndices < m_maxCollisionGroupIndices);
    for (int iSet = 0 ; iSet < m_numCollisionGroupIndices ; ++iSet)
    {
      int collisionGroupIndexToActivate = m_collisionGroupIndicesToActivate[iSet];
      if (collisionGroupIndexToActivate >= 0 && (allowedMask & 1 << collisionGroupIndexToActivate))
      {
        NMP_ASSERT(collisionGroupIndexToActivate < m_maxCollisionGroupIndices);
        desiredMask |= 1 << collisionGroupIndexToActivate;
      }
    }
    // If the desired mask has changed then set it on all the shapes.
    if (desiredMask != partPhysX->getCurrentCollisionGroupMask())
    {
      partPhysX->setCurrentCollisionGroupMask(desiredMask);
    }

    // increase the skin width to provide pseudo-CCD
    const PhysicsActorDriverDataJoltPhys* actorDriverData =
      (const PhysicsActorDriverDataJoltPhys*)physicsRigDef->m_parts[i].actor.driverData;

    float maxSkinWidthIncrease = actorDriverData->m_maxContactOffsetIncrease;
    float speed = partPhysX->getVel().magnitude();
    float skinWidthIncrease = NMP::minimum(speed * timeStep, maxSkinWidthIncrease);
    if (skinWidthIncrease > 0.0f)
    {
      // don't call less than 0 as it resets what the behaviours have asked for (and calling with 0
      // does nothing).
      setSkinWidthIncrease(i, skinWidthIncrease);
    }

    // Reset properties that weren't modified since the last pre-physics call
    if (!(partPhysX->m_modifiedFlags & PartJoltPhysRagdoll::MODIFIED_EXTRA_MASS))
    {
      partPhysX->setExtraMass(0.0f, NMP::Vector3::InitZero);
    }
    if (!(partPhysX->m_modifiedFlags & PartJoltPhysRagdoll::MODIFIED_INERTIA))
    {
      partPhysX->setMassSpaceInertia(partPhysX->getOriginalMassSpaceInertia() * partPhysX->m_massMultiplier);
    }
    partPhysX->m_modifiedFlags = 0;
  }

  // Update physX joint limits.
  writeJointLimits();

  // Prevent large compliances from letting the ISF value in physx get < 1, which would lead to
  // instability.
  uint32_t numJoints = getNumJoints();
  for (uint32_t iJoint = 0 ; iJoint != numJoints ; ++iJoint)
  {
    JointJoltPhysRagdoll* joint = getJointJoltPhysRagdoll(iJoint);

    float s = joint->getStrength();
    float d = joint->getDamping();
    float c = joint->getExternalCompliance();

    float isf = (1 + d * timeStep + s * timeStep * timeStep) / c;
    if (isf < 1.0f)
    {
      c *= isf;
      joint->setExternalCompliance(c);
    }
  }

  // reset the list of collision sets to activate
  m_numCollisionGroupIndices = 0;

  // Set the separation properties - these will get reset in updatePostPhysics
  //m_ragdoll->setSeparationTolerance(m_desiredJointProjectionLinearTolerance);
  // Note that there is no angular tolerance
  //m_ragdoll->setMaxProjectionIterations(m_desiredJointProjectionIterations);

  updateRegisteredJoints();
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::updatePostPhysics(float timeStep)
{
  if (!isReferenced())
  {
    return;
  }

  reenableSleeping();

  PhysicsRigJoltPhysRagdoll *physicsRigPhysX = (PhysicsRigJoltPhysRagdoll *)this;
  physicsRigPhysX->generateCachedValues(timeStep);

  // Reset all the joint drives to be relaxed - this is needed so that behaviours can interact
  // nicely with simple physics and we don't have to worry about overwriting physics nodes when
  // trying to stop their actions persisting... However, the multiple calls to the physics api are
  // not likely to be efficient - we should make it so the physx api calls are all done in a
  // pre/post-physics function only, and then this could just be done by setting some state
  // variables. See MORPH-11292
  for (uint32_t i = 0 ;i < physicsRigPhysX->getNumJoints(); ++i)
  {
    JointJoltPhysRagdoll *joint = getJointJoltPhysRagdoll(i);
    joint->m_rotationDirty = true;
    joint->setStrength(0.0f);
    joint->setDamping(joint->getMaxDamping());
    joint->setDriveCompensation(0.0f);
  }

#if defined(MR_OUTPUT_DEBUGGING)
  // Copy joint limits from joint into serialisation data structure.
  for (uint32_t i = 0, numJoints = getNumJoints(); i < numJoints; ++i)
  {
    static_cast< JointJoltPhys* >(getJoint(i))->updateSerializeTxFrameData();
  }
#endif // MR_OUTPUT_DEBUGGING

  resetJointLimits();
  m_desiredJointProjectionIterations = 0;
  m_desiredJointProjectionLinearTolerance = FLT_MAX;

  for (uint32_t i = 0 ; i < physicsRigPhysX->getNumParts() ; ++i)
  {
    physicsRigPhysX->setSkinWidthIncrease(i, 0.0f);
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::setExtraMass(float mass, const NMP::Vector3& massCOMPosition)
{
  m_extraMass = mass;
  m_extraMassCOMPosition = massCOMPosition;
  m_modifiedFlags |= MODIFIED_EXTRA_MASS;
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Vector3 PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::getAugmentedCOMPosition() const
{
  return (m_cache.cachedCOMPosition * m_mass + m_extraMassCOMPosition * m_extraMass) / (m_mass + m_extraMass);
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Vector3 PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::getCOMPosition() const 
{
  return m_cache.cachedCOMPosition; 
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Vector3 PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::getVelocityAtPoint(const NMP::Vector3 &point) const
{
  NMP::Vector3 rpoint = point - getCOMPosition();
  return getVel() + NMP::vCross(getAngVel(), rpoint);
}

//---------------------------------------------------------------------------------------------------------------------- 
NMP::Vector3 PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::getAngularMomentum() const  
{
  NMP::Matrix34 inertia = getGlobalInertiaTensor();
  NMP::Vector3 angVel = getAngVel();
  NMP::Vector3 angMom;
  inertia.rotateVector(angVel, angMom);
  return angMom;
}  

//----------------------------------------------------------------------------------------------------------------------  
NMP::Vector3 PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::getLinearMomentum() const  
{    
  return getVel() * getMass();
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Vector3 PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::getMassSpaceInertiaTensor() const
{
  return nmJPHVec3ToVector3(m_rigidBody->GetMotionProperties()->GetLocalSpaceInverseInertia().GetDiagonal3().Reciprocal());
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Matrix34 PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::getGlobalInertiaTensor() const
{
  NMP::Vector3 t = getMassSpaceInertiaTensor();
  NMP::Matrix34 massSpaceInertiaTensor(NMP::Matrix34::kIdentity);
  massSpaceInertiaTensor.scale3x3(t);
  NMP::Matrix34 localOffset = getCOMOffsetLocal();
  NMP::Matrix34 result = massSpaceInertiaTensor * localOffset * getTransform();
  return result;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::setMassSpaceInertia(const NMP::Vector3& inertia)
{
  // Node that this does little more than copying the data on the PhysX side, so should be pretty
  // fast.

  JPH::Vec3 inv_inertia = nmVector3ToJPHVec3(inertia).Reciprocal();
  JPH::Mat44 inv_inertia_tensor = JPH::Mat44::sScale(inv_inertia);

  //m_rigidBody->GetMotionProperties()->SetInverseInertia(inv_inertia_tensor.GetTranslation(), inv_inertia_tensor.GetQuaternion());
  m_modifiedFlags |= MODIFIED_INERTIA;
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Quat PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::getQuaternion() const
{
  return nmJPHQuatToQuat(m_rigidBody->GetWorldTransform().GetQuaternion());
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::setVel(const NMP::Vector3 &v)
{
  m_rigidBody->SetLinearVelocity(nmVector3ToJPHVec3(v));
  m_cache.cachedVel = v;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::setAngVel(const NMP::Vector3 &v)
{
  m_rigidBody->SetAngularVelocity(nmVector3ToJPHVec3(v));
  m_cache.cachedAngVel = v;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::moveTo(const NMP::Matrix34 &tm, bool updateCache)
{
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::setPosition(const NMP::Vector3 &NMP_UNUSED(p))
{
  // This function is not currently implemented for PhysX3
  NMP_ASSERT_FAIL();
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::setQuaternion(const NMP::Quat &NMP_UNUSED(q))
{
  // This function is not currently implemented for PhysX3
  NMP_ASSERT_FAIL();
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::setTransform(const NMP::Matrix34 &tm)
{
  NMP_ASSERT(tm.isValidTM(0.1f));
  //m_rigidBody->setGlobalPose(nmMatrix34ToPxTransform(tm));
  //if (m_isKinematic && m_kinematicBody)
  //  m_kinematicBody->setGlobalPose(nmMatrix34ToPxTransform(tm));
  m_cache.cachedTransform = tm;
  //m_cache.cachedCOMPosition = nmJPHVec3ToVector3(nmMatrix34ToJPHMat44(tm).transform(m_rigidBody->getCMassLocalPose().p));
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::makeKinematic(
  bool kinematic, float massMultiplier, bool enableConstraint)
{
  if (!kinematic)
  {
    m_isBeingKeyframed = false;
    massMultiplier = 1.0f;
    enableConstraint = false;
  }

  if (
    kinematic == m_isKinematic && 
    massMultiplier == m_massMultiplier && 
    enableConstraint == (m_constraintToKinematic ? true : false)
    )
  {
    return;
  }

  JPH::Body *body = getBody();
  NMP_ASSERT(body);

  if (!kinematic)
  {
    // Move the kinematic shape somewhere far.
    if (m_kinematicBody)
    {
      //m_kinematicBody->setGlobalPose(nmMatrix34ToPxTransform(
      //  ((PhysicsRigJoltPhysRagdoll*)m_physicsRig)->m_kinematicPose));
    }
  }
  else
  {
    if (m_kinematicBody)
    {
      //m_kinematicBody->setGlobalPose(body->getGlobalPose());
    }
  }
  m_isKinematic = kinematic;

  if (massMultiplier != m_massMultiplier)
  {
    body->GetMotionProperties()->SetInverseMass(1.0 / (m_mass * massMultiplier));
    setMassSpaceInertia(m_inertia * massMultiplier);
    m_massMultiplier = massMultiplier;
  }

  //if (enableConstraint && !m_constraintToKinematic)
  //{
  //  m_constraintToKinematic = PxD6JointCreate(
  //    PxGetPhysics(), 
  //    m_kinematicActor,
  //    physx::PxTransform(physx::PxIdentity),
  //    link,
  //    physx::PxTransform(physx::PxIdentity));
  //  m_constraintToKinematic->setMotion(physx::PxD6Axis::eX, physx::PxD6Motion::eLOCKED);
  //  m_constraintToKinematic->setMotion(physx::PxD6Axis::eY, physx::PxD6Motion::eLOCKED);
  //  m_constraintToKinematic->setMotion(physx::PxD6Axis::eZ, physx::PxD6Motion::eLOCKED);
  //  m_constraintToKinematic->setMotion(physx::PxD6Axis::eSWING1, physx::PxD6Motion::eLOCKED);
  //  m_constraintToKinematic->setMotion(physx::PxD6Axis::eSWING2, physx::PxD6Motion::eLOCKED);
  //  m_constraintToKinematic->setMotion(physx::PxD6Axis::eTWIST, physx::PxD6Motion::eLOCKED);
  //}
  //else if (!enableConstraint && m_constraintToKinematic)
  //{
  //  m_constraintToKinematic->Release();
  //  m_constraintToKinematic = 0;
  //}

}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::isKinematic() const
{
  return m_isKinematic;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::enableBodyCollision(JPH::Body *body, bool enable)
{
    //undone
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::enableCollision(bool enable)
{
  if (m_isKinematic && m_kinematicBody)
  {
    enableBodyCollision(m_kinematicBody, enable);
  }
  else
  {
    enableBodyCollision(m_rigidBody, enable);
  }
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::getCollisionEnabled() const
{
    return false;//undone
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::storeState(PhysicsSerialisationBuffer &savedState)
{
  savedState.addValue(getTransform());
  savedState.addValue(getVel());
  savedState.addValue(getAngVel());
  return true;
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::restoreState(PhysicsSerialisationBuffer &savedState)
{
  NMP::Matrix34 m = savedState.getValue<NMP::Matrix34>();
  setTransform(m);
  NMP::Vector3 vel = savedState.getValue<NMP::Vector3>();
  setVel(vel);
  NMP::Vector3 angVel = savedState.getValue<NMP::Vector3>();
  setAngVel(angVel);
  return true;
}

#if defined(MR_OUTPUT_DEBUGGING)

//----------------------------------------------------------------------------------------------------------------------
uint32_t PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::serializeTxPersistentData(
  uint16_t nameToken, 
  uint32_t objectID, 
  void* outputBuffer, 
  uint32_t NMP_USED_FOR_ASSERTS(outputBufferSize)) const
{
  uint32_t dataSize = sizeof(PhysicsPartPersistentData);

  uint32_t numBoxes = 0;
  uint32_t numCapsules = 0;
  uint32_t numSpheres = 0;

  uint32_t numShapes = 1;
  const JPH::Shape* partshape = m_kinematicBody->GetShape();
  if (partshape->GetType() == JPH::EShapeType::Compound)
  {
      numShapes = ((const JPH::StaticCompoundShape*)partshape)->GetNumSubShapes();
  }
  else
  {
      NMP_ASSERT(partshape->GetType() == JPH::EShapeType::Decorated);
      NMP_ASSERT(partshape->GetSubType() == JPH::EShapeSubType::RotatedTranslated);
      const JPH::Shape* innershape = ((const JPH::RotatedTranslatedShape*)partshape)->GetInnerShape();
      switch (innershape->GetSubType())
      {
      case JPH::EShapeSubType::Sphere:
          ++numSpheres;
          break;
      case JPH::EShapeSubType::Box:
          ++numBoxes;
          break;
      case JPH::EShapeSubType::Capsule:
          ++numCapsules;
          break;
      default:
          break;
      }
      dataSize += numBoxes * sizeof(PhysicsBoxPersistentData);
      dataSize += numCapsules * sizeof(PhysicsCapsulePersistentData);
      dataSize += numSpheres * sizeof(PhysicsSpherePersistentData);
      if (outputBuffer != 0)
      {
          NMP_ASSERT(outputBufferSize >= dataSize);
          PhysicsPartPersistentData* partPersistentData = (PhysicsPartPersistentData*)outputBuffer;

          partPersistentData->m_parentIndex = getParentPartIndex();
          partPersistentData->m_physicsObjectID = objectID;
          partPersistentData->m_numBoxes = numBoxes;
          partPersistentData->m_numCapsules = numCapsules;
          partPersistentData->m_numSpheres = numSpheres;
          partPersistentData->m_nameToken = nameToken;

          // convert to capsule orientated along y, not z, this code mirrors the creation code.
          NMP::Matrix34 yToZ(NMP::Matrix34::kIdentity);

          // convert to capsule orientated along z, not y
          NMP::Matrix34 capsuleConversionTx(NMP::Matrix34::kIdentity);
          capsuleConversionTx.fromEulerXYZ(NMP::Vector3(0, NM_PI_OVER_TWO, 0));

          uint32_t indexBox = 0;
          uint32_t indexCapsule = 0;
          uint32_t indexSphere = 0;

          switch (innershape->GetSubType())
          {
          case JPH::EShapeSubType::Sphere:
          {
              const JPH::SphereShape* pxSphere = (const JPH::SphereShape*)innershape;
              NMP_ASSERT(pxSphere);

              PhysicsSpherePersistentData* persistentData = partPersistentData->getSphere(indexSphere);

              //JPH::Mat44 localPose = 
              persistentData->m_localPose = NMP::Matrix34Identity(); //nmPxTransformToNmMatrix34(localPose);

              persistentData->m_radius = pxSphere->GetRadius();

              persistentData->m_parentIndex = 0;
              NMP::netEndianSwap(persistentData->m_parentIndex);
              NMP::netEndianSwap(persistentData->m_localPose);
              NMP::netEndianSwap(persistentData->m_radius);

              ++indexSphere;
              break;
          }
          case JPH::EShapeSubType::Box:
          {
              const JPH::BoxShape* pxBox = (const JPH::BoxShape*)(innershape);
              NMP_ASSERT(pxBox);

              PhysicsBoxPersistentData* persistentData = partPersistentData->getBox(indexBox);

              //JPH::Mat44 localPose = 
              persistentData->m_localPose = NMP::Matrix34Identity(); //nmPxTransformToNmMatrix34(localPose);

              persistentData->m_width = 2.0f * pxBox->GetHalfExtent().GetX();
              persistentData->m_height = 2.0f * pxBox->GetHalfExtent().GetY();
              persistentData->m_depth = 2.0f * pxBox->GetHalfExtent().GetZ();

              persistentData->m_parentIndex = 0;
              NMP::netEndianSwap(persistentData->m_parentIndex);
              NMP::netEndianSwap(persistentData->m_localPose);
              NMP::netEndianSwap(persistentData->m_width);
              NMP::netEndianSwap(persistentData->m_height);
              NMP::netEndianSwap(persistentData->m_depth);

              ++indexBox;
              break;
          }
          case JPH::EShapeSubType::Capsule:
          {
              const JPH::CapsuleShape* pxCapsule = (const JPH::CapsuleShape*)innershape;
              NMP_ASSERT(pxCapsule);

              PhysicsCapsulePersistentData* persistentData = partPersistentData->getCapsule(indexCapsule);

              //JPH::Mat44 localPose = 
              persistentData->m_localPose = NMP::Matrix34Identity(); //nmPxTransformToNmMatrix34(localPose);

              persistentData->m_radius = pxCapsule->GetRadius();
              persistentData->m_height = 2.0f * pxCapsule->GetHalfHeightOfCylinder();

              persistentData->m_parentIndex = 0;
              NMP::netEndianSwap(persistentData->m_parentIndex);
              NMP::netEndianSwap(persistentData->m_localPose);
              NMP::netEndianSwap(persistentData->m_radius);
              NMP::netEndianSwap(persistentData->m_height);

              ++indexCapsule;
              break;
          }
          default:
              break;
          }
          

          NMP_ASSERT(indexBox == numBoxes);
          NMP_ASSERT(indexCapsule == numCapsules);
          NMP_ASSERT(indexSphere == numSpheres);

          NMP::netEndianSwap(partPersistentData->m_numSpheres);
          NMP::netEndianSwap(partPersistentData->m_numCapsules);
          NMP::netEndianSwap(partPersistentData->m_numBoxes);
          NMP::netEndianSwap(partPersistentData->m_nameToken);
          NMP::netEndianSwap(partPersistentData->m_parentIndex);
          NMP::netEndianSwap(partPersistentData->m_physicsObjectID);
      }

      return dataSize;
  }

  NMP_ASSERT(numShapes < MAX_SHAPES_IN_VOLUME);

  JPH::CompoundShape::SubShapes subshapes = ((const JPH::CompoundShape*)partshape)->GetSubShapes();
  for (uint32_t i = 0; i != numShapes; ++i)
  {
    const JPH::Shape *shape = subshapes[i].mShape;

    JPH::EShapeSubType type = shape->GetSubType();
    switch (type)
    {
    case JPH::EShapeSubType::Sphere:
      ++numSpheres;
      break;
    case JPH::EShapeSubType::Box:
      ++numBoxes;
      break;
    case JPH::EShapeSubType::Capsule:
      ++numCapsules;
      break;
    default:
      break;
    }
  }

  dataSize += numBoxes * sizeof(PhysicsBoxPersistentData);
  dataSize += numCapsules * sizeof(PhysicsCapsulePersistentData);
  dataSize += numSpheres * sizeof(PhysicsSpherePersistentData);

  if (outputBuffer != 0)
  {
    NMP_ASSERT(outputBufferSize >= dataSize);
    PhysicsPartPersistentData *partPersistentData = (PhysicsPartPersistentData *)outputBuffer;

    partPersistentData->m_parentIndex = getParentPartIndex();
    partPersistentData->m_physicsObjectID = objectID;
    partPersistentData->m_numBoxes = numBoxes;
    partPersistentData->m_numCapsules = numCapsules;
    partPersistentData->m_numSpheres = numSpheres;
    partPersistentData->m_nameToken = nameToken;

    // convert to capsule orientated along y, not z, this code mirrors the creation code.
    NMP::Matrix34 yToZ(NMP::Matrix34::kIdentity);

    // convert to capsule orientated along z, not y
    NMP::Matrix34 capsuleConversionTx(NMP::Matrix34::kIdentity);
    capsuleConversionTx.fromEulerXYZ(NMP::Vector3(0, NM_PI_OVER_TWO, 0));

    uint32_t indexBox = 0;
    uint32_t indexCapsule = 0;
    uint32_t indexSphere = 0;
    for (uint32_t i = 0; i != numShapes; ++i)
    {
      const JPH::Shape *pxShape = subshapes[i].mShape;

      JPH::EShapeSubType type = pxShape->GetSubType();
      switch (type)
      {
        case JPH::EShapeSubType::Sphere:
        {
          JPH::SphereShape *pxSphere = dynamic_cast<JPH::SphereShape*>((JPH::Shape*)pxShape);
          NMP_ASSERT(pxSphere);

          PhysicsSpherePersistentData* persistentData = partPersistentData->getSphere(indexSphere);

          //JPH::Mat44 localPose = 
          persistentData->m_localPose = NMP::Matrix34Identity(); //nmPxTransformToNmMatrix34(localPose);

          persistentData->m_radius = pxSphere->GetRadius();

          persistentData->m_parentIndex = i;
          NMP::netEndianSwap(persistentData->m_parentIndex);
          NMP::netEndianSwap(persistentData->m_localPose);
          NMP::netEndianSwap(persistentData->m_radius);

          ++indexSphere;
          break;
        }
        case JPH::EShapeSubType::Box:
        {
          JPH::BoxShape* pxBox = dynamic_cast<JPH::BoxShape*>((JPH::Shape*)pxShape);
          NMP_ASSERT(pxBox);

          PhysicsBoxPersistentData* persistentData = partPersistentData->getBox(indexBox);

          //JPH::Mat44 localPose = 
          persistentData->m_localPose = NMP::Matrix34Identity(); //nmPxTransformToNmMatrix34(localPose);

          persistentData->m_width = 2.0f * pxBox->GetHalfExtent().GetX();
          persistentData->m_height = 2.0f * pxBox->GetHalfExtent().GetY();
          persistentData->m_depth = 2.0f * pxBox->GetHalfExtent().GetZ();

          persistentData->m_parentIndex = i;
          NMP::netEndianSwap(persistentData->m_parentIndex);
          NMP::netEndianSwap(persistentData->m_localPose);
          NMP::netEndianSwap(persistentData->m_width);
          NMP::netEndianSwap(persistentData->m_height);
          NMP::netEndianSwap(persistentData->m_depth);

          ++indexBox;
          break;
        }
        case JPH::EShapeSubType::Capsule:
        {
          JPH::CapsuleShape* pxCapsule = dynamic_cast<JPH::CapsuleShape*>((JPH::Shape*)pxShape);
          NMP_ASSERT(pxCapsule);

          PhysicsCapsulePersistentData* persistentData = partPersistentData->getCapsule(indexCapsule);

          //JPH::Mat44 localPose = 
          persistentData->m_localPose = NMP::Matrix34Identity(); //nmPxTransformToNmMatrix34(localPose);

          persistentData->m_radius = pxCapsule->GetRadius();
          persistentData->m_height = 2.0f * pxCapsule->GetHalfHeightOfCylinder();

          persistentData->m_parentIndex = i;
          NMP::netEndianSwap(persistentData->m_parentIndex);
          NMP::netEndianSwap(persistentData->m_localPose);
          NMP::netEndianSwap(persistentData->m_radius);
          NMP::netEndianSwap(persistentData->m_height);

          ++indexCapsule;
          break;
        }
      default:
        break;
      }
    }

    NMP_ASSERT(indexBox == numBoxes);
    NMP_ASSERT(indexCapsule == numCapsules);
    NMP_ASSERT(indexSphere == numSpheres);

    NMP::netEndianSwap(partPersistentData->m_numSpheres);
    NMP::netEndianSwap(partPersistentData->m_numCapsules);
    NMP::netEndianSwap(partPersistentData->m_numBoxes);
    NMP::netEndianSwap(partPersistentData->m_nameToken);
    NMP::netEndianSwap(partPersistentData->m_parentIndex);
    NMP::netEndianSwap(partPersistentData->m_physicsObjectID);
  }

  return dataSize;
}

//----------------------------------------------------------------------------------------------------------------------
uint32_t PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::serializeTxFrameData(void* outputBuffer, uint32_t NMP_USED_FOR_ASSERTS(outputBufferSize)) const
{
  uint32_t dataSize = sizeof(PhysicsPartFrameData);

  if (outputBuffer != 0)
  {
    NMP_ASSERT(outputBufferSize >= dataSize);
    PhysicsPartFrameData *partFrameData = (PhysicsPartFrameData *)outputBuffer;

    JPH::Mat44 globalPose;
    if (m_isKinematic)
    {
      globalPose = m_kinematicBody->GetWorldTransform();
    }
    else
    {
      globalPose = m_rigidBody->GetWorldTransform();
    }
    partFrameData->m_globalPose = nmJPHMat44ToNmMatrix34(globalPose);

    NMP::netEndianSwap(partFrameData->m_globalPose);
  }

  return dataSize;
}
#endif

//----------------------------------------------------------------------------------------------------------------------
// PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::JointJoltPhysRagdoll(JPH::TwoBodyConstraint* constraint, const PhysicsSixDOFJointDef* const def)
: m_jointInternal(constraint), JointJoltPhys(def)
{}

#if defined(MR_OUTPUT_DEBUGGING)
//----------------------------------------------------------------------------------------------------------------------
uint32_t PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::serializeTxPersistentData(
  const PhysicsJointDef* jointDef,
  uint16_t  stringToken,
  void*     outputBuffer,
  uint32_t  NMP_USED_FOR_ASSERTS(outputBufferSize)) const
{
  uint32_t dataSize = sizeof(PhysicsSixDOFJointPersistentData);
  if (outputBuffer)
  {
    NMP_ASSERT(outputBufferSize >= dataSize);

    PhysicsSixDOFJointPersistentData* persistentData = (PhysicsSixDOFJointPersistentData*)outputBuffer;

    persistentData->m_parentLocalFrame = jointDef->m_parentPartFrame;
    persistentData->m_childLocalFrame = jointDef->m_childPartFrame;
    persistentData->m_parentPartIndex = jointDef->m_parentPartIndex;
    persistentData->m_childPartIndex = jointDef->m_childPartIndex;

    persistentData->m_jointType = PhysicsJointPersistentData::JOINT_TYPE_SIX_DOF;

    float swingLimitY = 0.0f;
    float swingLimitZ = 0.0f;
    //m_jointInternal->getSwingLimit(swingLimitY, swingLimitZ);

    // for some reason PxArticulationJoint returns the swing limit values
    // the wrong way round so swap the y and z values.
    persistentData->m_swing1Limit = swingLimitZ;
    persistentData->m_swing2Limit = swingLimitY;

    float twistLimitLow = 0.0f;
    float twistLimitHigh = 0.0f;
    //m_jointInternal->getTwistLimit(twistLimitLow, twistLimitHigh);

    persistentData->m_twistLimitLow = twistLimitLow;
    persistentData->m_twistLimitHigh = twistLimitHigh;
    persistentData->m_nameToken = stringToken;

    PhysicsSixDOFJointPersistentData::endianSwap(persistentData);
  }

  return dataSize;
}
#endif

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::storeState(PhysicsSerialisationBuffer& savedState)
{
  (void) savedState;
  return true;
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::restoreState(PhysicsSerialisationBuffer& savedState)
{
  (void) savedState;
  return true;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::enableLimit(bool enable)
{
  //m_jointInternal->setSwingLimitEnabled(enable);
  //m_jointInternal->setTwistLimitEnabled(enable);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::writeLimits()
{
  // PhysX crashes with zero swing range, as well as it just causing jitter when very small (e.g.
  // with "hinge" limits).
  float swing1 = NMP::maximum(m_modifiableLimits.getSwing1Limit(), s_minSwingLimit);
  float swing2 = NMP::maximum(m_modifiableLimits.getSwing2Limit(), s_minSwingLimit);
  float twistLow = m_modifiableLimits.getTwistLimitLow();
  float twistHigh = m_modifiableLimits.getTwistLimitHigh();

  // Optimise this with MORPH-16668. Note that swing1/2 are reversed (compare with the jointed rig
  // where they're not reversed) - this is intentional - just how the joint is in PhysX.
  //m_jointInternal->setSwingLimit(swing2, swing1);
  //m_jointInternal->setTwistLimit(twistLow, twistHigh);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::setStrength(float strength)
{
  //NMP_ASSERT(m_jointInternal);
  NMP_ASSERT(strength >= 0.0f && strength < MAX_STRENGTH);

  m_strength = strength;
  ((JPH::SwingTwistConstraint*)m_jointInternal)->GetSwingMotorSettings().mSpringSettings.mStiffness = strength;
  ((JPH::SwingTwistConstraint*)m_jointInternal)->GetTwistMotorSettings().mSpringSettings.mStiffness = strength;
  //m_jointInternal->setStiffness(strength);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::setDamping(float damping)
{
  //NMP_ASSERT(m_jointInternal);
  NMP_ASSERT(damping >= 0.0f && damping < MAX_DAMPING);
  m_damping = damping;
  ((JPH::SwingTwistConstraint*)m_jointInternal)->GetSwingMotorSettings().mSpringSettings.mDamping = damping;
  ((JPH::SwingTwistConstraint*)m_jointInternal)->GetTwistMotorSettings().mSpringSettings.mDamping = damping;
  //m_jointInternal->setDamping(damping);
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::supportsDriveCompensation()
{
  return true;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::setDriveCompensation(float driveCompensation)
{
  //NMP_ASSERT(m_jointInternal);
  NMP_ASSERT(driveCompensation >= 0.f); 
  setInternalCompliance(1.f/(1.f + driveCompensation));
}

//----------------------------------------------------------------------------------------------------------------------
float PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::getStrength() const
{
  return m_strength;
}

//----------------------------------------------------------------------------------------------------------------------
float PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::getDamping() const
{
  return m_damping;
}

//----------------------------------------------------------------------------------------------------------------------
float PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::getDriveCompensation() const
{
  //NMP_ASSERT(m_jointInternal);
  //float internalCompliance = m_jointInternal->getInternalCompliance();
  //if (internalCompliance < MINIMUM_COMPLIANCE)
  //  internalCompliance = MINIMUM_COMPLIANCE;
  //return (1.0f / internalCompliance) - 1.0f;
  return 0;
}

//----------------------------------------------------------------------------------------------------------------------
float PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::getInternalCompliance() const
{
  //return m_jointInternal->getInternalCompliance();
    return 0;
}

//----------------------------------------------------------------------------------------------------------------------
float PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::getExternalCompliance() const
{
  //return m_jointInternal->getExternalCompliance();
    return 0;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::setExternalCompliance(float compliance) 
{
  //if (compliance < MINIMUM_COMPLIANCE)
  //  compliance = MINIMUM_COMPLIANCE;
  //m_jointInternal->setExternalCompliance(compliance);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::setInternalCompliance(float compliance) 
{
  //if (compliance < MINIMUM_COMPLIANCE)
  //  compliance = MINIMUM_COMPLIANCE;
  //m_jointInternal->setInternalCompliance(compliance);
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Quat PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::getTargetOrientation()
{
  //NMP_ASSERT(m_jointInternal);
  JPH::Quat returnval = JPH::Quat::sIdentity();
  JPH::SwingTwistConstraint* st_joint = dynamic_cast<JPH::SwingTwistConstraint*>(m_jointInternal);
  if (st_joint)
      returnval = st_joint->GetTargetOrientationCS();

  return nmJPHQuatToQuat(returnval);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::setTargetOrientation(const NMP::Quat &orientation)
{
  NMP_ASSERT(orientation.isValid());
  // This check allows characters to go to sleep when a constant target is being passed in.
  // Since setTargetOrientation wakes up the character.
  if (orientation != m_lastTargetOrientation)
  {
      JPH::SwingTwistConstraint* st_joint = dynamic_cast<JPH::SwingTwistConstraint*>(m_jointInternal);
      if (st_joint)
          st_joint->SetTargetOrientationCS(nmQuatToJPHQuat(orientation));
  }
  m_lastTargetOrientation = orientation;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::setVelocity(const NMP::Vector3 &velocity)
{
  NMP_ASSERT(velocity.isValid());
  JPH::SwingTwistConstraint* st_joint = dynamic_cast<JPH::SwingTwistConstraint*>(m_jointInternal);
  if (st_joint)
      st_joint->SetTargetAngularVelocityCS(nmVector3ToJPHVec3(velocity));
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::makeKinematic(bool moveToKinematicPos)
{
  NMP_ASSERT(m_refCount == 0);

  for (uint32_t i = 0; i < getNumParts(); ++i)
  {
    PartJoltPhysRagdoll *part = (PartJoltPhysRagdoll*)m_parts[i];
    part->makeKinematic(true, 1.0f, false);
    if (part->m_kinematicBody)
      part->enableBodyCollision(part->m_kinematicBody, true);
    part->enableBodyCollision(part->m_rigidBody, false);
  }

  if (moveToKinematicPos)
  {
    moveAllToKinematicPos();
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::removeFromScene()
{
  NMP_ASSERT(m_refCount == 0);

  m_ragdoll->Activate();

  removeRagdollFromScene();
}
//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::addToScene()
{
  NMP_ASSERT(m_refCount == 0);
  addRagdollToScene();
  PhysicsSceneJoltPhys* joltphys_scene = (PhysicsSceneJoltPhys*)getPhysicsScene();
  const JPH::BodyLockInterfaceLocking& lockinterface = joltphys_scene->m_joltPhysScene->GetBodyLockInterface();
  for (int i = 0; i < getNumParts(); i++)
  {
      PartJoltPhysRagdoll* ragdollpart = (PartJoltPhysRagdoll*)m_parts[i];
      ragdollpart->m_rigidBody = ragdollpart->m_kinematicBody = lockinterface.TryGetBody(m_ragdoll->GetBodyID(i));
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::addRagdollToScene()
{
  if (m_isRagdollAddedToScene)
    return;
  for (uint32_t i = 0; i < getNumParts(); ++i)
  {
    // use the fact that the position of the PhysX bone is offset so that it is at the same
    // location as the joint morpheme joint
    PhysicsRig::Part* part = m_parts[i];
    part->setTransform(part->getTransform());
    part->setVel(part->getVel());
    part->setAngVel(part->getAngVel());
  }

  m_ragdoll->AddToPhysicsSystem(JPH::EActivation::Activate);

  m_isRagdollAddedToScene = true;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::removeRagdollFromScene()
{
  if (!m_isRagdollAddedToScene)
    return;

  m_ragdoll->RemoveFromPhysicsSystem();

  m_isRagdollAddedToScene = false;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::makeDynamic()
{
  if (!m_isRagdollAddedToScene)
    addRagdollToScene();

  for (uint32_t i = 0; i < getNumParts(); ++i)
  {
    PartJoltPhysRagdoll *part = (PartJoltPhysRagdoll*)m_parts[i];
    part->makeKinematic(false, 1.0f, false);
    if (part->m_kinematicBody)
      part->enableBodyCollision(part->m_kinematicBody, false);
    part->enableBodyCollision(part->m_rigidBody, true);
  }

  for (uint32_t i=0; i<getNumJoints(); ++i)
  {
    PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll *joint = (JointJoltPhysRagdoll*)getJoint(i);
    // enable the joint limit
    joint->enableLimit(true);
  }

  // Re-enable gravity.  This will have been disabled by moveAllToKinematicPos if it was previously called.
  if (m_refCount == 0)
  {
    for (uint32_t i = 0; i < getNumParts(); ++i)
    {
      PartJoltPhysRagdoll *part = (PartJoltPhysRagdoll*)m_parts[i];
      // re-enable gravity
      part->m_rigidBody->GetMotionProperties()->SetGravityFactor(1);
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::moveAllToKinematicPos()
{
  JPH::Vec3 delta = nmVector3ToJPHVec3(m_kinematicPose.translation()) - ((PartJoltPhysRagdoll*)getPart(0))->m_rigidBody->GetWorldTransform().GetTranslation();

  // Move the kinematic shape somewhere far.
  JPH::Mat44 kinematicPose = nmMatrix34ToJPHMat44(m_kinematicPose);

  //for (uint32_t i = 0; i < getNumParts(); ++i)
  //{
  //  // move the dynamic part
  //  PartJoltPhysRagdoll *part = (PartJoltPhysRagdoll*)m_parts[i];
  //  JPH::Mat44 transform = part->m_rigidBody->GetWorldTransform();
  //  transform.SetTranslation(transform.GetTranslation() + delta);
  //  getPhysicsSceneJoltPhys()->m_joltPhysScene->GetBodyInterface().
  //      SetPositionAndRotation(part->m_rigidBody->GetID(), transform.GetTranslation(), transform.GetQuaternion(), JPH::EActivation::Activate);
  //  part->m_rigidBody->SetLinearVelocity(JPH::Vec3(0,0,0));
  //  part->m_rigidBody->SetAngularVelocity(JPH::Vec3(0,0,0));
  //  // disable gravity
  //  part->m_rigidBody->GetMotionProperties()->SetGravityFactor(0.0f);
  //  if (part->m_kinematicBody)
  //  {
  //      getPhysicsSceneJoltPhys()->m_joltPhysScene->GetBodyInterface().
  //          SetPositionAndRotation(part->m_kinematicBody->GetID(), kinematicPose.GetTranslation(), kinematicPose.GetQuaternion(), JPH::EActivation::Activate);
  //  }
  //}
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::restoreAllJointDrivesToDefault()
{
  for (uint32_t i = 0 ;i < getNumJoints(); ++i)
  {
    PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll *joint = (JointJoltPhysRagdoll*)m_joints[i];
    joint->setStrength(0.0f);
    joint->setDamping(joint->getMaxDamping());
    joint->setExternalCompliance(1.0f);
    joint->setInternalCompliance(1.0f);
  }

  m_desiredJointProjectionIterations = 0;
  m_desiredJointProjectionLinearTolerance = FLT_MAX;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::disableSleeping()
{
  //float threshold = m_articulation->getSleepThreshold();
  //if (threshold > 0.0f)
  //  m_cachedSleepThreshold = threshold;
  //m_articulation->setSleepThreshold(0.0f);
  //m_articulation->wakeUp();
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::reenableSleeping()
{
  //if (m_ragdoll->getSleepThreshold() == 0.0f)
  //  m_articulation->setSleepThreshold(m_cachedSleepThreshold);
}

//----------------------------------------------------------------------------------------------------------------------
#if !defined(NMP_PLATFORM_SIMD) || defined(NM_HOST_IOS)
void PhysicsRigJoltPhysRagdoll::applyHardKeyframing(
  const NMP::DataBuffer &targetBuffer,
  const NMP::DataBuffer *previousTargetBuffer,
  const NMP::DataBuffer &fallbackBuffer,
  const NMP::Matrix34   &worldRoot,
  const NMP::Matrix34   *previousWorldRoot,
  bool                   enableCollision,
  float                  massMultiplier,
  bool                   enableConstraint,
  float                  dt,
  const PartChooser     &partChooser)
{
  NMP_ASSERT_MSG(m_physicsRigDef != NULL, "No RigDef exists. Check AnimationSets to have physics rigs defined");

  bool hasPrevious = previousTargetBuffer && previousWorldRoot;

  int32_t numParts =  getNumParts();
  NMP::Matrix34* targetTMs = (NMP::Matrix34*)alloca(sizeof(NMP::Matrix34) * (numParts+1));
  NMP::Matrix34* targetTMsOld = 0;
  if (hasPrevious)
  {
    targetTMsOld = (NMP::Matrix34*)alloca(sizeof(NMP::Matrix34) * (numParts+1));
    calculateWorldSpacePartTMsCacheWithVelocity(
      targetTMs,
      targetTMsOld,
      targetBuffer,
      *previousTargetBuffer,
      fallbackBuffer,
      worldRoot,
      *previousWorldRoot);
  }
  else
  {
    calculateWorldSpacePartTMsCache(targetTMs, targetBuffer, fallbackBuffer, worldRoot);
  }

  bool wholeBodyHK = true;
  for (int32_t j = 0; j < numParts; ++j)
  {
    if (!partChooser.usePart(j))
    {
      wholeBodyHK = false;
      continue;
    }

    PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll *part = (PartJoltPhysRagdoll*)m_parts[j];
    part->makeKinematic(true, massMultiplier, enableConstraint);
    part->m_isBeingKeyframed = true;

    // Disable the collision on the dynamic actor.
    part->enableActorCollision(part->m_rigidBody, false);
    // Enable collision on the kinematic actor if desired.
    if (part->m_kinematicActor)
      part->enableActorCollision(part->m_kinematicActor, enableCollision);

    // PhysX applies the velocity before the position update (I think) - so if we move to the target
    // position, we have to set the velocity to zero, otherwise the final position is actually
    // position+velocity*dt, which will be a whole frame's offset. This causes big problems with
    // split body physics, as the other parts are being constrained to a part that has zero
    // velocity, but it keeps moving! When we have the velocity data set the position to
    // targetPosition-velocity*dt, and set the velocity as well. This makes sure that the velocity
    // is passed correctly across the interface when there's split body physics.
    //
    // TODO Note that this relies on a fixed dt, since we assume that the time since the last
    // transforms is the same as the time of the upcoming step...
    if (dt > 0.0f && hasPrevious)
    {
      // Use the fact that the position of the PhysX part is offset so that it is at the same
      // location as the morpheme joint
      NMP::Vector3 offset = part->getCOMPosition();
      NMP::Matrix34 offsetTM(NMP::Matrix34::kIdentity), invOffsetTM(NMP::Matrix34::kIdentity);
      offsetTM.translation() = -offset;
      invOffsetTM.translation() = offset;

      // calculate the motion of the target
      NMP::Matrix34 invTargetTMOld(targetTMsOld[j+1]); invTargetTMOld.invertFast();
      // The following pre- and post-multiplication converts diffTM into the actual motion TM
      // centered at the COM.
      NMP::Matrix34 targetMotionTM = invOffsetTM * invTargetTMOld * targetTMs[j+1] * offsetTM;

      NMP::Vector3 targetVel = targetMotionTM.translation() / dt;
      NMP::Vector3 targetAngVel = targetMotionTM.toQuat().toRotationVector(false) / dt;

      // set the dynamic part TMs
      part->moveTo(targetTMsOld[j+1], false);

      // override the kinematic part so that it's where the dynamic parts will be
      if (part->isKinematic() && part->getKinematicActor())
        part->getKinematicActor()->setKinematicTarget(nmMatrix34ToPxTransform(targetTMs[j+1]));

      part->setVel(targetVel);
      part->setAngVel(targetAngVel);
    }
    else
    {
      part->moveTo(targetTMs[j+1], false);
      part->setVel(NMP::Vector3::InitZero);
      part->setAngVel(NMP::Vector3::InitZero);
    }

    part->recalcNextVel();
  }

  if (wholeBodyHK)
  {
    removeArticulationFromScene();
  }
  else
  {
    // Go through all the joints and disable joint limits when both parts are HK. This is undone by
    // the call to enable the joint limit in makeDynamic
    for (uint32_t i = 0 ; i < getNumJoints() ; ++i)
    {
      PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll *joint = (JointJoltPhysRagdoll*)getJoint(i);
      const PhysicsJointDef* jointDef = m_physicsRigDef->m_joints[i];
      uint32_t i1 = jointDef->m_parentPartIndex;

      if (getPart(i1)->isKinematic())
      {
        uint32_t i2 = jointDef->m_childPartIndex;

        if (getPart(i2)->isKinematic())
        {
          // disable the joint limit
          joint->enableLimit(false);
          // Decrease the compliance so that split body interactions work much better. Better than
          // setting the damping to a large value because that tends to stop the joints reaching their
          // targets.
          // Note that setting this to a smaller value than about 0.1 results in the cm rig
          // exploding...
          joint->setExternalCompliance(0.1f);
          joint->setInternalCompliance(0.1f);
        }
      }
    }
  }
}

#else
//----------------------------------------------------------------------------------------------------------------------
// NMP_PLATFORM_SIMD
void PhysicsRigJoltPhysRagdoll::applyHardKeyframing(
  const NMP::DataBuffer &targetBuffer,
  const NMP::DataBuffer *previousTargetBuffer,
  const NMP::DataBuffer &fallbackBuffer,
  const NMP::Matrix34   &worldRoot,
  const NMP::Matrix34   *previousWorldRoot,
  bool                   enableCollision,
  float                  massMultiplier,
  bool                   enableConstraint,
  float                  dt,
  const PartChooser     &partChooser)
{
  NMP_ASSERT_MSG(m_physicsRigDef != NULL, "No RigDef exists. Check AnimationSets to have physics rigs defined");

  bool hasPrevious = previousTargetBuffer && previousWorldRoot;

  int32_t numParts = getNumParts();
  NMP::vpu::Matrix* targetTMs = (NMP::vpu::Matrix*)alloca(sizeof(NMP::vpu::Matrix) * (numParts+1));
  NMP::vpu::Matrix* targetTMsOld = 0;
  if (hasPrevious)
  {
    targetTMsOld = (NMP::vpu::Matrix*)alloca(sizeof(NMP::vpu::Matrix) * (numParts+1));
    calculateWorldSpacePartTMsCacheWithVelocity(
      targetTMs,
      targetTMsOld,
      targetBuffer,
      *previousTargetBuffer,
      fallbackBuffer,
      worldRoot,
      *previousWorldRoot);
  }
  else
  {
    calculateWorldSpacePartTMsCache(targetTMs, targetBuffer, fallbackBuffer, worldRoot);
  }
 
  bool wholeBodyHK = true;
  for (int32_t j = 0; j < numParts; ++j)
  {   
    if (!partChooser.usePart(j))
    {
      wholeBodyHK = false;
      continue;
    }

    PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll *part = (PartJoltPhysRagdoll*)m_parts[j];
    part->makeKinematic(true, massMultiplier, enableConstraint);
    part->m_isBeingKeyframed = true;

    // Disable the collision on the dynamic actor.
    part->enableBodyCollision(part->m_rigidBody, false);
    // Enable collision on the kinematic actor if desired.
    if (part->m_kinematicBody)
      part->enableBodyCollision(part->m_kinematicBody, enableCollision);

    // PhysX applies the velocity before the position update (I think) - so if we move to the target
    // position, we have to set the velocity to zero, otherwise the final position is actually
    // position+velocity*dt, which will be a whole frame's offset. This causes big problems with
    // split body physics, as the other parts are being constrained to a part that has zero
    // velocity, but it keeps moving! When we have the velocity data set the position to
    // targetPosition-velocity*dt, and set the velocity as well. This makes sure that the velocity
    // is passed correctly across the interface when there's split body physics.
    //
    // TODO Note that this relies on a fixed dt, since we assume that the time since the last
    // transforms is the same as the time of the upcoming step...
    if (dt > 0.0f && hasPrevious)
    {
      // Use the fact that the position of the PhysX part is offset so that it is at the same
      // location as the morpheme joint 
      NMP::Matrix34 currentTM = part->getTransform();
      NMP::Vector3 offset = part->getCOMPosition();
      NMP::Matrix34 offsetTM(NMP::Matrix34::kIdentity), invOffsetTM(NMP::Matrix34::kIdentity);
      offsetTM.translation() = -offset;
      invOffsetTM.translation() = offset;

      // calculate the motion of the target
      NMP::Matrix34 invTargetTMOld(M34vpu(targetTMsOld[j+1])); invTargetTMOld.invertFast();
      // The following pre- and post-multiplication converts diffTM into the actual motion TM
      // centered at the COM.
      NMP::Matrix34 targetMotionTM = invOffsetTM * invTargetTMOld * M34vpu(targetTMs[j+1]) * offsetTM;

      NMP::Vector3 targetVel = targetMotionTM.translation() / dt;
      NMP::Vector3 targetAngVel = targetMotionTM.toQuat().toRotationVector(false) / dt;

      // set the dynamic part TMs
      part->moveTo(M34vpu(targetTMsOld[j+1]), false);
      part->setVel(targetVel);
      part->setAngVel(targetAngVel);

      // override the kinematic part so that it's where the dynamic parts will be
      if (part->isKinematic() && part->getKinematicBody())
      {
          JPH::Mat44 transformtarget = nmMatrix34ToJPHMat44(M34vpu(targetTMs[j + 1]));
          const float deltatime = 1.0 / 60.f; //fix
          part->getKinematicBody()->MoveKinematic(transformtarget.GetTranslation(), transformtarget.GetQuaternion(), deltatime);
      }

    }
    else
    {
      part->moveTo(M34vpu(targetTMs[j+1]), false);
      part->setVel(NMP::Vector3::InitZero);
      part->setAngVel(NMP::Vector3::InitZero);
    }

    part->recalcNextVel();
  }

  if (wholeBodyHK)
  {
    removeRagdollFromScene();
  }
  else
  {
    // Go through all the joints and disable joint limits when both parts are HK. This is undone by
    // the call to enable the joint limit in makeDynamic
    for (uint32_t i = 0 ; i < getNumJoints() ; ++i)
    {
      PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll *joint = (JointJoltPhysRagdoll*)getJoint(i);
      const PhysicsJointDef* jointDef = m_physicsRigDef->m_joints[i];

      uint32_t i1 = jointDef->m_parentPartIndex;

      if (getPart(i1)->isKinematic())
      {
        uint32_t i2 = jointDef->m_childPartIndex;

        if (getPart(i2)->isKinematic())
        {
          // disable the joint limit
          joint->enableLimit(false);
          // Decrease the compliance so that split body interactions work much better. Better than
          // setting the damping to a large value because that tends to stop the joints reaching their
          // targets.
          // Note that setting this to a smaller value than about 0.1 results in the cm rig
          // exploding...
          joint->setExternalCompliance(0.1f);
          joint->setInternalCompliance(0.1f);
        }
      }
    }
  }
}
#endif

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::applySoftKeyframing(
    const NMP::DataBuffer &targetBuffer,
    const NMP::DataBuffer &targetBufferOld,
    const NMP::DataBuffer &fallbackBuffer,
    const NMP::Matrix34   &worldRoot,
    const NMP::Matrix34   &worldRootOld,
    bool                   enableCollision,
    bool                   enableJointLimits,
    bool                   preserveMomentum,
    float                  externalJointCompliance,
    float                  gravityCompensationFrac,
    float                  dt,
    float                  weight,
    float                  maxAccel,
    float                  maxAngAccel,
    const PartChooser     &partChooser)
{
  NMP_ASSERT_MSG(m_physicsRigDef != NULL, "No RigDef exists. Check AnimationSets to have physics rigs defined");

  // Do a first pass to set the properties that need to be set even if the weight is zero
  int32_t numParts = getNumParts();
  for (int32_t i = 0; i < numParts; ++i)
  {
    if (!partChooser.usePart(i))
    {
      continue;
    }

    PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll *part = (PartJoltPhysRagdoll*)m_parts[i];

    part->makeKinematic(false, 1.0f, false);
    part->m_isBeingKeyframed = true;
    // Enable the collision on the dynamic actor if desired.
    part->enableBodyCollision(part->m_rigidBody, enableCollision);
    // Disable collision on the kinematic actor.
    if (part->m_kinematicBody)
      part->enableBodyCollision(part->m_kinematicBody, false);

    // Set the external compliance on the associated joint (joint index = part index - 1, so there's
    // no joint for the root part).
    if (i != 0)
    {
      PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll *joint = (JointJoltPhysRagdoll*)getJoint(i-1);
      joint->setExternalCompliance(externalJointCompliance);
    }

    // Enable/disable joint limits on the parent joint, but only if the parent part is also soft
    // keyframed by this node.
    int32_t parentPartIndex = part->getParentPartIndex();
    if (parentPartIndex >= 0)
    {
      if (partChooser.usePart(parentPartIndex))
      {
        int32_t parentJointIndex = i - 1;
        if (parentJointIndex >= 0)
        {
          PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll* joint = (JointJoltPhysRagdoll*)getJoint(parentJointIndex);
          joint->enableLimit(enableJointLimits);
        }
      }
    }
  }

#ifdef DEBUG_SK
  static NMP::Vector3 prevVelTargets[64];
  static NMP::Vector3 prevAngVelTargets[64];
#endif

#if !defined(NMP_PLATFORM_SIMD) || defined(NM_HOST_IOS)
  NMP::Matrix34* targetTMs = (NMP::Matrix34*)alloca(sizeof(NMP::Matrix34) * (numParts+1));
  NMP::Matrix34* targetTMsOld = (NMP::Matrix34*)alloca(sizeof(NMP::Matrix34) * (numParts+1));
#else
  NMP::vpu::Matrix* targetTMs = (NMP::vpu::Matrix*)alloca(sizeof(NMP::vpu::Matrix) * (numParts+1));
  NMP::vpu::Matrix* targetTMsOld = (NMP::vpu::Matrix*)alloca(sizeof(NMP::vpu::Matrix) * (numParts+1));
#endif

  calculateWorldSpacePartTMsCacheWithVelocity( targetTMs,
    targetTMsOld,
    targetBuffer,
    targetBufferOld,
    fallbackBuffer,
    worldRoot,
                                               worldRootOld );

  // If the weight is effectively zero then we are only concerned with deviation
  bool weightIsEffectivelyZero = (weight <= 0.0000001f);
 
  // Adjust these for weight
  maxAccel *= weight;
  maxAngAccel *= weight;

  // Calculate this once for use in the loop
  NMP::Vector3 gravityDeltaVel = getPhysicsSceneJoltPhys()->getGravity() * (weight * dt * gravityCompensationFrac);

  // The variables required if we are preserving momentum.
  NMP::Vector3 originalCOMVel(NMP::Vector3::InitZero);
  NMP::Vector3 newCOMVel(NMP::Vector3::InitZero);
  float totalPreservedMass = 0.0f;

  // Iterate over the parts calculating deviation, velocity and angular velocity for each one.
  for (int32_t i = 0; i < numParts; ++i)
  {
    if (!partChooser.usePart(i))
    {
      continue;
    }

    PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll *part = (PartJoltPhysRagdoll*)m_parts[i];

    if (preserveMomentum)
    {
      originalCOMVel += part->getVel() * part->getMass();
      totalPreservedMass += part->getMass();
    }

    // Use the fact that the position of the PhysX bone is offset so that it is at the same
    // location as the morpheme joint 
    if (dt > 0.0f)
    {
      NMP::Matrix34 currentTM = part->getTransform();

      NMP::Vector3 offset = part->getCOMPosition();
      NMP::Matrix34 offsetTM(NMP::Matrix34::kIdentity), invOffsetTM(NMP::Matrix34::kIdentity);
      offsetTM.translation() = -offset;
      invOffsetTM.translation() = offset;

      // Calculate the motion to go from current to new
      NMP::Matrix34 invCurrentTM(currentTM); invCurrentTM.invertFast();
      // The following pre- and post-multiplication converts diffTM into the actual motion TM
      // centered at the COM. 
#if !defined(NMP_PLATFORM_SIMD) || defined(NM_HOST_IOS)
      NMP::Matrix34 motionTM = invOffsetTM * invCurrentTM * targetTMs[i+1] * offsetTM;
#else
      NMP::Matrix34 motionTM = invOffsetTM * invCurrentTM * M34vpu(targetTMs[i+1]) * offsetTM;
#endif

      // Store the distance/angle error
      part->m_SKDeviation = motionTM.translation().magnitude();
      part->m_SKDeviationAngle = motionTM.toRotationVector().magnitude();

      // If the weight is effectively zero then the velocity and angular velocity
      // for each part will be unchanged. Only calculate deviation here.
      if (weightIsEffectivelyZero)
      {
        continue;
      }

      // Calculate the motion of the target itself
#if !defined(NMP_PLATFORM_SIMD) || defined(NM_HOST_IOS)
      NMP::Matrix34 invTargetTMOld(targetTMsOld[i+1]); invTargetTMOld.invertFast();
      NMP::Matrix34 targetMotionTM = invOffsetTM * invTargetTMOld * targetTMs[i+1] * offsetTM;
#else
      NMP::Matrix34 invTargetTMOld(M34vpu(targetTMsOld[i+1])); invTargetTMOld.invertFast();
      NMP::Matrix34 targetMotionTM = invOffsetTM * invTargetTMOld * M34vpu(targetTMs[i+1]) * offsetTM;
#endif

      // This is where the velocity multiplier can be applied
      NMP::Vector3 translation = motionTM.translation();
      NMP::Vector3 rotation = motionTM.toQuat().toRotationVector(false);

      NMP::Vector3 newVel = translation / dt;
      NMP::Vector3 curVel = part->getVel();
      if (maxAccel >= 0.0f)
      {
        // Prevent overshoot by calculating the max speed we can have in the direction towards
        // the target given that we cannot decelerate faster than maxAccel
        NMP::Vector3 deltaVel = newVel - curVel;
        NMP::Vector3 targetVel = targetMotionTM.translation() / dt;
        NMP::Vector3 translationDir = translation;
        float distToTarget = translationDir.normaliseGetLength();
        float curVelAlongTranslation = curVel.dot(translationDir);
        float targetVelAlongTranslation = targetVel.dot(translationDir);

        if (curVelAlongTranslation > targetVelAlongTranslation)
        {
          float timeToCatchUp = distToTarget / (curVelAlongTranslation - targetVelAlongTranslation);
          float maxCurVelAlongTranslation = targetVelAlongTranslation + timeToCatchUp * maxAccel;
          if (curVelAlongTranslation > maxCurVelAlongTranslation)
          {
            // Replace the old component along the translation with the new max value
            newVel += translationDir * (maxCurVelAlongTranslation - newVel.dot(translationDir));
            deltaVel = newVel - curVel;
          }
        }

        // Clamp the acceleration
        float deltaVelMag = deltaVel.magnitude();
        if (deltaVelMag > maxAccel * dt)
          deltaVel *= maxAccel * dt / deltaVelMag;

        // Apply gravity compensation
        deltaVel -= gravityDeltaVel;
        newVel = curVel + deltaVel;
      }
      part->setVel(newVel);

      if (preserveMomentum)
        newCOMVel += newVel * part->getMass();

      NMP::Vector3 newAngVel = rotation / dt;
      NMP::Vector3 curAngVel = part->getAngVel();
      if (maxAngAccel >= 0.0f)
      {
        // Limit the max angular velocity target - this is just a straight conversion of the linear velocity code, so I
        // _think_ it's "correct"!
        NMP::Vector3 deltaAngVel = newAngVel - curAngVel;
        NMP::Vector3 targetAngVel = targetMotionTM.toQuat().toRotationVector(false) / dt;
        NMP::Vector3 rotationDir = rotation;
        float distToTarget = rotationDir.normaliseGetLength();
        float curAngVelAlongRotation = curAngVel.dot(rotationDir);
        float targetAngVelAlongRotation = targetAngVel.dot(rotationDir);

        if (curAngVelAlongRotation > targetAngVelAlongRotation)
        {
          float timeToCatchUp = distToTarget / (curAngVelAlongRotation - targetAngVelAlongRotation);
          float maxCurAngVelAlongRotation = targetAngVelAlongRotation + timeToCatchUp * maxAngAccel;

          if (curAngVelAlongRotation > maxCurAngVelAlongRotation)
          {
            // Replace the old component along the translation with the new max value
            newAngVel += rotationDir * (maxCurAngVelAlongRotation - newAngVel.dot(rotationDir));
            deltaAngVel = newAngVel - curAngVel;
          }
        }

        // Clamp the acceleration
        float deltaAngVelMag = deltaAngVel.magnitude();
        if (deltaAngVelMag > maxAngAccel * dt)
          deltaAngVel *= maxAngAccel * dt / deltaAngVelMag;
        newAngVel = curAngVel + deltaAngVel;
      }
      part->setAngVel(newAngVel);

#ifdef DEBUG_SK
      if (i == 0)
      {
        printf("VelTarget = (%6.2f %6.2f %6.2f) CurVel = (%6.2f %6.2f %6.2f) next VelTarget = (%6.2f %6.2f %6.2f)     dt = %6.4f\n",
          prevVelTargets[i].x, prevVelTargets[i].y, prevVelTargets[i].z, 
          curVel.x, curVel.y, curVel.z,
          newVel.x, newVel.y, newVel.z,
          dt);
    }
      prevVelTargets[i] = newVel;
      prevAngVelTargets[i] = newAngVel;
#endif

  }
  }

  if (preserveMomentum && weightIsEffectivelyZero == false)
  {
    originalCOMVel /= totalPreservedMass;
    newCOMVel /= totalPreservedMass;
    NMP::Vector3 correctionVel = originalCOMVel - newCOMVel;
    for (uint32_t i = 0; i < getNumParts(); ++i)
    {
      if (!partChooser.usePart(i))
        continue;
      PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll *part = (PartJoltPhysRagdoll*)m_parts[i];
      NMP::Vector3 partVel = part->getVel();
      part->setVel(partVel + correctionVel);
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------- 
void PhysicsRigJoltPhysRagdoll::applyActiveAnimation(uint32_t jointIndex, const NMP::Quat &targetQuat, bool makeChildDynamic) 
{ 
  NMP_ASSERT(jointIndex < getNumJoints());  
  JPH::TwoBodyConstraint *joint = ((JointJoltPhysRagdoll*)m_joints[jointIndex])->m_jointInternal;  
  const PhysicsJointDef* jointDef = m_physicsRigDef->m_joints[jointIndex];
  if (makeChildDynamic)  
  {   
    PartJoltPhysRagdoll *childPart = (PartJoltPhysRagdoll*)m_parts[jointDef->m_childPartIndex];     
    childPart->makeKinematic(false, 1.0f, false);
    childPart->m_isBeingKeyframed = false;
  }   
  // Don't force either of the parts to have collision - no way we could know which one _should_ have collision if it's
  // disabled elsewhere.
  JPH::SwingTwistConstraint* st_joint = dynamic_cast<JPH::SwingTwistConstraint*>(joint);
  if(st_joint)
      st_joint->SetTargetOrientationCS(nmQuatToJPHQuat(targetQuat));
  return; 
}

//----------------------------------------------------------------------------------------------------------------------
// Drives the joints to the targets given by the input animation buffer.
void PhysicsRigJoltPhysRagdoll::applyActiveAnimation(
    const NMP::DataBuffer& targetBuffer,
    const NMP::DataBuffer& fallbackBuffer,
    float                  strengthMultiplier,
    float                  dampingMultiplier,
    float                  internalCompliance,
    float                  externalCompliance,
    bool                   enableJointLimits,
    const JointChooser    &jointChooser,
    float                  limitClampFraction)
{
  NMP_ASSERT_MSG(m_physicsRigDef != NULL, "No RigDef exists. Check AnimationSets to have physics rigs defined");

  for (uint32_t i = 0; i < getNumJoints(); ++i)
  {
    if (!jointChooser.useJoint(i))
      continue;
  
    PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll *joint = (JointJoltPhysRagdoll*)m_joints[i];
    JPH::SwingTwistConstraint* joltjoint = (JPH::SwingTwistConstraint*)joint->m_jointInternal;
    const PhysicsSixDOFJointDef* jointDef = static_cast<const PhysicsSixDOFJointDef*>(m_physicsRigDef->m_joints[i]);
    PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll *childPart = (PartJoltPhysRagdoll*)m_parts[jointDef->m_childPartIndex];
    childPart->makeKinematic(false, 1.0f, false);
    childPart->m_isBeingKeyframed = false;
  
    // Don't force either of the bones to have collision - no way we could know which one _should_
    // have collision if it's disabled elsewhere.
  
    float newStrength = joint->getMaxStrength() * strengthMultiplier;
    float newDamping  = joint->getMaxDamping() * dampingMultiplier;
    joint->setStrength(newStrength);
    joint->setDamping(newDamping);
    joint->setInternalCompliance(internalCompliance);
    joint->setExternalCompliance(externalCompliance);
    joint->enableLimit(enableJointLimits);
  
    if (strengthMultiplier < 0.0000001f)
      continue;
  
    NMP::Quat curQ;
    getQuatFromTransformBuffer(jointDef->m_childPartIndex, targetBuffer, fallbackBuffer, curQ);
  
    // q is the rotation of the child relative to the parent (in parent space).
    // We need to account for the offset axes in the joint.
  
    // Get the local joint axes in each frame as l0, l1
    NMP::Quat l0 = jointDef->m_parentFrameQuat;
    NMP::Quat l1 = jointDef->m_childFrameQuat;
  
    // Now "assuming" the parent is at the origin (since we already have the relative rotation q)
    // we want to calculate rot, the relative rotation of the child local frame from the parent local frame
    NMP::Quat l0Inv = ~l0;
  
    // Target orientations outside the limits cause oscillations when physical limits are enabled
    if (limitClampFraction >= 0.0f)
    {
      joint->clampToLimits(curQ, limitClampFraction, NULL);
    }
    NMP::Quat curFrameQ = l0Inv * curQ * l1;
    JPH::Vec3 pos = joltjoint->GetBody1()->GetCenterOfMassTransform() * joltjoint->GetLocalSpacePosition1();
    NMP::Matrix34 transform = NMP::Matrix34Identity();
    transform.setTranslation(MR::nmJPHVec3ToVector3(pos));

    MR_DEBUG_DRAW_SPHERE_GLOBAL(transform, 2.f, NMP::Colour::DARK_GREEN);

    joltjoint->SetTargetOrientationCS(nmQuatToJPHQuat(curFrameQ));
    
  }
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Quat PhysicsRigJoltPhysRagdoll::getJointQuat(uint32_t jointIndex) 
{
  NMP_ASSERT(jointIndex < getNumJoints());
  JointJoltPhysRagdoll *joint = (JointJoltPhysRagdoll*)m_joints[jointIndex];
  const PhysicsJointDef* jointDef = m_physicsRigDef->m_joints[jointIndex];
  if (joint->m_rotationDirty)
  {
    NMP::Quat part1Quat = (getPartJoltPhysRagdoll(jointDef->m_parentPartIndex))->getQuaternion();
    NMP::Quat part2Quat = (getPartJoltPhysRagdoll(jointDef->m_childPartIndex))->getQuaternion();
    joint->m_actualRotation = ~(part1Quat * jointDef->m_parentFrameQuat) * (part2Quat * jointDef->m_childFrameQuat); 
    joint->m_rotationDirty = false;
  }
  return joint->m_actualRotation;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::setCollisionGroupsToActivate(const int *collisionGroupIndices, int numCollisionGroupIndices)
{
  NMP_ASSERT(numCollisionGroupIndices <= m_maxCollisionGroupIndices);
  m_numCollisionGroupIndices = NMP::minimum(m_maxCollisionGroupIndices, numCollisionGroupIndices);
  
  for (int i = 0 ; i < m_numCollisionGroupIndices ; ++i)
  {
    m_collisionGroupIndicesToActivate[i] = collisionGroupIndices[i];
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::setExplosionHandler(RagdollExplosionHandler* handler)
{
  s_explosionHandler = handler;
}

} // namespace MR

//----------------------------------------------------------------------------------------------------------------------
