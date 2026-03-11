// Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
// Not to be copied, adapted, modified, used, distributed, sold,
// licensed or commercially exploited in any manner without the
// written consent of NaturalMotion.
//
// All non public elements of this software are the confidential
// information of NaturalMotion and may not be disclosed to any
// person nor used for any purpose not expressly approved by
// NaturalMotion in writing.

//----------------------------------------------------------------------------------------------------------------------
#include <cstdio>
#include "NMPlatform/NMFastHeapAllocator.h"
#include "NMPlatform/NMPlatform.h"
#include "NMPlatform/NMProfiler.h"
#include "NMGeomUtils/NMGeomUtils.h"
#include "morpheme/mrAttribData.h"
#include "morpheme/mrBlendOps.h"
#include "morpheme/mrRig.h"
#include "physics/mrPhysicsRigDef.h"
#include "physics/mrPhysicsSerialisationBuffer.h"
#include "physics/mrPhysicsAttribData.h"
#include "physics/JoltPhys/mrJoltPhys.h"
#include "physics/JoltPhys/mrPhysicsDriverDataJoltPhys.h"
#include "physics/JoltPhys/mrPhysicsRigJoltPhysJointed.h"
#include "physics/JoltPhys/mrPhysicsSceneJoltPhys.h"
#include "sharedDefines/mPhysicsDebugInterface.h"

//----------------------------------------------------------------------------------------------------------------------

#define DISABLE_JOINTSx

namespace MR
{

// this is like a limit "skin width" - small and rather arbitrary.
    PhysicsRigJoltPhysJointed;

// This limit isn't nice, but PhysX can't handle 0 swing ranges. 
static const float s_minSwingLimit = NMP::degreesToRadians(0.01f);

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysJointed::PhysicsRigJoltPhysJointed(PhysicsSceneJoltPhys*physicsScene)
{
  m_physicsScene = physicsScene;
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Memory::Format PhysicsRigJoltPhysJointed::getMemoryRequirements(PhysicsRigDef* physicsRigDef)
{
  uint32_t numBones = physicsRigDef->getNumParts();
  uint32_t numJoints = physicsRigDef->getNumJoints();
  uint32_t numMaterials = physicsRigDef->getNumMaterials();

  NMP::Memory::Format result(sizeof(PhysicsRigJoltPhysJointed), NMP_VECTOR_ALIGNMENT);

  // Space for the part pointers
  result += NMP::Memory::Format(sizeof(PhysicsRigJoltPhysJointed::PartJoltPhysJointed*) * numBones, NMP_NATURAL_TYPE_ALIGNMENT);

  // Space for the joint pointers
  result += NMP::Memory::Format(sizeof(PhysicsRigJoltPhysJointed::JointJoltPhysJointed*) * numJoints, NMP_NATURAL_TYPE_ALIGNMENT);

  // Space for the parts
  result += NMP::Memory::Format(NMP::Memory::align(
    sizeof(PhysicsRigJoltPhysJointed::PartJoltPhysJointed), NMP_NATURAL_TYPE_ALIGNMENT) * numBones, NMP_NATURAL_TYPE_ALIGNMENT);

  // Space for the joints
  result += NMP::Memory::Format(NMP::Memory::align(
    sizeof(PhysicsRigJoltPhysJointed::JointJoltPhysJointed), NMP_NATURAL_TYPE_ALIGNMENT) * numJoints, NMP_NATURAL_TYPE_ALIGNMENT);

  return result;
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysJointed::PartJoltPhysJointed::PartJoltPhysJointed()
{
  m_userData = NULL;
  m_parentPartIndex = -1;
  m_dirtyFlags = 0;
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysJointed::PartJoltPhysJointed::~PartJoltPhysJointed()
{
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysJointed::PartJoltPhysJointed::PartJoltPhysJointed(
  const PhysicsRigJoltPhysJointed::PartJoltPhysJointed& other)
{
  *this = other;
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysJointed::PartJoltPhysJointed& PhysicsRigJoltPhysJointed::PartJoltPhysJointed::operator=(
  const PhysicsRigJoltPhysJointed::PartJoltPhysJointed& other)
{
  if (this == &other)
    return *this;

  m_parentPartIndex = other.m_parentPartIndex;
  return *this;
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Matrix34 PhysicsRigJoltPhysJointed::PartJoltPhysJointed::getCOMOffsetLocal() const
{
  // We don't expose methods that modifies the local com offset, so just get the value from the cache.
  return m_cache.COMOffsetLocal;
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Vector3 PhysicsRigJoltPhysJointed::PartJoltPhysJointed::getCOMPosition() const
{
  return m_cache.COMPosition;
}

//----------------------------------------------------------------------------------------------------------------------
float PhysicsRigJoltPhysJointed::PartJoltPhysJointed::getMass() const
{
  // We don't expose methods that modifies the mass, so just get the value from the cache.
  return m_cache.mass;
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Vector3 PhysicsRigJoltPhysJointed::PartJoltPhysJointed::getPosition() const
{
  return getTransform().translation();
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Vector3 PhysicsRigJoltPhysJointed::PartJoltPhysJointed::getVel() const
{
    return NMP::Vector3(NMP::Vector3::InitZero);
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Vector3 PhysicsRigJoltPhysJointed::PartJoltPhysJointed::getAngVel() const
{
    return NMP::Vector3(NMP::Vector3::InitZero);
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Vector3 PhysicsRigJoltPhysJointed::PartJoltPhysJointed::getVelocityAtPoint(const NMP::Vector3& point) const
{
  NMP::Vector3 rpoint = point - getCOMPosition();
  return getVel() + NMP::vCross(getAngVel(), rpoint);
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Vector3 PhysicsRigJoltPhysJointed::PartJoltPhysJointed::getAngularMomentum() const
{
  NMP_ASSERT_FAIL();
  return NMP::Vector3(0,0,0);
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Vector3 PhysicsRigJoltPhysJointed::PartJoltPhysJointed::getLinearMomentum() const
{
  // there appears to be a bug in the physX implementation so rather than call
  // return nmNxVec3ToVector3(m_actor->getLinearMomentum());
  // for now we'll do this
  return getVel() * getMass();
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::PartJoltPhysJointed::generateCachedValues()
{
  updateCOMPosition();

  m_dirtyFlags = 0;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::PartJoltPhysJointed::applyModifiedValues()
{
}

//----------------------------------------------------------------------------------------------------------------------
// inertia and summations thereof
//----------------------------------------------------------------------------------------------------------------------
NMP::Vector3 PhysicsRigJoltPhysJointed::PartJoltPhysJointed::getMassSpaceInertiaTensor() const
{
  return NMP::Vector3(NMP::Vector3::InitZero);
}

//----------------------------------------------------------------------------------------------------------------------
// returns the mass moment of inertia in the top 3x3 components
// along with the com position in the translation component
NMP::Matrix34 PhysicsRigJoltPhysJointed::PartJoltPhysJointed::getGlobalInertiaTensor() const
{
    return NMP::Matrix34();
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Quat PhysicsRigJoltPhysJointed::PartJoltPhysJointed::getQuaternion() const
{
  return getTransform().toQuat();
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Matrix34 PhysicsRigJoltPhysJointed::PartJoltPhysJointed::getTransform() const
{
  return m_cache.globalPose;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::PartJoltPhysJointed::updateCOMPosition()
{
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::PartJoltPhysJointed::moveTo(const NMP::Matrix34& tm)
{
  m_dirtyFlags |= kDirty_KinematicTarget;
  m_cache.kinematicTarget = tm;

  updateCOMPosition();
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::PartJoltPhysJointed::setPosition(const NMP::Vector3& p)
{
  m_dirtyFlags |= kDirty_GlobalPose;
  m_cache.globalPose.translation() = p;

  updateCOMPosition();
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::PartJoltPhysJointed::setQuaternion(const NMP::Quat& q)
{
  m_dirtyFlags |= kDirty_GlobalPose;
  m_cache.globalPose.fromQuat(q);

  updateCOMPosition();
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::PartJoltPhysJointed::setVel(const NMP::Vector3& v)
{
  m_dirtyFlags |= kDirty_LinearVel;
  m_cache.linearVel = v;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::PartJoltPhysJointed::setAngVel(const NMP::Vector3& angVel)
{
  m_dirtyFlags |= kDirty_AngularVel;
  m_cache.angularVel = angVel;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::PartJoltPhysJointed::setTransform(const NMP::Matrix34& tm)
{
  m_dirtyFlags |= kDirty_GlobalPose;
  m_cache.globalPose = tm;

  updateCOMPosition();
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::PartJoltPhysJointed::makeKinematic(bool kinematic, float NMP_UNUSED(massMultiplier), bool NMP_UNUSED(enableConstraint))
{
  if (!kinematic)
    m_isBeingKeyframed = false;

  if (kinematic == isKinematic())
    return;

  m_dirtyFlags |= kDirty_BodyFlags;
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysJointed::PartJoltPhysJointed::isKinematic() const
{
  return false;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::PartJoltPhysJointed::enableCollision(bool enable)
{
  m_dirtyFlags |= kDirty_Collision;
  m_cache.collisionOn = enable;
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysJointed::PartJoltPhysJointed::getCollisionEnabled() const
{
    return false;
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysJointed::PartJoltPhysJointed::storeState(MR::PhysicsSerialisationBuffer& savedState)
{
  savedState.addValue(getPosition());
  savedState.addValue(getQuaternion());
  savedState.addValue(getVel());
  savedState.addValue(getAngVel());
  savedState.addValue(getCachedData());
  return true;
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysJointed::PartJoltPhysJointed::restoreState(MR::PhysicsSerialisationBuffer& savedState)
{
  setPosition(savedState.getValue<NMP::Vector3>());
  setQuaternion(savedState.getValue<NMP::Quat>());
  setVel(savedState.getValue<NMP::Vector3>());
  setAngVel(savedState.getValue<NMP::Vector3>());
  setCachedData(savedState.getValue<MR::PhysicsRigJoltPhysJointed::PartJoltPhysJointed::CachedData>());
  return true;
}

#if defined(MR_OUTPUT_DEBUGGING)

//----------------------------------------------------------------------------------------------------------------------
uint32_t PhysicsRigJoltPhysJointed::PartJoltPhysJointed::serializeTxPersistentData(
  uint16_t nameToken, 
  uint32_t objectID, 
  void* outputBuffer, 
  uint32_t NMP_USED_FOR_ASSERTS(outputBufferSize)) const
{
  return 0;
}

//----------------------------------------------------------------------------------------------------------------------
uint32_t PhysicsRigJoltPhysJointed::PartJoltPhysJointed::serializeTxFrameData(
  void* outputBuffer, 
  uint32_t NMP_USED_FOR_ASSERTS(outputBufferSize)) const
{
  uint32_t dataSize = sizeof(PhysicsPartFrameData);

  if (outputBuffer != 0)
  {
    NMP_ASSERT(outputBufferSize >= dataSize);
    PhysicsPartFrameData *partFrameData = (PhysicsPartFrameData *)outputBuffer;
    partFrameData->m_globalPose = getTransform();
    NMP::netEndianSwap(partFrameData->m_globalPose);
  }

  return dataSize;
}
#endif

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysJointed::JointJoltPhysJointed::JointJoltPhysJointed(const PhysicsSixDOFJointDef* const def)
: JointJoltPhys(def)
{}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::JointJoltPhysJointed::generateCachedValues()
{
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::JointJoltPhysJointed::applyModifiedValues()
{
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysJointed::JointJoltPhysJointed::storeState(MR::PhysicsSerialisationBuffer& savedState)
{
  (void) savedState;
  return true;
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysJointed::JointJoltPhysJointed::restoreState(MR::PhysicsSerialisationBuffer& savedState)
{
  (void) savedState;
  return true;
}

#if defined(MR_OUTPUT_DEBUGGING)
//----------------------------------------------------------------------------------------------------------------------
uint32_t PhysicsRigJoltPhysJointed::JointJoltPhysJointed::serializeTxPersistentData(
  const MR::PhysicsJointDef* jointDef,
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

    persistentData->m_nameToken = stringToken;

    PhysicsSixDOFJointPersistentData::endianSwap(persistentData);
  }

  return dataSize;
}
#endif

//----------------------------------------------------------------------------------------------------------------------
NMP::Quat PhysicsRigJoltPhysJointed::JointJoltPhysJointed::getRotation(
  const MR::PhysicsJointDef* jointDef,
  const NMP::Matrix34& part1TM,
  const NMP::Matrix34& part2TM) const
{
  NMP::Matrix34 frame1 = jointDef->m_parentPartFrame;
  NMP::Matrix34 frame2 = jointDef->m_childPartFrame;

  NMP::Matrix34 joint1inverse;
  joint1inverse.multiply3x3(frame1, part1TM);
  joint1inverse.invertFast3x3();
  NMP::Matrix34 joint2;
  joint2.multiply3x3(frame2, part2TM);
  NMP::Matrix34 jointTM;
  jointTM.multiply3x3(joint2, joint1inverse);
  return jointTM.toQuat();
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::JointJoltPhysJointed::enableLimit(bool enable)
{
  if (enable == m_limitsEnabled)
    return;
#ifdef DISABLE_JOINTS
  return;
#else
  m_limitsEnabled = enable;

  m_dirtyFlags |= kDirty_Limits;
#endif
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::JointJoltPhysJointed::writeLimits()
{
  // PhysX crashes with zero swing range
  float swing1 = NMP::clampValue(m_modifiableLimits.getSwing1Limit(), s_minSwingLimit, NM_PI - 0.001f);
  float swing2 = NMP::clampValue(m_modifiableLimits.getSwing2Limit(), s_minSwingLimit, NM_PI - 0.001f);
  float twistLow = NMP::clampValue(m_modifiableLimits.getTwistLimitLow(), -NM_PI_OVER_TWO , NM_PI);
  float twistHigh = NMP::clampValue(m_modifiableLimits.getTwistLimitHigh(), -NM_PI_OVER_TWO , NM_PI);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::JointJoltPhysJointed::setDriveOrientation(const NMP::Quat &quat)
{
  // This is what allows unchanging drive to go to sleep, since setDriveOrientation wakes up
  // regardless of whether the drive has changed
  if (quat != m_cache.driveOrientation)
  {
    m_dirtyFlags |= kDirty_DriveOrientation;
    m_cache.driveOrientation = quat;
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::JointJoltPhysJointed::setDriveStrength(float twistStrength, float swingStrength, float slerpStrength)
{
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::JointJoltPhysJointed::setDriveDamping(float twistDamping, float swingDamping, float slerpDamping)
{
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::JointJoltPhysJointed::setStrength(float NMP_UNUSED(strength))
{
  NMP_ASSERT_FAIL_MSG("This function should never get called");
}

//----------------------------------------------------------------------------------------------------------------------
float PhysicsRigJoltPhysJointed::JointJoltPhysJointed::getStrength() const
{
  NMP_ASSERT_FAIL_MSG("This function should never get called");
  return 0.0f;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::JointJoltPhysJointed::setDamping(float NMP_UNUSED(damping))
{
  NMP_ASSERT_FAIL_MSG("This function should never get called");
}

//----------------------------------------------------------------------------------------------------------------------
float PhysicsRigJoltPhysJointed::JointJoltPhysJointed::getDamping() const
{
  NMP_ASSERT_FAIL_MSG("This function should never get called");
  return 0.0f;
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Quat PhysicsRigJoltPhysJointed::JointJoltPhysJointed::getTargetOrientation()
{
  NMP_ASSERT_FAIL_MSG("This function should never get called");
  return NMP::Quat::kIdentity;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::JointJoltPhysJointed::setTargetOrientation(const NMP::Quat &NMP_UNUSED(orientation))
{
  NMP_ASSERT_FAIL_MSG("This function should never get called");
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::removeFromScene()
{
  NMP_ASSERT(m_refCount == 0);
}
//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::addToScene()
{
  NMP_ASSERT(m_refCount == 0);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::makeKinematic(bool moveToKinematicPos)
{
  NMP_ASSERT(m_refCount == 0);
  for (uint32_t i = 0; i < getNumParts(); ++i)
  {
    PartJoltPhysJointed* part = (PartJoltPhysJointed*)m_parts[i];
    part->makeKinematic(true, 1.0f, false);
    part->enableCollision(false);
    if (moveToKinematicPos)
    {
      part->moveTo(m_kinematicPose);
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::makeDynamic()
{
  for (uint32_t i = 0; i < getNumParts(); ++i)
  {
    PartJoltPhysJointed* part = (PartJoltPhysJointed*)m_parts[i];
    part->makeKinematic(false, 1.0f, false);
    part->enableCollision(true);
  }

  for (uint32_t i = 0; i < getNumJoints(); ++i)
  {
    PhysicsRigJoltPhysJointed::JointJoltPhysJointed* joint = (JointJoltPhysJointed*)getJoint(i);
    // enable the joint limit
    joint->enableLimit(true);
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::restoreAllJointDrivesToDefault()
{
  for (uint32_t i = 0; i < getNumJoints(); ++i)
  {
    JointJoltPhysJointed* joint = (JointJoltPhysJointed*)m_joints[i];
    joint->setDriveStrength(0.0f, 0.0f, 0.0f);
    joint->setDriveDamping(joint->getMaxTwistDamping(), joint->getMaxSwingDamping(), joint->getMaxSlerpDamping());
    joint->enableLimit(true);
  }

  m_desiredJointProjectionIterations = 0;
  m_desiredJointProjectionLinearTolerance = FLT_MAX;
  m_desiredJointProjectionAngularTolerance = NM_PI;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::applyHardKeyframing(
  const NMP::DataBuffer& targetBuffer,
  const NMP::DataBuffer* NMP_UNUSED(previousTargetBuffer),
  const NMP::DataBuffer& fallbackBuffer,
  const NMP::Matrix34&   worldRoot,
  const NMP::Matrix34*   NMP_UNUSED(previousWorldRoot),
  bool                   enableCollision,
  float                  NMP_UNUSED(massMultiplier),
  bool                   NMP_UNUSED(enableConstraint),
  float                  NMP_UNUSED(dt),
  const PartChooser&     partChooser)
{
  NMP_ASSERT_MSG(m_physicsRigDef != NULL, "No RigDef exists. Check AnimationSets to have physics rigs defined");

  for (uint32_t i = 0; i < getNumParts(); ++i)
  {
    if (!partChooser.usePart(i))
      continue;

    PartJoltPhysJointed* part = (PartJoltPhysJointed*)m_parts[i];
    part->makeKinematic(true, 1.0f, false);
    part->m_isBeingKeyframed = true;

    part->enableCollision(enableCollision);

    NMP::Matrix34 targetTM;
    calculateWorldSpacePartTM(targetTM, i, targetBuffer, fallbackBuffer, worldRoot, false);

    // use the fact that the position of the PhysX part is offset so that it is at the same
    // location as the morpheme joint
    part->moveTo(targetTM);
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::applySoftKeyframing(
  const NMP::DataBuffer& targetBuffer,
  const NMP::DataBuffer& previousTargetBuffer,
  const NMP::DataBuffer& fallbackBuffer,
  const NMP::Matrix34&   worldRoot,
  const NMP::Matrix34&   previousWorldRoot,
  bool                   enableCollision,
  bool                   enableJointLimits,
  bool                   preserveMomentum,
  float                  NMP_UNUSED(externalJointCompliance),
  float                  gravityCompensationFrac,
  float                  dt,
  float                  weight,
  float                  maxAccel,
  float                  maxAngAccel,
  const PartChooser&     partChooser)
{
  NMP_ASSERT_MSG(m_physicsRigDef != NULL, "No RigDef exists. Check AnimationSets to have physics rigs defined");

  if (dt == 0.0f)
    return;

  NMP::Vector3 gravityDeltaVel = getPhysicsSceneJoltPhys()->getGravity() *
                                 (weight * dt * gravityCompensationFrac);

  maxAccel *= weight;
  maxAngAccel *= weight;

  NMP::Vector3 originalCOMVel(NMP::Vector3::InitZero);
  NMP::Vector3 newCOMVel(NMP::Vector3::InitZero);
  float totalPreservedMass = 0.0f;

  for (uint32_t partIndex = 0; partIndex < getNumParts(); ++partIndex)
  {
    // Skip this part if the part chooser tells us not to apply SK to it.
    if (!partChooser.usePart(partIndex))
      continue;

    PartJoltPhysJointed* part = (PartJoltPhysJointed*)m_parts[partIndex];
    part->makeKinematic(false, 1.0f, false);
    part->m_isBeingKeyframed = true;

    part->enableCollision(enableCollision);

    if (preserveMomentum)
    {
      originalCOMVel += part->getVel() * part->getMass();
      totalPreservedMass += part->getMass();
    }

    // enable/disable joint limits on the parent joint, but only if the parent part is also soft
    // keyframed by this node.
    int32_t parentPartIndex = part->getParentPartIndex();
    if (parentPartIndex >= 0)
    {
      if (partChooser.usePart(parentPartIndex))
      {
        int32_t parentJointIndex = partIndex - 1;
        if (parentJointIndex >= 0)
        {
          // This would be faster if the joint limit enabled state was cached
          JointJoltPhysJointed* joint = (JointJoltPhysJointed*)getJoint(parentJointIndex);
          joint->enableLimit(enableJointLimits);
        }
      }
    }

    NMP::Matrix34 targetTM, targetTMOld;
    calculateWorldSpacePartTM(targetTM, partIndex, targetBuffer, fallbackBuffer, worldRoot, false);
    calculateWorldSpacePartTM(targetTMOld, partIndex, previousTargetBuffer, fallbackBuffer, previousWorldRoot, false);

    // use the fact that the position of the PhysX part is offset so that it is at the same
    // location as the morpheme joint
    NMP::Matrix34 currentTM = part->getTransform();

    NMP::Vector3 offset = part->getCOMPosition();
    NMP::Matrix34 offsetTM(NMP::Matrix34::kIdentity), invOffsetTM(NMP::Matrix34::kIdentity);
    offsetTM.translation() = -offset;
    invOffsetTM.translation() = offset;

    // calculate the motion to go from current to new
    NMP::Matrix34 invCurrentTM(currentTM); invCurrentTM.invertFast();
    // The following pre- and post-multiplication converts diffTM into the actual motion TM
    // centered at the COM.
    NMP::Matrix34 motionTM = invOffsetTM * invCurrentTM * targetTM * offsetTM;

    // Store the distance/angle error
    part->m_SKDeviation = motionTM.translation().magnitude();
    part->m_SKDeviationAngle = motionTM.toRotationVector().magnitude();

    // calculate the motion of the target itself
    NMP::Matrix34 invTargetTMOld(targetTMOld); invTargetTMOld.invertFast();
    NMP::Matrix34 targetMotionTM = invOffsetTM * invTargetTMOld * targetTM * offsetTM;

    // This is where the velocity multiplier can be applied
    NMP::Vector3 translation = motionTM.translation();
    NMP::Vector3 rotation = motionTM.toQuat().toRotationVector(false);

    NMP::Vector3 newVel = translation / dt;
    NMP::Vector3 curVel = part->getVel();
    NMP::Vector3 deltaVel = newVel - curVel;
    if (maxAccel >= 0.0f)
    {
      // prevent overshoot by calculating the max speed we can have in the direction towards
      // the target given that we cannot decelerate faster than maxAccel
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
          // replace the old component along the translation with the new max value
          newVel += translationDir * (maxCurVelAlongTranslation - newVel.dot(translationDir));
          deltaVel = newVel - curVel;
        }
      }

      // clamp the acceleration
      float deltaVelMag = deltaVel.magnitude();
      if (deltaVelMag > maxAccel * dt)
        deltaVel *= maxAccel * dt / deltaVelMag;
    }

    // apply gravity compensation
    deltaVel -= gravityDeltaVel;

    newVel = curVel + deltaVel;
    part->setVel(newVel);

    if (preserveMomentum)
    {
      newCOMVel += newVel * part->getMass();
    }

    NMP::Vector3 newAngVel = rotation / dt;
    NMP::Vector3 curAngVel = part->getAngVel();
    NMP::Vector3 deltaAngVel = newAngVel - curAngVel;
    if (maxAngAccel >= 0.0f)
    {
      // limit the max angular velocity target - this is just a straight conversion of the linear velocity code,
      // so I _think_ it's "correct"!
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
          // replace the old component along the translation with the new max value
          newAngVel += rotationDir * (maxCurAngVelAlongRotation - newAngVel.dot(rotationDir));
          deltaAngVel = newAngVel - curAngVel;
        }
      }

      // clamp the acceleration
      float deltaAngVelMag = deltaAngVel.magnitude();
      if (deltaAngVelMag > maxAngAccel * dt)
        deltaAngVel *= maxAngAccel * dt / deltaAngVelMag;
    }
    newAngVel = curAngVel + deltaAngVel;
    part->setAngVel(newAngVel);
  }

  if (preserveMomentum)
  {
    originalCOMVel /= totalPreservedMass;
    newCOMVel /= totalPreservedMass;
    NMP::Vector3 correctionVel = originalCOMVel - newCOMVel;
    for (uint32_t i = 0; i < getNumParts(); ++i)
    {
      if (!partChooser.usePart(i))
        continue;
      PhysicsRigJoltPhysJointed::PartJoltPhysJointed *part = (PartJoltPhysJointed*)m_parts[i];
      NMP::Vector3 partVel = part->getVel();
      part->setVel(partVel + correctionVel);
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------
// Drives the joints to the targets given by the input animation buffer.   
// Using twist/swing drives seems to result in significant jittering - slerp drive works much
// better. However, currently the physx description defines the params using twist/swing, so we just
// assume the swing strengths can be applied to slerp.
void PhysicsRigJoltPhysJointed::applyActiveAnimation(
  const NMP::DataBuffer& targetBuffer,
  const NMP::DataBuffer& fallbackBuffer,
  float                  strengthMultiplier,
  float                  dampingMultiplier,
  float                  NMP_UNUSED(internalCompliance),
  float                  NMP_UNUSED(externalCompliance),
  bool                   enableJointLimits,
  const JointChooser&    jointChooser,
  float                  limitClampFraction)
{
  NMP_ASSERT_MSG(m_physicsRigDef != NULL, "No RigDef exists. Check AnimationSets to have physics rigs defined");

  for (uint32_t i = 0; i < getNumJoints(); ++i)
  {
    if (!jointChooser.useJoint(i))
      continue;

    JointJoltPhysJointed* joint = (JointJoltPhysJointed*)m_joints[i];
    const PhysicsSixDOFJointDef* jointDef = static_cast<const PhysicsSixDOFJointDef*>(m_physicsRigDef->m_joints[i]);

    PartJoltPhysJointed* childPart = (PartJoltPhysJointed*)m_parts[jointDef->m_childPartIndex];
    childPart->makeKinematic(false, 1.0f, false);
    childPart->m_isBeingKeyframed = false;

    // don't force either of the parts to have collision - no way we could know which one _should_
    // have collision if it's disabled elsewhere.

    joint->setDriveStrength(
      joint->getMaxTwistStrength() * strengthMultiplier,
      joint->getMaxSwingStrength() * strengthMultiplier,
      joint->getMaxSlerpStrength() * strengthMultiplier);
    joint->setDriveDamping(
      joint->getMaxTwistDamping() * dampingMultiplier,
      joint->getMaxSwingDamping() * dampingMultiplier,
      joint->getMaxSlerpDamping() * dampingMultiplier);
    joint->enableLimit(enableJointLimits);

    if (strengthMultiplier < 0.0000001f)
      continue;

    NMP::Quat curQ;
    getQuatFromTransformBuffer(jointDef->m_childPartIndex, targetBuffer, fallbackBuffer, curQ);

    // q is the rotation of the child relative to the parent (in parent space).
    // We need to account for the offset axes in the joint.

    // Get the local joint axes in each frame as l0, l1
    NMP::Quat l0 = m_physicsRigDef->m_joints[i]->m_parentPartFrame.toQuat();
    NMP::Quat l1 = m_physicsRigDef->m_joints[i]->m_childPartFrame.toQuat();

    // now "assuming" the parent is at the origin (since we already have the relative rotation q)
    // we want to calculate rot, the relative rotation of the child local frame from the parent local frame
    NMP::Quat l0Inv = ~l0;

    // target orientations outside the limits cause oscillations when physical limits are enabled
    if (limitClampFraction >= 0.0f)
    {
      NMP_ASSERT(m_physicsRigDef->m_joints[i]->m_jointType == PhysicsJointDef::JOINT_TYPE_SIX_DOF);
      joint->clampToLimits(curQ, limitClampFraction, NULL);
    }

    NMP::Quat curFrameQ = l0Inv * curQ * l1;
    joint->setDriveOrientation(curFrameQ);
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::applyActiveAnimation(uint32_t jointIndex, const NMP::Quat& targetQuat, bool makeChildDynamic)
{
  NMP_ASSERT(jointIndex < getNumJoints());
  JointJoltPhysJointed* joint = (JointJoltPhysJointed*)m_joints[jointIndex];
  const PhysicsJointDef* jointDef = m_physicsRigDef->m_joints[jointIndex];
  if (makeChildDynamic)
  {
    PartJoltPhysJointed *childPart = (PartJoltPhysJointed*)m_parts[jointDef->m_childPartIndex];
    childPart->makeKinematic(false, 1.0f, false);
    childPart->m_isBeingKeyframed = false;
  }
  // don't force either of the parts to have collision - no way we could know which one _should_
  // have collision if it's disabled elsewhere.
  joint->setDriveOrientation(targetQuat);
}

//----------------------------------------------------------------------------------------------------------------------
// TODO move this into PhysicsRig
NMP::Quat PhysicsRigJoltPhysJointed::getJointQuat(uint32_t jointIndex)
{
  JointJoltPhysJointed* joint = (JointJoltPhysJointed*)getJoint(jointIndex);
  const PhysicsJointDef* jointDef = m_physicsRigDef->m_joints[jointIndex];
  uint32_t p1 = jointDef->m_parentPartIndex;
  uint32_t p2 = jointDef->m_childPartIndex;
  NMP::Matrix34 part1TM = getPart(p1)->getTransform();
  NMP::Matrix34 part2TM = getPart(p2)->getTransform();
  NMP::Quat result = joint->getRotation(jointDef, part1TM, part2TM);

  return result;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::generateCachedValues()
{
  for (uint32_t i = 0; i < getNumParts(); ++i)
  {
    ((PartJoltPhysJointed*)m_parts[i])->generateCachedValues();
  }
  for (uint32_t i = 0; i < getNumJoints(); ++i)
  {
    ((JointJoltPhysJointed*)m_joints[i])->generateCachedValues();
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::applyModifiedValues()
{
  for (uint32_t i = 0; i < getNumParts(); ++i)
  {
    ((PartJoltPhysJointed*)m_parts[i])->applyModifiedValues();
  }

  bool doProjection = m_desiredJointProjectionIterations != 0;
  for (uint32_t i = 0; i < getNumJoints(); ++i)
  {
    JointJoltPhysJointed* j = (JointJoltPhysJointed*) m_joints[i];
    j->applyModifiedValues();
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::disableSleeping()
{
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::updatePrePhysics(float NMP_UNUSED(timeStep))
{
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::updatePostPhysics(float NMP_UNUSED(timeStep))
{
}

} // namespace MR
//----------------------------------------------------------------------------------------------------------------------
