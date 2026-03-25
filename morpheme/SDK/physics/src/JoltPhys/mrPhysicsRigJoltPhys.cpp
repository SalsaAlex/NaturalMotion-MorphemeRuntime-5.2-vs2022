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
#include "physics/JoltPhys/mrPhysicsRigJoltPhys.h"
#include "physics/JoltPhys/mrPhysicsSceneJoltPhys.h"
#include "physics/JoltPhys/mrPhysicsDriverDataJoltPhys.h"

#if defined(NM_HOST_CELL_PPU)
  #include <sys/process.h>
  SYS_PROCESS_PARAM (1001, 98304) // increase the primary ppu thread stack size from 64k to 96k 
                                  // to avoid physx3 simulate() running out of stack space.
#endif
//----------------------------------------------------------------------------------------------------------------------

// This rig ID must start at 1, because the default value that dynamic objects ignore is 0. If the
// game deletes and inits rigs more than 2^32 times this will be a problem. See MORPH-11270
static int g_rigID = 1; 

namespace MR
{
//----------------------------------------------------------------------------------------------------------------------
bool locatePhysicsRigDefJoltPhys(uint32_t NMP_USED_FOR_ASSERTS(assetType), void* assetMemory)
{
  NMP_ASSERT(assetType == Manager::kAsset_PhysicsRigDef);
  PhysicsRigDef* physicsRigDef = (PhysicsRigDef*)assetMemory;
  bool result = physicsRigDef->locate();
  if (result)
  {
    return locateDriverDataJoltPhys(physicsRigDef);
  }

  return false;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::init(PhysicsRigJoltPhys* physicsRigJoltPhys, Type type)
{
  physicsRigJoltPhys->m_type = type;
  physicsRigJoltPhys->m_rigID = g_rigID++;
  physicsRigJoltPhys->m_characterControllerBody = 0;
  physicsRigJoltPhys->m_kinematicPose.identity();
  physicsRigJoltPhys->m_desiredJointProjectionLinearTolerance = FLT_MAX;
  physicsRigJoltPhys->m_desiredJointProjectionAngularTolerance = NM_PI;
  physicsRigJoltPhys->m_desiredJointProjectionIterations = 0;

  NMP_ASSERT(physicsRigJoltPhys->m_registeredJoints.getNumUsedSlots() == 0);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::setKinematicPos(const NMP::Vector3& pos)
{
  m_kinematicPose.translation() = pos;
  if (!isReferenced())
  {
    makeKinematic(true);
  }
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhys::PartJoltPhys::PartJoltPhys()
{
  m_rigidBody = 0;
  m_currentSkinWidthIncrease = 0.0f;
  m_isBeingKeyframed = false;
  m_SKDeviation = 0.0f;
  m_SKDeviationAngle = 0.0f;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::PartJoltPhys::addVelocityChange(const NMP::Vector3& velChange, const NMP::Vector3& worldPos, float angularMultiplier)
{
  MR::addVelocityChangeToBody(m_rigidBody, velChange, worldPos, angularMultiplier);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::PartJoltPhys::addImpulse(const NMP::Vector3 &impulse)
{
  MR::addImpulseToBody(m_rigidBody, impulse);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::PartJoltPhys::addTorqueImpulse(const NMP::Vector3& torqueImpulse)
{
  MR::addTorqueImpulseToBody(m_rigidBody, torqueImpulse);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::PartJoltPhys::addTorque(const NMP::Vector3& torque)
{
  MR::addTorqueToBody(m_rigidBody, torque);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::PartJoltPhys::addForce(const NMP::Vector3 &force)
{
  MR::addForceToBody(m_rigidBody, force);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::PartJoltPhys::addLinearVelocityChange(const NMP::Vector3& velChange)
{
  MR::addLinearVelocityChangeToBody(m_rigidBody, velChange);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::PartJoltPhys::addAngularAcceleration(const NMP::Vector3& angularAcceleration)
{
  MR::addAngularAccelerationToBody(m_rigidBody, angularAcceleration);
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhys::PartJoltPhys* PhysicsRigJoltPhys::getPartJoltPhys(uint32_t index)
{
  return (PhysicsRigJoltPhys::PartJoltPhys*) getPart(index);
}

//----------------------------------------------------------------------------------------------------------------------
const PhysicsRigJoltPhys::PartJoltPhys* PhysicsRigJoltPhys::getPartJoltPhys(uint32_t index) const
{
  return (const PhysicsRigJoltPhys::PartJoltPhys*) getPart(index);
}

//----------------------------------------------------------------------------------------------------------------------
float PhysicsRigJoltPhys::PartJoltPhys::getSKDeviation() const
{
  // Note that the deviation is only set when actually being soft keyframed
  if (!m_isBeingKeyframed || isKinematic())
    return 0.0;
  return m_SKDeviation;
}

//----------------------------------------------------------------------------------------------------------------------
float PhysicsRigJoltPhys::PartJoltPhys::getSKDeviationAngle() const
{
  // Note that the deviation is only set when actually being soft keyframed
  if (!m_isBeingKeyframed || isKinematic())
    return 0.0;
  return m_SKDeviationAngle;
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhys::JointJoltPhys::JointJoltPhys(const PhysicsSixDOFJointDef* const def)
: m_def(def)
{
  NMP_ASSERT(def); // Null check.

  if (def)
  {
    m_modifiableLimits = def->m_hardLimits.getModifiableLimits();
  }

#if defined(MR_OUTPUT_DEBUGGING)
  // Initialise serialization data.
  updateSerializeTxFrameData();
#endif // MR_OUTPUT_DEBUGGING
}


//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::JointJoltPhys::clampToLimits(
  NMP::Quat& orientation,
  float limitFrac,
  const NMP::Quat* origQ) const
{
  if (origQ)
    m_modifiableLimits.clampToLimits(orientation, limitFrac, *m_def, *origQ);
  else
    m_modifiableLimits.clampToLimits(orientation, limitFrac, *m_def);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::JointJoltPhys::expandLimits(const NMP::Quat& orientation)
{
  m_modifiableLimits.expand(orientation, *m_def);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::JointJoltPhys::scaleLimits(float scaleFactor)
{
  m_modifiableLimits.scale(scaleFactor);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::JointJoltPhys::resetLimits()
{
#if defined(MR_OUTPUT_DEBUGGING)
  // Copy current limits used this frame into frame data cache before they are reset so that they can be made availible
  // to the debug render system later in the update.
  updateSerializeTxFrameData();
#endif // MR_OUTPUT_DEBUGGING

  m_modifiableLimits.setSwingLimit(
    m_def->m_hardLimits.getSwing1Limit(), 
    m_def->m_hardLimits.getSwing2Limit());
  m_modifiableLimits.setTwistLimit(
    m_def->m_hardLimits.getTwistLimitLow(), 
    m_def->m_hardLimits.getTwistLimitHigh());
}

#if defined(MR_OUTPUT_DEBUGGING)
//----------------------------------------------------------------------------------------------------------------------
uint32_t PhysicsRigJoltPhys::JointJoltPhys::serializeTxFrameData(
  void*     outputBuffer,
  uint32_t  NMP_USED_FOR_ASSERTS(outputBufferSize)) const
{
  uint32_t dataSize = sizeof(PhysicsSixDOFJointFrameData);

  if (outputBuffer)
  {
    NMP_ASSERT(outputBufferSize >= dataSize);

    PhysicsSixDOFJointFrameData* frameData = reinterpret_cast<PhysicsSixDOFJointFrameData*>(outputBuffer);

    *frameData = m_serializeTxFrameData;

    PhysicsSixDOFJointFrameData::endianSwap(frameData);
  }

  return dataSize;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::JointJoltPhys::updateSerializeTxFrameData()
{
  m_serializeTxFrameData.m_jointType = PhysicsJointFrameData::JOINT_TYPE_SIX_DOF;

  m_serializeTxFrameData.m_swing1Limit = m_modifiableLimits.getSwing1Limit();
  m_serializeTxFrameData.m_swing2Limit = m_modifiableLimits.getSwing2Limit();
  m_serializeTxFrameData.m_twistLimitLow = m_modifiableLimits.getTwistLimitLow();
  m_serializeTxFrameData.m_twistLimitHigh = m_modifiableLimits.getTwistLimitHigh();
}
#endif // MR_OUTPUT_DEBUGGING

//----------------------------------------------------------------------------------------------------------------------
float PhysicsRigJoltPhys::getMaxSKDeviation() const
{
  float maxDeviation = 0.0f;
  for (uint32_t i = 0; i < getNumParts(); ++i)
  {
    const PhysicsRigJoltPhys::PartJoltPhys *part = getPartJoltPhys(i);
    float SKDeviation = part->getSKDeviation();

    if (SKDeviation > maxDeviation)
      maxDeviation = SKDeviation;
  }
  return maxDeviation;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::receiveImpulse(int32_t inputPartIndex,
                                      const NMP::Vector3& inputPosition,
                                      const NMP::Vector3& inputDirection,
                                      float inputLocalMagnitude,
                                      float inputLocalAngularMultiplier,
                                      float inputLocalResponseRatio,
                                      float inputFullBodyMagnitude,
                                      float inputFullBodyAngularMultiplier,
                                      float inputFullBodyLinearMultiplier,
                                      float inputFullBodyResponseRatio,
                                      bool positionInWorldSpace,
                                      bool directionInWorldSpace,
                                      bool applyAsVelocityChange)
{
  bool invalidInputPartIndex = inputPartIndex < 0 || inputPartIndex >= (int32_t) getNumParts();
  if (invalidInputPartIndex)
  {
    inputPartIndex = 0;
    inputLocalMagnitude = 0.0f;
  }

  MR::PhysicsRigJoltPhys::PartJoltPhys* hitPart = getPartJoltPhys(inputPartIndex);
  JPH::Body* rigidBody = hitPart->getRigidBody();

  NMP::Matrix34 actorToWorld;
  MR::getBodyGlobalPoseTM(rigidBody, actorToWorld);

  // Get the world space position
  NMP::Vector3 positionWorld = inputPosition;
  if (!positionInWorldSpace)
  {
    // Local space positions are specified relative to the COM, not the actor, position.
    NMP::Matrix34 actorCOMToWorld = actorToWorld;
    actorCOMToWorld.translation() = MR::getBodyCOMPos(rigidBody);
    actorCOMToWorld.transformVector(positionWorld);
  }

  // Get world space direction
  NMP::Vector3 directionWorld = inputDirection;
  if (!directionInWorldSpace)
  {
    actorToWorld.rotateVector(directionWorld);
  }

  float totalRigMass = calculateMass();
  float averagePartMass = totalRigMass / getNumParts();

  // Here we apply the impulse to the part, scaled according to distribution
  if (inputLocalMagnitude != 0.0f)
  {
    // World space impulse or velocity change
    NMP::Vector3 impulseWorld = directionWorld * inputLocalMagnitude;

    float hitPartMass = hitPart->getMass();
    float multiplier = powf(hitPartMass / averagePartMass, inputLocalResponseRatio);

    if (applyAsVelocityChange == false)
    {
      // impulse
      MR::addImpulseToBody(
        rigidBody, 
        impulseWorld * multiplier, 
        positionWorld, 
        inputLocalAngularMultiplier);
    }
    else if (applyAsVelocityChange == true)
    {
      // velocity change
      MR::addVelocityChangeToBody(
        rigidBody, 
        impulseWorld * multiplier, 
        positionWorld);
    }
  }

  // Apply the impulse to the character as a whole, treating it as a rigid body with the
  // character's COM and inertia properties.
  if (inputFullBodyMagnitude != 0.0f)
  {
    // World space impulse or velocity change
    NMP::Vector3 impulseWorld = directionWorld * inputFullBodyMagnitude;

    NMP::Matrix34 invInertiaWorld = calculateGlobalInertiaTensor();
    invInertiaWorld.invert3x3();
    NMP::Vector3  COMPositionWorld = calculateCentreOfMass();

    NMP::Vector3 impulsePositionRelCOMPositionWorld = 
      invalidInputPartIndex ? NMP::Vector3::InitZero : positionWorld - COMPositionWorld;

    if (applyAsVelocityChange == true)
    {
      // If applying velocity changes we need to convert impulseWorld into a real impulse (rather
      // than velocity change). This is done using the same equations as found in collision
      // response code - i.e. relating a required change in velocity in a certain direction to
      // an impulse.

      float velChangePerUnitImpulse =  
        (1.0f / totalRigMass) + 
        NMP::vDot(directionWorld, 
        NMP::vCross(invInertiaWorld.getRotatedVector(NMP::vCross(impulsePositionRelCOMPositionWorld, 
        directionWorld)), 
        impulsePositionRelCOMPositionWorld));

      impulseWorld = impulseWorld / velChangePerUnitImpulse;
    }

    // Apply the impulse by calculating the response of a rigid body with the mass/inertia
    // properties of the physics rig
    {
      NMP::Vector3 linearVelocityChange = impulseWorld/totalRigMass;
      NMP::Vector3 angularImpulse = NMP::vCross(impulsePositionRelCOMPositionWorld, impulseWorld);
      NMP::Vector3 angularVelocityChange = angularImpulse;
      angularVelocityChange.rotate(invInertiaWorld);

      // apply the multipliers, and also the distribution
      linearVelocityChange *= inputFullBodyLinearMultiplier;
      angularVelocityChange *= inputFullBodyAngularMultiplier;

      for (uint32_t iPart = 0; iPart < getNumParts(); ++iPart)
      {
        MR::PhysicsRigJoltPhys::PartJoltPhys* part = getPartJoltPhys(iPart);

        float partMass = part->getMass();
        float multiplier = powf(partMass / averagePartMass, inputFullBodyResponseRatio);

        // add angular velocity due to torque
        MR::addAngularVelocityChangeToBody(part->getRigidBody(), angularVelocityChange * multiplier);

        // Add linear velocity due to linear push and torque around COM
        NMP::Vector3 partCOMOffset = part->getCOMPosition() - COMPositionWorld;
        NMP::Vector3 rotationalLinearVelocityChange = NMP::vCross(angularVelocityChange, partCOMOffset);
        MR::addLinearVelocityChangeToBody(
          part->getRigidBody(), 
          (linearVelocityChange + rotationalLinearVelocityChange) * multiplier);
      }
    }
  } // inputDistribution > 0
} 

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::receiveTorqueImpulse(
  int32_t inputPartIndex,
  const NMP::Vector3& inputDirection,
  float inputLocalMagnitude,
  float inputLocalResponseRatio,
  float inputFullBodyMagnitude,
  float inputFullBodyResponseRatio,
  bool directionInWorldSpace,
  bool applyAsVelocityChange)
{
  bool invalidInputPartIndex = inputPartIndex < 0 || inputPartIndex >= (int32_t) getNumParts();
  if (invalidInputPartIndex)
  {
    inputPartIndex = 0;
    inputLocalMagnitude = 0.0f;
  }

  MR::PhysicsRigJoltPhys::PartJoltPhys* hitPart = getPartJoltPhys(inputPartIndex);
  JPH::Body* rigidBody = hitPart->getRigidBody();

  NMP::Matrix34 actorToWorld;
  MR::getBodyGlobalPoseTM(rigidBody, actorToWorld);

  // Get world space direction
  NMP::Vector3 directionWorld = inputDirection;
  if (!directionInWorldSpace)
  {
    actorToWorld.rotateVector(directionWorld);
  }

  float totalRigMass = calculateMass();
  float averagePartMass = totalRigMass / getNumParts();

  // Here we apply the impulse to the part, scaled according to distribution
  if (inputLocalMagnitude != 0.0f)
  {
    // World space impulse or velocity change
    NMP::Vector3 torqueImpulseWorld = directionWorld * inputLocalMagnitude;

    float hitPartMass = hitPart->getMass();
    float multiplier = powf(hitPartMass / averagePartMass, inputLocalResponseRatio);

    if (applyAsVelocityChange == false)
    {
      // impulse
      MR::addTorqueImpulseToBody(
        rigidBody, 
        torqueImpulseWorld * multiplier);
    }
    else if (applyAsVelocityChange == true)
    {
      // velocity change
      MR::addAngularVelocityChangeToBody(
        rigidBody, 
        torqueImpulseWorld * multiplier);
    }
  }

  // Apply the impulse to the character as a whole, treating it as a rigid body with the
  // character's COM and inertia properties.
  if (inputFullBodyMagnitude != 0.0f)
  {
    // World space impulse or velocity change
    NMP::Vector3 torqueImpulseWorld = directionWorld * inputFullBodyMagnitude;

    NMP::Matrix34 invInertiaWorld = calculateGlobalInertiaTensor();
    invInertiaWorld.invert3x3();
    NMP::Vector3  COMPositionWorld = calculateCentreOfMass();

    // Apply the impulse by calculating the response of a rigid body with the mass/inertia
    // properties of the physics rig
    {
      NMP::Vector3 angularVelocityChange = 
        applyAsVelocityChange ? torqueImpulseWorld : invInertiaWorld.getRotatedVector(torqueImpulseWorld);

      for (uint32_t iPart = 0; iPart < getNumParts(); ++iPart)
      {
        MR::PhysicsRigJoltPhys::PartJoltPhys* part = getPartJoltPhys(iPart);

        float partMass = part->getMass();
        float multiplier = powf(partMass / averagePartMass, inputFullBodyResponseRatio);

        // add angular velocity due to torque
        MR::addAngularVelocityChangeToBody(part->getRigidBody(), angularVelocityChange * multiplier);

        // Add linear velocity due to linear push and torque around COM
        NMP::Vector3 partCOMOffset = part->getCOMPosition() - COMPositionWorld;
        NMP::Vector3 rotationalLinearVelocityChange = NMP::vCross(angularVelocityChange, partCOMOffset);
        MR::addLinearVelocityChangeToBody(
          part->getRigidBody(), 
          rotationalLinearVelocityChange * multiplier);
      }
    }
  } // inputDistribution > 0
} 

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::setSkinWidthIncrease(uint32_t partIndex, float skinWidthIncrease)
{
  PartJoltPhys *partPhysX = getPartJoltPhys(partIndex);
  // Note that if a skin width increase is in use, then it will end up getting set and then returned
  // to zero each update. However, this will prevent excessive calls in the normal cases.
  if (partPhysX->m_currentSkinWidthIncrease == skinWidthIncrease)
  {
    return;
  }

  // Only allow increases, unless this is supposed to reset to the authored value
  if (partPhysX->m_currentSkinWidthIncrease > skinWidthIncrease && skinWidthIncrease != 0.0f)
  {
    return;
  }

  //incomplete
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::scaleFrictionOnPart(const int32_t partIndex, const float frictionScale)
{
  const MR::PhysicsRigDef::Part& partDef = m_physicsRigDef->m_parts[partIndex];

  // Build a list of all the material id's that belong to this part.
  int32_t materialID[MAX_SHAPES_IN_VOLUME];
  int32_t materialIDCount = 0;

  for (int32_t i = 0; i < partDef.volume.numSpheres; ++i)
  {
    materialID[materialIDCount++] = partDef.volume.spheres[i].materialID;
  }

  for (int32_t i = 0; i < partDef.volume.numBoxes; ++i)
  {
    materialID[materialIDCount++] = partDef.volume.boxes[i].materialID;
  }

  for (int32_t i = 0; i < partDef.volume.numCapsules; ++i)
  {
    materialID[materialIDCount++] = partDef.volume.capsules[i].materialID;
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::registerJointOnRig(JPH::SixDOFConstraint* joint)
{
  if (!m_registeredJoints.replace(joint, true))
  {
    m_registeredJoints.insert(joint, true);
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::deRegisterJointOnRig(JPH::SixDOFConstraint* joint)
{
  if (!m_registeredJoints.erase(joint))
  {
    NMP_ASSERT_FAIL_MSG("Tried to deregister an unregistered joint");
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::updateRegisteredJoints()
{
  RegisteredJoints::value_walker it = m_registeredJoints.walker();
  while (it.next())
  {
    bool registered = it();
    if (!registered)
    {
      // The joint hasn't been deregistered, and it wasn't updated, so it needs to be released.
      JPH::SixDOFConstraint* joint = it.key();
      joint->Release(); //fix
      m_registeredJoints.erase(joint);
      // Erasing invalidates the walker so just start again
      it.reset();
    }
  }

  // Now clear the entries ready for the next update
  it.reset();
  while (it.next())
  {
    it() = false;
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::requestJointProjectionParameters(int iterations, float linearTolerance, float angularTolerance)
{
  if (iterations > 0)
  {
    if (iterations > m_desiredJointProjectionIterations)
    {
      m_desiredJointProjectionIterations = iterations;
    }
    if (linearTolerance < m_desiredJointProjectionLinearTolerance)
    {
      m_desiredJointProjectionLinearTolerance = linearTolerance;
    }
    if (angularTolerance < m_desiredJointProjectionAngularTolerance)
    {
      m_desiredJointProjectionAngularTolerance = angularTolerance;
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::addQueryFilterFlagToParts(uint32_t word0, uint32_t word1, uint32_t word2, uint32_t word3)
{
}



PhysicsRigJoltPhysBodyData::BodyToPhysicsRigJoltPhysBodyData* PhysicsRigJoltPhysBodyData::m_bodyToMorphemeMap = 0;
uint32_t PhysicsRigJoltPhysBodyData::m_bodyMapRefCount = 0;

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysBodyData::init()
{
  ++m_bodyMapRefCount;

  if (m_bodyMapRefCount == 1)
  {
    NMP_ASSERT(!m_bodyToMorphemeMap);
    if (!m_bodyToMorphemeMap)
    {
      m_bodyToMorphemeMap = (BodyToPhysicsRigJoltPhysBodyData*)
        NMPMemoryAlloc(sizeof(BodyToPhysicsRigJoltPhysBodyData));
      NMP_ASSERT(m_bodyToMorphemeMap);
      new(m_bodyToMorphemeMap) BodyToPhysicsRigJoltPhysBodyData(32);
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysBodyData::term()
{
  --m_bodyMapRefCount;

  if (m_bodyMapRefCount == 0)
  {
    if (m_bodyToMorphemeMap)
    {
      m_bodyToMorphemeMap->~BodyToPhysicsRigJoltPhysBodyData();
      NMP::Memory::memFree(m_bodyToMorphemeMap);
      m_bodyToMorphemeMap = 0;
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysBodyData *PhysicsRigJoltPhysBodyData::create(JPH::Body* body, 
                                                             PhysicsRig::Part *owningRigPart,
                                                             PhysicsRig *owningRig)
{
  PhysicsRigJoltPhysBodyData *data = 
    (PhysicsRigJoltPhysBodyData*) NMPMemoryAlloc(sizeof(PhysicsRigJoltPhysBodyData));
  NMP_ASSERT(data);
  new(data) PhysicsRigJoltPhysBodyData(body, owningRigPart, owningRig);
  return data;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysBodyData::destroy(PhysicsRigJoltPhysBodyData *data, JPH::Body* body)
{
  if (!data)
  {
    return;
  }
  if (body)
  {
    PhysicsRigJoltPhysBodyData* element = 0;
    m_bodyToMorphemeMap->find(body, &element);
    NMP_ASSERT(m_bodyToMorphemeMap->getNumUsedSlots()>0);
    NMP_ASSERT(element == data);
    m_bodyToMorphemeMap->erase(body);
  }
  NMP::Memory::memFree(data);
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysBodyData::PhysicsRigJoltPhysBodyData(
  JPH::Body *body, 
  PhysicsRig::Part *owningRigPart,
  PhysicsRig *owningRig) 
  : m_owningRigPart(owningRigPart), m_owningRig(owningRig), m_userData(0) 
{
  NMP_ASSERT(getFromBody(body) == 0);
  m_bodyToMorphemeMap->insert(body, this); // ensures it definitely goes in, even if the old value exists!
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysBodyData *PhysicsRigJoltPhysBodyData::getFromBody(JPH::Body *body)
{
  PhysicsRigJoltPhysBodyData* data = NULL;
  m_bodyToMorphemeMap->find(body, &data);
  return data;
}

} // namespace MR
//----------------------------------------------------------------------------------------------------------------------
