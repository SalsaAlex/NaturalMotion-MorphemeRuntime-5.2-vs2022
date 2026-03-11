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
  m_isBeingKeyframed = false;
  m_SKDeviation = 0.0f;
  m_SKDeviationAngle = 0.0f;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::PartJoltPhys::addVelocityChange(const NMP::Vector3& velChange, const NMP::Vector3& worldPos, float angularMultiplier)
{
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::PartJoltPhys::addImpulse(const NMP::Vector3 &impulse)
{
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::PartJoltPhys::addTorqueImpulse(const NMP::Vector3& torqueImpulse)
{
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::PartJoltPhys::addTorque(const NMP::Vector3& torque)
{
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::PartJoltPhys::addForce(const NMP::Vector3 &force)
{
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::PartJoltPhys::addLinearVelocityChange(const NMP::Vector3& velChange)
{
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::PartJoltPhys::addAngularAcceleration(const NMP::Vector3& angularAcceleration)
{
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysArticulation* PhysicsRigJoltPhys::getPhysicsRigJoltPhysArticulation()
{
  return m_type == TYPE_ARTICULATED ? (PhysicsRigJoltPhysArticulation*) this : 0;
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysJointed* PhysicsRigJoltPhys::getPhysicsRigJoltPhysJointed()
{
  return m_type == TYPE_JOINTED ? (PhysicsRigJoltPhysJointed*) this : 0;
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
} 

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhys::setSkinWidthIncrease(uint32_t partIndex, float skinWidthIncrease)
{
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
void PhysicsRigJoltPhys::updateRegisteredJoints()
{
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

} // namespace MR
//----------------------------------------------------------------------------------------------------------------------
