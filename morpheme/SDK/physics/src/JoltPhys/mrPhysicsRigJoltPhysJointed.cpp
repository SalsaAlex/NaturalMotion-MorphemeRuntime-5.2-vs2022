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
const float PhysicsRigJoltPhysJointed::s_limitContactAngle = NM_PI * 0.1f;

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
PhysicsRigJoltPhysJointed* PhysicsRigJoltPhysJointed::init(
  NMP::Memory::Resource& resource,
  PhysicsRigDef*         physicsRigDef,
  PhysicsScene*          physicsScene,
  AnimRigDef*            animRigDef,
  AnimToPhysicsMap*      animToPhysicsMap,
  int32_t                collisionTypeMask,
  int32_t                collisionIgnoreMask)
{
  PhysicsRigJoltPhysJointed* result = (PhysicsRigJoltPhysJointed*)resource.ptr;
  resource.increment(sizeof(PhysicsRigJoltPhysJointed));

  new (result) PhysicsRigJoltPhysJointed((PhysicsSceneJoltPhys*) physicsScene);
  PhysicsRigJoltPhys::init(result, PhysicsRigJoltPhys::TYPE_JOINTED);

  uint32_t numParts = physicsRigDef->getNumParts();
  uint32_t numJoints = physicsRigDef->getNumJoints();
  uint32_t numMaterials = physicsRigDef->getNumMaterials();

  // Part pointers
  resource.align(NMP::Memory::Format(sizeof(PhysicsRig::Part*) * numParts, NMP_NATURAL_TYPE_ALIGNMENT));
  result->m_parts = (PhysicsRig::Part**)resource.ptr;
  resource.increment(NMP::Memory::Format(sizeof(PhysicsRig::Part*) * numParts, NMP_NATURAL_TYPE_ALIGNMENT));

  // Joint pointers
  resource.align(NMP::Memory::Format(sizeof(PhysicsRig::Joint*) * numJoints, NMP_NATURAL_TYPE_ALIGNMENT));
  result->m_joints = (PhysicsRig::Joint**)resource.ptr;
  resource.increment(NMP::Memory::Format(sizeof(PhysicsRig::Joint*) * numJoints, NMP_NATURAL_TYPE_ALIGNMENT));

  // Parts
  for (uint32_t i = 0; i < numParts; i++)
  {
    resource.align(NMP::Memory::Format(sizeof(PhysicsRigJoltPhysJointed::PartJoltPhysJointed), NMP_NATURAL_TYPE_ALIGNMENT));
    result->m_parts[i] = (PhysicsRigJoltPhysJointed::PartJoltPhysJointed*)resource.ptr;
    resource.increment(NMP::Memory::Format(sizeof(PhysicsRigJoltPhysJointed::PartJoltPhysJointed), NMP_NATURAL_TYPE_ALIGNMENT));
  }

  // Joints
  for (uint32_t i = 0; i < numJoints; i++)
  {
    resource.align(NMP::Memory::Format(sizeof(PhysicsRigJoltPhysJointed::JointJoltPhysJointed), NMP_NATURAL_TYPE_ALIGNMENT));
    result->m_joints[i] = (PhysicsRigJoltPhysJointed::JointJoltPhysJointed*)resource.ptr;
    resource.increment(NMP::Memory::Format(sizeof(PhysicsRigJoltPhysJointed::JointJoltPhysJointed), NMP_NATURAL_TYPE_ALIGNMENT));
  }

  result->m_animRigDef = animRigDef;
  result->m_animToPhysicsMap = animToPhysicsMap;
  result->m_physicsRigDef = physicsRigDef;

  result->m_collisionTypeMask = collisionTypeMask;
  result->m_collisionIgnoreMask = collisionIgnoreMask;

  uint32_t totalNumShapes = 0;
  for (uint32_t iPart = 0 ; iPart < numParts ; ++iPart)
  {
    PhysicsRigDef::Part &part = physicsRigDef->m_parts[iPart];
    PhysicsRigDef::Part::Volume &volume = part.volume;
    uint32_t numShapes = volume.numSpheres + volume.numBoxes + volume.numCapsules;
    totalNumShapes += numShapes;
  }

  // convert collision sets into a mask per part
  int *partGroupMasks = (int *)alloca(numParts * sizeof(int));
  int *allowedPartGroupMasks = (int *)alloca(numParts * sizeof(int));
  for (uint32_t i = 0; i<numParts; i++)
  {
    partGroupMasks[i] = 0;
    allowedPartGroupMasks[i] = 0;
  }
  for (int32_t i = 0; i<physicsRigDef->m_numCollisionGroups; i++)
  {
    if (physicsRigDef->m_collisionGroups[i].enabled)
    {
      for (int j = 0; j<physicsRigDef->m_collisionGroups[i].numIndices; j++)
      {
        // indices can't reference a part out of range
        NMP_ASSERT(physicsRigDef->m_collisionGroups[i].indices[j] < (int)numParts); 
        partGroupMasks[physicsRigDef->m_collisionGroups[i].indices[j]] |= 1<<i;
      }
    }

    for (int j = 0; j<physicsRigDef->m_collisionGroups[i].numIndices; j++)
    {
      // indices can't reference a part out of range
      NMP_ASSERT(physicsRigDef->m_collisionGroups[i].indices[j] < (int)numParts); 
      allowedPartGroupMasks[physicsRigDef->m_collisionGroups[i].indices[j]] |= 1<<i;
    }
  }

  // incremented as we work on each shape
  uint32_t iShape = 0;

  // The parent joint index of a part is always the part index - 1, since its a tree.
  for (int32_t i = 0; i < physicsRigDef->m_numParts; i++)
    result->getPart(i)->setParentPartIndex(i ? physicsRigDef->m_joints[i-1]->m_parentPartIndex : -1);
  
  // Need to generate the cached values.
  result->generateCachedValues(); // timestep is irrelevant at this point

  result->restoreAllJointDrivesToDefault();

  result->makeKinematic(true);

  result->applyModifiedValues();

  return result;
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysJointed::term()
{
  JPH::PhysicsSystem* joltPhysScene = getPhysicsSceneJoltPhys()->m_joltPhysScene;
  if (joltPhysScene)
  {
    for (int32_t i = (int32_t) getNumJoints(); i-- != 0; )
      ((JointJoltPhysJointed*)m_joints[i])->m_joint->Release();
    for (int32_t i = (int32_t) getNumParts(); i-- != 0; )
      ((PartJoltPhysJointed*)m_parts[i])->m_rigidBody->Release();
  }
  m_refCount = 0;
  return true;
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

  m_rigidBody = other.m_rigidBody;
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
  return nmJPHVec3ToVector3(m_rigidBody->GetLinearVelocity());
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Vector3 PhysicsRigJoltPhysJointed::PartJoltPhysJointed::getAngVel() const
{
  return nmJPHVec3ToVector3(m_rigidBody->GetAngularVelocity());
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
    const JPH::Mat44 currentGlobalPose = m_rigidBody->GetWorldTransform();

  m_cache.motiontype = m_rigidBody->GetMotionType();
  m_cache.collisionOn = getCollisionEnabled();

  m_cache.mass = 1.0 / m_rigidBody->GetMotionProperties()->GetInverseMass();
  m_cache.COMOffsetLocal = nmJPHMat44ToNmMatrix34(m_rigidBody->getCMassLocalPose());
  m_cache.globalPose = nmJPHMat44ToNmMatrix34(currentGlobalPose);

  // The transforms from PhysX can be pretty bad, so orthonormalise the result.
  m_cache.globalPose.orthonormalise();
  m_cache.kinematicTarget = m_cache.globalPose;

  m_cache.angularVel = nmJPHVec3ToVector3(m_rigidBody->GetAngularVelocity());
  m_cache.linearVel = nmJPHVec3ToVector3(m_rigidBody->GetLinearVelocity());

  updateCOMPosition();

  m_dirtyFlags = 0;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::PartJoltPhysJointed::applyModifiedValues()
{
  if (m_dirtyFlags)
  {
    if (m_dirtyFlags & kDirty_Collision)
    {
      physx::PxShape *shapes[MAX_SHAPES_IN_VOLUME];
      NMP_ASSERT(m_rigidBody->getNbShapes() <= MAX_SHAPES_IN_VOLUME);
      physx::PxU32 numShapes = m_rigidBody->getShapes(&shapes[0], MAX_SHAPES_IN_VOLUME);
      NMP_ASSERT(numShapes && shapes[0]);

      // Assume all have the same collision enabled/disabled status. However, note that they may have
      // different collision and query status.
      physx::PxShapeFlags flags = shapes[0]->getFlags(); 

      bool enabledSimulation = flags & physx::PxShapeFlag::eSIMULATION_SHAPE;
      if (enabledSimulation != m_cache.collisionOn)
      {
        for (physx::PxU32 i = 0 ; i < numShapes ; ++i)
        {
          shapes[i]->setFlag(physx::PxShapeFlag::eSIMULATION_SHAPE, m_cache.collisionOn);
        }
      }

      bool enabledQuery = flags & physx::PxShapeFlag::eSCENE_QUERY_SHAPE;
      if (enabledQuery != m_cache.collisionOn)
      {
        for (physx::PxU32 i = 0 ; i < numShapes ; ++i)
        {
          shapes[i]->setFlag(physx::PxShapeFlag::eSCENE_QUERY_SHAPE, m_cache.collisionOn);
        }
      }
      // Shouldn't need to call resetFiltering here, as the docs don't indicate that it's necessary
      // after setting the flags since the filter function uses eSUPPRESS (i.e. the filter shader will
      // get called again if the flags change).
    }

    if (m_dirtyFlags & kDirty_BodyFlags)
    {
      getRigidDynamic()->setRigidBodyFlags((physx::PxRigidBodyFlags)m_cache.bodyFlags);
    }

    if (m_dirtyFlags & kDirty_GlobalPose)
    {
      getRigidDynamic()->setGlobalPose(nmMatrix34ToPxTransform(m_cache.globalPose));
    }

    if (m_cache.bodyFlags & (physx::PxRigidBodyFlag::eKINEMATIC))
    {
      //Body is kinematic.
      if (m_dirtyFlags & kDirty_KinematicTarget)
      {
        getRigidDynamic()->setKinematicTarget(nmMatrix34ToPxTransform(m_cache.kinematicTarget));
      }
    }
    else
    {
      // Body is dynamic.
      if (m_dirtyFlags & kDirty_LinearVel)
      {
        m_rigidBody->SetLinearVelocity(nmVector3ToJPHVec3(m_cache.linearVel));
      }

      if (m_dirtyFlags & kDirty_AngularVel)
      {
        m_rigidBody->SetAngularVelocity(nmVector3ToJPHVec3(m_cache.angularVel));
      }
    }

    m_dirtyFlags = 0;
  }
}

//----------------------------------------------------------------------------------------------------------------------
// inertia and summations thereof
//----------------------------------------------------------------------------------------------------------------------
NMP::Vector3 PhysicsRigJoltPhysJointed::PartJoltPhysJointed::getMassSpaceInertiaTensor() const
{
  return nmJPHVec3ToVector3(m_rigidBody->GetMotionProperties()->GetLocalSpaceInverseInertia().GetDiagonal3());
}

//----------------------------------------------------------------------------------------------------------------------
// returns the mass moment of inertia in the top 3x3 components
// along with the com position in the translation component
NMP::Matrix34 PhysicsRigJoltPhysJointed::PartJoltPhysJointed::getGlobalInertiaTensor() const
{
  NMP::Vector3 t = nmJPHVec3ToVector3(m_rigidBody->GetMotionProperties()->GetLocalSpaceInverseInertia().GetDiagonal3());
  NMP::Matrix34 massSpaceInertiaTensor(NMP::Matrix34::kIdentity);
  massSpaceInertiaTensor.scale3x3(t);
  NMP::Matrix34 localOffset = getCOMOffsetLocal();
  NMP::Matrix34 result = massSpaceInertiaTensor * localOffset * getTransform();
  return result;
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Quat PhysicsRigJoltPhysJointed::PartJoltPhysJointed::getQuaternion() const
{
  return getTransform().toQuat();
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Matrix34 PhysicsRigJoltPhysJointed::PartJoltPhysJointed::getTransform() const
{
  if ((m_cache.motiontype == JPH::EMotionType::Kinematic) != 0)
  {
    return m_cache.kinematicTarget;
  }

  return m_cache.globalPose;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::PartJoltPhysJointed::updateCOMPosition()
{
  m_cache.COMPosition = 
      nmJPHVec3ToVector3(
          nmMatrix34ToJPHMat44(getTransform()).transform(nmVector3ToJPHVec3(m_cache.COMOffsetLocal.translation())));
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
  if (kinematic)
  {
      m_cache.motiontype = JPH::EMotionType::Kinematic;
  }
  else
  {
      m_cache.motiontype = JPH::EMotionType::Dynamic;
  }
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysJointed::PartJoltPhysJointed::isKinematic() const
{
  return (m_rigidBody->GetMotionType() == JPH::EMotionType::Kinematic);
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
  static const physx::PxU32 maxShapes = 1;
  physx::PxShape *shapes[maxShapes];
  m_rigidBody->getShapes(&shapes[0], maxShapes);
  NMP_ASSERT(shapes[0]);
  physx::PxShapeFlags flags = shapes[0]->getFlags(); // assume all have the same collision enabled/disabled status
  bool enabledOnDynamic = flags & physx::PxShapeFlag::eSIMULATION_SHAPE;//getActorFlags() & NX_AF_DISABLE_COLLISION;
  return enabledOnDynamic;
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
  uint32_t dataSize = sizeof(PhysicsPartPersistentData);

  uint32_t numBoxes = 0;
  uint32_t numCapsules = 0;
  uint32_t numSpheres = 0;

  JPH::CompoundShape* multipleshapes = dynamic_cast<JPH::CompoundShape*>(const_cast<JPH::Shape*>(m_rigidBody->GetShape()));

  uint32_t numShapes = multipleshapes ? multipleshapes->GetNumSubShapes() : 1;
  NMP_ASSERT(numShapes <= MAX_SHAPES_IN_VOLUME);

  JPH::Shape* shapes[MAX_SHAPES_IN_VOLUME];

  for (uint32_t i = 0; i != numShapes; ++i)
  {
    const JPH::Shape *shape = shapes[i];

    shape->GetSubType();

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
      const JPH::Shape *pxShape = shapes[i];

      JPH::EShapeSubType type = pxShape->GetSubType();
      switch (type)
      {
      case JPH::EShapeSubType::Sphere:
        {
          JPH::SphereShape* sphereshape = dynamic_cast<JPH::SphereShape*>((JPH::Shape*)pxShape);
          NMP_ASSERT(sphereshape);

          PhysicsSpherePersistentData* persistentData = partPersistentData->getSphere(indexSphere);

          //JPH::Mat44 localPose =
          persistentData->m_localPose = NMP::Matrix34Identity(); //nmPxTransformToNmMatrix34(localPose);

          persistentData->m_radius = sphereshape->GetRadius();

          NMP::netEndianSwap(persistentData->m_localPose);
          NMP::netEndianSwap(persistentData->m_radius);

          ++indexSphere;
          break;
        }
      case JPH::EShapeSubType::Box:
        {
          JPH::BoxShape* boxshape = dynamic_cast<JPH::BoxShape*>((JPH::Shape*)pxShape);
          NMP_ASSERT(boxshape);

          PhysicsBoxPersistentData* persistentData = partPersistentData->getBox(indexBox);

          //JPH::Mat44 localPose =
          persistentData->m_localPose = NMP::Matrix34Identity(); //nmPxTransformToNmMatrix34(localPose);

          persistentData->m_width = 2.0f * boxshape->GetHalfExtent().GetX();
          persistentData->m_height = 2.0f * boxshape->GetHalfExtent().GetY();
          persistentData->m_depth = 2.0f * boxshape->GetHalfExtent().GetZ();

          NMP::netEndianSwap(persistentData->m_localPose);
          NMP::netEndianSwap(persistentData->m_width);
          NMP::netEndianSwap(persistentData->m_height);
          NMP::netEndianSwap(persistentData->m_depth);

          ++indexBox;
          break;
        }
      case JPH::EShapeSubType::Capsule:
        {
          JPH::CapsuleShape* capsuleshape = dynamic_cast<JPH::CapsuleShape*>((JPH::Shape*)pxShape);
          NMP_ASSERT(capsuleshape);

          PhysicsCapsulePersistentData* persistentData = partPersistentData->getCapsule(indexCapsule);

          //JPH::Mat44 localPose = 
          persistentData->m_localPose = capsuleConversionTx; //* nmPxTransformToNmMatrix34(localPose);

          persistentData->m_radius = capsuleshape->GetRadius();
          persistentData->m_height = 2.0f * capsuleshape->GetHalfHeightOfCylinder();

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
  for (uint32_t i=0; i< JPH::SixDOFConstraintSettings::EAxis::Num; ++i)
  {
      m_cache.motions[i] = !m_joint->IsFreeAxis((JPH::SixDOFConstraintSettings::EAxis)i) ? 
          (!m_joint->IsFixedAxis((JPH::SixDOFConstraintSettings::EAxis)i) ? 
              eJoltphys_d6movement::eLimited : eJoltphys_d6movement::eFree) : eJoltphys_d6movement::eLocked;
  }

  m_cache.swingDrive = m_joint->GetMotorSettings(JPH::SixDOFConstraintSettings::EAxis::RotationY);
  m_cache.twistDrive = m_joint->GetMotorSettings(JPH::SixDOFConstraintSettings::EAxis::RotationX);
  m_cache.slerpDrive = m_joint->GetMotorSettings(JPH::SixDOFConstraintSettings::EAxis::RotationZ);
  
  JPH::Mat44 target = JPH::Mat44::sIdentity();
  target.sTranslation(m_joint->GetTargetPositionCS());
  target.sRotation(m_joint->GetTargetOrientationCS());
  NMP::Matrix34 tm = nmJPHMat44ToNmMatrix34(target);
  m_cache.driveOrientation = tm.toQuat();

  m_dirtyFlags = 0;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::JointJoltPhysJointed::applyModifiedValues()
{
  if (m_dirtyFlags)
  {
    if (m_dirtyFlags & kDirty_DriveOrientation)
    {
      m_joint->SetTargetOrientationCS(nmQuatToJPHQuat(m_cache.driveOrientation));
    }

    if (m_dirtyFlags & kDirty_Limits)
    {
      // Note that the current api only disable/enable the limits, modifying the motions, so those are the only
      // values that get cached. This could be extended with the swing and twist limits, if necessary.
      for (uint32_t i=0; i<physx::PxD6Axis::eCOUNT; ++i)
      {
        m_joint->setMotion((physx::PxD6Axis::Enum)i, m_cache.motions[i]);
      }
    }

    if (m_dirtyFlags & kDirty_SwingDrive)
    {
      m_joint->GetMotorSettings(JPH::SixDOFConstraintSettings::EAxis::RotationY) = m_cache.swingDrive;
    }

    if (m_dirtyFlags & kDirty_TwistDrive)
    {
      m_joint->GetMotorSettings(JPH::SixDOFConstraintSettings::EAxis::RotationX) = m_cache.twistDrive;
    }
    
    if (m_dirtyFlags & kDirty_SlerpDrive)
    {
      m_joint->GetMotorSettings(JPH::SixDOFConstraintSettings::EAxis::RotationZ) = m_cache.slerpDrive;
    }

    m_dirtyFlags = 0;
  }
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

    physx::PxJointLimitCone swingLimit = m_joint->getSwingLimit();

    persistentData->m_swing1Limit = swingLimit.yAngle;
    persistentData->m_swing2Limit = swingLimit.zAngle;

    physx::PxJointAngularLimitPair twistLimit = m_joint->getTwistLimit();

    persistentData->m_twistLimitLow = twistLimit.lower;
    persistentData->m_twistLimitHigh = twistLimit.upper;
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

  physx::PxD6Motion::Enum motion = enable ? physx::PxD6Motion::eLIMITED : physx::PxD6Motion::eFREE;

  m_cache.motions[physx::PxD6Axis::eSWING1] = motion;
  m_cache.motions[physx::PxD6Axis::eSWING2] = motion;
  m_cache.motions[physx::PxD6Axis::eTWIST] = motion;

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

  // Optimise this with MORPH-16668
  physx::PxJointLimitCone swingLimit(swing1, swing2, m_def->m_hardLimits.getSwingLimitContactDistance());
  physx::PxJointAngularLimitPair twistLimit(twistLow, twistHigh, m_def->m_hardLimits.getTwistLimitContactDistance());

  m_joint->SetRotationLimits
  m_joint->setSwingLimit(swingLimit);
  m_joint->setTwistLimit(twistLimit);
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
  if (slerpStrength != m_cache.slerpDrive.mSpringSettings.mStiffness)
  {
    m_dirtyFlags |= kDirty_SlerpDrive;
    m_cache.slerpDrive.mSpringSettings.mStiffness = slerpStrength;
  }
  if (swingStrength != m_cache.swingDrive.mSpringSettings.mStiffness)
  {
    m_dirtyFlags |= kDirty_SwingDrive;
    m_cache.swingDrive.mSpringSettings.mStiffness = swingStrength;
  }
  if (twistStrength != m_cache.twistDrive.mSpringSettings.mStiffness)
  {
    m_dirtyFlags |= kDirty_TwistDrive;
    m_cache.twistDrive.mSpringSettings.mStiffness = twistStrength;
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::JointJoltPhysJointed::setDriveDamping(float twistDamping, float swingDamping, float slerpDamping)
{
  if (slerpDamping != m_cache.slerpDrive.mSpringSettings.mDamping)
  {
    m_dirtyFlags |= kDirty_SlerpDrive;
    m_cache.slerpDrive.mSpringSettings.mDamping = slerpDamping;
  }
  if (swingDamping != m_cache.swingDrive.mSpringSettings.mDamping)
  {
    m_dirtyFlags |= kDirty_SwingDrive;
    m_cache.swingDrive.mSpringSettings.mDamping = swingDamping;
  }
  if (twistDamping != m_cache.twistDrive.mSpringSettings.mDamping)
  {
    m_dirtyFlags |= kDirty_TwistDrive;
    m_cache.twistDrive.mSpringSettings.mDamping = twistDamping;
  }
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
  for (uint32_t i = 0; i < getNumParts(); ++i)
  {
    PartJoltPhysJointed *part = (PartJoltPhysJointed*)m_parts[i];
    getPhysicsSceneJoltPhys()->m_joltPhysScene->GetBodyInterface().RemoveBody(part->m_rigidBody->GetID());
  }
}
//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::addToScene()
{
  NMP_ASSERT(m_refCount == 0);
  for (uint32_t i = 0; i < getNumParts(); ++i)
  {
    PartJoltPhysJointed *part = (PartJoltPhysJointed*)m_parts[i];
    getPhysicsSceneJoltPhys()->m_joltPhysScene->GetBodyInterface().AddBody(part->m_rigidBody->GetID(), JPH::EActivation::Activate);
  }
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

    // Set the separation properties - these will get reset in updatePostPhysics
    //j->m_joint->setProjectionLinearTolerance(m_desiredJointProjectionLinearTolerance);
    //j->m_joint->setProjectionAngularTolerance(m_desiredJointProjectionAngularTolerance);
    //j->m_joint->setConstraintFlag(physx::PxConstraintFlag::ePROJECTION, doProjection);
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::disableSleeping()
{
    getPhysicsSceneJoltPhys()->m_joltPhysScene->GetBodyInterface().ActivateBody(getPartJoltPhysJointed(0)->m_rigidBody->GetID());
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::updatePrePhysics(float NMP_UNUSED(timeStep))
{
  applyModifiedValues();
  updateRegisteredJoints();
  writeJointLimits();
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysJointed::updatePostPhysics(float NMP_UNUSED(timeStep))
{
  if (!isReferenced())
    return;
  reenableSleeping(); 
  PhysicsRigJoltPhysJointed* physicsRigPhysX = (PhysicsRigJoltPhysJointed*)this;
  physicsRigPhysX->generateCachedValues();

#if defined(MR_OUTPUT_DEBUGGING)
  // Copy joint limits from joint into serialisation data structure.
  for (uint32_t i = 0, numJoints = getNumJoints(); i < numJoints; ++i)
  {
    static_cast< JointJoltPhys* >(getJoint(i))->updateSerializeTxFrameData();
  }
#endif // MR_OUTPUT_DEBUGGING

  resetJointLimits();
}

} // namespace MR
//----------------------------------------------------------------------------------------------------------------------
