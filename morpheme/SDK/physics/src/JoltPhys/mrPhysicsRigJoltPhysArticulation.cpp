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
#include "physics/JoltPhys/mrJoltPhys.h"
#include "physics/JoltPhys/mrPhysicsDriverDataJoltPhys.h"
#include "physics/JoltPhys/mrPhysicsRigJoltPhysArticulation.h"
#include "physics/mrPhysicsRigDef.h"
#include "physics/mrPhysicsAttribData.h"
#include "physics/JoltPhys/mrPhysicsSceneJoltPhys.h"
#include "physics/mrPhysicsSerialisationBuffer.h"
#include "morpheme/mrAttribData.h"

#include "NMPlatform/NMProfiler.h"

#include "NMPlatform/NMvpu.h"

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


ArticulationExplosionHandler* PhysicsRigJoltPhysArticulation::s_explosionHandler = 0;

//----------------------------------------------------------------------------------------------------------------------
NMP::Memory::Format PhysicsRigJoltPhysArticulation::getMemoryRequirements(PhysicsRigDef *physicsRigDef)
{
  uint32_t numBones = physicsRigDef->getNumParts();
  uint32_t numJoints = physicsRigDef->getNumJoints();
  uint32_t numMaterials = physicsRigDef->getNumMaterials();

  NMP::Memory::Format result(sizeof(PhysicsRigJoltPhysArticulation), NMP_VECTOR_ALIGNMENT);

  // Space for the part pointers
  result += NMP::Memory::Format(sizeof(PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation*) * numBones, NMP_NATURAL_TYPE_ALIGNMENT);

  // Space for the joint pointers
  result += NMP::Memory::Format(sizeof(PhysicsRigJoltPhysArticulation::JointJoltPhysArticulation*) * numJoints, NMP_NATURAL_TYPE_ALIGNMENT);

  // Space for the parts
  result += NMP::Memory::Format(
    NMP::Memory::align(sizeof(PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation), NMP_NATURAL_TYPE_ALIGNMENT) * numBones,
    NMP_NATURAL_TYPE_ALIGNMENT);

  // Space for the joints
  result += NMP::Memory::Format(
    NMP::Memory::align(sizeof(PhysicsRigJoltPhysArticulation::JointJoltPhysArticulation), NMP_NATURAL_TYPE_ALIGNMENT) * numJoints,
    NMP_NATURAL_TYPE_ALIGNMENT);

  return result;
}


//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysArticulation::term()
{
  return true;
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysArticulation::PhysicsRigJoltPhysArticulation(PhysicsSceneJoltPhys *physicsScene)
{
  m_physicsScene = physicsScene;
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::PartJoltPhysArticulation(
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
  m_massMultiplier = 1.0f;
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::~PartJoltPhysArticulation() 
{
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::PartJoltPhysArticulation(const PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation &other) 
{
  *this = other;
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation &PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::operator=(const PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation &other) 
{
  if (this == &other)
    return *this;

#ifdef STORE_PART_AND_JOINT_NAMES
  memcpy(m_name, other.m_name, MAX_NAME_SIZE);
#endif
  m_parentPartIndex = other.m_parentPartIndex;
  return *this;
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::generateCachedValues(float timeStep)
{
  return true;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::handleExplosion(const NMP::Matrix34& worldRoot)
{
  if (m_isArticulationAddedToScene)
  {
    removeArticulationFromScene();
    addArticulationToScene();
  }

  for (uint32_t i = 0; i < getNumParts(); i++)
  {
    PartJoltPhysArticulation* partJoltPhysArticulation = (PartJoltPhysArticulation*)m_parts[i];

    NMP::Matrix34 tm;
    calculateWorldSpacePartTM(
      tm, 
      i, 
      *m_animRigDef->getBindPose()->m_transformBuffer, 
      *m_animRigDef->getBindPose()->m_transformBuffer, 
      worldRoot, 
      false);

    partJoltPhysArticulation->setTransform(tm);
    partJoltPhysArticulation->setVel(NMP::Vector3::InitZero);
    partJoltPhysArticulation->setAngVel(NMP::Vector3::InitZero);
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::generateCachedValues(float timeStep)
{
  bool OK = true;
  for (uint32_t i = 0; i < getNumParts(); i++)
  {
    if (!((PartJoltPhysArticulation*)m_parts[i])->generateCachedValues(timeStep))
    {
      OK = false;
    }
  }
  if (!OK)
  {
    NMP_MSG("PhysX has exploded - reinitialising at the last known position\n");

    PartJoltPhysArticulation* rootPartJoltPhysArticulation = (PartJoltPhysArticulation*)m_parts[0];
    NMP::Matrix34 worldRoot = rootPartJoltPhysArticulation->getTransform();
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
void PhysicsRigJoltPhysArticulation::updatePrePhysics(float timeStep)
{
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::updatePostPhysics(float timeStep)
{
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::setExtraMass(float mass, const NMP::Vector3& massCOMPosition)
{
  m_extraMass = mass;
  m_extraMassCOMPosition = massCOMPosition;
  m_modifiedFlags |= MODIFIED_EXTRA_MASS;
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Vector3 PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::getAugmentedCOMPosition() const
{
  return (m_cache.cachedCOMPosition * m_mass + m_extraMassCOMPosition * m_extraMass) / (m_mass + m_extraMass);
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Vector3 PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::getCOMPosition() const 
{
  return m_cache.cachedCOMPosition; 
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Vector3 PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::getVelocityAtPoint(const NMP::Vector3 &point) const
{
  NMP::Vector3 rpoint = point - getCOMPosition();
  return getVel() + NMP::vCross(getAngVel(), rpoint);
}

//---------------------------------------------------------------------------------------------------------------------- 
NMP::Vector3 PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::getAngularMomentum() const  
{
  NMP::Matrix34 inertia = getGlobalInertiaTensor();
  NMP::Vector3 angVel = getAngVel();
  NMP::Vector3 angMom;
  inertia.rotateVector(angVel, angMom);
  return angMom;
}  

//----------------------------------------------------------------------------------------------------------------------  
NMP::Vector3 PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::getLinearMomentum() const  
{    
  return getVel() * getMass();
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Vector3 PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::getMassSpaceInertiaTensor() const
{
  return nmJPHVec3ToVector3(m_rigidBody->getMassSpaceInertiaTensor());
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Matrix34 PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::getGlobalInertiaTensor() const
{
  NMP::Vector3 t = nmJPHVec3ToVector3(m_rigidBody->getMassSpaceInertiaTensor());
  NMP::Matrix34 massSpaceInertiaTensor(NMP::Matrix34::kIdentity);
  massSpaceInertiaTensor.scale3x3(t);
  NMP::Matrix34 localOffset = getCOMOffsetLocal();
  NMP::Matrix34 result = massSpaceInertiaTensor * localOffset * getTransform();
  return result;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::setMassSpaceInertia(const NMP::Vector3& inertia)
{
  // Node that this does little more than copying the data on the PhysX side, so should be pretty
  // fast.
  m_rigidBody->setMassSpaceInertiaTensor(MR::nmVector3ToJPHVec3(inertia));
  m_modifiedFlags |= MODIFIED_INERTIA;
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Quat PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::getQuaternion() const
{
  return nmJPHQuatToQuat(m_rigidBody->getGlobalPose().q);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::setVel(const NMP::Vector3 &v)
{
  m_rigidBody->setLinearVelocity(nmVector3ToPxVec3(v));
  m_cache.cachedVel = v;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::setAngVel(const NMP::Vector3 &v)
{
  m_rigidBody->setAngularVelocity(nmVector3ToPxVec3(v));
  m_cache.cachedAngVel = v;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::moveTo(const NMP::Matrix34 &tm, bool updateCache)
{
  // Note that with PhysX3 hardkeyframing the kinematic part uses move, but the dynamic part
  // position gets set.
  m_rigidBody->setGlobalPose(nmMatrix34ToPxTransform(tm));
  if (m_isKinematic && m_kinematicActor)
    m_kinematicActor->setKinematicTarget(nmMatrix34ToPxTransform(tm));
  if (updateCache)
  {
    m_cache.cachedTransform = tm;
    m_cache.cachedCOMPosition = nmPxVec3ToVector3(nmMatrix34ToPxTransform(tm).transform(m_rigidBody->getCMassLocalPose().p));
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::setPosition(const NMP::Vector3 &NMP_UNUSED(p))
{
  // This function is not currently implemented for PhysX3
  NMP_ASSERT_FAIL();
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::setQuaternion(const NMP::Quat &NMP_UNUSED(q))
{
  // This function is not currently implemented for PhysX3
  NMP_ASSERT_FAIL();
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::setTransform(const NMP::Matrix34 &tm)
{
  NMP_ASSERT(tm.isValidTM(0.1f));
  m_rigidBody->setGlobalPose(nmMatrix34ToPxTransform(tm));
  if (m_isKinematic && m_kinematicActor)
    m_kinematicActor->setGlobalPose(nmMatrix34ToPxTransform(tm));
  m_cache.cachedTransform = tm;
  m_cache.cachedCOMPosition = nmPxVec3ToVector3(nmMatrix34ToPxTransform(tm).transform(m_rigidBody->getCMassLocalPose().p));
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::makeKinematic(
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

  physx::PxArticulationLink *link = getArticulationLink();
  NMP_ASSERT(link);

  if (!kinematic)
  {
    // Move the kinematic shape somewhere far.
    if (m_kinematicActor)
    {
      m_kinematicActor->setGlobalPose(nmMatrix34ToPxTransform(
        ((PhysicsRigJoltPhysArticulation*)m_physicsRig)->m_kinematicPose));
    }
  }
  else
  {
    if (m_kinematicActor)
    {
      m_kinematicActor->setGlobalPose(link->getGlobalPose());
    }
  }
  m_isKinematic = kinematic;

  if (massMultiplier != m_massMultiplier)
  {
    link->setMass(m_mass * massMultiplier);
    link->setMassSpaceInertiaTensor(nmVector3ToPxVec3(m_inertia) * massMultiplier);
    m_massMultiplier = massMultiplier;
  }

  if (enableConstraint && !m_constraintToKinematic)
  {
    m_constraintToKinematic = PxD6JointCreate(
      PxGetPhysics(), 
      m_kinematicActor,
      physx::PxTransform(physx::PxIdentity),
      link,
      physx::PxTransform(physx::PxIdentity));
    m_constraintToKinematic->setMotion(physx::PxD6Axis::eX, physx::PxD6Motion::eLOCKED);
    m_constraintToKinematic->setMotion(physx::PxD6Axis::eY, physx::PxD6Motion::eLOCKED);
    m_constraintToKinematic->setMotion(physx::PxD6Axis::eZ, physx::PxD6Motion::eLOCKED);
    m_constraintToKinematic->setMotion(physx::PxD6Axis::eSWING1, physx::PxD6Motion::eLOCKED);
    m_constraintToKinematic->setMotion(physx::PxD6Axis::eSWING2, physx::PxD6Motion::eLOCKED);
    m_constraintToKinematic->setMotion(physx::PxD6Axis::eTWIST, physx::PxD6Motion::eLOCKED);
  }
  else if (!enableConstraint && m_constraintToKinematic)
  {
    m_constraintToKinematic->release();
    m_constraintToKinematic = 0;
  }

}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::isKinematic() const
{
  return m_isKinematic;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::enableCollision(bool enable)
{
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::getCollisionEnabled() const
{
  return enabledOnDynamic;
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::storeState(PhysicsSerialisationBuffer &savedState)
{
  savedState.addValue(getTransform());
  savedState.addValue(getVel());
  savedState.addValue(getAngVel());
  return true;
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::restoreState(PhysicsSerialisationBuffer &savedState)
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
uint32_t PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::serializeTxPersistentData(
  uint16_t nameToken, 
  uint32_t objectID, 
  void* outputBuffer, 
  uint32_t NMP_USED_FOR_ASSERTS(outputBufferSize)) const
{
  uint32_t dataSize = sizeof(PhysicsPartPersistentData);

  uint32_t numBoxes = 0;
  uint32_t numCapsules = 0;
  uint32_t numSpheres = 0;

  physx::PxU32 numShapes = m_kinematicActor->getNbShapes();
  NMP_ASSERT(numShapes < MAX_SHAPES_IN_VOLUME);

  physx::PxShape* shapes[MAX_SHAPES_IN_VOLUME];
  numShapes = m_kinematicActor->getShapes(shapes, MAX_SHAPES_IN_VOLUME);
  for (physx::PxU32 i = 0; i != numShapes; ++i)
  {
    const physx::PxShape *shape = shapes[i];

    physx::PxGeometryType::Enum type = shape->getGeometryType();
    switch (type)
    {
    case physx::PxGeometryType::eSPHERE:
      ++numSpheres;
      break;
    case physx::PxGeometryType::eBOX:
      ++numBoxes;
      break;
    case physx::PxGeometryType::eCAPSULE:
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
    for (physx::PxU32 i = 0; i != numShapes; ++i)
    {
      const physx::PxShape *pxShape = shapes[i];

      physx::PxGeometryType::Enum type = pxShape->getGeometryType();
      switch (type)
      {
      case physx::PxGeometryType::eSPHERE:
        {
          physx::PxSphereGeometry pxSphere;
          NMP_USED_FOR_ASSERTS(bool result =) pxShape->getSphereGeometry(pxSphere);
          NMP_ASSERT(result);

          PhysicsSpherePersistentData* persistentData = partPersistentData->getSphere(indexSphere);

          physx::PxTransform localPose = pxShape->getLocalPose();
          persistentData->m_localPose = nmPxTransformToNmMatrix34(localPose);

          persistentData->m_radius = pxSphere.radius;

          persistentData->m_parentIndex = i;
          NMP::netEndianSwap(persistentData->m_parentIndex);
          NMP::netEndianSwap(persistentData->m_localPose);
          NMP::netEndianSwap(persistentData->m_radius);

          ++indexSphere;
          break;
        }
      case physx::PxGeometryType::eBOX:
        {
          physx::PxBoxGeometry pxBox;
          NMP_USED_FOR_ASSERTS(bool result =) pxShape->getBoxGeometry(pxBox);
          NMP_ASSERT(result);

          PhysicsBoxPersistentData* persistentData = partPersistentData->getBox(indexBox);

          physx::PxTransform localPose = pxShape->getLocalPose();
          persistentData->m_localPose = nmPxTransformToNmMatrix34(localPose);

          persistentData->m_width = 2.0f * pxBox.halfExtents.x;
          persistentData->m_height = 2.0f * pxBox.halfExtents.y;
          persistentData->m_depth = 2.0f * pxBox.halfExtents.z;

          persistentData->m_parentIndex = i;
          NMP::netEndianSwap(persistentData->m_parentIndex);
          NMP::netEndianSwap(persistentData->m_localPose);
          NMP::netEndianSwap(persistentData->m_width);
          NMP::netEndianSwap(persistentData->m_height);
          NMP::netEndianSwap(persistentData->m_depth);

          ++indexBox;
          break;
        }
      case physx::PxGeometryType::eCAPSULE:
        {
          physx::PxCapsuleGeometry pxCapsule;
          NMP_USED_FOR_ASSERTS(bool result =) pxShape->getCapsuleGeometry(pxCapsule);
          NMP_ASSERT(result);

          PhysicsCapsulePersistentData* persistentData = partPersistentData->getCapsule(indexCapsule);

          physx::PxTransform localPose = pxShape->getLocalPose();
          persistentData->m_localPose = capsuleConversionTx * nmPxTransformToNmMatrix34(localPose);

          persistentData->m_radius = pxCapsule.radius;
          persistentData->m_height = 2.0f * pxCapsule.halfHeight;

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
uint32_t PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation::serializeTxFrameData(void* outputBuffer, uint32_t NMP_USED_FOR_ASSERTS(outputBufferSize)) const
{
  uint32_t dataSize = sizeof(PhysicsPartFrameData);

  if (outputBuffer != 0)
  {
    NMP_ASSERT(outputBufferSize >= dataSize);
    PhysicsPartFrameData *partFrameData = (PhysicsPartFrameData *)outputBuffer;

    physx::PxTransform globalPose;
    if (m_isKinematic)
    {
      globalPose = m_kinematicActor->getGlobalPose();
    }
    else
    {
      globalPose = m_rigidBody->getGlobalPose();
    }
    partFrameData->m_globalPose = nmPxTransformToNmMatrix34(globalPose);

    NMP::netEndianSwap(partFrameData->m_globalPose);
  }

  return dataSize;
}
#endif

//----------------------------------------------------------------------------------------------------------------------
// PhysicsRigJoltPhysArticulation::JointPhysX3Articulation
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysArticulation::JointPhysX3Articulation::JointPhysX3Articulation(const PhysicsSixDOFJointDef* const def)
: JointPhysX3(def)
{}

#if defined(MR_OUTPUT_DEBUGGING)
//----------------------------------------------------------------------------------------------------------------------
uint32_t PhysicsRigJoltPhysArticulation::JointPhysX3Articulation::serializeTxPersistentData(
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

    physx::PxReal swingLimitY = 0.0f;
    physx::PxReal swingLimitZ = 0.0f;
    m_jointInternal->getSwingLimit(swingLimitY, swingLimitZ);

    // for some reason PxArticulationJoint returns the swing limit values
    // the wrong way round so swap the y and z values.
    persistentData->m_swing1Limit = swingLimitZ;
    persistentData->m_swing2Limit = swingLimitY;

    physx::PxReal twistLimitLow = 0.0f;
    physx::PxReal twistLimitHigh = 0.0f;
    m_jointInternal->getTwistLimit(twistLimitLow, twistLimitHigh);

    persistentData->m_twistLimitLow = twistLimitLow;
    persistentData->m_twistLimitHigh = twistLimitHigh;
    persistentData->m_nameToken = stringToken;

    PhysicsSixDOFJointPersistentData::endianSwap(persistentData);
  }

  return dataSize;
}
#endif

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysArticulation::JointPhysX3Articulation::storeState(PhysicsSerialisationBuffer& savedState)
{
  (void) savedState;
  return true;
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysArticulation::JointPhysX3Articulation::restoreState(PhysicsSerialisationBuffer& savedState)
{
  (void) savedState;
  return true;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::JointPhysX3Articulation::enableLimit(bool enable)
{
  m_jointInternal->setSwingLimitEnabled(enable);
  m_jointInternal->setTwistLimitEnabled(enable);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::JointPhysX3Articulation::writeLimits()
{
  // PhysX crashes with zero swing range, as well as it just causing jitter when very small (e.g.
  // with "hinge" limits).
  float swing1 = NMP::maximum(m_modifiableLimits.getSwing1Limit(), s_minSwingLimit);
  float swing2 = NMP::maximum(m_modifiableLimits.getSwing2Limit(), s_minSwingLimit);
  float twistLow = m_modifiableLimits.getTwistLimitLow();
  float twistHigh = m_modifiableLimits.getTwistLimitHigh();

  // Optimise this with MORPH-16668. Note that swing1/2 are reversed (compare with the jointed rig
  // where they're not reversed) - this is intentional - just how the joint is in PhysX.
  m_jointInternal->setSwingLimit(swing2, swing1);
  m_jointInternal->setTwistLimit(twistLow, twistHigh);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::JointPhysX3Articulation::setStrength(float strength)
{
  NMP_ASSERT(m_jointInternal);
  NMP_ASSERT(strength >= 0.0f && strength < MAX_STRENGTH);

  m_strength = strength;
  m_jointInternal->setStiffness(strength);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::JointPhysX3Articulation::setDamping(float damping)
{
  NMP_ASSERT(m_jointInternal);
  NMP_ASSERT(damping >= 0.0f && damping < MAX_DAMPING);
  m_damping = damping;
  m_jointInternal->setDamping(damping);
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysArticulation::JointPhysX3Articulation::supportsDriveCompensation()
{
  return true;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::JointPhysX3Articulation::setDriveCompensation(float driveCompensation)
{
  NMP_ASSERT(m_jointInternal);
  NMP_ASSERT(driveCompensation >= 0.f); 
  setInternalCompliance(1.f/(1.f + driveCompensation));
}

//----------------------------------------------------------------------------------------------------------------------
float PhysicsRigJoltPhysArticulation::JointPhysX3Articulation::getStrength() const
{
  return m_strength;
}

//----------------------------------------------------------------------------------------------------------------------
float PhysicsRigJoltPhysArticulation::JointPhysX3Articulation::getDamping() const
{
  return m_damping;
}

//----------------------------------------------------------------------------------------------------------------------
float PhysicsRigJoltPhysArticulation::JointPhysX3Articulation::getDriveCompensation() const
{
  NMP_ASSERT(m_jointInternal);
  float internalCompliance = m_jointInternal->getInternalCompliance();
  if (internalCompliance < MINIMUM_COMPLIANCE)
    internalCompliance = MINIMUM_COMPLIANCE;
  return (1.0f / internalCompliance) - 1.0f;
}

//----------------------------------------------------------------------------------------------------------------------
float PhysicsRigJoltPhysArticulation::JointPhysX3Articulation::getInternalCompliance() const
{
  return m_jointInternal->getInternalCompliance();
}

//----------------------------------------------------------------------------------------------------------------------
float PhysicsRigJoltPhysArticulation::JointPhysX3Articulation::getExternalCompliance() const
{
  return m_jointInternal->getExternalCompliance();
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::JointPhysX3Articulation::setExternalCompliance(float compliance) 
{
  if (compliance < MINIMUM_COMPLIANCE)
    compliance = MINIMUM_COMPLIANCE;
  m_jointInternal->setExternalCompliance(compliance);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::JointPhysX3Articulation::setInternalCompliance(float compliance) 
{
  if (compliance < MINIMUM_COMPLIANCE)
    compliance = MINIMUM_COMPLIANCE;
  m_jointInternal->setInternalCompliance(compliance);
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Quat PhysicsRigJoltPhysArticulation::JointPhysX3Articulation::getTargetOrientation()
{
  NMP_ASSERT(m_jointInternal);
  return nmPxQuatToQuat(m_jointInternal->getTargetOrientation());
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::JointPhysX3Articulation::setTargetOrientation(const NMP::Quat &orientation)
{
  NMP_ASSERT(orientation.isValid());
  // This check allows characters to go to sleep when a constant target is being passed in.
  // Since setTargetOrientation wakes up the character.
  if (orientation != m_lastTargetOrientation)
  {
    m_jointInternal->setTargetOrientation(nmQuatToPxQuat(orientation));
  }
  m_lastTargetOrientation = orientation;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::JointPhysX3Articulation::setVelocity(const NMP::Vector3 &velocity)
{
  NMP_ASSERT(velocity.isValid());
  m_jointInternal->setTargetVelocity(nmVector3ToPxVec3(velocity));
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::makeKinematic(bool moveToKinematicPos)
{
  NMP_ASSERT(m_refCount == 0);

  for (uint32_t i = 0; i < getNumParts(); ++i)
  {
    PartJoltPhysArticulation *part = (PartJoltPhysArticulation*)m_parts[i];
    part->makeKinematic(true, 1.0f, false);
    if (part->m_kinematicActor)
      part->enableActorCollision(part->m_kinematicActor, true);
    part->enableActorCollision(part->m_rigidBody, false);
  }

  if (moveToKinematicPos)
  {
    moveAllToKinematicPos();
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::removeFromScene()
{
  NMP_ASSERT(m_refCount == 0);

  // Workaround for MORPH-15443 where physx sometimes crashes if the articulation is asleep when
  // added to the scene. Still there in PhysX 3.2.1
  m_articulation->wakeUp();

  removeArticulationFromScene();
  for (uint32_t i = 0; i < getNumParts(); ++i)
  {
    PartJoltPhysArticulation *part = (PartJoltPhysArticulation*)m_parts[i];
    if (part->m_kinematicActor)
    {
      getPhysicsScenePhysX()->getPhysXScene()->removeActor(*part->m_kinematicActor);
    }
  }
}
//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::addToScene()
{
  NMP_ASSERT(m_refCount == 0);
  for (uint32_t i = 0; i < getNumParts(); ++i)
  {
    PartJoltPhysArticulation *part = (PartJoltPhysArticulation*)m_parts[i];
    if (part->m_kinematicActor)
    {
      getPhysicsScenePhysX()->getPhysXScene()->addActor(*part->m_kinematicActor);
    }
  }
  addArticulationToScene();
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::addArticulationToScene()
{
  if (m_isArticulationAddedToScene)
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
#ifdef USE_ARTICULATION_AGGREGATE
  getPhysicsScenePhysX()->getPhysXScene()->addAggregate(*m_aggregate);
#else
  getPhysicsScenePhysX()->getPhysXScene()->addArticulation(*m_articulation);
#endif

  m_isArticulationAddedToScene = true;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::removeArticulationFromScene()
{
  if (!m_isArticulationAddedToScene)
    return;
#ifdef USE_ARTICULATION_AGGREGATE
  getPhysicsScenePhysX()->getPhysXScene()->removeAggregate(*m_aggregate);
#else
  getPhysicsScenePhysX()->getPhysXScene()->removeArticulation(*m_articulation);
#endif
  m_isArticulationAddedToScene = false;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::makeDynamic()
{
  if (!m_isArticulationAddedToScene)
    addArticulationToScene();

  for (uint32_t i = 0; i < getNumParts(); ++i)
  {
    PartJoltPhysArticulation *part = (PartJoltPhysArticulation*)m_parts[i];
    part->makeKinematic(false, 1.0f, false);
    if (part->m_kinematicActor)
      part->enableActorCollision(part->m_kinematicActor, false);
    part->enableActorCollision(part->m_rigidBody, true);
  }

  for (uint32_t i=0; i<getNumJoints(); ++i)
  {
    PhysicsRigJoltPhysArticulation::JointPhysX3Articulation *joint = (JointPhysX3Articulation*)getJoint(i);
    // enable the joint limit
    joint->enableLimit(true);
  }

  // Re-enable gravity.  This will have been disabled by moveAllToKinematicPos if it was previously called.
  if (m_refCount == 0)
  {
    for (uint32_t i = 0; i < getNumParts(); ++i)
    {
      PartJoltPhysArticulation *part = (PartJoltPhysArticulation*)m_parts[i];
      // re-enable gravity
      part->m_rigidBody->setActorFlag(physx::PxActorFlag::eDISABLE_GRAVITY, false);
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::moveAllToKinematicPos()
{
  physx::PxVec3 delta = nmVector3ToPxVec3(m_kinematicPose.translation()) - ((PartJoltPhysArticulation*)getPart(0))->m_rigidBody->getGlobalPose().p;

  // Move the kinematic shape somewhere far.
  physx::PxTransform kinematicPose = nmMatrix34ToPxTransform(m_kinematicPose);

  for (uint32_t i = 0; i < getNumParts(); ++i)
  {
    // move the dynamic part
    PartJoltPhysArticulation *part = (PartJoltPhysArticulation*)m_parts[i];
    physx::PxTransform t = part->m_rigidBody->getGlobalPose();
    t.p += delta;
    part->m_rigidBody->setGlobalPose(t);
    part->m_rigidBody->setLinearVelocity(physx::PxVec3(0,0,0));
    part->m_rigidBody->setAngularVelocity(physx::PxVec3(0,0,0));
    // disable gravity
    part->m_rigidBody->setActorFlag(physx::PxActorFlag::eDISABLE_GRAVITY, true);
    if (part->m_kinematicActor)
      part->m_kinematicActor->setGlobalPose(kinematicPose);
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::restoreAllJointDrivesToDefault()
{
  for (uint32_t i = 0 ;i < getNumJoints(); ++i)
  {
    PhysicsRigJoltPhysArticulation::JointPhysX3Articulation *joint = (JointPhysX3Articulation*)m_joints[i];
    joint->setStrength(0.0f);
    joint->setDamping(joint->getMaxDamping());
    joint->setExternalCompliance(1.0f);
    joint->setInternalCompliance(1.0f);
  }

  m_desiredJointProjectionIterations = 0;
  m_desiredJointProjectionLinearTolerance = FLT_MAX;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::disableSleeping()
{
  float threshold = m_articulation->getSleepThreshold();
  if (threshold > 0.0f)
    m_cachedSleepThreshold = threshold;
  m_articulation->setSleepThreshold(0.0f);
  m_articulation->wakeUp();
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::reenableSleeping()
{
  if (m_articulation->getSleepThreshold() == 0.0f)
    m_articulation->setSleepThreshold(m_cachedSleepThreshold);
}

//----------------------------------------------------------------------------------------------------------------------
#if !defined(NMP_PLATFORM_SIMD) || defined(NM_HOST_IOS)
void PhysicsRigJoltPhysArticulation::applyHardKeyframing(
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

    PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation *part = (PartJoltPhysArticulation*)m_parts[j];
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
      PhysicsRigJoltPhysArticulation::JointPhysX3Articulation *joint = (JointPhysX3Articulation*)getJoint(i);
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
void PhysicsRigJoltPhysArticulation::applyHardKeyframing(
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

    PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation *part = (PartJoltPhysArticulation*)m_parts[j];
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
      if (part->isKinematic() && part->getKinematicActor())
        part->getKinematicActor()->setKinematicTarget(nmMatrix34ToPxTransform(M34vpu(targetTMs[j+1])));

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
    removeArticulationFromScene();
  }
  else
  {
    // Go through all the joints and disable joint limits when both parts are HK. This is undone by
    // the call to enable the joint limit in makeDynamic
    for (uint32_t i = 0 ; i < getNumJoints() ; ++i)
    {
      PhysicsRigJoltPhysArticulation::JointPhysX3Articulation *joint = (JointPhysX3Articulation*)getJoint(i);
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
void PhysicsRigJoltPhysArticulation::applySoftKeyframing(
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

    PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation *part = (PartJoltPhysArticulation*)m_parts[i];

    part->makeKinematic(false, 1.0f, false);
    part->m_isBeingKeyframed = true;
    // Enable the collision on the dynamic actor if desired.
    part->enableActorCollision(part->m_rigidBody, enableCollision);
    // Disable collision on the kinematic actor.
    if (part->m_kinematicActor)
      part->enableActorCollision(part->m_kinematicActor, false);

    // Set the external compliance on the associated joint (joint index = part index - 1, so there's
    // no joint for the root part).
    if (i != 0)
    {
      PhysicsRigJoltPhysArticulation::JointPhysX3Articulation *joint = (JointPhysX3Articulation*)getJoint(i-1);
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
          PhysicsRigJoltPhysArticulation::JointPhysX3Articulation* joint = (JointPhysX3Articulation*)getJoint(parentJointIndex);
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
  NMP::Vector3 gravityDeltaVel = getPhysicsScenePhysX()->getGravity() * (weight * dt * gravityCompensationFrac);

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

    PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation *part = (PartJoltPhysArticulation*)m_parts[i];

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
      PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation *part = (PartJoltPhysArticulation*)m_parts[i];
      NMP::Vector3 partVel = part->getVel();
      part->setVel(partVel + correctionVel);
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------- 
void PhysicsRigJoltPhysArticulation::applyActiveAnimation(uint32_t jointIndex, const NMP::Quat &targetQuat, bool makeChildDynamic) 
{ 
  NMP_ASSERT(jointIndex < getNumJoints());  
  physx::PxArticulationJoint *joint = ((JointPhysX3Articulation*)m_joints[jointIndex])->m_jointInternal;  
  const PhysicsJointDef* jointDef = m_physicsRigDef->m_joints[jointIndex];
  if (makeChildDynamic)  
  {   
    PartJoltPhysArticulation *childPart = (PartJoltPhysArticulation*)m_parts[jointDef->m_childPartIndex];     
    childPart->makeKinematic(false, 1.0f, false);
    childPart->m_isBeingKeyframed = false;
  }   
  // Don't force either of the parts to have collision - no way we could know which one _should_ have collision if it's
  // disabled elsewhere.
  joint->setTargetOrientation(nmQuatToPxQuat(targetQuat));  
  return; 
}

//----------------------------------------------------------------------------------------------------------------------
// Drives the joints to the targets given by the input animation buffer.
void PhysicsRigJoltPhysArticulation::applyActiveAnimation(
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

    PhysicsRigJoltPhysArticulation::JointPhysX3Articulation *joint = (JointPhysX3Articulation*)m_joints[i];
    const PhysicsSixDOFJointDef* jointDef = static_cast<const PhysicsSixDOFJointDef*>(m_physicsRigDef->m_joints[i]);
    PhysicsRigJoltPhysArticulation::PartJoltPhysArticulation *childPart = (PartJoltPhysArticulation*)m_parts[jointDef->m_childPartIndex];
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
    joint->setTargetOrientation(curFrameQ);
  }
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Quat PhysicsRigJoltPhysArticulation::getJointQuat(uint32_t jointIndex) 
{
  NMP_ASSERT(jointIndex < getNumJoints());
  JointPhysX3Articulation *joint = (JointPhysX3Articulation*)m_joints[jointIndex];
  const PhysicsJointDef* jointDef = m_physicsRigDef->m_joints[jointIndex];
  if (joint->m_rotationDirty)
  {
    NMP::Quat part1Quat = (getPartJoltPhysArticulation(jointDef->m_parentPartIndex))->getQuaternion();
    NMP::Quat part2Quat = (getPartJoltPhysArticulation(jointDef->m_childPartIndex))->getQuaternion();
    joint->m_actualRotation = ~(part1Quat * jointDef->m_parentFrameQuat) * (part2Quat * jointDef->m_childFrameQuat); 
    joint->m_rotationDirty = false;
  }
  return joint->m_actualRotation;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::setCollisionGroupsToActivate(const int *collisionGroupIndices, int numCollisionGroupIndices)
{
  NMP_ASSERT(numCollisionGroupIndices <= m_maxCollisionGroupIndices);
  m_numCollisionGroupIndices = NMP::minimum(m_maxCollisionGroupIndices, numCollisionGroupIndices);
  
  for (int i = 0 ; i < m_numCollisionGroupIndices ; ++i)
  {
    m_collisionGroupIndicesToActivate[i] = collisionGroupIndices[i];
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::dumpToRepX(physx::repx::RepXCollection *collection, 
                                              physx::repx::RepXIdToRepXObjectMap *idMap) const
{
    //salsatobias: jesus christ physx, why did you change things so much ?
//  for (uint32_t iMaterial = 0 ; iMaterial < m_physicsRigDef->m_numMaterials ; ++iMaterial)
//  {
//    physx::repx::RepXObject material = physx::repx::createRepXObject(m_materials[iMaterial]);
//#ifdef NMP_ENABLE_ASSERTS
//    physx::repx::RepXAddToCollectionResult result =
//#endif
//      collection->addRepXObjectToCollection(material, *idMap);
//    NMP_ASSERT(result.isValid());
//  }
//
//  physx::repx::RepXObject articulation = physx::repx::createRepXObject(m_articulation);
//#ifdef NMP_ENABLE_ASSERTS
//  physx::repx::RepXAddToCollectionResult result =
//#endif
//    collection->addRepXObjectToCollection(articulation, *idMap);
//  NMP_ASSERT(result.isValid());
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysArticulation::setExplosionHandler(ArticulationExplosionHandler* handler)
{
  s_explosionHandler = handler;
}

} // namespace MR

//----------------------------------------------------------------------------------------------------------------------
