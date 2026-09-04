// Copyright (c) 2009 NaturalMotion.  All Rights Reserved.
// Not to be copied, adapted, modified, used, distributed, sold,
// licensed or commercially exploited in any manner without the
// written consent of NaturalMotion.  
//
// All non public elements of this software are the confidential
// information of NaturalMotion and may not be disclosed to any
// person nor used for any purpose not expressly approved by
// NaturalMotion in writing.

//----------------------------------------------------------------------------------------------------------------------
#include "mrPhysicsSceneKarma.h"
#include "mrKarma.h"
#include "mrPhysicsRigKarma.h"
#include "mrCharacterControllerInterfaceKarma.h"
#include "physics/mrPhysicsSerialisationBuffer.h"
#include "mrKarma.h"
//----------------------------------------------------------------------------------------------------------------------

namespace MR
{

KarmaPerBodyData::BodyToDataMap* PhysXPerShapeData::s_shapeToDataMap = 0;
NMP::HeapAllocator* PhysXPerShapeData::s_mapAllocator = 0;

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsScenePhysX3::rayCollide(physx::PxRaycastHit &raycastHit,
                                    const NMP::Vector3 &position, 
                                    const NMP::Vector3 &direction, 
                                    float distance, 
                                    uint32_t collisionIgnoreMask,
                                    physx::PxClientID ownerClientID) const
{
  // Use default PxFilterData here to bypass the internal filtering, and we pass the real flags
  // into the callback.
  physx::PxSceneQueryFilterData filterData; // by default hits all static & dynamic - requires masks to be all zero(?!)

  filterData.flags = physx::PxQueryFlags(
    physx::PxQueryFlag::eSTATIC | 
    physx::PxQueryFlag::eDYNAMIC | 
    physx::PxQueryFlag::ePREFILTER);

  physx::PxSceneQueryFlags flags(
    physx::PxSceneQueryFlag::ePOSITION | 
    physx::PxSceneQueryFlag::eNORMAL | 
    physx::PxSceneQueryFlag::eDISTANCE);

  MorphemePhysX3QueryFilterCallback morphemePhysX3QueryFilterCallback(physx::PxFilterData(
    0,
    collisionIgnoreMask,
    0,
    0));

  physx::PxRaycastBuffer raycastBuffer;

  bool result = m_physXScene->raycast(
    nmVector3ToPxVec3(position), 
    nmVector3ToPxVec3(direction), 
    distance, 
    raycastBuffer,
    flags,
    filterData,
    &morphemePhysX3QueryFilterCallback,
    NULL);
  raycastHit = raycastBuffer.block;
  return result;
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsScenePhysX3::PhysicsScenePhysX3(physx::PxScene *physXScene) :
  m_physXScene(physXScene)
{
  PhysicsRigPhysX3ActorData::init();

  if (physXScene)
  {
    setPhysXScene(physXScene);
  }
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsScenePhysX3::~PhysicsScenePhysX3()
{
  PhysicsRigPhysX3ActorData::term();
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsScenePhysX3::setPhysXScene(physx::PxScene *physXScene) 
{ 
  m_physXScene = physXScene;   
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Vector3 PhysicsScenePhysX3::getFloorPositionBelow(
  const NMP::Vector3 &pos, 
  const PhysicsRig *skipChar, 
  float distToCheck) const
{
  // Rather than skipping the character, skip all characters - that means we won't detect another
  // ragdoll etc as the ground...
  uint32_t ignore = 1<<MR::GROUP_NON_COLLIDABLE | 1<<MR::GROUP_INTERACTION_PROXY | 1<<MR::GROUP_CHARACTER_CONTROLLER;
  if (skipChar)
  {
    ignore |= 1<<MR::GROUP_CHARACTER_PART;
  }
  NMP::Vector3 rayDirection = m_worldUpDirection * -1.0f;
  physx::PxRaycastHit hit;
  const PhysicsRigPhysX3* physicsRigPhysX3 = (const PhysicsRigPhysX3*) skipChar;
  physx::PxClientID clientID = physicsRigPhysX3 ? physicsRigPhysX3->getClientID() : physx::PX_DEFAULT_CLIENT;
  if (rayCollide(hit, pos, rayDirection, distToCheck, ignore, clientID))
  {
    return nmPxVec3ToVector3(hit.position);
  }
  else
  {
    return pos + (rayDirection * distToCheck);
  }
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsScenePhysX3::castRay(
    const NMP::Vector3& start,
    const NMP::Vector3& delta,
    const PhysicsRig* skipChar,
    const CharacterControllerInterface* skipCharController,
    float& hitDistance,
    NMP::Vector3& hitPosition,
    NMP::Vector3& hitNormal,
    NMP::Vector3& hitVelocity) const
{
  NMP::Vector3 dir = delta;
  float len = dir.normaliseGetLength();
  // Rather than skipping the character, skip all characters - that means we won't detect another
  // ragdoll etc as the ground...
  uint32_t ignore = 1<<MR::GROUP_NON_COLLIDABLE | 1<<MR::GROUP_INTERACTION_PROXY;
  if (skipChar)
  {
    ignore |= 1<<MR::GROUP_CHARACTER_PART;
  }
  if (skipCharController)
  {
    ignore |= 1<<MR::GROUP_CHARACTER_CONTROLLER;
  }

  physx::PxRaycastHit hit;

  const PhysicsRigPhysX3* physicsRigPhysX3 = (const PhysicsRigPhysX3*) skipChar;
  physx::PxClientID clientID = physicsRigPhysX3 ? physicsRigPhysX3->getClientID() : physx::PX_DEFAULT_CLIENT;
  if (rayCollide(hit, start, dir, len, ignore, clientID))
  {
    hitPosition = nmPxVec3ToVector3(hit.position);
    hitNormal = nmPxVec3ToVector3(hit.normal);
    hitDistance = hit.distance;
    hitVelocity.setToZero();

    if (hit.shape)
    {
      const physx::PxRigidBody* rigidBody = hit.shape->getActor()->is<physx::PxRigidBody>();
      if (rigidBody)
      {
        hitVelocity = MR::nmPxVec3ToVector3(physx::PxRigidBodyExt::getVelocityAtPos(*rigidBody, hit.position));
      }
    }
    return true;
  }
  return false;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsScenePhysX3::setGravity(const NMP::Vector3& gravity)
{
  physx::PxVec3 nxGravity(gravity.x, gravity.y, gravity.z);
  getPhysXScene()->setGravity(nxGravity);
}

//----------------------------------------------------------------------------------------------------------------------
// aux used to convert position from actorlocal to world frame
void actorToWorldPoint(const physx::PxActor * actor, const NMP::Vector3& xLocal, NMP::Vector3& xWorld)
{
  NMP::Matrix34 actorToWorld(NMP::Matrix34::kIdentity);
  MR::getActorGlobalPoseTM(*actor, actorToWorld);
  actorToWorld.transformVector(xLocal, xWorld);
}

//----------------------------------------------------------------------------------------------------------------------
// aux used to convert normal from actorlocal to world frame
void actorToWorldNormal(const physx::PxActor * actor, const NMP::Vector3& nLocal, NMP::Vector3& nWorld)
{
  NMP::Matrix34 actorToWorld;
  MR::getActorGlobalPoseTM(*actor, actorToWorld);
  actorToWorld.rotateVector(nLocal, nWorld);
}

//----------------------------------------------------------------------------------------------------------------------
// aux used to convert position from world to actorlocal frame
void worldToActorPoint(const physx::PxActor* actor, const NMP::Vector3& xWorld, NMP::Vector3& xLocal)
{
  NMP::Matrix34 worldToActor;
  MR::getActorGlobalPoseTM(*actor, worldToActor);
  worldToActor.invertFast();
  worldToActor.transformVector(xWorld, xLocal);
}

//----------------------------------------------------------------------------------------------------------------------
// aux used to convert normal from world to actorlocal frame
void worldToActorNormal(const physx::PxActor* actor, const NMP::Vector3& nWorld, NMP::Vector3& nLocal)
{
  NMP::Matrix34 worldToActor;
  MR::getActorGlobalPoseTM(*actor, worldToActor);
  worldToActor.invertFast();
  worldToActor.rotateVector(nWorld, nLocal);
}

void actorToWorldImpulse(physx::PxActor* actor, const NMP::Vector3 &dirL, const NMP::Vector3 &posL, NMP::Vector3 &dirW, NMP::Vector3 &posW)
{
  NMP::Matrix34 actorToWorld;
  getActorGlobalPoseTM(*actor, actorToWorld);  
  actorToWorld.rotateVector(dirL, dirW);
  actorToWorld.transformVector(posL, posW);
}

//----------------------------------------------------------------------------------------------------------------------
void addLocalImpulseAtLocalPosToActor(physx::PxActor& actor, const NMP::Vector3 &impulse, const NMP::Vector3 &position, float torqueMultiplier /*= 1.0f*/)
{
  NMP::Matrix34 actorToWorld;
  getActorGlobalPoseTM(actor, actorToWorld);
  NMP::Vector3 impulseW, positionW;
  actorToWorld.rotateVector(impulse, impulseW);
  actorToWorld.transformVector(position, positionW);

  addImpulseToActor(actor, impulseW, positionW, torqueMultiplier);
}

//----------------------------------------------------------------------------------------------------------------------
void addImpulseToActor(physx::PxActor& actor, const NMP::Vector3 &impulse, const NMP::Vector3 &position, float torqueMultiplier)
{
  if (actor.is<physx::PxRigidBody>() && actor.getScene())
  {
    NMP::Vector3 actorCOM = getActorCOMPos(&actor);
    NMP::Vector3 torque = NMP::vCross(position - actorCOM, impulse) * torqueMultiplier;
    actor.is<physx::PxRigidBody>()->addForce(nmVector3ToPxVec3(impulse), physx::PxForceMode::eIMPULSE);
    actor.is<physx::PxRigidBody>()->addTorque(nmVector3ToPxVec3(torque), physx::PxForceMode::eIMPULSE);
  }
}

//----------------------------------------------------------------------------------------------------------------------
void addVelocityChangeToActor(physx::PxActor& actor, const NMP::Vector3& velChange, const NMP::Vector3& worldPos, float torqueMultiplier)
{
  if (velChange.magnitude() == 0.0f)
    return;
  physx::PxRigidBody* rigidBody = actor.is<physx::PxRigidBody>();
  if (!rigidBody)
    return;

  // Note that physx::PxRigidBodyExt::addForceAtPos doesn't work with velocity changes, so do the
  // maths ourselves.

  // calculate global inertia tensor
  NMP::Vector3 t = nmPxVec3ToVector3(rigidBody->getMassSpaceInertiaTensor());
  NMP::Matrix34 massSpaceInertiaTensor(NMP::Matrix34::kIdentity);
  massSpaceInertiaTensor.scale3x3(t);
  NMP::Matrix34 localOffset = nmPxTransformToNmMatrix34(rigidBody->getCMassLocalPose());
  NMP::Matrix34 globalInertiaTensor = massSpaceInertiaTensor * 
    localOffset * nmPxTransformToNmMatrix34(rigidBody->getGlobalPose());

  // Calculate inv inertia
  NMP::Matrix34 invInertiaWorld = globalInertiaTensor;
  invInertiaWorld.invert3x3();
  NMP::Vector3 COMPositionWorld = MR::getActorCOMPos(rigidBody);

  NMP::Vector3 localImpulsePos = worldPos - COMPositionWorld;
  NMP::Vector3 directionWorld = velChange.getNormalised();

  // Calculate impulse using the same equations as found in collision response code - i.e. relating
  // a required change in velocity in a certain direction to an impulse.
   float velChangePerUnitImpulse =  
    (1.0f / rigidBody->getMass()) + 
    NMP::vDot(directionWorld, 
    NMP::vCross(invInertiaWorld.getRotatedVector(NMP::vCross(localImpulsePos, 
    directionWorld)), 
    localImpulsePos));

  NMP::Vector3 impulse = velChange / velChangePerUnitImpulse;

  addImpulseToActor(actor, impulse, worldPos, 1.0f);

  if (torqueMultiplier != 1.0f)
  {
    // Apply a rotational boost by applying the scaled velocity change either side of the CoM
    NMP::Vector3 actorCOMW = MR::getActorCOMPos(&actor);
    NMP::Vector3 offset = worldPos - actorCOMW;

    MR::addVelocityChangeToActor(actor, velChange * (torqueMultiplier - 1.0f) * 0.5f, worldPos + offset);
    MR::addVelocityChangeToActor(actor, velChange * -(torqueMultiplier - 1.0f) * 0.5f, worldPos - offset);
  }
}

//----------------------------------------------------------------------------------------------------------------------
void addForceToActor(physx::PxActor& actor, const NMP::Vector3 &force, const NMP::Vector3 &position)
{
  NMP::Vector3 actorCOM = getActorCOMPos(&actor);
  NMP::Vector3 torque = NMP::vCross(position - actorCOM, force);
  if (actor.is<physx::PxRigidBody>())
  {
    actor.is<physx::PxRigidBody>()->addForce(nmVector3ToPxVec3(force), physx::PxForceMode::eFORCE);
    actor.is<physx::PxRigidBody>()->addTorque(nmVector3ToPxVec3(torque), physx::PxForceMode::eFORCE);
  }
}

//----------------------------------------------------------------------------------------------------------------------
// a (sub-optimal) implementation of a function to get the inertia matrix of an actor in world frame
// todo: investigate loosing some conversions to and from physX types
NMP::Matrix34 getActorInertiaWorld(const physx::PxActor* actor)
{
  // get the (mass-aligned) body frame and it's inverse
  NMP::Matrix34 bodyTM = getActorCOMTM(actor);
  bodyTM.setTranslation(NMP::Vector3Zero());
  NMP::Matrix34 bodyTMInv(bodyTM);

  bodyTMInv.invertFast3x3();
  
  // get the body frame inertia matrix
  NMP::Matrix34 inertiaBody;
  inertiaBody.identity();
  const physx::PxRigidBody* rigidBody = actor->is<physx::PxRigidBody>();
  physx::PxVec3 I = rigidBody->getMassSpaceInertiaTensor();
  inertiaBody.setXAxis(NMP::Vector3(I.x, 0, 0));
  inertiaBody.setYAxis(NMP::Vector3(0, I.y, 0));
  inertiaBody.setZAxis(NMP::Vector3(0, 0, I.z));
  
  // return world frame inertia
  return bodyTMInv * inertiaBody * bodyTM;
}

} // namespace MR

//----------------------------------------------------------------------------------------------------------------------
