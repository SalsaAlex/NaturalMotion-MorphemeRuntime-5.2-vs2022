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
#include "mrPhysicsSceneJoltPhys.h"
#include "mrJoltPhys.h"
#include "mrPhysicsRigJoltPhys.h"
#include "mrCharacterControllerInterfaceJoltPhys.h"
#include "physics/mrPhysicsSerialisationBuffer.h"
#include "mrJoltPhysIncludes.h"

MR::NM_ObjectLayerPairFilterImpl s_object_vs_object_layer_filter;

static MR::NM_BPLayerInterfaceImpl s_broad_phase_layer_interface;

static MR::NM_ObjectVsBroadPhaseLayerFilterImpl s_object_vs_broadphase_layer_filter;

// max amount of rigid bodies that you can add to the physics system. If you try to add more you'll get an error.
const uint32_t cMaxBodies = 32768;

// how many mutexes to allocate to protect rigid bodies from concurrent access. Set it to 0 for the default settings.
const uint32_t cNumBodyMutexes = 0;

// max amount of body pairs that can be queued at any time (the broad phase will detect overlapping
// body pairs based on their bounding boxes and will insert them into a queue for the narrowphase). If you make this buffer
// too small the queue will fill up and the broad phase jobs will start to do narrow phase work. This is slightly less efficient.
const uint32_t cMaxBodyPairs = 32768;

// maximum size of the contact constraint buffer. If more contacts (collisions between bodies) are detected than this
// number then these contacts will be ignored and bodies will start interpenetrating / fall through the world.
const uint32_t cMaxContactConstraints = 8192;

//----------------------------------------------------------------------------------------------------------------------

namespace MR
{

JoltPhysPerBodyData::BodyToDataMap* JoltPhysPerBodyData::s_bodyToDataMap = 0;
NMP::HeapAllocator* JoltPhysPerBodyData::s_mapAllocator = 0;

//----------------------------------------------------------------------------------------------------------------------
PhysicsSceneJoltPhys::PhysicsSceneJoltPhys(JPH::TempAllocator* joltAllocator, 
    JPH::JobSystem* joltJobSystem, 
    JPH::PhysicsSystem* joltPhysScene) 
    :
  m_joltAllocator(joltAllocator),
  m_joltJobSystem(joltJobSystem),
  m_joltPhysScene(joltPhysScene)
{
    joltPhysScene->Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints,
        s_broad_phase_layer_interface, s_object_vs_broadphase_layer_filter,
        s_object_vs_object_layer_filter);

    PhysicsRigJoltPhysBodyData::init();
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsSceneJoltPhys::~PhysicsSceneJoltPhys()
{
  PhysicsRigJoltPhysBodyData::term();
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Vector3 PhysicsSceneJoltPhys::getFloorPositionBelow(
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

  //incomplete

  return NMP::Vector3(0, 0, 0);
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsSceneJoltPhys::castRay(
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

  //incomplete


  return false;
}


//----------------------------------------------------------------------------------------------------------------------
// aux used to convert position from bodylocal to world frame
void bodyToWorldPoint(const JPH::Body* body, const NMP::Vector3& xLocal, NMP::Vector3& xWorld)
{
    NMP::Matrix34 bodyToWorld(NMP::Matrix34::kIdentity);
    MR::getBodyGlobalPoseTM(body, bodyToWorld);
    bodyToWorld.transformVector(xLocal, xWorld);
}

//----------------------------------------------------------------------------------------------------------------------
// aux used to convert normal from bodylocal to world frame
void bodyToWorldNormal(const JPH::Body* body, const NMP::Vector3& nLocal, NMP::Vector3& nWorld)
{
    NMP::Matrix34 bodyToWorld;
    MR::getBodyGlobalPoseTM(body, bodyToWorld);
    bodyToWorld.rotateVector(nLocal, nWorld);
}

//----------------------------------------------------------------------------------------------------------------------
// aux used to convert position from world to bodylocal frame
void worldToBodyPoint(const JPH::Body* body, const NMP::Vector3& xWorld, NMP::Vector3& xLocal)
{
    NMP::Matrix34 worldToBody;
    MR::getBodyGlobalPoseTM(body, worldToBody);
    worldToBody.invertFast();
    worldToBody.transformVector(xWorld, xLocal);
}

//----------------------------------------------------------------------------------------------------------------------
// aux used to convert normal from world to bodylocal frame
void worldToBodyNormal(const JPH::Body* body, const NMP::Vector3& nWorld, NMP::Vector3& nLocal)
{
    NMP::Matrix34 worldToBody;
    MR::getBodyGlobalPoseTM(body, worldToBody);
    worldToBody.invertFast();
    worldToBody.rotateVector(nWorld, nLocal);
}

void bodyToWorldImpulse(JPH::Body* body, const NMP::Vector3& dirL, const NMP::Vector3& posL, NMP::Vector3& dirW, NMP::Vector3& posW)
{
    NMP::Matrix34 bodyToWorld;
    getBodyGlobalPoseTM(body, bodyToWorld);
    bodyToWorld.rotateVector(dirL, dirW);
    bodyToWorld.transformVector(posL, posW);
}

//----------------------------------------------------------------------------------------------------------------------
void addLocalImpulseAtLocalPosToBody(JPH::Body* body, const NMP::Vector3& impulse, const NMP::Vector3& position, float torqueMultiplier /*= 1.0f*/)
{
    NMP::Matrix34 bodyToWorld;
    getBodyGlobalPoseTM(body, bodyToWorld);
    NMP::Vector3 impulseW, positionW;
    bodyToWorld.rotateVector(impulse, impulseW);
    bodyToWorld.transformVector(position, positionW);

    addImpulseToBody(body, impulseW, positionW, torqueMultiplier);
}

//----------------------------------------------------------------------------------------------------------------------
void addImpulseToBody(JPH::Body* body, const NMP::Vector3& impulse, const NMP::Vector3& position, float torqueMultiplier)
{
    if (body->IsDynamic())
    {
        NMP::Vector3 bodyCOM = getBodyCOMPos(body);
        NMP::Vector3 torque = NMP::vCross(position - bodyCOM, impulse) * torqueMultiplier;
        body->AddImpulse(nmVector3ToJPHVec3(impulse), nmVector3ToJPHVec3(position));
        body->AddTorque(nmVector3ToJPHVec3(torque));
    }
}

//----------------------------------------------------------------------------------------------------------------------
void addVelocityChangeToBody(JPH::Body* body, const NMP::Vector3& velChange, const NMP::Vector3& worldPos, float torqueMultiplier)
{
    if (velChange.magnitude() == 0.0f)
        return;
    if (body->IsStatic())
        return;

    // Note that physx::PxRigidBodyExt::addForceAtPos doesn't work with velocity changes, so do the
    // maths ourselves.

    // calculate global inertia tensor
    NMP::Vector3 t = nmJPHVec3ToVector3(getBodyMassSpaceInertiaTensor(body));
    NMP::Matrix34 massSpaceInertiaTensor(NMP::Matrix34::kIdentity);
    massSpaceInertiaTensor.scale3x3(t);
    NMP::Matrix34 localOffset = nmJPHMat44ToNmMatrix34(getBodyCenterMassLocalPose(body));
    NMP::Matrix34 globalInertiaTensor = massSpaceInertiaTensor *
        localOffset * nmJPHMat44ToNmMatrix34(body->GetWorldTransform());

    // Calculate inv inertia
    NMP::Matrix34 invInertiaWorld = globalInertiaTensor;
    invInertiaWorld.invert3x3();
    NMP::Vector3 COMPositionWorld = MR::getBodyCOMPos(body);

    NMP::Vector3 localImpulsePos = worldPos - COMPositionWorld;
    NMP::Vector3 directionWorld = velChange.getNormalised();

    // Calculate impulse using the same equations as found in collision response code - i.e. relating
    // a required change in velocity in a certain direction to an impulse.
    float velChangePerUnitImpulse =
        body->GetMotionProperties()->GetInverseMass() +
        NMP::vDot(directionWorld,
            NMP::vCross(invInertiaWorld.getRotatedVector(NMP::vCross(localImpulsePos,
                directionWorld)),
                localImpulsePos));

    NMP::Vector3 impulse = velChange / velChangePerUnitImpulse;

    addImpulseToBody(body, impulse, worldPos, 1.0f);

    if (torqueMultiplier != 1.0f)
    {
        // Apply a rotational boost by applying the scaled velocity change either side of the CoM
        NMP::Vector3 bodyCOMW = MR::getBodyCOMPos(body);
        NMP::Vector3 offset = worldPos - bodyCOMW;

        MR::addVelocityChangeToBody(body, velChange * (torqueMultiplier - 1.0f) * 0.5f, worldPos + offset);
        MR::addVelocityChangeToBody(body, velChange * -(torqueMultiplier - 1.0f) * 0.5f, worldPos - offset);
    }
}

//----------------------------------------------------------------------------------------------------------------------
void addForceToBody(JPH::Body* body, const NMP::Vector3& force, const NMP::Vector3& position)
{
    NMP::Vector3 bodyCOM = getBodyCOMPos(body);
    NMP::Vector3 torque = NMP::vCross(position - bodyCOM, force);
    if (!body->IsStatic())
    {
        body->AddForce(nmVector3ToJPHVec3(force));
        body->AddTorque(nmVector3ToJPHVec3(torque));
    }
}

//----------------------------------------------------------------------------------------------------------------------
// a (sub-optimal) implementation of a function to get the inertia matrix of a body in world frame
// todo: investigate loosing some conversions to and from physX types
NMP::Matrix34 getBodyInertiaWorld(const JPH::Body* body)
{
    // get the (mass-aligned) body frame and it's inverse
    NMP::Matrix34 bodyTM = getBodyCOMTM(body);
    bodyTM.setTranslation(NMP::Vector3Zero());
    NMP::Matrix34 bodyTMInv(bodyTM);

    bodyTMInv.invertFast3x3();

    // get the body frame inertia matrix
    NMP::Matrix34 inertiaBody;
    inertiaBody.identity();
    JPH::Vec3 I = getBodyMassSpaceInertiaTensor(body);
    inertiaBody.setXAxis(NMP::Vector3(I.GetX(), 0, 0));
    inertiaBody.setYAxis(NMP::Vector3(0, I.GetY(), 0));
    inertiaBody.setZAxis(NMP::Vector3(0, 0, I.GetZ()));

    // return world frame inertia
    return bodyTMInv * inertiaBody * bodyTM;
}





} // namespace MR

//----------------------------------------------------------------------------------------------------------------------
