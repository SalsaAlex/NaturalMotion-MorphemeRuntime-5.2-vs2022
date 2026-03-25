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
#ifdef _MSC_VER
  #pragma once
#endif
#ifndef MR_PHYSICS_SCENE_JOLTPHYS_H
#define MR_PHYSICS_SCENE_JOLTPHYS_H
//----------------------------------------------------------------------------------------------------------------------
#include "NMPlatform/NMPlatform.h"
#include "NMPlatform/NMVector3.h"
#include "NMPlatform/NMMatrix34.h"
#include "NMPlatform/NMQuat.h"
#include "NMPlatform/NMHashMap.h"
#include "physics/mrPhysicsScene.h"
#include "physics/JoltPhys/mrJoltPhys.h"
#include "mrJoltPhysIncludes.h"
//----------------------------------------------------------------------------------------------------------------------

namespace MR
{

/// user data attached to Jolt Bodies etc must be of this type - see the mrPhysicsRigJoltPhys.h for the declaration.
struct PhysicsRigJoltPhysBodyData;

class CharacterControllerInterface;
struct PhysicsSerialisationBuffer;
class PhysicsRigJoltPhys;

//----------------------------------------------------------------------------------------------------------------------
/// \class PhysicsSceneJoltPhys
///
/// Interface used to access the physics scene by morpheme runtime.
//----------------------------------------------------------------------------------------------------------------------
class PhysicsSceneJoltPhys : public PhysicsScene
{
public:
  //----------------------------------------------------------------------
  // Functions that the application should call at the appropriate times
  //----------------------------------------------------------------------
  PhysicsSceneJoltPhys(JPH::PhysicsSystem* joltPhysScene = 0);

  virtual ~PhysicsSceneJoltPhys();

  //----------------------------------------------------------------------
  // Functions that the application implement unless the default implementation is OK
  //----------------------------------------------------------------------

  /// Cast a ray. Returns true/false to indicate a hit, and if there is a hit then hitDist etc will
  /// be set (hit Velocity is the velocity of the point on the object hit). Can pass in objects to
  /// ignore.
  bool castRay(
    const NMP::Vector3&                 start,
    const NMP::Vector3&                 delta,
    const PhysicsRig*                   skipChar,
    const CharacterControllerInterface* skipCharController,
    float&                              hitDist,
    NMP::Vector3&                       hitPosition,
    NMP::Vector3&                       hitNormal,
    NMP::Vector3&                       hitVelocity) const NM_OVERRIDE;

  /// This will return the floor position below pos.
  /// Default implementation simply ray casts. The application may wish to improve on this.
  /// skipChar indicates the character that needs to be skipped form the tests
  NMP::Vector3 getFloorPositionBelow(
    const NMP::Vector3& pos,
    const PhysicsRig*   skipChar,
    float               distToCheck) const NM_OVERRIDE;

  /// Returns the gravity in the physics simulation.
  /// Default implementation returns the Jolt scene gravity
  NMP::Vector3 getGravity() NM_OVERRIDE;

  /// Sets the gravity used in the physics simulation
  /// Default implementation sets the Jolt scene gravity
  void setGravity(const NMP::Vector3& gravity) NM_OVERRIDE;

public:
    JPH::PhysicsSystem* m_joltPhysScene;
};


#define INITIAL_SHAPEMAP_SIZE 256
//----------------------------------------------------------------------------------------------------------------------
/// \struct JoltPhysPerShapeData
///
/// Per-shape data that is needed on physics objects if Euphoria is being used
//----------------------------------------------------------------------------------------------------------------------
struct JoltPhysPerShapeData
{
  static void initialiseMap()
  {
    NMP_ASSERT_MSG(s_mapAllocator == NULL && s_shapeToDataMap == NULL, "JoltPhysPerShapeData map already initialised.");

    NMP::Memory::Resource resource = NMPMemoryAllocateFromFormat(NMP::HeapAllocator::getMemoryRequirements());
    s_mapAllocator = NMP::HeapAllocator::init(resource);

    NMP::Memory::Format mapFormat(sizeof(ShapeToDataMap));
    NMP::Memory::Resource mapResource = NMPMemoryAllocateFromFormat(mapFormat);
    s_shapeToDataMap = ShapeToDataMap::init(mapResource, INITIAL_SHAPEMAP_SIZE, s_mapAllocator);
    NMP_ASSERT(mapResource.format.size == 0);
  }

  static void destroyMap()
  {
    if (s_shapeToDataMap)
    {
      s_shapeToDataMap->~ShapeToDataMap();
      NMP::Memory::memFree(s_shapeToDataMap);
      s_shapeToDataMap = NULL;
    }

    if (s_mapAllocator)
    {
      s_mapAllocator->term();
      s_mapAllocator->~HeapAllocator();
      NMP::Memory::memFree(s_mapAllocator);
      s_mapAllocator = NULL;
    }
  }

  static JoltPhysPerShapeData* create(JPH::Shape* shape)
  {
    if (!s_shapeToDataMap)
      return 0;
    JoltPhysPerShapeData* data = (JoltPhysPerShapeData*) NMPMemoryAlloc(sizeof(JoltPhysPerShapeData));
    NMP_ASSERT(data);
    new (data) JoltPhysPerShapeData(shape);
    return data;
  }

  static void destroy(JoltPhysPerShapeData* data, JPH::Shape* shape)
  {
    if (!s_shapeToDataMap)
      return;
    if (!data)
      return;
    if (shape)
    {
      s_shapeToDataMap->erase(shape);
    }
    NMP::Memory::memFree(data);
  }

  static JoltPhysPerShapeData* getFromShape(JPH::Shape* shape)
  {
    if (!s_shapeToDataMap)
      return 0;
    JoltPhysPerShapeData* pVal = 0;
    s_shapeToDataMap->find(shape, &pVal);
    return pVal;
  }

  // Data
#if defined(MR_OUTPUT_DEBUGGING)
  NMP::Vector3 m_debugColour;
#endif
  int32_t m_dataIndex; // Index into the array of EA objects

private:
  typedef NMP::hash_map<JPH::Shape*, JoltPhysPerShapeData*>  ShapeToDataMap;

  JoltPhysPerShapeData(JPH::Shape* shape) : m_dataIndex(0)
  {
    NMP_ASSERT(getFromShape(shape) == 0);
    s_shapeToDataMap->insert(shape, this);
#if defined(MR_OUTPUT_DEBUGGING)
    m_debugColour.setToZero();
#endif
  }

  static ShapeToDataMap*      s_shapeToDataMap;
  static NMP::HeapAllocator*  s_mapAllocator;
};



/// returns the body tm
void getBodyGlobalPoseTM(const JPH::Body* body, NMP::Matrix34& bodyToWorld);
// sets the body TM - This method instantaneously changes the body space to world space transformation.
void setBodyGlobalPoseTM(JPH::Body* body, const NMP::Matrix34& bodyToWorld);
/// applies an impulse to the point on the body: impulse and position coordinates are relative and wrt global frame
void addImpulseToBody(JPH::Body* body, const NMP::Vector3& impulse, const NMP::Vector3& position, float torqueMultiplier = 1.0f);
/// applies an impulse to the point on the body: impulse and position coordinates are relative and wrt body frame
void addLocalImpulseAtLocalPosToBody(JPH::Body* body, const NMP::Vector3& impulse, const NMP::Vector3& position, float torqueMultiplier = 1.0f);
/// applies a force to the point on the body: force and position coordinates are wrt global frame
void addForceToBody(JPH::Body* body, const NMP::Vector3& force, const NMP::Vector3& position);
/// applies a force to the body: force and is in the global frame
void addForceToBody(JPH::Body* body, const NMP::Vector3& force);
/// applies pure linear impulse (i.e. as if to body centre of mass): impulse coordinates wrt global frame
void addImpulseToBody(JPH::Body* body, const NMP::Vector3& impulse);
/// applies a pure torque (i.e. as if via a couple): torque coordinates wrt global frame
void addTorqueToBody(JPH::Body* body, const NMP::Vector3& torque);
/// applies a pure torqueImpulse (i.e. as if via a couple): torqueImpulse coordinates wrt global frame
void addTorqueImpulseToBody(JPH::Body* body, const NMP::Vector3& torqueImpulse);

/// applies an acceleration to the body - accel is in the global frame
void addAccelerationToBody(JPH::Body* body, const NMP::Vector3& accel);

/// Applies a velocity change at the world space point, with an optional multiplier for the angular amount
void addVelocityChangeToBody(JPH::Body* body, const NMP::Vector3& velChange, const NMP::Vector3& worldPos, float angularMultiplier = 1.0f);
/// Applies a linear velocity change
void addLinearVelocityChangeToBody(JPH::Body* body, const NMP::Vector3& velChange);
/// Applies an angular velocity change
void addAngularVelocityChangeToBody(JPH::Body* body, const NMP::Vector3& angularVelChange);
/// Applies an angular acceleration
void addAngularAccelerationToBody(JPH::Body* body, const NMP::Vector3& angularAccel);

/// aux used to convert position from bodylocal to world frame
void bodyToWorldPoint(const JPH::Body* body, const NMP::Vector3& xLocal, NMP::Vector3& xWorld);
/// aux used to convert normal from bodylocal to world frame
void bodyToWorldNormal(const JPH::Body* body, const NMP::Vector3& nLocal, NMP::Vector3& nWorld);
/// aux to convert body local direction and point of application eg. of an impulse
void bodyToWorldImpulse(JPH::Body* body, const NMP::Vector3& dirL, const NMP::Vector3& posL, NMP::Vector3& dirW, NMP::Vector3& posW);
// aux used to convert normal from world to bodylocal frame
void worldToBodyNormal(const JPH::Body* body, const NMP::Vector3& nWorld, NMP::Vector3& nLocal);
// aux used to convert position from world to bodylocal frame
void worldToBodyPoint(const JPH::Body* body, const NMP::Vector3& xWorld, NMP::Vector3& xLocal);

/// returns the mass frame of the body
NMP::Matrix34 getBodyCOMTM(const JPH::Body* body);
/// returns the centre of mass position of the body
NMP::Vector3 getBodyCOMPos(const JPH::Body* body);
/// returns the angular velocity of the body
NMP::Vector3 getBodyAngVelW(const JPH::Body* body);
/// returns the linear velocity of the body
NMP::Vector3 getBodyLinVelW(const JPH::Body* body);
/// returns world frame body inertia matrix
NMP::Matrix34 getBodyInertiaWorld(const JPH::Body* body);
/// returns the body mass, or 0 if it's not dynamic
float getBodyMass(const JPH::Body* body);

/// zeros the linear velocity of the body
void setBodyLinVelW(JPH::Body* body, const NMP::Vector3& accel);
/// zeros the angular velocity of the body
void setBodyAngVelW(JPH::Body* body, const NMP::Vector3& accel);


//----------------------------------------------------------------------------------------------------------------------
// inline functions
//----------------------------------------------------------------------------------------------------------------------
NM_INLINE void setBodyLinVelW(JPH::Body* body, const NMP::Vector3& v)
{
    body->SetLinearVelocity(MR::nmVector3ToJPHVec3(v));
}
//----------------------------------------------------------------------------------------------------------------------
NM_INLINE void setBodyAngVelW(JPH::Body* body, const NMP::Vector3& v)
{
    body->SetAngularVelocity(MR::nmVector3ToJPHVec3(v));
}

//----------------------------------------------------------------------------------------------------------------------
NM_INLINE NMP::Matrix34 getBodyCOMTM(const JPH::Body* body)
{
    return MR::nmJPHMat44ToNmMatrix34(body->GetCenterOfMassTransform());
}

//----------------------------------------------------------------------------------------------------------------------
NM_INLINE NMP::Vector3 getBodyCOMPos(const JPH::Body* body)
{
    return MR::nmJPHVec3ToVector3(body->GetCenterOfMassPosition());
}

//----------------------------------------------------------------------------------------------------------------------
NM_INLINE void getBodyGlobalPoseTM(const JPH::Body* body, NMP::Matrix34& bodyToWorld)
{
    bodyToWorld = nmJPHMat44ToNmMatrix34(body->GetWorldTransform());
}

//----------------------------------------------------------------------------------------------------------------------
NM_INLINE void setBodyGlobalPoseTM(JPH::Body* body, const NMP::Matrix34& newBodyToWorld)
{
    //undone
}

//----------------------------------------------------------------------------------------------------------------------
NM_INLINE NMP::Vector3 getBodyAngVelW(const JPH::Body* body)
{
    return MR::nmJPHVec3ToVector3(body->GetAngularVelocity());
}

//----------------------------------------------------------------------------------------------------------------------
NM_INLINE NMP::Vector3 getBodyLinVelW(const JPH::Body* body)
{
    return MR::nmJPHVec3ToVector3(body->GetLinearVelocity());
}

//----------------------------------------------------------------------------------------------------------------------
NM_INLINE void addLinearVelocityChangeToBody(JPH::Body* body, const NMP::Vector3& velChange)
{
    body->SetLinearVelocity(nmVector3ToJPHVec3(velChange));
}

//----------------------------------------------------------------------------------------------------------------------
NM_INLINE void addAngularVelocityChangeToBody(JPH::Body* body, const NMP::Vector3& angularVelChange)
{
    body->SetAngularVelocity(nmVector3ToJPHVec3(angularVelChange));
}

//----------------------------------------------------------------------------------------------------------------------
NM_INLINE void addAngularAccelerationToBody(JPH::Body* body, const NMP::Vector3& angularAccel)
{
    body->AddTorque(nmVector3ToJPHVec3(angularAccel));
}

//----------------------------------------------------------------------------------------------------------------------
NM_INLINE void addForceToBody(JPH::Body* body, const NMP::Vector3& force)
{
    body->AddForce(nmVector3ToJPHVec3(force));
}

//----------------------------------------------------------------------------------------------------------------------
NM_INLINE void addAccelerationToBody(JPH::Body* body, const NMP::Vector3& accel)
{
    body->AddForce(nmVector3ToJPHVec3(accel));
}

//----------------------------------------------------------------------------------------------------------------------
NM_INLINE void addImpulseToBody(JPH::Body* body, const NMP::Vector3& impulse)
{
    body->AddImpulse(nmVector3ToJPHVec3(impulse));
}

//----------------------------------------------------------------------------------------------------------------------
NM_INLINE void addTorqueToBody(JPH::Body* body, const NMP::Vector3& torque)
{
    body->AddTorque(nmVector3ToJPHVec3(torque));
}

//----------------------------------------------------------------------------------------------------------------------
NM_INLINE void addTorqueImpulseToBody(JPH::Body* body, const NMP::Vector3& torqueImpulse)
{
    body->AddTorque(nmVector3ToJPHVec3(torqueImpulse));
}

//----------------------------------------------------------------------------------------------------------------------
NM_INLINE NMP::Vector3 PhysicsSceneJoltPhys::getGravity()
{
    return nmJPHVec3ToVector3(m_joltPhysScene->GetGravity());
}

//----------------------------------------------------------------------------------------------------------------------
NM_INLINE float getBodyMass(const JPH::Body* body)
{
    return 1.0 / body->GetMotionProperties()->GetInverseMass();
}


} // namespace

//----------------------------------------------------------------------------------------------------------------------
#endif // MR_PHYSICS_SCENE_JOLTPHYS_H
//----------------------------------------------------------------------------------------------------------------------
