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

// Layer that objects can be in, determines which other objects it can collide with
// Typically you at least want to have 1 layer for moving bodies and 1 layer for static bodies, but you can have more
// layers if you want. E.g. you could have a layer for high detail collision (which is not used by the physics simulation
// but only if you do collision testing).
namespace NMPhysLayers //must match GameGroup enum
{
    static constexpr JPH::ObjectLayer NON_COLLIDABLE(0);            ///< 1 Object does not interact
    static constexpr JPH::ObjectLayer COLLIDABLE_NON_PUSHABLE(1);   ///< 2 Object is (effectively) static/kinematic and can be interacted with
    static constexpr JPH::ObjectLayer COLLIDABLE_PUSHABLE(2);       ///< 4 Object is dynamic and can be interacted with
    static constexpr JPH::ObjectLayer CHARACTER_CONTROLLER(3);      ///< 8 Object is used as a character controller
    static constexpr JPH::ObjectLayer CHARACTER_PART(4);            ///< 16 Object is part of a morpheme physics rig
    static constexpr JPH::ObjectLayer INTERACTION_PROXY(5);         ///< 32 Interaction proxy object for character probes.
    static constexpr JPH::ObjectLayer CHARACTER_PART_WITH_PROXY(6); ///< 64 Object is part of a physics rig that has an interaction proxy
    static constexpr uint32_t NUM_LAYERS(7);
};

// Each broadphase layer results in a separate bounding volume tree in the broad phase. You at least want to have
// a layer for non-moving and moving objects to avoid having to update a tree full of static objects every frame.
// You can have a 1-on-1 mapping between object layers and broadphase layers (like in this case) but if you have
// many object layers you'll be creating many broad phase trees, which is not efficient. If you want to fine tune
// your broadphase layers define JPH_TRACK_BROADPHASE_STATS and look at the stats reported on the TTY.
namespace NMBroadPhaseLayers
{
    static constexpr JPH::BroadPhaseLayer STATIC(0);
    static constexpr JPH::BroadPhaseLayer DYNAMIC(1);
    static constexpr JPH::BroadPhaseLayer NO_COLLIDE(2);
    static constexpr uint32_t NUM_LAYERS(3);
};


/// Class that determines if two object layers can collide
class NM_ObjectLayerPairFilterImpl : public JPH::ObjectLayerPairFilter
{
public:
    virtual bool					ShouldCollide(JPH::ObjectLayer inObject1,
        JPH::ObjectLayer inObject2) const override
    {
        switch (inObject1)
        {
        case NMPhysLayers::NON_COLLIDABLE:
            return false;
        case NMPhysLayers::COLLIDABLE_NON_PUSHABLE:
        case NMPhysLayers::COLLIDABLE_PUSHABLE:
        case NMPhysLayers::CHARACTER_CONTROLLER:
        case NMPhysLayers::CHARACTER_PART:
        case NMPhysLayers::INTERACTION_PROXY:
        case NMPhysLayers::CHARACTER_PART_WITH_PROXY:
            return inObject2 != NMPhysLayers::NON_COLLIDABLE;
        default:
            JPH_ASSERT(false);
            return false;
        }
    }
};

// BroadPhaseLayerInterface implementation
// This defines a mapping between object and broadphase layers.
class NM_BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface
{
public:
    NM_BPLayerInterfaceImpl()
    {
        // Create a mapping table from object to broad phase layer
        mObjectToBroadPhase[NMPhysLayers::NON_COLLIDABLE] = NMBroadPhaseLayers::NO_COLLIDE;
        mObjectToBroadPhase[NMPhysLayers::COLLIDABLE_NON_PUSHABLE] = NMBroadPhaseLayers::STATIC;
        mObjectToBroadPhase[NMPhysLayers::COLLIDABLE_PUSHABLE] = NMBroadPhaseLayers::DYNAMIC;
        mObjectToBroadPhase[NMPhysLayers::CHARACTER_CONTROLLER] = NMBroadPhaseLayers::DYNAMIC;
        mObjectToBroadPhase[NMPhysLayers::CHARACTER_PART] = NMBroadPhaseLayers::DYNAMIC;
        mObjectToBroadPhase[NMPhysLayers::INTERACTION_PROXY] = NMBroadPhaseLayers::DYNAMIC;
        mObjectToBroadPhase[NMPhysLayers::CHARACTER_PART_WITH_PROXY] = NMBroadPhaseLayers::DYNAMIC;
    }

    virtual uint32_t					GetNumBroadPhaseLayers() const override
    {
        return NMBroadPhaseLayers::NUM_LAYERS;
    }

    virtual JPH::BroadPhaseLayer			GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override
    {
        JPH_ASSERT(inLayer < NMPhysLayers::NUM_LAYERS);
        return mObjectToBroadPhase[inLayer];
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    virtual const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override
    {
        switch ((JPH::BroadPhaseLayer::Type)inLayer)
        {
        case (JPH::BroadPhaseLayer::Type)NMBroadPhaseLayers::STATIC:            return "STATIC";
        case (JPH::BroadPhaseLayer::Type)NMBroadPhaseLayers::DYNAMIC:           return "DYNAMIC";
        case (JPH::BroadPhaseLayer::Type)NMBroadPhaseLayers::NO_COLLIDE:           return "NO_COLLIDE";
        default:													JPH_ASSERT(false);  return "INVALID";
        }
    }
#endif // JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED

private:
    JPH::BroadPhaseLayer					mObjectToBroadPhase[NMPhysLayers::NUM_LAYERS];
};

/// Class that determines if an object layer can collide with a broadphase layer
class NM_ObjectVsBroadPhaseLayerFilterImpl : public JPH::ObjectVsBroadPhaseLayerFilter
{
public:
    virtual bool				ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override
    {
        switch (inLayer1)
        {
        case NMPhysLayers::NON_COLLIDABLE:
            return false;
        case NMPhysLayers::COLLIDABLE_NON_PUSHABLE:
        case NMPhysLayers::COLLIDABLE_PUSHABLE:
        case NMPhysLayers::CHARACTER_CONTROLLER:
        case NMPhysLayers::CHARACTER_PART:
        case NMPhysLayers::INTERACTION_PROXY:
        case NMPhysLayers::CHARACTER_PART_WITH_PROXY:
            return inLayer2 != NMBroadPhaseLayers::NO_COLLIDE;
        default:
            JPH_ASSERT(false);
            return false;
        }
    }
};

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
  PhysicsSceneJoltPhys(JPH::TempAllocator* joltAllocator, JPH::JobSystem* joltJobSystem, JPH::PhysicsSystem* joltPhysScene = 0);

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
    JPH::JobSystem* m_joltJobSystem;
    JPH::TempAllocator* m_joltAllocator;
};


#define INITIAL_BODYMAP_SIZE 256
//----------------------------------------------------------------------------------------------------------------------
/// \struct JoltPhysPerBodyData
///
/// Per-body data that is needed on physics objects if Euphoria is being used
//----------------------------------------------------------------------------------------------------------------------
struct JoltPhysPerBodyData
{
  static void initialiseMap()
  {
    NMP_ASSERT_MSG(s_mapAllocator == NULL && s_bodyToDataMap == NULL, "JoltPhysPerBodyData map already initialised.");

    NMP::Memory::Resource resource = NMPMemoryAllocateFromFormat(NMP::HeapAllocator::getMemoryRequirements());
    s_mapAllocator = NMP::HeapAllocator::init(resource);

    NMP::Memory::Format mapFormat(sizeof(BodyToDataMap));
    NMP::Memory::Resource mapResource = NMPMemoryAllocateFromFormat(mapFormat);
    s_bodyToDataMap = BodyToDataMap::init(mapResource, INITIAL_BODYMAP_SIZE, s_mapAllocator);
    NMP_ASSERT(mapResource.format.size == 0);
  }

  static void destroyMap()
  {
    if (s_bodyToDataMap)
    {
      s_bodyToDataMap->~BodyToDataMap();
      NMP::Memory::memFree(s_bodyToDataMap);
      s_bodyToDataMap = NULL;
    }

    if (s_mapAllocator)
    {
      s_mapAllocator->term();
      s_mapAllocator->~HeapAllocator();
      NMP::Memory::memFree(s_mapAllocator);
      s_mapAllocator = NULL;
    }
  }

  static JoltPhysPerBodyData* create(JPH::Body* body)
  {
    if (!s_bodyToDataMap)
      return 0;
    JoltPhysPerBodyData* data = (JoltPhysPerBodyData*) NMPMemoryAlloc(sizeof(JoltPhysPerBodyData));
    NMP_ASSERT(data);
    new (data) JoltPhysPerBodyData(body);
    return data;
  }

  static void destroy(JoltPhysPerBodyData* data, JPH::Body* body)
  {
    if (!s_bodyToDataMap)
      return;
    if (!data)
      return;
    if (body)
    {
      s_bodyToDataMap->erase(body);
    }
    NMP::Memory::memFree(data);
  }

  static JoltPhysPerBodyData* getFromBody(JPH::Body* body)
  {
    if (!s_bodyToDataMap)
      return 0;
    JoltPhysPerBodyData* pVal = 0;
    s_bodyToDataMap->find(body, &pVal);
    return pVal;
  }

  // Data
#if defined(MR_OUTPUT_DEBUGGING)
  NMP::Vector3 m_debugColour;
#endif
  int32_t m_dataIndex; // Index into the array of EA objects

private:
  typedef NMP::hash_map<JPH::Body*, JoltPhysPerBodyData*>  BodyToDataMap;

  JoltPhysPerBodyData(JPH::Body* body) : m_dataIndex(0)
  {
    NMP_ASSERT(getFromBody(body) == 0);
    s_bodyToDataMap->insert(body, this);
#if defined(MR_OUTPUT_DEBUGGING)
    m_debugColour.setToZero();
#endif
  }

  static BodyToDataMap*      s_bodyToDataMap;
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
    if(!body->IsStatic())
        body->SetLinearVelocity(MR::nmVector3ToJPHVec3(v));
}
//----------------------------------------------------------------------------------------------------------------------
NM_INLINE void setBodyAngVelW(JPH::Body* body, const NMP::Vector3& v)
{
    if (!body->IsStatic())
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
NM_INLINE void PhysicsSceneJoltPhys::setGravity(const NMP::Vector3& gravity)
{
    m_joltPhysScene->SetGravity(nmVector3ToJPHVec3(gravity));
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
