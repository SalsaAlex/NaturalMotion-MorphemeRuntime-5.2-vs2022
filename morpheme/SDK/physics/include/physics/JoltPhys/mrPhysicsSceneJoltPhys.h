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

/// user data attached to Jolt Physics bodies etc must be of this type - see the mrPhysicsRigJoltPhys.h for the declaration.
struct PhysicsRigJoltPhysActorData;

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
  /// Default implementation returns the Jolt Physics scene gravity
  NMP::Vector3 getGravity() NM_OVERRIDE;

  /// Sets the gravity used in the physics simulation
  /// Default implementation sets the Jolt Physics scene gravity
  void setGravity(const NMP::Vector3& gravity) NM_OVERRIDE;

#ifdef NM_HOST_64_BIT
  uint32_t pad[2];
#endif // NM_HOST_64_BIT

};

} // namespace

//----------------------------------------------------------------------------------------------------------------------
#endif // MR_PHYSICS_SCENE_JOLTPHYS_H
//----------------------------------------------------------------------------------------------------------------------
