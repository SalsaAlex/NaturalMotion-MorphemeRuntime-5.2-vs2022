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
#include "physics/JoltPhys/mrPhysicsSceneJoltPhys.h"
#include "physics/JoltPhys/mrJoltPhys.h"
#include "physics/JoltPhys/mrPhysicsRigJoltPhys.h"
#include "physics/JoltPhys/mrCharacterControllerInterfaceJoltPhys.h"
#include "physics/mrPhysicsSerialisationBuffer.h"
#include "physics/JoltPhys/mrJoltPhysIncludes.h"
//----------------------------------------------------------------------------------------------------------------------

namespace MR
{

//----------------------------------------------------------------------------------------------------------------------
NMP::Vector3 PhysicsSceneJoltPhys::getFloorPositionBelow(
  const NMP::Vector3 &pos, 
  const PhysicsRig *skipChar, 
  float distToCheck) const
{
    return pos;
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
  return false;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsSceneJoltPhys::setGravity(const NMP::Vector3& gravity)
{
}

} // namespace MR

//----------------------------------------------------------------------------------------------------------------------
