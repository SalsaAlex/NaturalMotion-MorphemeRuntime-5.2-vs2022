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
//----------------------------------------------------------------------------------------------------------------------

namespace MR
{

JoltPhysPerShapeData::ShapeToDataMap* JoltPhysPerShapeData::s_shapeToDataMap = 0;
NMP::HeapAllocator* JoltPhysPerShapeData::s_mapAllocator = 0;

//----------------------------------------------------------------------------------------------------------------------
PhysicsSceneJoltPhys::PhysicsSceneJoltPhys(JPH::PhysicsSystem *joltPhysScene) :
  m_joltPhysScene(joltPhysScene)
{
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

} // namespace MR

//----------------------------------------------------------------------------------------------------------------------
