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
#include "physics/JoltPhys/mrCharacterControllerInterfaceJoltPhys.h"
#include "physics/JoltPhys/mrPhysicsSceneJoltPhys.h"
#include "physics/mrPhysics.h"
//----------------------------------------------------------------------------------------------------------------------

namespace MR
{

//----------------------------------------------------------------------------------------------------------------------
// Implementations of the functions for CharacterControllerJoltPhys
//----------------------------------------------------------------------------------------------------------------------
CharacterControllerInterfaceJoltPhys::CharacterControllerInterfaceJoltPhys()
{
  m_physicsScene = 0;  
}

//----------------------------------------------------------------------------------------------------------------------
bool CharacterControllerInterfaceJoltPhys::castRayIntoCollisionWorld(
  const NMP::Vector3& start,
  const NMP::Vector3& delta,
  float&              hitDist,
  NMP::Vector3&       hitPosition,
  NMP::Vector3&       hitNormal,
  NMP::Vector3&       hitVelocity,
  MR::Network*        network) const
{
  NMP_ASSERT_MSG(m_physicsScene, "CharacterControllerJoltPhys must have a physics scene to perform ray casts.\n");

  // Use the physics scene's ray cast function
  return false;
}

//----------------------------------------------------------------------------------------------------------------------
bool CharacterControllerInterfaceJoltPhys::setRequestedPropertyOverride(
  CCPropertyType     propertyType,
  AttribData*        property,
  CCOverridePriority priority,
  FrameCount         frameIndex)
{
  return m_overrideBasics.setRequestedPropertyOverride(propertyType, property, priority, frameIndex);
}

//----------------------------------------------------------------------------------------------------------------------
const AttribData* CharacterControllerInterfaceJoltPhys::getRequestedPropertyOverride(
  MR::CCPropertyType propertyType,
  FrameCount         frameIndex) const
{
  return m_overrideBasics.getRequestedPropertyOverride(propertyType, frameIndex);
}

} // namespace MR

//----------------------------------------------------------------------------------------------------------------------
