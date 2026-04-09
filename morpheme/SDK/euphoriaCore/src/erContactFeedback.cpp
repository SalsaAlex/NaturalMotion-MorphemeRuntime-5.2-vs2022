// Copyright (c) 2011 NaturalMotion.  All Rights Reserved.
// Not to be copied, adapted, modified, used, distributed, sold,
// licensed or commercially exploited in any manner without the
// written consent of NaturalMotion.  
//
// All non public elements of this software are the confidential
// information of NaturalMotion and may not be disclosed to any
// person nor used for any purpose not expressly approved by
// NaturalMotion in writing.

#include "euphoria/erContactFeedback.h"
#include "euphoria/erEuphoriaUserData.h"
#include "mrJoltPhys.h"
#include "mrPhysicsRigJoltPhys.h"
#include "mrPhysicsSceneJoltPhys.h"
#include "euphoria/erValueValidators.h"

bool g_debugDrawDetailedContacts = false;
bool g_debugDrawContacts = false;
static ER::ContactFeedback s_contactFeedback;

//ugh.. the way naturalmotion did this with physx3 is just not compatible with jolt
// without alot of changes..
static JPH::PhysicsSystem *s_physicsSystem;

static JPH::Body* bodyRefToPtr(const JPH::Body& body) { return s_physicsSystem->GetBodyLockInterface().TryGetBody(body.GetID()); }


namespace ER
{
//----------------------------------------------------------------------------------------------------------------------
// Add the callback if it isn't already added
void ContactFeedback::initialise(MR::PhysicsScene* physicsScene)
{
  // Setting an event callback is a global operation that only needs to be done once per scene
  // So we don't need to set it if it has already been set by a different character.
  MR::PhysicsSceneJoltPhys* scene = (MR::PhysicsSceneJoltPhys*)physicsScene;
  scene->m_joltPhysScene->SetContactListener(&s_contactFeedback);
  s_physicsSystem = scene->m_joltPhysScene;
}

//----------------------------------------------------------------------------------------------------------------------
// Remove the callback
void ContactFeedback::deinitialise(MR::PhysicsScene* physicsScene)
{
  MR::PhysicsSceneJoltPhys* scene = (MR::PhysicsSceneJoltPhys*)physicsScene;
  if (scene->m_joltPhysScene->GetContactListener() == &s_contactFeedback) //original code did not have this check.. hm
      scene->m_joltPhysScene->SetContactListener(NULL);
}

//----------------------------------------------------------------------------------------------------------------------
void ContactFeedback::setUserContactHandler(UserContactHandler* handler)
{
  s_contactFeedback.m_userContactHandler = handler;
}

//----------------------------------------------------------------------------------------------------------------------
bool ContactFeedback::getDrawDetailedContactsFlag() 
{
  return g_debugDrawDetailedContacts;
}

//----------------------------------------------------------------------------------------------------------------------
void ContactFeedback::setDrawDetailedContactsFlag(bool drawEnabled) 
{
  g_debugDrawDetailedContacts = drawEnabled;
}

//----------------------------------------------------------------------------------------------------------------------
bool ContactFeedback::getDrawContactsFlag() 
{
  return g_debugDrawContacts;
}

//----------------------------------------------------------------------------------------------------------------------
void ContactFeedback::setDrawContactsFlag(bool drawEnabled) 
{
  g_debugDrawContacts = drawEnabled;
}

//----------------------------------------------------------------------------------------------------------------------
void ER::EuphoriaRigPartUserData::processData(
  JPH::Body* contactedBody, 
  const NMP::Vector3& point, 
  const NMP::Vector3& normal, 
  float impulseMagnitude)
{
  NMP_ASSERT(contactedActor);
  if (!m_accumulating)
  {
    startNewContact();
  }

  const NMP::Vector3 impulse = normal * impulseMagnitude;

  m_accumulating = true;
  m_lastTotalImpulseMagnitude += impulseMagnitude;
  m_lastTotalImpulse += impulse;
  m_lastTotalPositionScaled += point * impulseMagnitude; 
  m_lastTotalNormalScaled += normal * impulseMagnitude;
  m_lastTotalVelocityScaled += MR::nmJPHVec3ToVector3(contactedBody->GetLinearVelocity()) * impulseMagnitude;
  m_lastCollisionID = (int64_t)(size_t)contactedBody;

  if (m_numContacts == m_maxNumContacts)
  {
    JPH::Body** origBodies = m_contactedBodies;
    uint16_t origNum = m_maxNumContacts;
    m_maxNumContacts *= 2;
    m_contactedBodies = 
      (JPH::Body**) NMP::Memory::memAlloc(sizeof(JPH::Body*)*m_maxNumContacts NMP_MEMORY_TRACKING_ARGS);
    memcpy(m_contactedBodies, origBodies, sizeof(JPH::Body*)*origNum);
    NMP::Memory::memFree(origBodies);
  }
  NMP_ASSERT(m_numContacts < m_maxNumContacts);

  m_contactedBodies[m_numContacts] = contactedBody;
  ++m_numContacts;
}

//----------------------------------------------------------------------------------------------------------------------
static void setBodiesInContact(const JPH::Body& body0, 
                               const JPH::Body& body1, 
                               const NMP::Vector3& point, 
                               const NMP::Vector3& normal, 
                               const float impulseMagnitude)
{
  // Check that the actor has a dynamic body so we don't get confused with the kinematic shapes used
  // for HK.
  MR::PhysicsRigJoltPhysBodyData* act0 = MR::PhysicsRigJoltPhysBodyData::getFromBody(body0);
  MR::PhysicsRigJoltPhysBodyData* act1 = MR::PhysicsRigJoltPhysBodyData::getFromBody(body1);
  MR::PhysicsRig::Part* p0 = act0 ? act0->m_owningRigPart : 0;
  MR::PhysicsRig::Part* p1 = act1 ? act1->m_owningRigPart : 0;

  NMP_ASSERT(Validators::Vector3Valid(point));
  NMP_ASSERT(Validators::FloatValid(impulseMagnitude));
  NMP_ASSERT(Validators::Vector3Normalised(normal));

#ifdef RECOVER_FROM_ERRORS
  if (
    !Validators::Vector3Valid(point) || 
    !Validators::FloatValid(impulseMagnitude) || 
    !Validators::Vector3Normalised(normal))
  {
    return;
  }
#endif

#if defined(MR_OUTPUT_DEBUGGING)
  NMP::Vector3 impulse = normal * impulseMagnitude;

  if (g_debugDrawContacts)
  {
    if (impulseMagnitude > 0.0f)
    {
      MR_DEBUG_DRAW_POINT_GLOBAL(point, 1, NMP::Colour::WHITE);
    }
    else
    {
      MR_DEBUG_DRAW_POINT_GLOBAL(point, 1, NMP::Colour::RED);
    }
  }

  if (g_debugDrawDetailedContacts && impulseMagnitude > 0.0f)
  {
    MR_DEBUG_DRAW_VECTOR_GLOBAL(MR::VT_Normal, point, normal, NMP::Colour::WHITE);
    MR_DEBUG_DRAW_VECTOR_GLOBAL(MR::VT_Normal, point, 
      MR::nmJPHVec3ToVector3(body0.GetWorldTransform().GetTranslation()) - point, NMP::Colour::DARK_RED);
    MR_DEBUG_DRAW_VECTOR_GLOBAL(MR::VT_Normal, point, 
      MR::nmJPHVec3ToVector3(body1.GetWorldTransform().GetTranslation()) - point, NMP::Colour::DARK_RED);
    MR_DEBUG_DRAW_VECTOR_GLOBAL(MR::VT_Impulse, point, impulse, NMP::Colour::BLUE);
  }

  // Only interested in active contacts, and zero force contacts were only passed if debug draw was
  // enabled at compile time
  if (impulseMagnitude <= 0.0f)
    return;

#endif // defined(MR_OUTPUT_DEBUGGING)

  // Only interested in characters
  NMP_ASSERT(p0 || p1);

  // don't register self collision
  if (p0 && p1 && p0->getOwningPhysicsRig() == p1->getOwningPhysicsRig())
  {
    return;
  }

  if (p0)
  {
    ER::EuphoriaRigPartUserData* data = ER::EuphoriaRigPartUserData::getFromPart(p0);
    if (data)
    {
      data->processData(bodyRefToPtr(body1), point, normal, impulseMagnitude);
    }
  }
  if (p1)
  {
    ER::EuphoriaRigPartUserData* data = ER::EuphoriaRigPartUserData::getFromPart(p1);
    if (data)
    {
      data->processData(bodyRefToPtr(body1), point, -normal, impulseMagnitude);
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------
JPH::ValidateResult ContactFeedback::OnContactValidate(const JPH::Body& inBody1, 
    const JPH::Body& inBody2,
    JPH::RVec3Arg inBaseOffset, 
    const JPH::CollideShapeResult& inCollisionResult)
{
  if (m_userContactHandler)
  {
    m_userContactHandler->onContact(inBody1, inBody2);
  }

  bool callSetActorsInContact = 
    MR::PhysicsRigJoltPhysBodyData::getFromBody(inBody1) || MR::PhysicsRigJoltPhysBodyData::getFromBody(inBody2);

  // Don't do the loop unless we would do something
  if (!callSetActorsInContact && !m_userContactHandler)
    return;

  const NMP::Vector3 point = MR::nmJPHVec3ToVector3(inCollisionResult.mContactPointOn1);
  const NMP::Vector3 normal = MR::nmJPHVec3ToVector3(-inCollisionResult.mPenetrationAxis.Normalized());
  float impulseMagnitude = inCollisionResult.mPenetrationDepth; //this prob not right
  // Don't just check if these are articulation links, as then we won't get hits on HK
  // parts. We can get the per-actor data even from the HK parts, so use that instead.
  // This is slightly more expensive, but shouldn't be too significant.
  if (
    MR::PhysicsRigJoltPhysBodyData::getFromBody(inBody1) || 
    MR::PhysicsRigJoltPhysBodyData::getFromBody(inBody2)
    )
  {
    setBodiesInContact(inBody1, inBody2, point, normal, impulseMagnitude);
  }
}

} // namespace
