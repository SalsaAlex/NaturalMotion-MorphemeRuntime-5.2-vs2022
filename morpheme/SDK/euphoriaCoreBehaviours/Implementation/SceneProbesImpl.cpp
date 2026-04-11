/*
 * Copyright (c) 2011 NaturalMotion Ltd. All rights reserved.
 *
 * Not to be copied, adapted, modified, used, distributed, sold,
 * licensed or commercially exploited in any manner without the
 * written consent of NaturalMotion.
 *
 * All non public elements of this software are the confidential
 * information of NaturalMotion and may not be disclosed to any
 * person nor used for any purpose not expressly approved by
 * NaturalMotion in writing.
 *
 */

//----------------------------------------------------------------------------------------------------------------------
#include "SceneProbes.h"
#include "SceneProbesPackaging.h"
#include "MyNetwork.h"
#include "MyNetworkPackaging.h"
#include "euphoria/erBody.h"
#include "euphoria/erLimb.h"

#define SCALING_SOURCE owner->data->dimensionalScaling
#include "euphoria/erDimensionalScalingHelpers.h"

//----------------------------------------------------------------------------------------------------------------------


#include <algorithm>
#include "euphoria/erCharacter.h"
#include "euphoria/erEuphoriaUserData.h"
#include "euphoria/erDebugDraw.h"
#include "mrPhysicsSceneJoltPhys.h"
#include "physics/mrPhysicsRigDef.h"
#include "mrPhysicsRigJoltPhysRagdoll.h"
#include "EnvironmentAwareness.h"
#include "Head.h"
#include "HeadEyes.h"

//----------------------------------------------------------------------------------------------------------------------
namespace NM_BEHAVIOUR_LIB_NAMESPACE
{

//----------------------------------------------------------------------------------------------------------------------
// Updates an existing Environment::Object from the physics shape's latest state
// Extracts a minimal set of state information from the physical shape, this minimal objects will be passed into the
// EnvironmentAwareness module
//----------------------------------------------------------------------------------------------------------------------
static void updateEnvironmentObject(
  Environment::Object* object,
  JPH::Body* body,
  int32_t objectDataIndex,
  MR::InstanceDebugInterface* MR_OUTPUT_DEBUG_ARG(pDebugDrawInst))
{
  JPH::AABox bounds = body->GetWorldSpaceBounds();
  JPH::Vec3 extents = bounds.GetExtent();
  JPH::Vec3 centre = bounds.GetCenter();
  object->state.aabb.setCentreAndHalfExtents(MR::nmJPHVec3ToVector3(centre), MR::nmJPHVec3ToVector3(extents));
  object->state.position = MR::nmJPHVec3ToVector3(body->GetWorldTransform().GetTranslation());
  object->matrix = MR::nmJPHMat44ToNmMatrix34(body->GetWorldTransform());

  if (!body->IsStatic())
  {
    // This is the vel of centre of mass
    object->state.velocity = MR::nmJPHVec3ToVector3(body->GetLinearVelocity());
    object->state.angularVelocity = MR::nmJPHVec3ToVector3(body->GetAngularVelocity());
  }

  object->dataIndex = objectDataIndex;

  // Debug code to colour objects based on if they're seen
#if defined(MR_OUTPUT_DEBUGGING)
  if (pDebugDrawInst) // if myNetwork colouring is enabled
  {
    if (pDebugDrawInst->debugOutputsAreOn(MR::DEBUG_OUTPUT_DEBUG_DRAW))
    {
      uint32_t currentModuleIndex = pDebugDrawInst->getCurrentModuleIndex();

      if (pDebugDrawInst->isModuleDebugEnabled(currentModuleIndex))
      {
        MR::JoltPhysPerBodyData* perBodyData = MR::JoltPhysPerBodyData::getFromBody(body);
        if (perBodyData)
        {
          perBodyData->m_debugColour = ER::getDefaultColourForControl(ER::objectSeenControl);
          perBodyData->m_debugColour.w = ER::getDefaultColourForControl(ER::objectSeenControl).w;
        }
      }
    }
  }
#endif
}

//----------------------------------------------------------------------------------------------------------------------
// This is called on potentially new environment objects, initialising them, such as their acceleration to 0,
// and filling information that won't change, such as the mass.
// then it calls updateEnvironmentObjects to update to the latest information from the physics body
//----------------------------------------------------------------------------------------------------------------------
static bool setEnvironmentObjectFromBody(
  Environment::Object* object,
  JPH::Body* body,
  int32_t objectDataIndex,
  const MyNetwork* owner,
  MR::InstanceDebugInterface* pDebugDrawInst)
{
  // Early out: don't collide with own character controller
  // FIXME
  //if (body->GetID() == ((MR::PhysicsRigJoltPhys*)owner->getCharacter()->getBody().getPhysicsRig())->getCharacterController()->GetBodyID())
  //{
  //  return false;
  //}

  bool isMoveable = !body->IsStatic();
  float bodymass = 0.0f;
  if (isMoveable)
  {
    bodymass = (1.0f / body->GetMotionProperties()->GetInverseMass());
    if (bodymass < owner->data->minInterestingRelativeMass * owner->data->totalMass)
    {
      // early out: too small to be dealing with
      return false;
    }
  }

  // Only interact with bodies that have got per-body data
  MR::JoltPhysPerBodyData* perBodyData = MR::JoltPhysPerBodyData::getFromBody(body);
  if (!perBodyData)
    return false;

  // Otherwise it's an object we're actually interested in
  updateEnvironmentObject(object, body, objectDataIndex, pDebugDrawInst);
  object->state.acceleration.setToZero(); // this is calculated inside environment awareness
  object->state.bodyID = (int64_t)(size_t)body;
  object->dataIndex = 0;

  if (isMoveable)
  {
    object->isStill = body->IsDynamic() ? body->IsActive() : false;
    // if you're using a proxy character then the individual parts don't need to use a data index
    if (perBodyData) 
    {
      object->dataIndex = perBodyData->m_dataIndex;
      perBodyData->m_dataIndex = objectDataIndex;
    }
    else
    {
      object->dataIndex = 0;
    }
    // Use the real mass even if kinematic so users can effectively flag some kinematic objects as
    // uninteresting, even though they are effectively infinitely massive.
    object->state.mass = bodymass;
    object->state.isStatic = false;
  }
  else
  {
    object->isStill = true;
    object->state.velocity.setToZero();
    object->state.angularVelocity.setToZero();
    object->state.mass = SCALE_MASS(1e8f);
    object->state.isStatic = true;
  }

  MR_DEBUG_DRAW_VECTOR(pDebugDrawInst, MR::VT_Velocity,
    object->state.position, object->state.velocity, NMP::Colour::RED);
  return true;
}

//#define SIMPLE_PROBES
#if defined(SIMPLE_PROBES)

//----------------------------------------------------------------------------------------------------------------------
// This is an example of how to replace the terrain probing system with a simpler one in this very simple example we
// replace it with a single ground plane and a list of vertical capsules so it could represent a flat level with
// other characters on it
//----------------------------------------------------------------------------------------------------------------------
void SceneProbes::update(float timeStep)
{
}

//----------------------------------------------------------------------------------------------------------------------
void SceneProbes::feedback(float timeStep)
{
  // here's the list of capsules
  const int numCapsules = 10;
  static struct Capsule
  {
    NMP::Vector3 position;
    float radius;
    float halfHeight;
    float mass;
  } capsules[numCapsules];

  for (int i = 0; i < numCapsules; i++)
  {
    capsules[i].mass = 100.0f;
    capsules[i].radius = 0.3f;
    capsules[i].halfHeight = 1.0f;
    capsules[i].position = NMP::Vector3((float)(i % 3) * 10, 1.0f, (float)(i / 3) * 10); // put them in a grid
  }

  // first we send up the ground object
  Environment::State state;
  state.extents.set(100.0f, 100.0f, 100.0f);
  state.mass = 1e8f; // flags as static - Special number - see Environment_State.cpp MORPH-16212
  state.bodyID = 0;
  EnvironmentAwareness* EA = owner->environmentAwareness;
  Environment::Object* object = &EA->data->staticSweepResultObject;
  object->state = state;
  object->dataIndex = 0;
  object->isStill = true;
  object->matrix.identity();

  // then we send up the list of object he can see... in this simple example we ignore where
  // he is looking and pass up all the objects
  Environment::Object* objects = EA->data->dynamicObjectsInRange;
  for (int i = 0; i < numCapsules; i++)
  {
    objects[i].state.mass = capsules[i].mass;
    objects[i].state.extents = NMP::Vector3(capsules[i].radius, capsules[i].halfHeight, capsules[i].radius);
    objects[i].state.boxCentre = capsules[i].position;
    objects[i].state.position = objects[i].state.boxCentre;
    objects[i].state.velocity.setToZero();
    objects[i].state.angularVelocity.setToZero();
    objects[i].state.acceleration.setToZero();
    objects[i].state.bodyID = i + 1; // unique ID

    objects[i].isStill = true;
    objects[i].matrix = NMP::Matrix34(NMP::Quat(0, 0, 0, 1), objects[i].state.position);
    objects[i].dataIndex = i + 1; // unique ID, contiguous
  }
  EA->data->numDynamicObjectsInRange = numCapsules;

  // make the static result always just the ground plane
  if (EA->out->getSphereSweepStaticImportance())
  {
    ER::SweepResult resultStatic;
    resultStatic.setFromPlane(NMP::Vector3(0, 0, 0), NMP::Vector3(0, 1, 0), 0, EA->out->getSphereSweepStatic().probeID);
    EA->feedIn->setSweepResultStatic(resultStatic);
  }

  // for dynamic results we need to probe the requested object
  // Since EA has already done a bounding box test, and since it will do a further sweep on the patch,
  // all we need to do is pass up the object in question
  if (EA->out->getSphereSweepDynamicImportance())
  {
    ER::SweepResult resultDynamic;
    int index = EA->out->getSphereSweepDynamic().targetShape;
    NMP::Vector3 halfHeight = NMP::Vector3(0, capsules[index].halfHeight, 0);
    resultDynamic.setFromCapsule(capsules[index].position - halfHeight, capsules[index].position + halfHeight,
                                 capsules[index].radius, capsules[index].halfHeight, index, EA->out->getSphereSweepDynamic().probeID);
    EA->feedIn->setSweepResultDynamic(resultDynamic);
  }

  // EA can be continually requesting, since the probes don't cost
  EA->feedIn->setRequestNextProbe(true);
}
#else

static int s_characterIndex = 0; // used for scheduling between characters, will need to be more sophisticated for SPU
static int s_framesFromStart = 0;

//----------------------------------------------------------------------------------------------------------------------
void SceneProbes::update(float NMP_UNUSED(timeStep))
{
  s_characterIndex = 0;
}

#if defined(MR_OUTPUT_DEBUGGING)
//----------------------------------------------------------------------------------------------------------------------
static void drawBox(const NMP::Vector3& MR_OUTPUT_DEBUG_ARG(focalCentre), 
             float MR_OUTPUT_DEBUG_ARG(focalRadius), 
             const NMP::Colour& MR_OUTPUT_DEBUG_ARG(colour), 
             MR::InstanceDebugInterface* MR_OUTPUT_DEBUG_ARG(pDebugDrawInst))
{
  NMP::Vector3 radii(focalRadius, focalRadius, focalRadius);
  MR_DEBUG_DRAW_BOX(pDebugDrawInst, focalCentre, radii, colour);
}
#endif

// For collecting the closest objects
//----------------------------------------------------------------------------------------------------------------------
struct BodyInfo
{
  JPH::Body* joltBody;
  float distance;
};

//----------------------------------------------------------------------------------------------------------------------
static int calculateBodyDistances(
  JPH::Body** hitBodies, int numHits, BodyInfo* bodies, const NMP::Vector3& focalRoot,
  const NMP::Vector3& focalCentre, float focalRadius)
{
  NMP::Vector3 lookDir = NMP::vNormalise(focalCentre - focalRoot);

  // higher values result in being more interested in what's ahead rather than to the side.
  static float aheadScale = 3.0f; // non-dimensional
  int nBodies = 0;
  for (int i = 0; i < numHits; i++)
  {
    JPH::Body* body = hitBodies[i];

    JPH::AABox bounds = body->GetWorldSpaceBounds();
    JPH::Vec3 extents = bounds.GetExtent();
    JPH::Vec3 centre = bounds.GetCenter();
    NMP::Vector3 deltaToBody = MR::nmJPHVec3ToVector3(centre) - focalCentre;
    float distFromFocalCentre = NMP::fastSqrt(deltaToBody.magnitudeSquared());
    float extentsMag = NMP::fastSqrt(extents.LengthSq());
    float distanceScale = NMP::maximum(distFromFocalCentre - extentsMag, 0.0f) / distFromFocalCentre;
    distFromFocalCentre *= distanceScale;
    if (distFromFocalCentre < focalRadius)
    {
      deltaToBody = MR::nmJPHVec3ToVector3(centre) - focalRoot;
      float aheadAmount = NMP::vDot(deltaToBody, lookDir);
      float sideAmountSqr = deltaToBody.magnitudeSquared() - NMP::sqr(aheadAmount);
      aheadAmount /= aheadScale;
      float distFromRoot = NMP::sqr(aheadAmount) + sideAmountSqr;
      bodies->joltBody = body;
      bodies->distance = distFromRoot * distanceScale;
      ++bodies;
      ++nBodies;
    }
  }
  return nBodies;
}

//----------------------------------------------------------------------------------------------------------------------
struct BodySorter
{
  bool operator()(const BodyInfo& first, const BodyInfo& second) const
  {
    return first.distance < second.distance;
  }
};

//----------------------------------------------------------------------------------------------------------------------
// SceneProbesFeedbackPackage
// This function is fairly heavy and a little complicated as it interleaves the work so for instance only probing every 4
// frames
//----------------------------------------------------------------------------------------------------------------------
void SceneProbes::feedback(float NMP_UNUSED(timeStep))
{
  // get info on where he is looking
  float focalRadiusImportance = in->getFocalRadiusImportance();
  float focalRadius = in->getFocalRadius();
  NMP::Vector3 focalCentre = in->getFocalCentre();

  const ER::RootModule* rootModule = getRootModule();
  MR_DEBUG_MODULE_ENTER(rootModule->getDebugInterface(), getClassName());
  ER::Character* character = owner->getCharacter();
  EnvironmentAwareness* EA = owner->environmentAwareness;
  MR::PhysicsSceneJoltPhys* scene = (MR::PhysicsSceneJoltPhys*)(character->getBody().m_physicsScene);
  MR::PhysicsRigJoltPhys* pRig = ((MR::PhysicsRigJoltPhys*)character->getBody().getPhysicsRig());
  if (s_characterIndex == 0)
  {
    s_framesFromStart++;
  }

  // special code for first frame, which ensures request probe will be true on subsequent frame
  // so its a jump start for each new character
  if (data->justStarted)
  {
    // this ensures that the following frame (when getFocalRadiusImportance is >0) a request probe will be performed
    s_framesFromStart = SceneProbesData::NUM_FRAMES_BETWEEN_PROBES - 1 - s_characterIndex % SceneProbesData::NUM_FRAMES_BETWEEN_PROBES;
    data->justStarted = false;
  }
  if (focalRadiusImportance == 0.0f)
  {
    return; // no info from eyes, he effectively has them shut
  }

  // interleave probe work with other characters
  int characterIndex = s_characterIndex++;
  int frameMod = (s_framesFromStart + characterIndex) % SceneProbesData::NUM_FRAMES_BETWEEN_PROBES;
  bool requestProbe = frameMod == 0;

  // TODO this is a sphere now
#if defined(MR_OUTPUT_DEBUGGING)
  drawBox(focalCentre, focalRadius, NMP::Colour::GREEN, rootModule->getDebugInterface());
#endif // defined(MR_OUTPUT_DEBUGGING)

  ER::SweepResult resultDynamic, resultStatic;
  resultDynamic.setType(ER::SweepResult::SR_NoContact);
  resultStatic.setType(ER::SweepResult::SR_NoContact);

  // apply sphere sweeps if they have been defined, note they only have importance > 0 if we request a probe,
  // which happens every 4 frames (done lower down)
  if (EA->out->getSphereSweepDynamicImportance() != 0.0f)
  {
    resultDynamic.applySweep(
      EA->out->getSphereSweepDynamic(),
      scene, character->m_collisionIgnoreGroups, 
      true, SCALE_DIST(1.0f), rootModule->getDebugInterface());
  }
  // static and dynamic probes are also interleaved with each other
  else if (EA->out->getSphereSweepStaticImportance() != 0.0f)
  {
    resultStatic.applySweep(
      EA->out->getSphereSweepStatic(),
      scene, character->m_collisionIgnoreGroups, 
      false, SCALE_DIST(1.0f), rootModule->getDebugInterface());
  }

  Environment::Object* objects = EA->data->dynamicObjectsInRange;
  const int maxBodies = 256;
  BodyInfo bodies[maxBodies];
  int numBodies = 0;
  const int maxClosestBodies = 16;
  /*
  if (requestProbe == true) // if we're going to request a probe then we should send up the latest information
  {
    physx::PxSphereGeometry sphere;
    sphere.radius = focalRadius;
    // Use default PxFilterData here to bypass the internal filtering, and we pass the real flags
    // into the callback.
    physx::PxSceneQueryFilterData filterData(
      physx::PxQueryFlag::eDYNAMIC | physx::PxQueryFlag::ePREFILTER);

    int characterID = character->getBody().getPhysicsRig()->getRigID();
    MR::MorphemePhysX3QueryFilterCallback morphemePhysX3QueryFilterCallback(physx::PxFilterData(
      0, character->m_collisionIgnoreGroups, 0, 0), characterID);

    // It doesn't really cost to have a large number here, and ensures objects don't get ignored
    JPH::Body* hitbodies[maxBodies];

    physx::PxOverlapHit overlaphits[maxBodies];
    physx::PxOverlapBuffer overlapCallback(overlaphits, maxBodies);


    int numHit = scene->m_joltPhysScene->overlap(
      sphere, 
      JPH::Mat44::sRotationTranslation(JPH::Quat(0, 0, 0, 1), MR::nmVector3ToJPHVec3(focalCentre)),
      overlapCallback,
      filterData, 
      &morphemePhysX3QueryFilterCallback);
    if (numHit == -1)
    {
      // -1 means it overflowed, so the buffer is missing some objects in range, so we just work with these known objects
      numHit = maxBodies;
    }

    for (int i = 0; i < numHit; i++)
    {
        hitBodies[i] = overlapCallback.getAnyHit(i).shape;
    }

    // using the focal centre, find all objects in-range (basically, the things you are looking at) Perhaps the actual
    // check should be done before the physics, it wouldn't make a difference but may be faster for synchronisation
    // reasons? Need to check
    NMP::Vector3 extents(focalRadius, focalRadius, focalRadius);
    JPH::AABox bounds;
#if defined(MR_OUTPUT_DEBUGGING)
    drawBox(focalCentre, focalRadius, NMP::Colour::GREEN, rootModule->getDebugInterface());
#endif // defined(MR_OUTPUT_DEBUGGING)
    bounds.boundsOfPoints(MR::nmVector3ToJPHVec3(focalCentre - extents), MR::nmVector3ToJPHVec3(focalCentre + extents));

    // sort the shapes as we're just interested in the closest maxClosestBodies
    NMP::Vector3 headPos = owner->data->headLimbSharedStates[0].m_endTM.translation();
    numBodies = calculateBodyDistances(hitBodies, numHit, bodies, headPos, focalCentre, focalRadius);

    if (numBodies > maxClosestBodies) // only sort if there are too many
    {
      std::sort(&bodies[0], &bodies[numBodies], BodySorter());
      numBodies = maxClosestBodies;
    }

    // now add the sorted objects into the objects structure for sending up to EA
    int numObs = 0;
    for (int bodyCount = 0; bodyCount != numBodies; ++bodyCount)
    {
      if (setEnvironmentObjectFromBody(
        &objects[numObs], bodies[bodyCount].joltBody, numObs, owner, rootModule->getDebugInterface()))
      {
        numObs++;
      }
    }
    EA->data->numDynamicObjectsInRange = numObs;

    // Now update touch to EA, this passes up a separate result, but adds the object to the list (in order that acc can
    // be calculated) note, we don't have to do this inside this loop, we could do it every frame, but here is less
    // frequent and fits a little better
    // find the first contacting point, which is a limb end point
    for (uint32_t iLimb = 0; iLimb < character->getBody().m_definition->m_numLimbs; ++iLimb)
    {
      ER::Limb& limb = character->getBody().getLimb(iLimb);
      ER::EuphoriaRigPartUserData* endData =
        ER::EuphoriaRigPartUserData::getFromPart(limb.getPart(limb.getNumPartsInChain() - 1));

      // If currently contacting
      if (endData->getNumContacts())
      {
        JPH::Body* body = endData->getContactedBody(0);
        // Only interact with bodies that have got per-body data
        MR::JoltPhysPerBodyData* perBodyData = MR::JoltPhysPerBodyData::getFromBody(body);
        if (!perBodyData)
        {
          continue;
        }

        ER::SweepResult result;
        // Generate a sweep result from the plane that we can obtain from the contact
        result.setFromContact(
          endData->getLastAveragePosition(),
          endData->getLastAverageNormal(),
          (int64_t) (size_t) &body);
        EA->feedIn->setContactResult(result);
        if (body->IsStatic())
        {
          Environment::Object* object = &EA->data->staticContactObject;
          setEnvironmentObjectFromBody(object, body, 1, owner, rootModule->getDebugInterface());
        }
        else
        {
          bool found = false;
          // Next we add the contacted object to the list, but we don't want to duplicate it so search through the list
          // first
          int& numObjs = EA->data->numDynamicObjectsInRange;
          for (int i = 0; i < numObjs; i++)
          {
            if (body == (JPH::Body*) (size_t) EA->data->dynamicObjectsInRange[i].state.bodyID)
            {
              found = true;
              break;
            }
          }
          if (!found && numObjs < maxClosestBodies)
          {
            // If it isn't in the list and we have room for this extra object then add it in
            Environment::Object* object = &EA->data->dynamicObjectsInRange[numObjs];
            if (setEnvironmentObjectFromBody(object, body, numObjs, owner, rootModule->getDebugInterface()))
            {
              numObjs++;
            }
          }
        }
        break;
      }
    }
  }
  else // Otherwise we can just assume the sort order is about right and the set of objects is about right
  */
  {
    // TODO We need to dirty the bounds cache so getCachedBounds updates properly, on the two
    // intermediate frames now add the sorted objects into the objects structure for sending up to
    // EA
    int numObs = EA->data->numDynamicObjectsInRange;
    for (int i = 0; i < numObs; i++)
    {
      // Check that this object hasn't been deleted
      JPH::Body* body = (JPH::Body*) (size_t) objects[i].state.bodyID;
      if (MR::JoltPhysPerBodyData::getFromBody(body))
      {
        updateEnvironmentObject(&objects[i], body, i, rootModule->getDebugInterface());
      }
    }
  }
  // Is probe has found something then send it up to environment awareness
  if (resultDynamic.getType() != ER::SweepResult::SR_NoContact)
  {
    EA->feedIn->setSweepResultDynamic(resultDynamic);
#if defined(MR_OUTPUT_DEBUGGING)
    // Draw the object that has just been seen light pink
    if (rootModule->getDebugInterface() && rootModule->getDebugInterface()->isDrawEnabled())
    {
      JPH::Body* body = (JPH::Body*) (size_t) resultDynamic.getBodyID();
      NMP_ASSERT(body);
      MR::JoltPhysPerBodyData* bodyData = MR::JoltPhysPerBodyData::getFromBody(body);
      if (bodyData)
      {
        bodyData->m_debugColour = ER::getDefaultColourForControl(ER::objectSweepSucceededControl);
        bodyData->m_debugColour.w = ER::getDefaultColourForControl(ER::objectSweepSucceededControl).w;
      }
    }
#endif
  }
  // Just get the single collided static shape to send up to EA
  if (resultStatic.getType() != ER::SweepResult::SR_NoContact)
  {
    Environment::Object* object = &EA->data->staticSweepResultObject;
    bool res = setEnvironmentObjectFromBody(
      object, (JPH::Body*) (size_t) resultStatic.getBodyID(), -1, owner, rootModule->getDebugInterface());
    NMP_ASSERT(res); // Should always be true
    (void)res;
    EA->feedIn->setSweepResultStatic(resultStatic);
  }

  // every 4th frame. This is what causes the probe requests to come from EA,
  // use higher up in this function
  if (requestProbe == true)
  {
    feedOut->setRequestNextProbe(true);
  }
}
#endif // SIMPLE_PROBES

//----------------------------------------------------------------------------------------------------------------------
void SceneProbes::entry()
{
  data->justStarted = true;
}

} //namespace NM_BEHAVIOUR_LIB_NAMESPACE

