// Not to be copied, adapted, modified, used, distributed, sold,
// licensed or commercially exploited in any manner without the
// written consent of NaturalMotion.  
//
// All non public elements of this software are the confidential
// information of NaturalMotion and may not be disclosed to any
// person nor used for any purpose not expressly approved by
// NaturalMotion in writing.

//----------------------------------------------------------------------------------------------------------------------
#define  _CRT_SECURE_NO_WARNINGS // for strncpy

#include "NMPlatform/NMPlatform.h"
#include "morpheme/mrRig.h"
#include "morpheme/mrBlendOps.h"
#include "mrJoltPhys.h"
#include "mrPhysicsDriverDataJoltPhys.h"
#include "mrPhysicsRigJoltPhysArticulation.h"
#include "physics/mrPhysicsRigDef.h"
#include "physics/mrPhysicsAttribData.h"
#include "mrPhysicsSceneJoltPhys.h"
#include "physics/mrPhysicsSerialisationBuffer.h"
#include "morpheme/mrAttribData.h"

#include "NMPlatform/NMProfiler.h"

#include "NMPlatform/NMvpu.h"

//----------------------------------------------------------------------------------------------------------------------
#define MINIMUM_COMPLIANCE 0.001f

// Sanity checks on passing strength/damping to physx
#define MAX_STRENGTH 1e12f
#define MAX_DAMPING 1e25f

namespace MR 
{

// This limit isn't nice, but PhysX is very jittery with tiny ranges, and currently we don't
// have a hinge joint type in PhysX. See MORPH-11273
static const float s_minSwingLimit = NMP::degreesToRadians(3.0f);


RagdollExplosionHandler* PhysicsRigJoltPhysRagdoll::s_explosionHandler = 0;

//----------------------------------------------------------------------------------------------------------------------
NMP::Memory::Format PhysicsRigJoltPhysRagdoll::getMemoryRequirements(PhysicsRigDef *physicsRigDef)
{
  uint32_t numBones = physicsRigDef->getNumParts();
  uint32_t numJoints = physicsRigDef->getNumJoints();
  uint32_t numMaterials = physicsRigDef->getNumMaterials();

  NMP::Memory::Format result(sizeof(PhysicsRigJoltPhysRagdoll), NMP_VECTOR_ALIGNMENT);

  // Space for the part pointers
  result += NMP::Memory::Format(sizeof(PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll*) * numBones, NMP_NATURAL_TYPE_ALIGNMENT);

  // Space for the joint pointers
  result += NMP::Memory::Format(sizeof(PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll*) * numJoints, NMP_NATURAL_TYPE_ALIGNMENT);

  // Space for the parts
  result += NMP::Memory::Format(
    NMP::Memory::align(sizeof(PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll), NMP_NATURAL_TYPE_ALIGNMENT) * numBones,
    NMP_NATURAL_TYPE_ALIGNMENT);

  // Space for the joints
  result += NMP::Memory::Format(
    NMP::Memory::align(sizeof(PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll), NMP_NATURAL_TYPE_ALIGNMENT) * numJoints,
    NMP_NATURAL_TYPE_ALIGNMENT);

  return result;
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysRagdoll*PhysicsRigJoltPhysRagdoll::init(
  NMP::Memory::Resource &resource, 
  PhysicsRigDef         *physicsRigDef, 
  PhysicsScene          *physicsScene, 
  AnimRigDef            *animRigDef,
  AnimToPhysicsMap      *animToPhysicsMap,
  int32_t                collisionTypeMask,
  int32_t                collisionIgnoreMask)
{
  PhysicsRigJoltPhysRagdoll*result = (PhysicsRigJoltPhysRagdoll*)resource.ptr;
  resource.increment(sizeof(PhysicsRigJoltPhysRagdoll));

  new (result) PhysicsRigJoltPhysRagdoll((PhysicsSceneJoltPhys*) physicsScene);
  PhysicsRigJoltPhys::init(result, PhysicsRigJoltPhys::TYPE_ARTICULATED);
  result->m_cachedSleepThreshold = 0.0f;

  uint32_t numParts = physicsRigDef->getNumParts();
  uint32_t numJoints = physicsRigDef->getNumJoints();
  //uint32_t numMaterials = physicsRigDef->getNumMaterials();

  // Materials
  //resource.align(NMP::Memory::Format(sizeof(physx::PxMaterial*) * numMaterials, NMP_NATURAL_TYPE_ALIGNMENT));
  //result->m_materials = (physx::PxMaterial**)resource.ptr;
  //resource.increment(NMP::Memory::Format(sizeof(physx::PxMaterial*) * numMaterials, NMP_NATURAL_TYPE_ALIGNMENT));

  // Part pointers
  resource.align(NMP::Memory::Format(sizeof(PhysicsRig::Part*) * numParts, NMP_NATURAL_TYPE_ALIGNMENT));
  result->m_parts = (PhysicsRig::Part **)resource.ptr;
  resource.increment(NMP::Memory::Format(sizeof(PhysicsRig::Part*) * numParts, NMP_NATURAL_TYPE_ALIGNMENT));

  // Joint pointers
  resource.align(NMP::Memory::Format(sizeof(PhysicsRig::Joint*) * numJoints, NMP_NATURAL_TYPE_ALIGNMENT));
  result->m_joints = (PhysicsRig::Joint **)resource.ptr;
  resource.increment(NMP::Memory::Format(sizeof(PhysicsRig::Joint*) * numJoints, NMP_NATURAL_TYPE_ALIGNMENT));

  // Parts
  for (uint32_t i = 0; i < numParts; i++)
  {
    resource.align(NMP::Memory::Format(sizeof(PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll), NMP_NATURAL_TYPE_ALIGNMENT));
    result->m_parts[i] = (PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll*)resource.ptr;
    resource.increment(NMP::Memory::Format(sizeof(PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll), NMP_NATURAL_TYPE_ALIGNMENT));
  }

  // Joints
  for (uint32_t i = 0; i < numJoints; i++)
  {
    resource.align(NMP::Memory::Format(sizeof(PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll), NMP_NATURAL_TYPE_ALIGNMENT));
    result->m_joints[i] = (PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll*)resource.ptr;
    resource.increment(NMP::Memory::Format(sizeof(PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll), NMP_NATURAL_TYPE_ALIGNMENT));
  }

  result->m_animRigDef = animRigDef;
  result->m_animToPhysicsMap = animToPhysicsMap;
  result->m_physicsRigDef = physicsRigDef;

  result->m_collisionTypeMask = collisionTypeMask;
  result->m_collisionIgnoreMask = collisionIgnoreMask;
  result->m_numCollisionGroupIndices = 0;

  // Fill m_materials in with the different materials in the physicsScene
  //for (uint32_t i = 0; i < physicsRigDef->m_numMaterials; ++i)
  //{
  //  result->m_materials[i] = createMaterial(physicsRigDef->m_materials[i]);
  //}

  uint32_t totalNumShapes = 0;
  for (uint32_t iPart = 0 ; iPart < numParts ; ++iPart)
  {
    PhysicsRigDef::Part &part = physicsRigDef->m_parts[iPart];
    PhysicsRigDef::Part::Volume &volume = part.volume;
    uint32_t numShapes = volume.numSpheres + volume.numBoxes + volume.numCapsules;
    totalNumShapes += numShapes;
  }

  // Allocate memory for link and shape descriptors - they all have to be filled
  // in before anything is created.
  JoltPhysRagdollJointDesc **jointDescs = (JoltPhysRagdollJointDesc**)NMPMemoryAlloc(
    sizeof(JoltPhysRagdollJointDesc*) * numParts);
  NMP_ASSERT(jointDescs);

  PxShapeDesc **shapeDescs = (PxShapeDesc **)NMPMemoryAlloc(
    sizeof(PxShapeDesc*) * totalNumShapes);
  NMP_ASSERT(shapeDescs);
  for (uint32_t iShape = 0 ; iShape < totalNumShapes ; ++iShape)
  {
    shapeDescs[iShape] = (PxShapeDesc *)NMPMemoryAlloc(
      sizeof(PxShapeDesc));
    NMP_ASSERT(shapeDescs[iShape]);
    (void) new (shapeDescs[iShape]) PxShapeDesc(PxGetPhysics().getTolerancesScale());
    shapeDescs[iShape]->name = "Articulation rig part (kinematic or dynamic)";
  }

  // Convert collision sets into a mask per part
  int *partGroupMasks = (int *)alloca(numParts * sizeof(int));
  int *allowedPartGroupMasks = (int *)alloca(numParts * sizeof(int));
  for (uint32_t i = 0; i<numParts; i++)
  {
    partGroupMasks[i] = 0;
    allowedPartGroupMasks[i] = 0;
  }
  for (int32_t i = 0; i<physicsRigDef->m_numCollisionGroups; i++)
  {
    if (physicsRigDef->m_collisionGroups[i].enabled)
    {
      for (int j = 0; j<physicsRigDef->m_collisionGroups[i].numIndices; j++)
      {
        // Indices can't reference a part out of range
        NMP_ASSERT(physicsRigDef->m_collisionGroups[i].indices[j] < (int)numParts); 
        partGroupMasks[physicsRigDef->m_collisionGroups[i].indices[j]] |= 1<<i;
      }
    }

    for (int j = 0; j<physicsRigDef->m_collisionGroups[i].numIndices; j++)
    {
      // Indices can't reference a part out of range
      NMP_ASSERT(physicsRigDef->m_collisionGroups[i].indices[j] < (int)numParts); 
      allowedPartGroupMasks[physicsRigDef->m_collisionGroups[i].indices[j]] |= 1<<i;
    }
  }

  // Incremented as we work on each shape
  uint32_t iShape = 0;

  // There's a maximum limit on the number of parts in an articulation imposed by physx
  NMP_ASSERT(numParts <= 64);

  // Loop setting up each part
  for (uint32_t iPart = 0; iPart < numParts; ++iPart)
  {
    const PhysicsRigDef::Part &part = physicsRigDef->m_parts[iPart];
    const PhysicsRigDef::Part::Volume &volume = part.volume;

    const PhysicsBodyDriverDataJoltPhys* bodyDriverData = (const PhysicsBodyDriverDataJoltPhys*)part.body.driverData;

    // Actor
    PxRigidDynamicDesc rigidDynamicDesc(PxGetPhysics().getTolerancesScale()); 
    rigidDynamicDesc.ownerClient = result->m_ownerClientID;
    rigidDynamicDesc.globalPose = nmMatrix34ToPxTransform(part.actor.globalPose);
    rigidDynamicDesc.name = part.name;
    rigidDynamicDesc.mass = 1e10f; // gets overridden
    rigidDynamicDesc.massSpaceInertia = 
      physx::PxVec3(rigidDynamicDesc.mass,rigidDynamicDesc.mass,rigidDynamicDesc.mass);

    NMP_ASSERT(part.actor.hasCollision);
    NMP_ASSERT(!part.actor.isFixed);
    rigidDynamicDesc.angularDamping = part.body.angularDamping;
    rigidDynamicDesc.linearDamping = part.body.linearDamping;
    rigidDynamicDesc.maxAngularVelocity = bodyDriverData->m_maxAngularVelocity;
    if (rigidDynamicDesc.maxAngularVelocity < 0.0f)
      rigidDynamicDesc.maxAngularVelocity = 1e6;

    // Note that the following iterations don't affect the internal character physics/drives as
    // this part of the descriptor never makes it into the articulation part.
    // Articulation-specific iterations are set up later. See MORPH-11268

    // This is the number iterations for resolving collisions
    rigidDynamicDesc.minPositionIterations = bodyDriverData->m_positionSolverIterationCount;
    // This is the number of iterations for resolving penetration
    rigidDynamicDesc.minVelocityIterations = bodyDriverData->m_velocitySolverIterationCount;

    uint32_t iFirstShapeForPart = iShape;
    uint32_t numShapes = volume.numBoxes + volume.numCapsules + volume.numSpheres;

    physx::PxFilterData filterData(
      result->m_collisionTypeMask, 
      result->m_collisionIgnoreMask, 
      result->getRigID(), 
      partGroupMasks[iPart]);
    physx::PxShapeFlags flags = physx::PxShapeFlag::eSCENE_QUERY_SHAPE | physx::PxShapeFlag::eSIMULATION_SHAPE;
#ifdef USE_PHYSX_SWEEPS_FOR_CHARACTER
    flags |= physx::PxShapeFlag::eUSE_SWEPT_BOUNDS;
#endif

    // Volumes
    PhysicsRigDef::Part::Volume::Shape* inOrderShapes[MAX_SHAPES_IN_VOLUME];
    JPH::EShapeSubType inOrderShapeTypes[MAX_SHAPES_IN_VOLUME];
    NMP_ASSERT(numShapes <= MAX_SHAPES_IN_VOLUME);

    for (int32_t j = 0 ; j < volume.numBoxes ; ++j)
    {
      int32_t parentIndex = volume.boxes[j].parentIndex;
      inOrderShapes[parentIndex] = &(volume.boxes[j]);
      inOrderShapeTypes[parentIndex] = JPH::EShapeSubType::Box;
    }
    for (int32_t j = 0 ; j < volume.numCapsules ; ++j)
    {
      int32_t parentIndex = volume.capsules[j].parentIndex;
      inOrderShapes[parentIndex] = &(volume.capsules[j]);
      inOrderShapeTypes[parentIndex] = JPH::EShapeSubType::Capsule;
    }
    for (int32_t j = 0 ; j < volume.numSpheres ; ++j)
    {
      int32_t parentIndex = volume.spheres[j].parentIndex;
      inOrderShapes[parentIndex] = &(volume.spheres[j]);
      inOrderShapeTypes[parentIndex] = JPH::EShapeSubType::Sphere;
    }

    // Volumes
    for (uint32_t j = 0 ; j < numShapes ; ++j)
    {
      PhysicsShapeDriverDataJoltPhys* shapeDriverData = (PhysicsShapeDriverDataJoltPhys*)inOrderShapes[j]->driverData;

      switch (inOrderShapeTypes[j])
      {
      case JPH::EShapeSubType::Box:
        {
          PhysicsRigDef::Part::Volume::Box* shape = reinterpret_cast<PhysicsRigDef::Part::Volume::Box *>(inOrderShapes[j]);

          PxShapeDesc *shapeDesc = shapeDescs[iShape++];
          void *shapeMem = NMPMemoryAlloc(sizeof(physx::PxBoxGeometry));
          NMP_ASSERT(shapeMem);
          physx::PxBoxGeometry *box = new(shapeMem) physx::PxBoxGeometry(nmVector3ToPxVec3(shape->dimensions));
          shapeDesc->geometry = box; 
          shapeDesc->localPose = nmMatrix34ToPxTransform(shape->localPose);
          shapeDesc->contactOffset = shapeDriverData->m_contactOffset;
          shapeDesc->restOffset = shapeDriverData->m_restOffset;
          shapeDesc->userData = &(shape->density);
        }
        break;

      case JPH::EShapeSubType::Capsule:
        {
          PhysicsRigDef::Part::Volume::Capsule* shape = reinterpret_cast<PhysicsRigDef::Part::Volume::Capsule *>(inOrderShapes[j]);

          PxShapeDesc *shapeDesc = shapeDescs[iShape++];
          void *shapeMem = NMPMemoryAlloc(sizeof(physx::PxCapsuleGeometry));
          NMP_ASSERT(shapeMem);
          physx::PxCapsuleGeometry *capsule = new(shapeMem) physx::PxCapsuleGeometry(shape->radius, shape->height * 0.5f);
          shapeDesc->geometry = capsule; 
          // convert to capsule orientated along z, not y
          NMP::Matrix34 m(NMP::Matrix34::kIdentity);
          m.fromEulerXYZ(NMP::Vector3(0, NM_PI/2.f, 0));
          shapeDesc->localPose = nmMatrix34ToPxTransform(m * shape->localPose);
          shapeDesc->contactOffset = shapeDriverData->m_contactOffset;
          shapeDesc->restOffset = shapeDriverData->m_restOffset;
          shapeDesc->userData = &(shape->density);
        }
        break;

      case JPH::EShapeSubType::Sphere:
        {
          PhysicsRigDef::Part::Volume::Sphere* shape = reinterpret_cast<PhysicsRigDef::Part::Volume::Sphere *>(inOrderShapes[j]);

          PxShapeDesc *shapeDesc = shapeDescs[iShape++];
          void *shapeMem = NMPMemoryAlloc(sizeof(physx::PxSphereGeometry));
          NMP_ASSERT(shapeMem);
          physx::PxSphereGeometry *sphere = new(shapeMem) physx::PxSphereGeometry(shape->radius);
          shapeDesc->geometry = sphere; 
          shapeDesc->localPose = nmMatrix34ToPxTransform(shape->localPose);
          shapeDesc->contactOffset = shapeDriverData->m_contactOffset;
          shapeDesc->restOffset = shapeDriverData->m_restOffset;

          shapeDesc->userData = &(shape->density);
        }
        break;

      default:
        NMP_ASSERT_FAIL();
      }
    }

    // Fix up common data
    for (uint32_t shapeIndex = 0 ; shapeIndex < numShapes ; ++shapeIndex)
    {
      PxShapeDesc *shapeDesc = shapeDescs[shapeIndex+iFirstShapeForPart];
      shapeDesc->simulationFilterData = filterData;
      shapeDesc->queryFilterData = filterData;
      shapeDesc->flags = flags;
    }

    rigidDynamicDesc.setShapes(&shapeDescs[iFirstShapeForPart], numShapes);

    // Freed at the end of the function
    void *linkMem = NMPMemoryAlloc(sizeof(PxArticulationLinkDesc));
    NMP_ASSERT(linkMem);
    PxArticulationLinkDesc *linkDesc = new(linkMem) PxArticulationLinkDesc(PxGetPhysics().getTolerancesScale());
    linkDescs[iPart] = linkDesc;
    linkDescs[iPart]->globalPose = nmMatrix34ToPxTransform(part.actor.globalPose);
    // Will make the following code calculate the mass using the densities. However, if +ve, it can
    // be made to use a single total mass.
    linkDescs[iPart]->mass = -1; 
    linkDescs[iPart]->massLocalPose = physx::PxTransform(physx::PxIdentity);

    linkDescs[iPart]->setShapes(&shapeDescs[iFirstShapeForPart], numShapes);
    linkDescs[iPart]->ownerClient = result->m_ownerClientID;
    linkDescs[iPart]->clientBehaviorFlags = result->m_clientBehaviourFlags;

    linkDescs[iPart]->inertiaSphericalisation = bodyDriverData->m_inertiaSphericalisation;

    new(result->m_parts[iPart]) PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll(partGroupMasks[iPart], allowedPartGroupMasks[iPart]);
    PartJoltPhysRagdoll *partPhysX = (PartJoltPhysRagdoll*)(result->m_parts[iPart]);
    partPhysX->m_physicsRig = result;
#ifdef STORE_PART_AND_JOINT_NAMES
    size_t partNameLength = NMP_STRLEN(part.name);
    NMP_ASSERT(partNameLength < PhysicsRigJoltPhysRagdoll::MAX_NAME_SIZE);
    NMP_STRNCPY_S(partPhysX->m_name, PhysicsRigJoltPhysRagdoll::MAX_NAME_SIZE, part.name);
#endif

#if 1
    // Also create a kinematic "ghost" that is used for collision in HK
    partPhysX->m_kinematicBody = PxCreateRigidDynamic(rigidDynamicDesc);
    NMP_ASSERT(partPhysX->m_kinematicActor);
    partPhysX->m_kinematicBody->setName("Physics rig kinematic part");

    // The following line shouldn't be necessary, but it isn't kinematic without it.
    partPhysX->m_kinematicBody->setRigidBodyFlag(physx::PxRigidBodyFlag::eKINEMATIC, true);

    NMP_ASSERT(partPhysX->m_kinematicActor);
    PhysicsRigJoltPhysBodyData::create(partPhysX->m_kinematicBody, partPhysX, result);
#endif
  }

  if (physicsRigDef->getNumJoints())
  {
    PhysicsRigJoltPhysRagdoll::createJoints((PhysicsSceneJoltPhys*) physicsScene, physicsRigDef, result, linkDescs);
  }

  // Set all the masks etc for the kinematic parts - might be better to do this at creation.
  for (uint32_t iPart = 0; iPart < numParts; ++iPart)
  {
    physx::PxShape *shapes[MAX_SHAPES_IN_VOLUME];
    physx::PxFilterData filterData(
      result->m_collisionTypeMask, 
      result->m_collisionIgnoreMask, result->getRigID(), 
      partGroupMasks[iPart]);

    physx::PxRigidDynamic *kinematicActor = ((PartJoltPhysRagdoll*)result->getPart(iPart))->getKinematicActor();
    if (kinematicActor)
    {
      filterData.word0 &= ~(1<<GROUP_COLLIDABLE_PUSHABLE);
      filterData.word0 |= 1<<GROUP_COLLIDABLE_NON_PUSHABLE;
      filterData.word1 |= 1<<GROUP_COLLIDABLE_NON_PUSHABLE;
      physx::PxU32 numShapes =  kinematicActor->getShapes(&shapes[0], MAX_SHAPES_IN_VOLUME);
      NMP_ASSERT(shapes[0]);
      for (physx::PxU32 j = 0 ; j < numShapes ; ++j)
      {
        shapes[j]->setQueryFilterData(filterData);
        shapes[j]->setSimulationFilterData(filterData);
        shapes[j]->userData = 0; // original value was just used to store the density
      }
    }
  }


  // Free the working memory.
  for (uint32_t shapeIndex = 0 ; shapeIndex < totalNumShapes ; ++shapeIndex)
  {
    if (shapeDescs[shapeIndex]->geometry)
    {
      NMP::Memory::memFree(shapeDescs[shapeIndex]->geometry);
    }
  }

  for (uint32_t i = 0; i < totalNumShapes; ++i)
  {
    NMP::Memory::memFree(shapeDescs[i]);
  }
  NMP::Memory::memFree(shapeDescs);

  for (uint32_t i = 0; i < numParts; ++i)
  {
    NMP::Memory::memFree(linkDescs[i]);
  }
  NMP::Memory::memFree(linkDescs);

  // Note that the parent joint index of a bone is guaranteed (in morpheme export) to always the
  // bone index - 1, since every part has a parent joint and parent part, except for the root part.
  Part* part = result->getPart(0);
  part->setParentPartIndex(-1);
  for (uint32_t i = 1; i < physicsRigDef->getNumParts(); i++)
  {
    part = result->getPart(i);
    part->setParentPartIndex(physicsRigDef->m_joints[i-1]->m_parentPartIndex);
  }

  result->restoreAllJointDrivesToDefault();

  result->m_isRagdollAddedToScene = false;

  // Need to generate the cached values.
  result->generateCachedValues(1.0f); // timestep is irrelevant at this point

  return result;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::createLink(
  JPH::RagdollSettings ragdoll, 
  JPH::Body* parent, 
  PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll* part, 
  PhysicsRigJoltPhysRagdoll* physicsRig,
  PxArticulationLinkDesc& linkDesc)
{
  
  m_rigidBody = ragdoll->createLink(parent, linkDesc.globalPose);
  NMP_ASSERT(m_rigidBody);
  uint32_t numShapes = linkDesc.shapes.size();
  for (uint32_t iShape = 0; iShape<numShapes; iShape++)
  {
    const PxShapeDesc *shapeDesc = linkDesc.shapes[iShape];
    physx::PxShape *shape = m_rigidBody->createShape(*shapeDesc->geometry, *shapeDesc->materials[0]);
    shape->setLocalPose(shapeDesc->localPose);
    NMP_ASSERT(shape);
#ifdef STORE_PART_AND_JOINT_NAMES
    m_rigidBody->setName(getName());
#endif
    shape->setContactOffset(shapeDesc->contactOffset);
    shape->setRestOffset(shapeDesc->restOffset);
    shape->setSimulationFilterData(shapeDesc->simulationFilterData);
    shape->setQueryFilterData(shapeDesc->queryFilterData);
    shape->setFlags(shapeDesc->flags | physx::PxShapeFlag::eVISUALIZATION);
    shape->userData = 0; // Original value was just used to store the density
  }

  if (linkDesc.mass < 0.0f)
  {
    float* densities = (float*)NMPMemoryAlloc(sizeof(float) * numShapes);
    NMP_ASSERT(densities);
    for (uint32_t iShape = 0; iShape < numShapes; iShape++)
    {
      densities[iShape] = *((float*)linkDesc.shapes[iShape]->userData);
    }

    physx::PxRigidBodyExt::updateMassAndInertia(*m_rigidBody, densities, numShapes); // uses density

    NMP::Memory::memFree(densities);

    if (linkDesc.inertiaSphericalisation > 0.0f)
    {
      // This piece of code makes the inertia tensors on articulations more even, it helps a little
      // with jitter, and artifacts are not likely to be big, since each link is constrained anyway.
      physx::PxVec3 massVec = m_rigidBody->getMassSpaceInertiaTensor();
      float averageMass = (massVec.x + massVec.y + massVec.z) / 3.0f;
      physx::PxVec3 evenMass(averageMass, averageMass, averageMass);
      massVec += (evenMass-massVec) * linkDesc.inertiaSphericalisation;
      m_rigidBody->setMassSpaceInertiaTensor(massVec);
    }
  }
  else
  {
    physx::PxRigidBodyExt::setMassAndUpdateInertia(*m_rigidBody, linkDesc.mass); // uses mass
  }

  m_mass = m_rigidBody->getMass();
  m_inertia = nmPxVec3ToVector3(m_rigidBody->getMassSpaceInertiaTensor());
  NMP_ASSERT(m_mass > 0.0f);

  PhysicsRigPhysX3ActorData::create(m_rigidBody, part, physicsRig);
  m_rigidBody->setClientBehaviorFlags((physx::PxActorClientBehaviorFlags)physicsRig->m_clientBehaviourFlags); 
  m_rigidBody->setOwnerClient(physicsRig->m_ownerClientID);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::createJoints(
  PhysicsSceneJoltPhys *physicsScene, 
  PhysicsRigDef *physicsRigDef, 
  PhysicsRigJoltPhysRagdoll *physicsRig, 
  PxArticulationLinkDesc **linkDescs)
{
  JPH::Ragdoll *articulation = new JPH::Ragdoll(physicsScene->m_joltPhysScene);
  NMP_ASSERT(articulation != NULL);
  // Since we use warm starting the number of iterations can be very low. However, it should be
  // exposed too (as accuracy will be better with a higher count, if the user can afford it). See MORPH-11268
  articulation->setInternalDriveIterations(1 | 0x80000000); // enable warm starting
  articulation->setExternalDriveIterations(1 | 0x80000000); // enable warm starting
  articulation->setName("PhysicsRigJoltPhysRagdoll");

  // TODO expose these in connect - MORPH-11268
  articulation->setSolverIterationCounts(4, 2);

  physicsRig->setRagdoll(articulation);

  // Add the root link
  const PhysicsJointDef* joint = physicsRigDef->m_joints[0];
  PartJoltPhysRagdoll *part = (PartJoltPhysRagdoll*)physicsRig->m_parts[joint->m_parentPartIndex];

  // Copy the first generated link into the morpheme rig part data... assuming it makes a link for this root.
  part->createLink(articulation, 0, part, physicsRig, *linkDescs[joint->m_parentPartIndex]);

  for (int32_t i = 0; i < physicsRigDef->m_numJoints; ++i)
  {
    NMP_ASSERT(physicsRigDef->m_joints[i]->m_jointType == PhysicsJointDef::JOINT_TYPE_SIX_DOF);
    const PhysicsSixDOFJointDef* jointDef = (const PhysicsSixDOFJointDef*)physicsRigDef->m_joints[i];
    const PhysicsJointDriverDataJoltPhys* driverData = (const PhysicsJointDriverDataPhysX3*)jointDef->m_driverData;

    new (physicsRig->m_joints[i]) PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll(jointDef);

    NMP_ASSERT(jointDef->m_parentPartIndex < (int32_t)physicsRig->getNumParts());
    NMP_ASSERT(jointDef->m_childPartIndex < (int32_t)physicsRig->getNumParts());
    part = (PartJoltPhysRagdoll*)physicsRig->m_parts[jointDef->m_childPartIndex];
    PartJoltPhysRagdoll* parentPart = ((PartJoltPhysRagdoll*)physicsRig->m_parts[jointDef->m_parentPartIndex]);

    part->createLink(
      articulation,
      parentPart->getArticulationLink(), 
      part, 
      physicsRig,
      *linkDescs[jointDef->m_childPartIndex]);

    JointJoltPhysRagdoll *jointPhysX = (JointJoltPhysRagdoll*)physicsRig->m_joints[i];
    physx::PxArticulationJoint *pxJoint = part->getArticulationLink()->getInboundJoint();
    jointPhysX->m_jointInternal = pxJoint;

    /// Internal and external compliances get set by behaviours, and also reset to 1 in
    /// restoreAllJointDrivesToDefault which is called when a reference is added/removed (i.e.
    /// whenever partial body physics configuration changes.
    pxJoint->setExternalCompliance(1.0f); // this gets overwritten in behaviours
    pxJoint->setInternalCompliance(1.0f); // this gets overwritten in behaviours 
    float s2Limit = NMP::maximum(jointDef->m_hardLimits.getSwing2Limit(), s_minSwingLimit);
    float s1Limit = NMP::maximum(jointDef->m_hardLimits.getSwing1Limit(), s_minSwingLimit);
    pxJoint->setSwingLimitContactDistance(jointDef->m_hardLimits.getSwingLimitContactDistance());
    pxJoint->setSwingLimitContactDistance(jointDef->m_hardLimits.getSwingLimitContactDistance());
    pxJoint->setSwingLimitEnabled(true);
    pxJoint->setTwistLimitContactDistance(jointDef->m_hardLimits.getTwistLimitContactDistance());
    pxJoint->setTwistLimitContactDistance(jointDef->m_hardLimits.getTwistLimitContactDistance());
    pxJoint->setTwistLimitEnabled(true);
    pxJoint->setStiffness(0.0f);
    pxJoint->setDamping(driverData->m_articulationDamping);
    // Set the limits, clamping if necessary
    jointPhysX->writeLimits();

    const float amountOfTangentialDamping = 0.8f; // TODO read this in from file?
    float limitCurvature = 2.0f / (s1Limit + s2Limit); 
    // Do we scale by curvature or curvature squared? well, it is somewhere between curvature and
    // curvature squared depending on how fast the joint is moving. Choosing curvature squared here
    // as it gives a better drop off. Why scale by curvature squared then? maybe because big
    // relative motions happen most on small (high curvature) limits.
    pxJoint->setTangentialDamping(amountOfTangentialDamping*limitCurvature*limitCurvature); 
    
    JPH::Mat44 jointToParentActor = nmMatrix34ToJPHMat44(jointDef->m_parentPartFrame);   // joint -> actor
    pxJoint->setParentPose(jointToParentActor);
    
    JPH::Mat44 jointToChildActor = nmMatrix34ToJPHMat44(jointDef->m_childPartFrame);    // joint -> actor
    pxJoint->setChildPose(jointToChildActor);

    // Set pointer to joint def in joint.
//    jointPhysX->setDefinition(jointDef);

#ifdef STORE_PART_AND_JOINT_NAMES
    NMP_ASSERT(strlen(jointDef->m_name) < PhysicsRigJoltPhysRagdoll::MAX_NAME_SIZE - 1);
    strncpy(jointPhysX->m_name, jointDef->m_name, PhysicsRigJoltPhysRagdoll::MAX_NAME_SIZE);
#endif

    jointPhysX->m_strength = 0.0f;
    jointPhysX->m_damping = driverData->m_articulationDamping;

    jointPhysX->m_maxStrength = driverData->m_articulationSpring;
    jointPhysX->m_maxDamping = driverData->m_articulationDamping;

    jointPhysX->m_driveStrengthScale = driverData->m_driveStrengthScale;
    jointPhysX->m_driveDampingScale = driverData->m_driveDampingScale;
    jointPhysX->m_driveCompensationScale = driverData->m_driveCompensationScale;
    jointPhysX->m_driveMinDampingScale = driverData->m_driveMinDampingScale;

    jointPhysX->m_lastTargetOrientation.setXYZW(0,0,0,0); // So will always update the physX internal target orientation on the first frame.

    jointPhysX->m_rotationDirty = true;
  }
 
#ifdef USE_ARTICULATION_AGGREGATE
  physicsRig->m_aggregate = physicsScene->getPhysXScene()->getPhysics().createAggregate(physicsRig->getNumParts(), true); 
  physicsRig->m_aggregate->addArticulation(*articulation);
#endif
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysRagdoll::term()
{
  JPH::PhysicsSystem *joltPhysScene = getPhysicsSceneJoltPhys()->m_joltPhysScene;
  if (joltPhysScene)
  {
    for (uint32_t i = getNumParts(); i-- != 0; )
    {
      PartJoltPhysRagdoll *part = (PartJoltPhysRagdoll*)m_parts[i];
      PhysicsRigJoltPhysBodyData::destroy(
        PhysicsRigJoltPhysBodyData::getFromBody(part->getBody()), part->getBody());
      if (part->getKinematicBody())
      {
        PhysicsRigJoltPhysBodyData::destroy(
          PhysicsRigJoltPhysBodyData::getFromBody(part->getKinematicBody()), part->getKinematicBody());
        joltPhysScene->GetBodyInterface().RemoveBody(part->getKinematicBody()->GetID());
      }
    }

    // Releasing the aggregate alone results in the contents being re-added to the scene directly.
    // So - release the articulation first. 
    m_ragdoll->Release();
  }

  return true;
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysRagdoll::PhysicsRigJoltPhysRagdoll(PhysicsSceneJoltPhys *physicsScene)
{
  m_physicsScene = physicsScene;
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::PartJoltPhysRagdoll(
  int32_t defaultCollisionGroupMask, int32_t allowedCollisionGroupMask)
{
  m_userData = NULL;
  m_recalcVels = false;
  m_parentPartIndex = -1;
  m_isKinematic = false;
  m_defaultCollisionGroupMask = m_currentCollisionGroupMask = defaultCollisionGroupMask;
  m_allowedCollisionGroupMask = allowedCollisionGroupMask;
  m_mass = 0.0f;
  m_extraMass = 0.0f;
  m_extraMassCOMPosition.setToZero();
  m_modifiedFlags = 0;
  m_kinematicBody = 0;
  m_constraintToKinematic = 0;
  m_massMultiplier = 1.0f;
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::~PartJoltPhysRagdoll() 
{
  if (m_constraintToKinematic)
    m_constraintToKinematic->Release();
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::PartJoltPhysRagdoll(const PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll &other) 
{
  *this = other;
}

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll &PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::operator=(const PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll &other) 
{
  if (this == &other)
    return *this;

#ifdef STORE_PART_AND_JOINT_NAMES
  memcpy(m_name, other.m_name, MAX_NAME_SIZE);
#endif
  m_rigidBody = other.m_rigidBody;
  m_parentPartIndex = other.m_parentPartIndex;
  return *this;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::setCurrentCollisionGroupMask(physx::PxU32 mask)
{
  m_currentCollisionGroupMask = mask;

  physx::PxShape *shapeBuffer[MAX_SHAPES_IN_VOLUME];
  physx::PxU32 numShapes = m_rigidBody->getShapes(shapeBuffer, MAX_SHAPES_IN_VOLUME);
  for (physx::PxU32 iShape = 0 ; iShape < numShapes ; ++iShape)
  {
    physx::PxShape *shape = shapeBuffer[iShape];
    physx::PxFilterData filterData = shape->getSimulationFilterData();
    if (filterData.word3 != mask)
    {
      filterData.word3 = mask;
      shape->setSimulationFilterData(filterData);
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::generateCachedValues(float timeStep)
{
  // Get the pose from the kinematic part if the articulation wasn't added,
  // otherwise it is delayed by a frame in HK (because in HK we set the position
  // to the last transform and expect physx to move it according to the
  // velocity).
  const JPH::Mat44 currentGlobalPose = 
    ((PhysicsRigJoltPhysRagdoll*)m_physicsRig)->m_isRagdollAddedToScene 
    ? m_rigidBody->GetWorldTransform()
    : m_kinematicBody->GetWorldTransform();

  //if (!currentGlobalPose.isFinite())
  //{
  //  return false;
  //}

  const NMP::Matrix34 currentTM = nmJPHMat44ToNmMatrix34(currentGlobalPose);

  // Note that the local COM offset wouldn't normally change, but it might if the part shapes are modified.
  NMP_ASSERT(m_rigidBody->getCMassLocalPose().isFinite());
  m_cache.cachedCOMOffset = nmJPHMat44ToNmMatrix34(m_rigidBody->getCMassLocalPose());

  NMP::Vector3 currentComPos = currentTM.getTransformedVector(m_cache.cachedCOMOffset.translation());

  if (m_recalcVels && timeStep > 0.0f) 
  {
    // reset velocities after physics based on new transform relative to cached (previous) transform

    // calculate the motion at the CoM for the velocities
    NMP::Matrix34 invPrevTM(m_cache.cachedTransform); invPrevTM.invertFast();
    NMP::Matrix34 motionTM = invPrevTM * currentTM;
    m_cache.cachedVel = (motionTM.getTransformedVector(currentComPos) - currentComPos) / timeStep;
    m_cache.cachedAngVel = motionTM.toQuat().toRotationVector(false) / timeStep;
    m_recalcVels = false;
  }
  else
  {
    JPH::Vec3 angVel = m_rigidBody->GetAngularVelocity();
    JPH::Vec3 linVel = m_rigidBody->GetLinearVelocity();
    if (angVel.IsNaN() || !linVel.IsNaN())
    {
      return false;
    }
    m_cache.cachedAngVel = nmJPHVec3ToVector3(angVel);
    m_cache.cachedVel = nmJPHVec3ToVector3(linVel);
  }

  m_cache.cachedTransform = currentTM;
  m_cache.cachedCOMPosition = currentComPos;

  // The transforms from PhysX can be pretty bad, so orthonormalise the result.
  m_cache.cachedTransform.orthonormalise();

  return true;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::handleExplosion(const NMP::Matrix34& worldRoot)
{
  if (m_isRagdollAddedToScene)
  {
    removeRagdollFromScene();
    addRagdollToScene();
  }

  for (uint32_t i = 0; i < getNumParts(); i++)
  {
    PartJoltPhysRagdoll* partJoltPhysRagdoll = (PartJoltPhysRagdoll*)m_parts[i];

    NMP::Matrix34 tm;
    calculateWorldSpacePartTM(
      tm, 
      i, 
      *m_animRigDef->getBindPose()->m_transformBuffer, 
      *m_animRigDef->getBindPose()->m_transformBuffer, 
      worldRoot, 
      false);

    partJoltPhysRagdoll->setTransform(tm);
    partJoltPhysRagdoll->setVel(NMP::Vector3::InitZero);
    partJoltPhysRagdoll->setAngVel(NMP::Vector3::InitZero);
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::generateCachedValues(float timeStep)
{
  bool OK = true;
  for (uint32_t i = 0; i < getNumParts(); i++)
  {
    if (!((PartJoltPhysRagdoll*)m_parts[i])->generateCachedValues(timeStep))
    {
      OK = false;
    }
  }
  if (!OK)
  {
    NMP_MSG("PhysX has exploded - reinitialising at the last known position\n");

    PartJoltPhysRagdoll* rootPartJoltPhysRagdoll = (PartJoltPhysRagdoll*)m_parts[0];
    NMP::Matrix34 worldRoot = rootPartJoltPhysRagdoll->getTransform();
    if (!worldRoot.isValidTM(0.001f))
    {
      worldRoot.identity();
    }

    if (s_explosionHandler)
    {
      (*s_explosionHandler)(this, worldRoot);
    }
    else
    {
      handleExplosion(worldRoot);
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::updatePrePhysics(float timeStep)
{
  const PhysicsRigDef* physicsRigDef = getPhysicsRigDef();

  // Activate requested collision sets
  for (uint32_t i = 0, numParts = getNumParts(); i < numParts; i++)
  {
    MR::PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll *partPhysX = getPartJoltPhysRagdoll(i);
    uint32_t desiredMask = partPhysX->getDefaultCollisionGroupMask();
    uint32_t allowedMask = partPhysX->getAllowedCollisionGroupMask();
    NMP_ASSERT(m_numCollisionGroupIndices < m_maxCollisionGroupIndices);
    for (int iSet = 0 ; iSet < m_numCollisionGroupIndices ; ++iSet)
    {
      int collisionGroupIndexToActivate = m_collisionGroupIndicesToActivate[iSet];
      if (collisionGroupIndexToActivate >= 0 && (allowedMask & 1 << collisionGroupIndexToActivate))
      {
        NMP_ASSERT(collisionGroupIndexToActivate < m_maxCollisionGroupIndices);
        desiredMask |= 1 << collisionGroupIndexToActivate;
      }
    }
    // If the desired mask has changed then set it on all the shapes.
    if (desiredMask != partPhysX->getCurrentCollisionGroupMask())
    {
      partPhysX->setCurrentCollisionGroupMask(desiredMask);
    }

    // increase the skin width to provide pseudo-CCD
    const PhysicsActorDriverDataJoltPhys* actorDriverData =
      (const PhysicsActorDriverDataJoltPhys*)physicsRigDef->m_parts[i].actor.driverData;

    float maxSkinWidthIncrease = actorDriverData->m_maxContactOffsetIncrease;
    float speed = partPhysX->getVel().magnitude();
    float skinWidthIncrease = NMP::minimum(speed * timeStep, maxSkinWidthIncrease);
    if (skinWidthIncrease > 0.0f)
    {
      // don't call less than 0 as it resets what the behaviours have asked for (and calling with 0
      // does nothing).
      setSkinWidthIncrease(i, skinWidthIncrease);
    }

    // Reset properties that weren't modified since the last pre-physics call
    if (!(partPhysX->m_modifiedFlags & PartJoltPhysRagdoll::MODIFIED_EXTRA_MASS))
    {
      partPhysX->setExtraMass(0.0f, NMP::Vector3::InitZero);
    }
    if (!(partPhysX->m_modifiedFlags & PartJoltPhysRagdoll::MODIFIED_INERTIA))
    {
      partPhysX->setMassSpaceInertia(partPhysX->getOriginalMassSpaceInertia() * partPhysX->m_massMultiplier);
    }
    partPhysX->m_modifiedFlags = 0;
  }

  // Update physX joint limits.
  writeJointLimits();

  // Prevent large compliances from letting the ISF value in physx get < 1, which would lead to
  // instability.
  uint32_t numJoints = getNumJoints();
  for (uint32_t iJoint = 0 ; iJoint != numJoints ; ++iJoint)
  {
    JointJoltPhysRagdoll* joint = getJointJoltPhysRagdoll(iJoint);

    float s = joint->getStrength();
    float d = joint->getDamping();
    float c = joint->getExternalCompliance();

    float isf = (1 + d * timeStep + s * timeStep * timeStep) / c;
    if (isf < 1.0f)
    {
      c *= isf;
      joint->setExternalCompliance(c);
    }
  }

  // reset the list of collision sets to activate
  m_numCollisionGroupIndices = 0;

  // Set the separation properties - these will get reset in updatePostPhysics
  //m_ragdoll->setSeparationTolerance(m_desiredJointProjectionLinearTolerance);
  // Note that there is no angular tolerance
  //m_ragdoll->setMaxProjectionIterations(m_desiredJointProjectionIterations);

  updateRegisteredJoints();
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::updatePostPhysics(float timeStep)
{
  if (!isReferenced())
  {
    return;
  }

  reenableSleeping();

  PhysicsRigJoltPhysRagdoll *physicsRigPhysX = (PhysicsRigJoltPhysRagdoll *)this;
  physicsRigPhysX->generateCachedValues(timeStep);

  // Reset all the joint drives to be relaxed - this is needed so that behaviours can interact
  // nicely with simple physics and we don't have to worry about overwriting physics nodes when
  // trying to stop their actions persisting... However, the multiple calls to the physics api are
  // not likely to be efficient - we should make it so the physx api calls are all done in a
  // pre/post-physics function only, and then this could just be done by setting some state
  // variables. See MORPH-11292
  for (uint32_t i = 0 ;i < physicsRigPhysX->getNumJoints(); ++i)
  {
    JointJoltPhysRagdoll *joint = getJointJoltPhysRagdoll(i);
    joint->m_rotationDirty = true;
    joint->setStrength(0.0f);
    joint->setDamping(joint->getMaxDamping());
    joint->setDriveCompensation(0.0f);
  }

#if defined(MR_OUTPUT_DEBUGGING)
  // Copy joint limits from joint into serialisation data structure.
  for (uint32_t i = 0, numJoints = getNumJoints(); i < numJoints; ++i)
  {
    static_cast< JointJoltPhys* >(getJoint(i))->updateSerializeTxFrameData();
  }
#endif // MR_OUTPUT_DEBUGGING

  resetJointLimits();
  m_desiredJointProjectionIterations = 0;
  m_desiredJointProjectionLinearTolerance = FLT_MAX;

  for (uint32_t i = 0 ; i < physicsRigPhysX->getNumParts() ; ++i)
  {
    physicsRigPhysX->setSkinWidthIncrease(i, 0.0f);
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::setExtraMass(float mass, const NMP::Vector3& massCOMPosition)
{
  m_extraMass = mass;
  m_extraMassCOMPosition = massCOMPosition;
  m_modifiedFlags |= MODIFIED_EXTRA_MASS;
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Vector3 PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::getAugmentedCOMPosition() const
{
  return (m_cache.cachedCOMPosition * m_mass + m_extraMassCOMPosition * m_extraMass) / (m_mass + m_extraMass);
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Vector3 PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::getCOMPosition() const 
{
  return m_cache.cachedCOMPosition; 
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Vector3 PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::getVelocityAtPoint(const NMP::Vector3 &point) const
{
  NMP::Vector3 rpoint = point - getCOMPosition();
  return getVel() + NMP::vCross(getAngVel(), rpoint);
}

//---------------------------------------------------------------------------------------------------------------------- 
NMP::Vector3 PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::getAngularMomentum() const  
{
  NMP::Matrix34 inertia = getGlobalInertiaTensor();
  NMP::Vector3 angVel = getAngVel();
  NMP::Vector3 angMom;
  inertia.rotateVector(angVel, angMom);
  return angMom;
}  

//----------------------------------------------------------------------------------------------------------------------  
NMP::Vector3 PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::getLinearMomentum() const  
{    
  return getVel() * getMass();
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Vector3 PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::getMassSpaceInertiaTensor() const
{
  return nmJPHVec3ToVector3(m_rigidBody->getMassSpaceInertiaTensor());
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Matrix34 PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::getGlobalInertiaTensor() const
{
  NMP::Vector3 t = nmJPHVec3ToVector3(m_rigidBody->getMassSpaceInertiaTensor());
  NMP::Matrix34 massSpaceInertiaTensor(NMP::Matrix34::kIdentity);
  massSpaceInertiaTensor.scale3x3(t);
  NMP::Matrix34 localOffset = getCOMOffsetLocal();
  NMP::Matrix34 result = massSpaceInertiaTensor * localOffset * getTransform();
  return result;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::setMassSpaceInertia(const NMP::Vector3& inertia)
{
  // Node that this does little more than copying the data on the PhysX side, so should be pretty
  // fast.
  m_rigidBody->setMassSpaceInertiaTensor(MR::nmVector3ToJPHVec3(inertia));
  m_modifiedFlags |= MODIFIED_INERTIA;
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Quat PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::getQuaternion() const
{
  return nmJPHQuatToQuat(m_rigidBody->GetWorldTransform().GetQuaternion());
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::setVel(const NMP::Vector3 &v)
{
  m_rigidBody->SetLinearVelocity(nmVector3ToJPHVec3(v));
  m_cache.cachedVel = v;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::setAngVel(const NMP::Vector3 &v)
{
  m_rigidBody->SetAngularVelocity(nmVector3ToJPHVec3(v));
  m_cache.cachedAngVel = v;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::moveTo(const NMP::Matrix34 &tm, bool updateCache)
{
  // Note that with PhysX3 hardkeyframing the kinematic part uses move, but the dynamic part
  // position gets set.
  m_rigidBody->setGlobalPose(nmMatrix34ToPxTransform(tm));
  if (m_isKinematic && m_kinematicActor)
    m_kinematicActor->setKinematicTarget(nmMatrix34ToPxTransform(tm));
  if (updateCache)
  {
    m_cache.cachedTransform = tm;
    m_cache.cachedCOMPosition = nmPxVec3ToVector3(nmMatrix34ToPxTransform(tm).transform(m_rigidBody->getCMassLocalPose().p));
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::setPosition(const NMP::Vector3 &NMP_UNUSED(p))
{
  // This function is not currently implemented for PhysX3
  NMP_ASSERT_FAIL();
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::setQuaternion(const NMP::Quat &NMP_UNUSED(q))
{
  // This function is not currently implemented for PhysX3
  NMP_ASSERT_FAIL();
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::setTransform(const NMP::Matrix34 &tm)
{
  NMP_ASSERT(tm.isValidTM(0.1f));
  m_rigidBody->setGlobalPose(nmMatrix34ToPxTransform(tm));
  if (m_isKinematic && m_kinematicActor)
    m_kinematicActor->setGlobalPose(nmMatrix34ToPxTransform(tm));
  m_cache.cachedTransform = tm;
  m_cache.cachedCOMPosition = nmPxVec3ToVector3(nmMatrix34ToPxTransform(tm).transform(m_rigidBody->getCMassLocalPose().p));
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::makeKinematic(
  bool kinematic, float massMultiplier, bool enableConstraint)
{
  if (!kinematic)
  {
    m_isBeingKeyframed = false;
    massMultiplier = 1.0f;
    enableConstraint = false;
  }

  if (
    kinematic == m_isKinematic && 
    massMultiplier == m_massMultiplier && 
    enableConstraint == (m_constraintToKinematic ? true : false)
    )
  {
    return;
  }

  physx::PxArticulationLink *link = getArticulationLink();
  NMP_ASSERT(link);

  if (!kinematic)
  {
    // Move the kinematic shape somewhere far.
    if (m_kinematicBody)
    {
      m_kinematicBody->setGlobalPose(nmMatrix34ToPxTransform(
        ((PhysicsRigJoltPhysRagdoll*)m_physicsRig)->m_kinematicPose));
    }
  }
  else
  {
    if (m_kinematicBody)
    {
      m_kinematicActor->setGlobalPose(link->getGlobalPose());
    }
  }
  m_isKinematic = kinematic;

  if (massMultiplier != m_massMultiplier)
  {
    link->setMass(m_mass * massMultiplier);
    link->setMassSpaceInertiaTensor(nmVector3ToPxVec3(m_inertia) * massMultiplier);
    m_massMultiplier = massMultiplier;
  }

  if (enableConstraint && !m_constraintToKinematic)
  {
    m_constraintToKinematic = PxD6JointCreate(
      PxGetPhysics(), 
      m_kinematicActor,
      physx::PxTransform(physx::PxIdentity),
      link,
      physx::PxTransform(physx::PxIdentity));
    m_constraintToKinematic->setMotion(physx::PxD6Axis::eX, physx::PxD6Motion::eLOCKED);
    m_constraintToKinematic->setMotion(physx::PxD6Axis::eY, physx::PxD6Motion::eLOCKED);
    m_constraintToKinematic->setMotion(physx::PxD6Axis::eZ, physx::PxD6Motion::eLOCKED);
    m_constraintToKinematic->setMotion(physx::PxD6Axis::eSWING1, physx::PxD6Motion::eLOCKED);
    m_constraintToKinematic->setMotion(physx::PxD6Axis::eSWING2, physx::PxD6Motion::eLOCKED);
    m_constraintToKinematic->setMotion(physx::PxD6Axis::eTWIST, physx::PxD6Motion::eLOCKED);
  }
  else if (!enableConstraint && m_constraintToKinematic)
  {
    m_constraintToKinematic->Release();
    m_constraintToKinematic = 0;
  }

}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::isKinematic() const
{
  return m_isKinematic;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::enableBodyCollision(JPH::Body *body, bool enable)
{
  NMP_ASSERT(actor);
  physx::PxShape *shapes[MAX_SHAPES_IN_VOLUME];
  physx::PxRigidActor *rigidActor = actor->is<physx::PxRigidActor>();
  NMP_ASSERT(rigidActor);
  NMP_ASSERT(rigidActor->getNbShapes() <= MAX_SHAPES_IN_VOLUME);
  physx::PxU32 numShapes = rigidActor->getShapes(&shapes[0], MAX_SHAPES_IN_VOLUME);
  NMP_ASSERT(numShapes && shapes[0]);

  // Assume all have the same collision enabled/disabled status. However, note that they may have
  // different collision and query status.
  physx::PxShapeFlags flags = shapes[0]->getFlags(); 

  bool enabledSimulation = flags & physx::PxShapeFlag::eSIMULATION_SHAPE;
  if (enabledSimulation != enable)
  {
    for (physx::PxU32 i = 0 ; i < numShapes ; ++i)
    {
      shapes[i]->setFlag(physx::PxShapeFlag::eSIMULATION_SHAPE, enable);
    }
  }

  bool enabledQuery = flags & physx::PxShapeFlag::eSCENE_QUERY_SHAPE;
  if (enabledQuery != enable)
  {
    for (physx::PxU32 i = 0 ; i < numShapes ; ++i)
    {
      shapes[i]->setFlag(physx::PxShapeFlag::eSCENE_QUERY_SHAPE, enable);
    }
  }
  // shouldn't need to call resetFiltering here, as the docs don't indicate that it's necessary
  // after setting the flags since the filter function uses eSUPPRESS (i.e. the filter shader will
  // get called again if the flags change).
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::enableCollision(bool enable)
{
  if (m_isKinematic && m_kinematicBody)
  {
    enableBodyCollision(m_kinematicBody, enable);
  }
  else
  {
    enableBodyCollision(m_rigidBody, enable);
  }
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::getCollisionEnabled() const
{
  static const physx::PxU32 maxShapes = 1;
  physx::PxShape *shapes[maxShapes];
  m_rigidBody->getShapes(&shapes[0], maxShapes);
  NMP_ASSERT(shapes[0]);
  physx::PxShapeFlags flags = shapes[0]->getFlags(); // assume all have the same collision enabled/disabled status
  bool enabledOnDynamic = flags & physx::PxShapeFlag::eSIMULATION_SHAPE;//getActorFlags() & NX_AF_DISABLE_COLLISION;

  return enabledOnDynamic;
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::storeState(PhysicsSerialisationBuffer &savedState)
{
  savedState.addValue(getTransform());
  savedState.addValue(getVel());
  savedState.addValue(getAngVel());
  return true;
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::restoreState(PhysicsSerialisationBuffer &savedState)
{
  NMP::Matrix34 m = savedState.getValue<NMP::Matrix34>();
  setTransform(m);
  NMP::Vector3 vel = savedState.getValue<NMP::Vector3>();
  setVel(vel);
  NMP::Vector3 angVel = savedState.getValue<NMP::Vector3>();
  setAngVel(angVel);
  return true;
}

#if defined(MR_OUTPUT_DEBUGGING)

//----------------------------------------------------------------------------------------------------------------------
uint32_t PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::serializeTxPersistentData(
  uint16_t nameToken, 
  uint32_t objectID, 
  void* outputBuffer, 
  uint32_t NMP_USED_FOR_ASSERTS(outputBufferSize)) const
{
  uint32_t dataSize = sizeof(PhysicsPartPersistentData);

  uint32_t numBoxes = 0;
  uint32_t numCapsules = 0;
  uint32_t numSpheres = 0;

  physx::PxU32 numShapes = m_kinematicBody->getNbShapes();
  NMP_ASSERT(numShapes < MAX_SHAPES_IN_VOLUME);

  physx::PxShape* shapes[MAX_SHAPES_IN_VOLUME];
  numShapes = m_kinematicActor->getShapes(shapes, MAX_SHAPES_IN_VOLUME);
  for (physx::PxU32 i = 0; i != numShapes; ++i)
  {
    const JPH::Shape *shape = shapes[i];

    JPH::EShapeSubType type = shape->GetSubType();
    switch (type)
    {
    case JPH::EShapeSubType::Sphere:
      ++numSpheres;
      break;
    case JPH::EShapeSubType::Box:
      ++numBoxes;
      break;
    case JPH::EShapeSubType::Capsule:
      ++numCapsules;
      break;
    default:
      break;
    }
  }

  dataSize += numBoxes * sizeof(PhysicsBoxPersistentData);
  dataSize += numCapsules * sizeof(PhysicsCapsulePersistentData);
  dataSize += numSpheres * sizeof(PhysicsSpherePersistentData);

  if (outputBuffer != 0)
  {
    NMP_ASSERT(outputBufferSize >= dataSize);
    PhysicsPartPersistentData *partPersistentData = (PhysicsPartPersistentData *)outputBuffer;

    partPersistentData->m_parentIndex = getParentPartIndex();
    partPersistentData->m_physicsObjectID = objectID;
    partPersistentData->m_numBoxes = numBoxes;
    partPersistentData->m_numCapsules = numCapsules;
    partPersistentData->m_numSpheres = numSpheres;
    partPersistentData->m_nameToken = nameToken;

    // convert to capsule orientated along y, not z, this code mirrors the creation code.
    NMP::Matrix34 yToZ(NMP::Matrix34::kIdentity);

    // convert to capsule orientated along z, not y
    NMP::Matrix34 capsuleConversionTx(NMP::Matrix34::kIdentity);
    capsuleConversionTx.fromEulerXYZ(NMP::Vector3(0, NM_PI_OVER_TWO, 0));

    uint32_t indexBox = 0;
    uint32_t indexCapsule = 0;
    uint32_t indexSphere = 0;
    for (uint32_t i = 0; i != numShapes; ++i)
    {
      const JPH::Shape *pxShape = shapes[i];

      JPH::EShapeSubType type = pxShape->GetSubType();
      switch (type)
      {
        case JPH::EShapeSubType::Sphere:
        {
          JPH::SphereShape *pxSphere = dynamic_cast<JPH::SphereShape*>((JPH::Shape*)pxShape);
          NMP_ASSERT(pxSphere);

          PhysicsSpherePersistentData* persistentData = partPersistentData->getSphere(indexSphere);

          //JPH::Mat44 localPose = 
          persistentData->m_localPose = NMP::Matrix34Identity(); //nmPxTransformToNmMatrix34(localPose);

          persistentData->m_radius = pxSphere->GetRadius();

          persistentData->m_parentIndex = i;
          NMP::netEndianSwap(persistentData->m_parentIndex);
          NMP::netEndianSwap(persistentData->m_localPose);
          NMP::netEndianSwap(persistentData->m_radius);

          ++indexSphere;
          break;
        }
        case JPH::EShapeSubType::Box:
        {
          JPH::BoxShape* pxBox = dynamic_cast<JPH::BoxShape*>((JPH::Shape*)pxShape);
          NMP_ASSERT(pxBox);

          PhysicsBoxPersistentData* persistentData = partPersistentData->getBox(indexBox);

          //JPH::Mat44 localPose = 
          persistentData->m_localPose = NMP::Matrix34Identity(); //nmPxTransformToNmMatrix34(localPose);

          persistentData->m_width = 2.0f * pxBox->GetHalfExtent().GetX();
          persistentData->m_height = 2.0f * pxBox->GetHalfExtent().GetY();
          persistentData->m_depth = 2.0f * pxBox->GetHalfExtent().GetZ();

          persistentData->m_parentIndex = i;
          NMP::netEndianSwap(persistentData->m_parentIndex);
          NMP::netEndianSwap(persistentData->m_localPose);
          NMP::netEndianSwap(persistentData->m_width);
          NMP::netEndianSwap(persistentData->m_height);
          NMP::netEndianSwap(persistentData->m_depth);

          ++indexBox;
          break;
        }
        case JPH::EShapeSubType::Capsule:
        {
          JPH::CapsuleShape* pxCapsule = dynamic_cast<JPH::CapsuleShape*>((JPH::Shape*)pxShape);
          NMP_ASSERT(pxCapsule);

          PhysicsCapsulePersistentData* persistentData = partPersistentData->getCapsule(indexCapsule);

          //JPH::Mat44 localPose = 
          persistentData->m_localPose = NMP::Matrix34Identity(); //nmPxTransformToNmMatrix34(localPose);

          persistentData->m_radius = pxCapsule->GetRadius();
          persistentData->m_height = 2.0f * pxCapsule->GetHalfHeightOfCylinder();

          persistentData->m_parentIndex = i;
          NMP::netEndianSwap(persistentData->m_parentIndex);
          NMP::netEndianSwap(persistentData->m_localPose);
          NMP::netEndianSwap(persistentData->m_radius);
          NMP::netEndianSwap(persistentData->m_height);

          ++indexCapsule;
          break;
        }
      default:
        break;
      }
    }

    NMP_ASSERT(indexBox == numBoxes);
    NMP_ASSERT(indexCapsule == numCapsules);
    NMP_ASSERT(indexSphere == numSpheres);

    NMP::netEndianSwap(partPersistentData->m_numSpheres);
    NMP::netEndianSwap(partPersistentData->m_numCapsules);
    NMP::netEndianSwap(partPersistentData->m_numBoxes);
    NMP::netEndianSwap(partPersistentData->m_nameToken);
    NMP::netEndianSwap(partPersistentData->m_parentIndex);
    NMP::netEndianSwap(partPersistentData->m_physicsObjectID);
  }

  return dataSize;
}

//----------------------------------------------------------------------------------------------------------------------
uint32_t PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll::serializeTxFrameData(void* outputBuffer, uint32_t NMP_USED_FOR_ASSERTS(outputBufferSize)) const
{
  uint32_t dataSize = sizeof(PhysicsPartFrameData);

  if (outputBuffer != 0)
  {
    NMP_ASSERT(outputBufferSize >= dataSize);
    PhysicsPartFrameData *partFrameData = (PhysicsPartFrameData *)outputBuffer;

    JPH::Mat44 globalPose;
    if (m_isKinematic)
    {
      globalPose = m_kinematicBody->GetWorldTransform();
    }
    else
    {
      globalPose = m_rigidBody->GetWorldTransform();
    }
    partFrameData->m_globalPose = nmJPHMat44ToNmMatrix34(globalPose);

    NMP::netEndianSwap(partFrameData->m_globalPose);
  }

  return dataSize;
}
#endif

//----------------------------------------------------------------------------------------------------------------------
// PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::JointJoltPhysRagdoll(const PhysicsSixDOFJointDef* const def)
: JointJoltPhys(def)
{}

#if defined(MR_OUTPUT_DEBUGGING)
//----------------------------------------------------------------------------------------------------------------------
uint32_t PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::serializeTxPersistentData(
  const PhysicsJointDef* jointDef,
  uint16_t  stringToken,
  void*     outputBuffer,
  uint32_t  NMP_USED_FOR_ASSERTS(outputBufferSize)) const
{
  uint32_t dataSize = sizeof(PhysicsSixDOFJointPersistentData);
  if (outputBuffer)
  {
    NMP_ASSERT(outputBufferSize >= dataSize);

    PhysicsSixDOFJointPersistentData* persistentData = (PhysicsSixDOFJointPersistentData*)outputBuffer;

    persistentData->m_parentLocalFrame = jointDef->m_parentPartFrame;
    persistentData->m_childLocalFrame = jointDef->m_childPartFrame;
    persistentData->m_parentPartIndex = jointDef->m_parentPartIndex;
    persistentData->m_childPartIndex = jointDef->m_childPartIndex;

    persistentData->m_jointType = PhysicsJointPersistentData::JOINT_TYPE_SIX_DOF;

    physx::PxReal swingLimitY = 0.0f;
    physx::PxReal swingLimitZ = 0.0f;
    m_jointInternal->getSwingLimit(swingLimitY, swingLimitZ);

    // for some reason PxArticulationJoint returns the swing limit values
    // the wrong way round so swap the y and z values.
    persistentData->m_swing1Limit = swingLimitZ;
    persistentData->m_swing2Limit = swingLimitY;

    physx::PxReal twistLimitLow = 0.0f;
    physx::PxReal twistLimitHigh = 0.0f;
    m_jointInternal->getTwistLimit(twistLimitLow, twistLimitHigh);

    persistentData->m_twistLimitLow = twistLimitLow;
    persistentData->m_twistLimitHigh = twistLimitHigh;
    persistentData->m_nameToken = stringToken;

    PhysicsSixDOFJointPersistentData::endianSwap(persistentData);
  }

  return dataSize;
}
#endif

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::storeState(PhysicsSerialisationBuffer& savedState)
{
  (void) savedState;
  return true;
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::restoreState(PhysicsSerialisationBuffer& savedState)
{
  (void) savedState;
  return true;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::enableLimit(bool enable)
{
  //m_jointInternal->setSwingLimitEnabled(enable);
  //m_jointInternal->setTwistLimitEnabled(enable);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::writeLimits()
{
  // PhysX crashes with zero swing range, as well as it just causing jitter when very small (e.g.
  // with "hinge" limits).
  float swing1 = NMP::maximum(m_modifiableLimits.getSwing1Limit(), s_minSwingLimit);
  float swing2 = NMP::maximum(m_modifiableLimits.getSwing2Limit(), s_minSwingLimit);
  float twistLow = m_modifiableLimits.getTwistLimitLow();
  float twistHigh = m_modifiableLimits.getTwistLimitHigh();

  // Optimise this with MORPH-16668. Note that swing1/2 are reversed (compare with the jointed rig
  // where they're not reversed) - this is intentional - just how the joint is in PhysX.
  //m_jointInternal->setSwingLimit(swing2, swing1);
  //m_jointInternal->setTwistLimit(twistLow, twistHigh);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::setStrength(float strength)
{
  NMP_ASSERT(m_jointInternal);
  NMP_ASSERT(strength >= 0.0f && strength < MAX_STRENGTH);

  m_strength = strength;
  //m_jointInternal->setStiffness(strength);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::setDamping(float damping)
{
  NMP_ASSERT(m_jointInternal);
  NMP_ASSERT(damping >= 0.0f && damping < MAX_DAMPING);
  m_damping = damping;
  //m_jointInternal->setDamping(damping);
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::supportsDriveCompensation()
{
  return true;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::setDriveCompensation(float driveCompensation)
{
  NMP_ASSERT(m_jointInternal);
  NMP_ASSERT(driveCompensation >= 0.f); 
  setInternalCompliance(1.f/(1.f + driveCompensation));
}

//----------------------------------------------------------------------------------------------------------------------
float PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::getStrength() const
{
  return m_strength;
}

//----------------------------------------------------------------------------------------------------------------------
float PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::getDamping() const
{
  return m_damping;
}

//----------------------------------------------------------------------------------------------------------------------
float PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::getDriveCompensation() const
{
  NMP_ASSERT(m_jointInternal);
  //float internalCompliance = m_jointInternal->getInternalCompliance();
  //if (internalCompliance < MINIMUM_COMPLIANCE)
  //  internalCompliance = MINIMUM_COMPLIANCE;
  //return (1.0f / internalCompliance) - 1.0f;
  return 0;
}

//----------------------------------------------------------------------------------------------------------------------
float PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::getInternalCompliance() const
{
  //return m_jointInternal->getInternalCompliance();
    return 0;
}

//----------------------------------------------------------------------------------------------------------------------
float PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::getExternalCompliance() const
{
  //return m_jointInternal->getExternalCompliance();
    return 0;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::setExternalCompliance(float compliance) 
{
  //if (compliance < MINIMUM_COMPLIANCE)
  //  compliance = MINIMUM_COMPLIANCE;
  //m_jointInternal->setExternalCompliance(compliance);
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::setInternalCompliance(float compliance) 
{
  //if (compliance < MINIMUM_COMPLIANCE)
  //  compliance = MINIMUM_COMPLIANCE;
  //m_jointInternal->setInternalCompliance(compliance);
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Quat PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::getTargetOrientation()
{
  NMP_ASSERT(m_jointInternal);
  return nmJPHQuatToQuat(m_jointInternal->GetTargetOrientationCS());
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::setTargetOrientation(const NMP::Quat &orientation)
{
  NMP_ASSERT(orientation.isValid());
  // This check allows characters to go to sleep when a constant target is being passed in.
  // Since setTargetOrientation wakes up the character.
  if (orientation != m_lastTargetOrientation)
  {
    m_jointInternal->SetTargetOrientationCS(nmQuatToJPHQuat(orientation));
  }
  m_lastTargetOrientation = orientation;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll::setVelocity(const NMP::Vector3 &velocity)
{
  NMP_ASSERT(velocity.isValid());
  m_jointInternal->SetTargetVelocityCS(nmVector3ToJPHVec3(velocity));
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::makeKinematic(bool moveToKinematicPos)
{
  NMP_ASSERT(m_refCount == 0);

  for (uint32_t i = 0; i < getNumParts(); ++i)
  {
    PartJoltPhysRagdoll *part = (PartJoltPhysRagdoll*)m_parts[i];
    part->makeKinematic(true, 1.0f, false);
    if (part->m_kinematicBody)
      part->enableBodyCollision(part->m_kinematicBody, true);
    part->enableBodyCollision(part->m_rigidBody, false);
  }

  if (moveToKinematicPos)
  {
    moveAllToKinematicPos();
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::removeFromScene()
{
  NMP_ASSERT(m_refCount == 0);

  m_ragdoll->Activate();

  removeRagdollFromScene();
  for (uint32_t i = 0; i < getNumParts(); ++i)
  {
    PartJoltPhysRagdoll *part = (PartJoltPhysRagdoll*)m_parts[i];
    if (part->m_kinematicBody)
    {
      getPhysicsSceneJoltPhys()->m_joltPhysScene->GetBodyInterface().RemoveBody(part->m_kinematicBody->GetID());
    }
  }
}
//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::addToScene()
{
  NMP_ASSERT(m_refCount == 0);
  for (uint32_t i = 0; i < getNumParts(); ++i)
  {
    PartJoltPhysRagdoll *part = (PartJoltPhysRagdoll*)m_parts[i];
    if (part->m_kinematicBody)
    {
      getPhysicsSceneJoltPhys()->m_joltPhysScene->GetBodyInterface().AddBody(part->m_kinematicBody->GetID(), JPH::EActivation::Activate);
    }
  }
  addRagdollToScene();
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::addRagdollToScene()
{
  if (m_isRagdollAddedToScene)
    return;
  for (uint32_t i = 0; i < getNumParts(); ++i)
  {
    // use the fact that the position of the PhysX bone is offset so that it is at the same
    // location as the joint morpheme joint
    PhysicsRig::Part* part = m_parts[i];
    part->setTransform(part->getTransform());
    part->setVel(part->getVel());
    part->setAngVel(part->getAngVel());
  }

  m_ragdoll->AddToPhysicsSystem(JPH::EActivation::Activate);

  m_isRagdollAddedToScene = true;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::removeRagdollFromScene()
{
  if (!m_isRagdollAddedToScene)
    return;

  m_ragdoll->RemoveFromPhysicsSystem();

  m_isRagdollAddedToScene = false;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::makeDynamic()
{
  if (!m_isRagdollAddedToScene)
    addRagdollToScene();

  for (uint32_t i = 0; i < getNumParts(); ++i)
  {
    PartJoltPhysRagdoll *part = (PartJoltPhysRagdoll*)m_parts[i];
    part->makeKinematic(false, 1.0f, false);
    if (part->m_kinematicBody)
      part->enableBodyCollision(part->m_kinematicBody, false);
    part->enableBodyCollision(part->m_rigidBody, true);
  }

  for (uint32_t i=0; i<getNumJoints(); ++i)
  {
    PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll *joint = (JointJoltPhysRagdoll*)getJoint(i);
    // enable the joint limit
    joint->enableLimit(true);
  }

  // Re-enable gravity.  This will have been disabled by moveAllToKinematicPos if it was previously called.
  if (m_refCount == 0)
  {
    for (uint32_t i = 0; i < getNumParts(); ++i)
    {
      PartJoltPhysRagdoll *part = (PartJoltPhysRagdoll*)m_parts[i];
      // re-enable gravity
      part->m_rigidBody->GetMotionProperties()->SetGravityFactor(0);
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::moveAllToKinematicPos()
{
  JPH::Vec3 delta = nmVector3ToJPHVec3(m_kinematicPose.translation()) - ((PartJoltPhysRagdoll*)getPart(0))->m_rigidBody->GetWorldTransform().GetTranslation();

  // Move the kinematic shape somewhere far.
  JPH::Mat44 kinematicPose = nmMatrix34ToJPHMat44(m_kinematicPose);

  for (uint32_t i = 0; i < getNumParts(); ++i)
  {
    // move the dynamic part
    PartJoltPhysRagdoll *part = (PartJoltPhysRagdoll*)m_parts[i];
    physx::PxTransform t = part->m_rigidBody->getGlobalPose();
    t.p += delta;
    part->m_rigidBody->setGlobalPose(t);
    part->m_rigidBody->SetLinearVelocity(JPH::Vec3(0,0,0));
    part->m_rigidBody->SetAngularVelocity(JPH::Vec3(0,0,0));
    // disable gravity
    part->m_rigidBody->setActorFlag(physx::PxActorFlag::eDISABLE_GRAVITY, true);
    if (part->m_kinematicBody)
      part->m_kinematicBody->setGlobalPose(kinematicPose);
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::restoreAllJointDrivesToDefault()
{
  for (uint32_t i = 0 ;i < getNumJoints(); ++i)
  {
    PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll *joint = (JointJoltPhysRagdoll*)m_joints[i];
    joint->setStrength(0.0f);
    joint->setDamping(joint->getMaxDamping());
    joint->setExternalCompliance(1.0f);
    joint->setInternalCompliance(1.0f);
  }

  m_desiredJointProjectionIterations = 0;
  m_desiredJointProjectionLinearTolerance = FLT_MAX;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::disableSleeping()
{
  //float threshold = m_articulation->getSleepThreshold();
  //if (threshold > 0.0f)
  //  m_cachedSleepThreshold = threshold;
  //m_articulation->setSleepThreshold(0.0f);
  //m_articulation->wakeUp();
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::reenableSleeping()
{
  //if (m_ragdoll->getSleepThreshold() == 0.0f)
  //  m_articulation->setSleepThreshold(m_cachedSleepThreshold);
}

//----------------------------------------------------------------------------------------------------------------------
#if !defined(NMP_PLATFORM_SIMD) || defined(NM_HOST_IOS)
void PhysicsRigJoltPhysRagdoll::applyHardKeyframing(
  const NMP::DataBuffer &targetBuffer,
  const NMP::DataBuffer *previousTargetBuffer,
  const NMP::DataBuffer &fallbackBuffer,
  const NMP::Matrix34   &worldRoot,
  const NMP::Matrix34   *previousWorldRoot,
  bool                   enableCollision,
  float                  massMultiplier,
  bool                   enableConstraint,
  float                  dt,
  const PartChooser     &partChooser)
{
  NMP_ASSERT_MSG(m_physicsRigDef != NULL, "No RigDef exists. Check AnimationSets to have physics rigs defined");

  bool hasPrevious = previousTargetBuffer && previousWorldRoot;

  int32_t numParts =  getNumParts();
  NMP::Matrix34* targetTMs = (NMP::Matrix34*)alloca(sizeof(NMP::Matrix34) * (numParts+1));
  NMP::Matrix34* targetTMsOld = 0;
  if (hasPrevious)
  {
    targetTMsOld = (NMP::Matrix34*)alloca(sizeof(NMP::Matrix34) * (numParts+1));
    calculateWorldSpacePartTMsCacheWithVelocity(
      targetTMs,
      targetTMsOld,
      targetBuffer,
      *previousTargetBuffer,
      fallbackBuffer,
      worldRoot,
      *previousWorldRoot);
  }
  else
  {
    calculateWorldSpacePartTMsCache(targetTMs, targetBuffer, fallbackBuffer, worldRoot);
  }

  bool wholeBodyHK = true;
  for (int32_t j = 0; j < numParts; ++j)
  {
    if (!partChooser.usePart(j))
    {
      wholeBodyHK = false;
      continue;
    }

    PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll *part = (PartJoltPhysRagdoll*)m_parts[j];
    part->makeKinematic(true, massMultiplier, enableConstraint);
    part->m_isBeingKeyframed = true;

    // Disable the collision on the dynamic actor.
    part->enableActorCollision(part->m_rigidBody, false);
    // Enable collision on the kinematic actor if desired.
    if (part->m_kinematicActor)
      part->enableActorCollision(part->m_kinematicActor, enableCollision);

    // PhysX applies the velocity before the position update (I think) - so if we move to the target
    // position, we have to set the velocity to zero, otherwise the final position is actually
    // position+velocity*dt, which will be a whole frame's offset. This causes big problems with
    // split body physics, as the other parts are being constrained to a part that has zero
    // velocity, but it keeps moving! When we have the velocity data set the position to
    // targetPosition-velocity*dt, and set the velocity as well. This makes sure that the velocity
    // is passed correctly across the interface when there's split body physics.
    //
    // TODO Note that this relies on a fixed dt, since we assume that the time since the last
    // transforms is the same as the time of the upcoming step...
    if (dt > 0.0f && hasPrevious)
    {
      // Use the fact that the position of the PhysX part is offset so that it is at the same
      // location as the morpheme joint
      NMP::Vector3 offset = part->getCOMPosition();
      NMP::Matrix34 offsetTM(NMP::Matrix34::kIdentity), invOffsetTM(NMP::Matrix34::kIdentity);
      offsetTM.translation() = -offset;
      invOffsetTM.translation() = offset;

      // calculate the motion of the target
      NMP::Matrix34 invTargetTMOld(targetTMsOld[j+1]); invTargetTMOld.invertFast();
      // The following pre- and post-multiplication converts diffTM into the actual motion TM
      // centered at the COM.
      NMP::Matrix34 targetMotionTM = invOffsetTM * invTargetTMOld * targetTMs[j+1] * offsetTM;

      NMP::Vector3 targetVel = targetMotionTM.translation() / dt;
      NMP::Vector3 targetAngVel = targetMotionTM.toQuat().toRotationVector(false) / dt;

      // set the dynamic part TMs
      part->moveTo(targetTMsOld[j+1], false);

      // override the kinematic part so that it's where the dynamic parts will be
      if (part->isKinematic() && part->getKinematicActor())
        part->getKinematicActor()->setKinematicTarget(nmMatrix34ToPxTransform(targetTMs[j+1]));

      part->setVel(targetVel);
      part->setAngVel(targetAngVel);
    }
    else
    {
      part->moveTo(targetTMs[j+1], false);
      part->setVel(NMP::Vector3::InitZero);
      part->setAngVel(NMP::Vector3::InitZero);
    }

    part->recalcNextVel();
  }

  if (wholeBodyHK)
  {
    removeArticulationFromScene();
  }
  else
  {
    // Go through all the joints and disable joint limits when both parts are HK. This is undone by
    // the call to enable the joint limit in makeDynamic
    for (uint32_t i = 0 ; i < getNumJoints() ; ++i)
    {
      PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll *joint = (JointJoltPhysRagdoll*)getJoint(i);
      const PhysicsJointDef* jointDef = m_physicsRigDef->m_joints[i];
      uint32_t i1 = jointDef->m_parentPartIndex;

      if (getPart(i1)->isKinematic())
      {
        uint32_t i2 = jointDef->m_childPartIndex;

        if (getPart(i2)->isKinematic())
        {
          // disable the joint limit
          joint->enableLimit(false);
          // Decrease the compliance so that split body interactions work much better. Better than
          // setting the damping to a large value because that tends to stop the joints reaching their
          // targets.
          // Note that setting this to a smaller value than about 0.1 results in the cm rig
          // exploding...
          joint->setExternalCompliance(0.1f);
          joint->setInternalCompliance(0.1f);
        }
      }
    }
  }
}

#else
//----------------------------------------------------------------------------------------------------------------------
// NMP_PLATFORM_SIMD
void PhysicsRigJoltPhysRagdoll::applyHardKeyframing(
  const NMP::DataBuffer &targetBuffer,
  const NMP::DataBuffer *previousTargetBuffer,
  const NMP::DataBuffer &fallbackBuffer,
  const NMP::Matrix34   &worldRoot,
  const NMP::Matrix34   *previousWorldRoot,
  bool                   enableCollision,
  float                  massMultiplier,
  bool                   enableConstraint,
  float                  dt,
  const PartChooser     &partChooser)
{
  NMP_ASSERT_MSG(m_physicsRigDef != NULL, "No RigDef exists. Check AnimationSets to have physics rigs defined");

  bool hasPrevious = previousTargetBuffer && previousWorldRoot;

  int32_t numParts = getNumParts();
  NMP::vpu::Matrix* targetTMs = (NMP::vpu::Matrix*)alloca(sizeof(NMP::vpu::Matrix) * (numParts+1));
  NMP::vpu::Matrix* targetTMsOld = 0;
  if (hasPrevious)
  {
    targetTMsOld = (NMP::vpu::Matrix*)alloca(sizeof(NMP::vpu::Matrix) * (numParts+1));
    calculateWorldSpacePartTMsCacheWithVelocity(
      targetTMs,
      targetTMsOld,
      targetBuffer,
      *previousTargetBuffer,
      fallbackBuffer,
      worldRoot,
      *previousWorldRoot);
  }
  else
  {
    calculateWorldSpacePartTMsCache(targetTMs, targetBuffer, fallbackBuffer, worldRoot);
  }
 
  bool wholeBodyHK = true;
  for (int32_t j = 0; j < numParts; ++j)
  {   
    if (!partChooser.usePart(j))
    {
      wholeBodyHK = false;
      continue;
    }

    PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll *part = (PartJoltPhysRagdoll*)m_parts[j];
    part->makeKinematic(true, massMultiplier, enableConstraint);
    part->m_isBeingKeyframed = true;

    // Disable the collision on the dynamic actor.
    part->enableBodyCollision(part->m_rigidBody, false);
    // Enable collision on the kinematic actor if desired.
    if (part->m_kinematicBody)
      part->enableBodyCollision(part->m_kinematicBody, enableCollision);

    // PhysX applies the velocity before the position update (I think) - so if we move to the target
    // position, we have to set the velocity to zero, otherwise the final position is actually
    // position+velocity*dt, which will be a whole frame's offset. This causes big problems with
    // split body physics, as the other parts are being constrained to a part that has zero
    // velocity, but it keeps moving! When we have the velocity data set the position to
    // targetPosition-velocity*dt, and set the velocity as well. This makes sure that the velocity
    // is passed correctly across the interface when there's split body physics.
    //
    // TODO Note that this relies on a fixed dt, since we assume that the time since the last
    // transforms is the same as the time of the upcoming step...
    if (dt > 0.0f && hasPrevious)
    {
      // Use the fact that the position of the PhysX part is offset so that it is at the same
      // location as the morpheme joint 
      NMP::Matrix34 currentTM = part->getTransform();
      NMP::Vector3 offset = part->getCOMPosition();
      NMP::Matrix34 offsetTM(NMP::Matrix34::kIdentity), invOffsetTM(NMP::Matrix34::kIdentity);
      offsetTM.translation() = -offset;
      invOffsetTM.translation() = offset;

      // calculate the motion of the target
      NMP::Matrix34 invTargetTMOld(M34vpu(targetTMsOld[j+1])); invTargetTMOld.invertFast();
      // The following pre- and post-multiplication converts diffTM into the actual motion TM
      // centered at the COM.
      NMP::Matrix34 targetMotionTM = invOffsetTM * invTargetTMOld * M34vpu(targetTMs[j+1]) * offsetTM;

      NMP::Vector3 targetVel = targetMotionTM.translation() / dt;
      NMP::Vector3 targetAngVel = targetMotionTM.toQuat().toRotationVector(false) / dt;

      // set the dynamic part TMs
      part->moveTo(M34vpu(targetTMsOld[j+1]), false);
      part->setVel(targetVel);
      part->setAngVel(targetAngVel);

      // override the kinematic part so that it's where the dynamic parts will be
      if (part->isKinematic() && part->getKinematicActor())
        part->getKinematicActor()->setKinematicTarget(nmMatrix34ToPxTransform(M34vpu(targetTMs[j+1])));

    }
    else
    {
      part->moveTo(M34vpu(targetTMs[j+1]), false);
      part->setVel(NMP::Vector3::InitZero);
      part->setAngVel(NMP::Vector3::InitZero);
    }

    part->recalcNextVel();
  }

  if (wholeBodyHK)
  {
    removeRagdollFromScene();
  }
  else
  {
    // Go through all the joints and disable joint limits when both parts are HK. This is undone by
    // the call to enable the joint limit in makeDynamic
    for (uint32_t i = 0 ; i < getNumJoints() ; ++i)
    {
      PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll *joint = (JointJoltPhysRagdoll*)getJoint(i);
      const PhysicsJointDef* jointDef = m_physicsRigDef->m_joints[i];

      uint32_t i1 = jointDef->m_parentPartIndex;

      if (getPart(i1)->isKinematic())
      {
        uint32_t i2 = jointDef->m_childPartIndex;

        if (getPart(i2)->isKinematic())
        {
          // disable the joint limit
          joint->enableLimit(false);
          // Decrease the compliance so that split body interactions work much better. Better than
          // setting the damping to a large value because that tends to stop the joints reaching their
          // targets.
          // Note that setting this to a smaller value than about 0.1 results in the cm rig
          // exploding...
          joint->setExternalCompliance(0.1f);
          joint->setInternalCompliance(0.1f);
        }
      }
    }
  }
}
#endif

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::applySoftKeyframing(
    const NMP::DataBuffer &targetBuffer,
    const NMP::DataBuffer &targetBufferOld,
    const NMP::DataBuffer &fallbackBuffer,
    const NMP::Matrix34   &worldRoot,
    const NMP::Matrix34   &worldRootOld,
    bool                   enableCollision,
    bool                   enableJointLimits,
    bool                   preserveMomentum,
    float                  externalJointCompliance,
    float                  gravityCompensationFrac,
    float                  dt,
    float                  weight,
    float                  maxAccel,
    float                  maxAngAccel,
    const PartChooser     &partChooser)
{
  NMP_ASSERT_MSG(m_physicsRigDef != NULL, "No RigDef exists. Check AnimationSets to have physics rigs defined");

  // Do a first pass to set the properties that need to be set even if the weight is zero
  int32_t numParts = getNumParts();
  for (int32_t i = 0; i < numParts; ++i)
  {
    if (!partChooser.usePart(i))
    {
      continue;
    }

    PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll *part = (PartJoltPhysRagdoll*)m_parts[i];

    part->makeKinematic(false, 1.0f, false);
    part->m_isBeingKeyframed = true;
    // Enable the collision on the dynamic actor if desired.
    part->enableBodyCollision(part->m_rigidBody, enableCollision);
    // Disable collision on the kinematic actor.
    if (part->m_kinematicBody)
      part->enableBodyCollision(part->m_kinematicBody, false);

    // Set the external compliance on the associated joint (joint index = part index - 1, so there's
    // no joint for the root part).
    if (i != 0)
    {
      PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll *joint = (JointJoltPhysRagdoll*)getJoint(i-1);
      joint->setExternalCompliance(externalJointCompliance);
    }

    // Enable/disable joint limits on the parent joint, but only if the parent part is also soft
    // keyframed by this node.
    int32_t parentPartIndex = part->getParentPartIndex();
    if (parentPartIndex >= 0)
    {
      if (partChooser.usePart(parentPartIndex))
      {
        int32_t parentJointIndex = i - 1;
        if (parentJointIndex >= 0)
        {
          PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll* joint = (JointJoltPhysRagdoll*)getJoint(parentJointIndex);
          joint->enableLimit(enableJointLimits);
        }
      }
    }
  }

#ifdef DEBUG_SK
  static NMP::Vector3 prevVelTargets[64];
  static NMP::Vector3 prevAngVelTargets[64];
#endif

#if !defined(NMP_PLATFORM_SIMD) || defined(NM_HOST_IOS)
  NMP::Matrix34* targetTMs = (NMP::Matrix34*)alloca(sizeof(NMP::Matrix34) * (numParts+1));
  NMP::Matrix34* targetTMsOld = (NMP::Matrix34*)alloca(sizeof(NMP::Matrix34) * (numParts+1));
#else
  NMP::vpu::Matrix* targetTMs = (NMP::vpu::Matrix*)alloca(sizeof(NMP::vpu::Matrix) * (numParts+1));
  NMP::vpu::Matrix* targetTMsOld = (NMP::vpu::Matrix*)alloca(sizeof(NMP::vpu::Matrix) * (numParts+1));
#endif

  calculateWorldSpacePartTMsCacheWithVelocity( targetTMs,
    targetTMsOld,
    targetBuffer,
    targetBufferOld,
    fallbackBuffer,
    worldRoot,
                                               worldRootOld );

  // If the weight is effectively zero then we are only concerned with deviation
  bool weightIsEffectivelyZero = (weight <= 0.0000001f);
 
  // Adjust these for weight
  maxAccel *= weight;
  maxAngAccel *= weight;

  // Calculate this once for use in the loop
  NMP::Vector3 gravityDeltaVel = getPhysicsSceneJoltPhys()->getGravity() * (weight * dt * gravityCompensationFrac);

  // The variables required if we are preserving momentum.
  NMP::Vector3 originalCOMVel(NMP::Vector3::InitZero);
  NMP::Vector3 newCOMVel(NMP::Vector3::InitZero);
  float totalPreservedMass = 0.0f;

  // Iterate over the parts calculating deviation, velocity and angular velocity for each one.
  for (int32_t i = 0; i < numParts; ++i)
  {
    if (!partChooser.usePart(i))
    {
      continue;
    }

    PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll *part = (PartJoltPhysRagdoll*)m_parts[i];

    if (preserveMomentum)
    {
      originalCOMVel += part->getVel() * part->getMass();
      totalPreservedMass += part->getMass();
    }

    // Use the fact that the position of the PhysX bone is offset so that it is at the same
    // location as the morpheme joint 
    if (dt > 0.0f)
    {
      NMP::Matrix34 currentTM = part->getTransform();

      NMP::Vector3 offset = part->getCOMPosition();
      NMP::Matrix34 offsetTM(NMP::Matrix34::kIdentity), invOffsetTM(NMP::Matrix34::kIdentity);
      offsetTM.translation() = -offset;
      invOffsetTM.translation() = offset;

      // Calculate the motion to go from current to new
      NMP::Matrix34 invCurrentTM(currentTM); invCurrentTM.invertFast();
      // The following pre- and post-multiplication converts diffTM into the actual motion TM
      // centered at the COM. 
#if !defined(NMP_PLATFORM_SIMD) || defined(NM_HOST_IOS)
      NMP::Matrix34 motionTM = invOffsetTM * invCurrentTM * targetTMs[i+1] * offsetTM;
#else
      NMP::Matrix34 motionTM = invOffsetTM * invCurrentTM * M34vpu(targetTMs[i+1]) * offsetTM;
#endif

      // Store the distance/angle error
      part->m_SKDeviation = motionTM.translation().magnitude();
      part->m_SKDeviationAngle = motionTM.toRotationVector().magnitude();

      // If the weight is effectively zero then the velocity and angular velocity
      // for each part will be unchanged. Only calculate deviation here.
      if (weightIsEffectivelyZero)
      {
        continue;
      }

      // Calculate the motion of the target itself
#if !defined(NMP_PLATFORM_SIMD) || defined(NM_HOST_IOS)
      NMP::Matrix34 invTargetTMOld(targetTMsOld[i+1]); invTargetTMOld.invertFast();
      NMP::Matrix34 targetMotionTM = invOffsetTM * invTargetTMOld * targetTMs[i+1] * offsetTM;
#else
      NMP::Matrix34 invTargetTMOld(M34vpu(targetTMsOld[i+1])); invTargetTMOld.invertFast();
      NMP::Matrix34 targetMotionTM = invOffsetTM * invTargetTMOld * M34vpu(targetTMs[i+1]) * offsetTM;
#endif

      // This is where the velocity multiplier can be applied
      NMP::Vector3 translation = motionTM.translation();
      NMP::Vector3 rotation = motionTM.toQuat().toRotationVector(false);

      NMP::Vector3 newVel = translation / dt;
      NMP::Vector3 curVel = part->getVel();
      if (maxAccel >= 0.0f)
      {
        // Prevent overshoot by calculating the max speed we can have in the direction towards
        // the target given that we cannot decelerate faster than maxAccel
        NMP::Vector3 deltaVel = newVel - curVel;
        NMP::Vector3 targetVel = targetMotionTM.translation() / dt;
        NMP::Vector3 translationDir = translation;
        float distToTarget = translationDir.normaliseGetLength();
        float curVelAlongTranslation = curVel.dot(translationDir);
        float targetVelAlongTranslation = targetVel.dot(translationDir);

        if (curVelAlongTranslation > targetVelAlongTranslation)
        {
          float timeToCatchUp = distToTarget / (curVelAlongTranslation - targetVelAlongTranslation);
          float maxCurVelAlongTranslation = targetVelAlongTranslation + timeToCatchUp * maxAccel;
          if (curVelAlongTranslation > maxCurVelAlongTranslation)
          {
            // Replace the old component along the translation with the new max value
            newVel += translationDir * (maxCurVelAlongTranslation - newVel.dot(translationDir));
            deltaVel = newVel - curVel;
          }
        }

        // Clamp the acceleration
        float deltaVelMag = deltaVel.magnitude();
        if (deltaVelMag > maxAccel * dt)
          deltaVel *= maxAccel * dt / deltaVelMag;

        // Apply gravity compensation
        deltaVel -= gravityDeltaVel;
        newVel = curVel + deltaVel;
      }
      part->setVel(newVel);

      if (preserveMomentum)
        newCOMVel += newVel * part->getMass();

      NMP::Vector3 newAngVel = rotation / dt;
      NMP::Vector3 curAngVel = part->getAngVel();
      if (maxAngAccel >= 0.0f)
      {
        // Limit the max angular velocity target - this is just a straight conversion of the linear velocity code, so I
        // _think_ it's "correct"!
        NMP::Vector3 deltaAngVel = newAngVel - curAngVel;
        NMP::Vector3 targetAngVel = targetMotionTM.toQuat().toRotationVector(false) / dt;
        NMP::Vector3 rotationDir = rotation;
        float distToTarget = rotationDir.normaliseGetLength();
        float curAngVelAlongRotation = curAngVel.dot(rotationDir);
        float targetAngVelAlongRotation = targetAngVel.dot(rotationDir);

        if (curAngVelAlongRotation > targetAngVelAlongRotation)
        {
          float timeToCatchUp = distToTarget / (curAngVelAlongRotation - targetAngVelAlongRotation);
          float maxCurAngVelAlongRotation = targetAngVelAlongRotation + timeToCatchUp * maxAngAccel;

          if (curAngVelAlongRotation > maxCurAngVelAlongRotation)
          {
            // Replace the old component along the translation with the new max value
            newAngVel += rotationDir * (maxCurAngVelAlongRotation - newAngVel.dot(rotationDir));
            deltaAngVel = newAngVel - curAngVel;
          }
        }

        // Clamp the acceleration
        float deltaAngVelMag = deltaAngVel.magnitude();
        if (deltaAngVelMag > maxAngAccel * dt)
          deltaAngVel *= maxAngAccel * dt / deltaAngVelMag;
        newAngVel = curAngVel + deltaAngVel;
      }
      part->setAngVel(newAngVel);

#ifdef DEBUG_SK
      if (i == 0)
      {
        printf("VelTarget = (%6.2f %6.2f %6.2f) CurVel = (%6.2f %6.2f %6.2f) next VelTarget = (%6.2f %6.2f %6.2f)     dt = %6.4f\n",
          prevVelTargets[i].x, prevVelTargets[i].y, prevVelTargets[i].z, 
          curVel.x, curVel.y, curVel.z,
          newVel.x, newVel.y, newVel.z,
          dt);
    }
      prevVelTargets[i] = newVel;
      prevAngVelTargets[i] = newAngVel;
#endif

  }
  }

  if (preserveMomentum && weightIsEffectivelyZero == false)
  {
    originalCOMVel /= totalPreservedMass;
    newCOMVel /= totalPreservedMass;
    NMP::Vector3 correctionVel = originalCOMVel - newCOMVel;
    for (uint32_t i = 0; i < getNumParts(); ++i)
    {
      if (!partChooser.usePart(i))
        continue;
      PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll *part = (PartJoltPhysRagdoll*)m_parts[i];
      NMP::Vector3 partVel = part->getVel();
      part->setVel(partVel + correctionVel);
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------- 
void PhysicsRigJoltPhysRagdoll::applyActiveAnimation(uint32_t jointIndex, const NMP::Quat &targetQuat, bool makeChildDynamic) 
{ 
  NMP_ASSERT(jointIndex < getNumJoints());  
  JPH::SixDOFConstraint *joint = ((JointJoltPhysRagdoll*)m_joints[jointIndex])->m_jointInternal;  
  const PhysicsJointDef* jointDef = m_physicsRigDef->m_joints[jointIndex];
  if (makeChildDynamic)  
  {   
    PartJoltPhysRagdoll *childPart = (PartJoltPhysRagdoll*)m_parts[jointDef->m_childPartIndex];     
    childPart->makeKinematic(false, 1.0f, false);
    childPart->m_isBeingKeyframed = false;
  }   
  // Don't force either of the parts to have collision - no way we could know which one _should_ have collision if it's
  // disabled elsewhere.
  joint->SetTargetOrientationCS(nmQuatToJPHQuat(targetQuat));
  return; 
}

//----------------------------------------------------------------------------------------------------------------------
// Drives the joints to the targets given by the input animation buffer.
void PhysicsRigJoltPhysRagdoll::applyActiveAnimation(
    const NMP::DataBuffer& targetBuffer,
    const NMP::DataBuffer& fallbackBuffer,
    float                  strengthMultiplier,
    float                  dampingMultiplier,
    float                  internalCompliance,
    float                  externalCompliance,
    bool                   enableJointLimits,
    const JointChooser    &jointChooser,
    float                  limitClampFraction)
{
  NMP_ASSERT_MSG(m_physicsRigDef != NULL, "No RigDef exists. Check AnimationSets to have physics rigs defined");

  for (uint32_t i = 0; i < getNumJoints(); ++i)
  {
    if (!jointChooser.useJoint(i))
      continue;

    PhysicsRigJoltPhysRagdoll::JointJoltPhysRagdoll *joint = (JointJoltPhysRagdoll*)m_joints[i];
    const PhysicsSixDOFJointDef* jointDef = static_cast<const PhysicsSixDOFJointDef*>(m_physicsRigDef->m_joints[i]);
    PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll *childPart = (PartJoltPhysRagdoll*)m_parts[jointDef->m_childPartIndex];
    childPart->makeKinematic(false, 1.0f, false);
    childPart->m_isBeingKeyframed = false;

    // Don't force either of the bones to have collision - no way we could know which one _should_
    // have collision if it's disabled elsewhere.

    float newStrength = joint->getMaxStrength() * strengthMultiplier;
    float newDamping  = joint->getMaxDamping() * dampingMultiplier;
    joint->setStrength(newStrength);
    joint->setDamping(newDamping);
    joint->setInternalCompliance(internalCompliance);
    joint->setExternalCompliance(externalCompliance);
    joint->enableLimit(enableJointLimits);

    if (strengthMultiplier < 0.0000001f)
      continue;

    NMP::Quat curQ;
    getQuatFromTransformBuffer(jointDef->m_childPartIndex, targetBuffer, fallbackBuffer, curQ);

    // q is the rotation of the child relative to the parent (in parent space).
    // We need to account for the offset axes in the joint.

    // Get the local joint axes in each frame as l0, l1
    NMP::Quat l0 = jointDef->m_parentFrameQuat;
    NMP::Quat l1 = jointDef->m_childFrameQuat;

    // Now "assuming" the parent is at the origin (since we already have the relative rotation q)
    // we want to calculate rot, the relative rotation of the child local frame from the parent local frame
    NMP::Quat l0Inv = ~l0;

    // Target orientations outside the limits cause oscillations when physical limits are enabled
    if (limitClampFraction >= 0.0f)
    {
      joint->clampToLimits(curQ, limitClampFraction, NULL);
    }
    NMP::Quat curFrameQ = l0Inv * curQ * l1;
    joint->setTargetOrientation(curFrameQ);
  }
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Quat PhysicsRigJoltPhysRagdoll::getJointQuat(uint32_t jointIndex) 
{
  NMP_ASSERT(jointIndex < getNumJoints());
  JointJoltPhysRagdoll *joint = (JointJoltPhysRagdoll*)m_joints[jointIndex];
  const PhysicsJointDef* jointDef = m_physicsRigDef->m_joints[jointIndex];
  if (joint->m_rotationDirty)
  {
    NMP::Quat part1Quat = (getPartJoltPhysRagdoll(jointDef->m_parentPartIndex))->getQuaternion();
    NMP::Quat part2Quat = (getPartJoltPhysRagdoll(jointDef->m_childPartIndex))->getQuaternion();
    joint->m_actualRotation = ~(part1Quat * jointDef->m_parentFrameQuat) * (part2Quat * jointDef->m_childFrameQuat); 
    joint->m_rotationDirty = false;
  }
  return joint->m_actualRotation;
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::setCollisionGroupsToActivate(const int *collisionGroupIndices, int numCollisionGroupIndices)
{
  NMP_ASSERT(numCollisionGroupIndices <= m_maxCollisionGroupIndices);
  m_numCollisionGroupIndices = NMP::minimum(m_maxCollisionGroupIndices, numCollisionGroupIndices);
  
  for (int i = 0 ; i < m_numCollisionGroupIndices ; ++i)
  {
    m_collisionGroupIndicesToActivate[i] = collisionGroupIndices[i];
  }
}

//----------------------------------------------------------------------------------------------------------------------
void PhysicsRigJoltPhysRagdoll::setExplosionHandler(RagdollExplosionHandler* handler)
{
  s_explosionHandler = handler;
}

} // namespace MR

//----------------------------------------------------------------------------------------------------------------------
