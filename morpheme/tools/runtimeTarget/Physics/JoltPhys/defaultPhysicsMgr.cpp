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
#include "defaultPhysicsMgr.h"
#include "NMPlatform/NMProfiler.h"
#include "NMPlatform/NMPlatform.h"
#include "NMPlatform/NMSocket.h"

#include "comms/mcomms.h"
#include "comms/packet.h"

#include "../../iPhysicsMgr.h"
#include "../../iControllerMgr.h"

#include "morpheme/mrNetwork.h"

#include "physics/JoltPhys/mrJoltPhys.h"
#include "physics/mrPhysicsRigDef.h"
#include "physics/JoltPhys/mrPhysicsRigJoltPhysRagdoll.h"
#include "physics/JoltPhys/mrPhysicsRigJoltPhysJointed.h"
#include "physics/JoltPhys/mrPhysicsSceneJoltPhys.h"
#include "morpheme/mrCoreTaskIDs.h"
#include "physics/mrPhysicsRig.h"
#include "physics/mrPhysicsRigDef.h"
#include "../../defaultSceneObjectMgr.h"
#include "../../defaultAssetMgr.h"
#include "../../networkInstanceRecordManager.h"
#include "../../runtimeTargetContext.h"
#include "../../sceneObjectRecordManager.h"
#include "defaultControllerMgr.h"
#include "NMPlatform/NMvpu.h"
#include "physics/Nodes/mrPhysicsNodes.h"
#include "morpheme/mrDefines.h"
#include "morpheme/mrManager.h"
#include "physics/mrPhysics.h"

#include "memoryStream.h"

#include "morpheme/mrDispatcher.h"
#ifdef NM_HOST_CELL_PPU
  #include "morpheme/mrDispatcherPS3.h"
#endif // NM_HOST_CELL_PPU

#if defined(NM_HOST_CELL_PPU)
  #include "NMPlatform/ps3/NMSPUManager.h"
#endif // defined(NM_HOST_CELL_PPU)

#include "../../runtimeTargetLogger.h"

//----------------------------------------------------------------------------------------------------------------------

class JoltPhysAllocator
{
#ifdef WIN32
  // on win32 we only have 8-byte alignment guaranteed, but the CRT provides special aligned
  // allocation fns
  static void* allocate(size_t size, const char*, const char*, int)
  {
    return _aligned_malloc(size, 16);
  }
  static void deallocate(void* ptr)
  {
    _aligned_free(ptr);
  }
#elif defined(NM_HOST_ANDROID)
  static void* allocate(size_t size, const char*, const char*, int)
  {
    void *ptr = memalign(16, size);
    PX_ASSERT((reinterpret_cast<size_t>(ptr) & 15)==0);
    return ptr;
  }

  static void deallocate(void* ptr)
  {
    free(ptr);
  }
#else
  // on PS3, XBox and Win64 we get 16-byte alignment by default
  static void* allocate(size_t size, const char*, const char*, int)
  {
    void *ptr = ::malloc(size);
    PX_ASSERT((reinterpret_cast<size_t>(ptr) & 15)==0);
    return ptr;
  }
  static void deallocate(void* ptr)
  {
    ::free(ptr);
  }
#endif
};

class JoltPhysMaterial : public JPH::PhysicsMaterialSimple
{
public:
    //etc etc
};


//----------------------------------------------------------------------------------------------------------------------
#define ANIM_GRAVITY
//----------------------------------------------------------------------------------------------------------------------
/// \class PhysicsSDK
/// \brief A singleton class to keep track of the instance of the physics sdk object
/// \ingroup RuntimeTarget
//----------------------------------------------------------------------------------------------------------------------
class PhysicsSDK
{
public:
  static bool init(float physicsToleranceScale);
  static bool term();

protected:

  PhysicsSDK() {}
  ~PhysicsSDK() {}
  PhysicsSDK& operator=(PhysicsSDK& liveLink);
  PhysicsSDK(const PhysicsSDK& liveLink);

  enum
  {
    kMaxFilenameLength = 260
  };

  static bool                           sm_created;

public:

  static JPH::TempAllocatorImpl*        sm_joltTempAllocator;
  static JPH::JobSystemThreadPool*      sm_joltJobSystem;
  static JPH::PhysicsMaterialSimple*    sm_defaultJoltPhysMaterial;
};

//----------------------------------------------------------------------------------------------------------------------
// Statics
bool                        PhysicsSDK::sm_created = false;
JPH::TempAllocatorImpl*     PhysicsSDK::sm_joltTempAllocator = nullptr;
JPH::JobSystemThreadPool*   PhysicsSDK::sm_joltJobSystem = nullptr;
JPH::PhysicsMaterialSimple* PhysicsSDK::sm_defaultJoltPhysMaterial;

//----------------------------------------------------------------------------------------------------------------------
/// \class PhysicsUserData
/// \brief Physics Engine user-data object for connecting a physics actor to the scene object that represents it
/// \ingroup RuntimeTarget
//----------------------------------------------------------------------------------------------------------------------
class PhysicsUserData
{
public:
  PhysicsUserData(MCOMMS::SceneObjectID objID)
  {
    objectID = objID;
    dummyBody = 0;
    constraint = 0;
    magicString[0] = 'N';
    magicString[1] = 'M';
    magicString[2] = 'S';
    magicString[3] = 'O';
  }

  char magicString[4]; // initialize ad NMSO (NM-scene object)

  bool isValid()
  {
    return magicString[0] == 'N' && magicString[1] == 'M' && magicString[2] == 'S' && magicString[3] == 'O';
  }

  MCOMMS::SceneObjectID objectID;
  JPH::Body* dummyBody;
  JPH::Constraint* constraint;
};

//----------------------------------------------------------------------------------------------------------------------
// Physics SDK singleton class
//----------------------------------------------------------------------------------------------------------------------
bool PhysicsSDK::init(float physicsToleranceScale)
{
  term();

  // Register allocation hook. In this example we'll just let Jolt use malloc / free but you can override these if you want (see Memory.h).
  // This needs to be done before any other Jolt function is called.
  JPH::RegisterDefaultAllocator();

  sm_joltTempAllocator = new JPH::TempAllocatorImpl(10 * 1024 * 1024);
  sm_joltJobSystem = new JPH::JobSystemThreadPool(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers,
      JPH::thread::hardware_concurrency() - 1);

  // Create a factory, this class is responsible for creating instances of classes based on their name or hash and is mainly used for deserialization of saved data.
  // It is not directly used in this example but still required.
  JPH::Factory::sInstance = new JPH::Factory();

  // Register all physics types with the factory and install their collision handlers with the CollisionDispatch class.
  // If you have your own custom shape types you probably need to register their handlers with the CollisionDispatch before calling this function.
  // If you implement your own default material (PhysicsMaterial::sDefault) make sure to initialize it before this function or else this function will create one for you.
  JPH::RegisterTypes();

  // Create a default material
  sm_defaultJoltPhysMaterial = new JPH::PhysicsMaterialSimple();
  //sm_defaultJoltPhysMaterial->setFrictionCombineMode(physx::PxCombineMode::eMULTIPLY);
  //sm_defaultJoltPhysMaterial->setRestitutionCombineMode(physx::PxCombineMode::eMULTIPLY);
  NMP_ASSERT(sm_defaultJoltPhysMaterial);

  sm_created = true;
  return true;
}

//----------------------------------------------------------------------------------------------------------------------
bool PhysicsSDK::term()
{
  if (!sm_created)
  {
    return false;
  }

  if (sm_defaultJoltPhysMaterial)
  {
    sm_defaultJoltPhysMaterial->Release();
    sm_defaultJoltPhysMaterial = 0;
  }

  // Unregisters all types with the factory and cleans up the default material
  JPH::UnregisterTypes();

  delete sm_joltTempAllocator;
  delete sm_joltJobSystem;

  // Destroy the factory
  delete JPH::Factory::sInstance;
  JPH::Factory::sInstance = nullptr;

  sm_created = false;
  return true;
}

//----------------------------------------------------------------------------------------------------------------------
// Default physics manager
//----------------------------------------------------------------------------------------------------------------------

namespace
{

void incPhysicsAssetRefCounts(MR::NetworkDef* networkDef);
void decPhysicsAssetRefCounts(MR::NetworkDef* networkDef);
void deleteOrphanedPhysicsAsset(MR::NetworkDef* networkDef);

#ifdef NM_PROFILING
  float g_totalTiming = 0.0f;
  float g_maxTime = 0.0f;
  uint32_t g_totalSamples = 0;
#endif

} // anonymous namespace

//----------------------------------------------------------------------------------------------------------------------
DefaultPhysicsMgr::DefaultPhysicsMgr(
  RuntimeTargetContext* context,
  DefaultAssetMgr* assetMgr,
  const NMP::CommandLineProcessor& commandLineArguments) :
  IPhysicsMgr(),
  m_assetMgr(assetMgr),
  m_context(context),
  m_physicsRigType(MR::PhysicsRigJoltPhys::TYPE_ARTICULATED),
  m_frameIndex(0),
  m_physicsScene(NULL),
  m_characterControllerManager(NULL),
  m_nextPhysicsObjectID(0),
  m_toleranceScalingValue(1.0f),
  m_physicsAndCharacterControllerUpdate(MR::PHYSICS_AND_CC_UPDATE_SEPARATE)
{
  m_assetMgr->setPhysicsManager(this);

  const char* physicsRigType = NULL;
  commandLineArguments.getOptionValue("-physicsRigType", &physicsRigType);

  if (physicsRigType)
  {
    if (strcmp(physicsRigType, "TYPE_ARTICULATED") == 0)
    {
      m_physicsRigType = MR::PhysicsRigJoltPhys::TYPE_ARTICULATED;
      NMP_MSG("Using physics rig type TYPE_ARTICULATED\n");
    }
    else if (strcmp(physicsRigType, "TYPE_JOINTED") == 0)
    {
      m_physicsRigType = MR::PhysicsRigJoltPhys::TYPE_JOINTED;
      NMP_MSG("Using physics rig type TYPE_JOINTED\n");
    }
  }

  resetSDKS();

  m_assetMgr->registerRefCountIncCallback(&incPhysicsAssetRefCounts);
  m_assetMgr->registerRefCountDecCallback(&decPhysicsAssetRefCounts);
  m_assetMgr->registerDeleteOrphanedAssetCallback(&deleteOrphanedPhysicsAsset);
  
}

//----------------------------------------------------------------------------------------------------------------------
DefaultPhysicsMgr::~DefaultPhysicsMgr()
{
  m_assetMgr->setPhysicsManager(NULL);

  deleteScene();
}

//----------------------------------------------------------------------------------------------------------------------
void DefaultPhysicsMgr::getPhysicsEngineID(char* buffer, uint32_t bufferLength) const
{
  //NMP_STRNCPY_S(buffer, bufferLength, "JoltPhys");
  NMP_STRNCPY_S(buffer, bufferLength, "PhysX3"); // we are gonna larp this information out of safety
}

//----------------------------------------------------------------------------------------------------------------------
void DefaultPhysicsMgr::updateEnvironment()
{
  NMP::Vector3 worldUpVector(0.0f, 1.0f, 0.0f);

  // Get the gravity settings from the environment.
  MCOMMS::EnvironmentManagementInterface* environmentManager = MCOMMS::getRuntimeTarget()->getEnvironmentManager();
  NMP_ASSERT(environmentManager);
  bool gravityEnabled = false;
  NMP::Vector3 gravityVector(0.0f, 0.0f, 0.0f);

  // Configure gravity and up axis
  {
    MCOMMS::Attribute* gravityAttr = environmentManager->getEnvironmentAttribute(MCOMMS::Attribute::SEMANTIC_GRAVITY_VECTOR);
    NMP_ASSERT(gravityAttr); // Must exist for us to initialise the gravity correctly.
    gravityVector = *((NMP::Vector3*)gravityAttr->getData());

    // For now set the up vector to be the inverse of the gravity direction. However, this is
    // not always correct, and also check that everything works when gravity = 0.  This line below is
    // OK, but there may be inconsistencies elsewhere.  See MORPH-11297
    worldUpVector = gravityVector * -1.0f;
    worldUpVector.normalise(NMP::Vector3::InitTypeOneY);
  }

  // Get whether gravity is enabled
  {
    MCOMMS::Attribute* gravityAttr = environmentManager->getEnvironmentAttribute(MCOMMS::Attribute::SEMANTIC_GRAVITY_ENABLED);
    NMP_ASSERT(gravityAttr); // Must exist for us to initialise the gravity correctly.

    gravityEnabled = (*(int*)gravityAttr->getData() != 0);
    if (!gravityEnabled)
    {
      // gravity is not enabled, so zero the vector.
      gravityVector.setToZero();
      // Note that the up axis must remain configured!
    }
  }

  NMP_ASSERT(m_physicsScene);
  m_physicsScene->setGravity(gravityVector);
  m_physicsScene->setWorldUpDirection(worldUpVector);
}

//----------------------------------------------------------------------------------------------------------------------
float DefaultPhysicsMgr::getActualTimestep(float deltaTime) const
{
  return deltaTime;
}

//----------------------------------------------------------------------------------------------------------------------
void DefaultPhysicsMgr::simulate(float deltaTime)
{
  if (deltaTime < 0.000001f)
    return;

  if (deltaTime > m_maxTimeStep)
    deltaTime = m_maxTimeStep;

  float maxSeparationSpeed = 0.5f*m_toleranceScalingValue;

  m_physicsScene->m_joltPhysScene->Update(deltaTime, 1, PhysicsSDK::sm_joltTempAllocator, PhysicsSDK::sm_joltJobSystem);

  m_physicsScene->setLastPhysicsTimeStep(deltaTime);
}

//----------------------------------------------------------------------------------------------------------------------
// Create a PhysX actor of specified geometry type (box, capsule, sphere)
//----------------------------------------------------------------------------------------------------------------------
JPH::Body* DefaultPhysicsMgr::createBody(
  JPH::Shape* shape,
  bool dynamic,
  const NMP::Vector3& pos,
  const NMP::Quat& rot,
  float density,
  bool hasCollision,
  JPH::PhysicsMaterial* material,
  float staticFriction,
  float dynamicFriction,
  float skinWidth,
  float restitution,
  float sleepThreshold,
  float maxAngularVelocity,
  float linearDamping,
  float angularDamping,
  NMP::Matrix34* NMP_UNUSED(localPose))
{
  JPH::Mat44 globalPose = JPH::Mat44::sIdentity();
  globalPose = globalPose.sRotation(MR::nmQuatToJPHQuat(rot));
  globalPose.SetTranslation(MR::nmVector3ToJPHVec3(pos));

  //-----------------------
  // create the body
  JPH::BodyCreationSettings bodysettings(shape,
      globalPose.GetTranslation(), globalPose.GetQuaternion(),
      dynamic ? JPH::EMotionType::Dynamic : JPH::EMotionType::Static,
      dynamic ? MR::NMPhysLayers::COLLIDABLE_PUSHABLE : MR::NMPhysLayers::COLLIDABLE_NON_PUSHABLE);
  if (dynamic)
  {
    bodysettings.mLinearDamping = linearDamping;
    bodysettings.mAngularDamping = angularDamping;
    bodysettings.mMaxAngularVelocity = maxAngularVelocity;
    bodysettings.mAllowSleeping = true;
    //set sleep threshold how ? hm
  }

  JPH::BodyInterface& bodyinterface = m_physicsScene->m_joltPhysScene->GetBodyInterface();

  JPH::Body* body = bodyinterface.CreateBody(bodysettings);

  NMP_ASSERT(body);

  //-----------------------
  // adjust local pose for capsules so they are oriented correctly
  // :is this needed for jolt physics ? we`ll see..
  //physx::PxTransform shapeLocalPose = physx::PxTransform(physx::PxIdentity);
  //if (geomType == physx::PxGeometryType::eCAPSULE)
  //{
  //  shapeLocalPose.q = physx::PxQuat(NM_PI_OVER_TWO, physx::PxVec3(0.0f, 1.0f, 0.0f));
  //}

  //-----------------------
  // create a material
  //if(!material)
  //{
  //  material = new JoltPhysMaterial(staticFriction, dynamicFriction, restitution);
  //  material->setFrictionCombineMode(physx::PxCombineMode::eMULTIPLY);
  //  material->setRestitutionCombineMode(physx::PxCombineMode::eMULTIPLY);
  //  NMP_ASSERT(material);
  //  m_materials.push_back(material);
  //}

  //-----------------------
  //shape->setLocalPose(shapeLocalPose); figure this out in jolt physics, if necessary

  //-----------------------
  // For simplicity, make the contact offset of statics effectively zero, and the offset of dynamics > 0
  //shape->setContactOffset(dynamic ? skinWidth : minContactOffset);
  //shape->setRestOffset(0.0f);

  //physx::PxU32 filterWord0 = (dynamic ? 1 << MR::GROUP_COLLIDABLE_PUSHABLE : 1 << MR::GROUP_COLLIDABLE_NON_PUSHABLE);

  //-----------------------
  // set up filter data
  //shape->setQueryFilterData(physx::PxFilterData(filterWord0, 0, 0, 0));
  //if (!hasCollision)
  //{
  //  shape->setSimulationFilterData(physx::PxFilterData(0, 0xffffffff, 0, 0));
  //}
  //else
  //{
  //  shape->setSimulationFilterData(physx::PxFilterData(filterWord0, 0, 0, 0));
  //}

  //if (dynamic)
  //{
  //  NMP_ASSERT(density >= 0.0f);
  //  physx::PxRigidBodyExt::updateMassAndInertia(*static_cast<physx::PxRigidDynamic*>(actor), density);
  //}

  // Add per shape data - needed for the object to be recognised by Euphoria whether it's static or
  // dynamic.
  //physx::PxShape *tempShapes[1];
  //int numShapes = actor->getShapes(&tempShapes[0], 1);
  //for (int i = 0; i<numShapes; i++)
  //{
  //  MR::PhysXPerShapeData::create(tempShapes[i]);
  //}
  MR::JoltPhysPerShapeData::create(shape);

  return body;
}

//----------------------------------------------------------------------------------------------------------------------
MR::PhysicsObjectID DefaultPhysicsMgr::createNewPhysicsBody(
  float                 dynamicFriction,
  float                 staticFriction,
  float                 restitution,
  bool                  isDynamic,
  MCOMMS::Attribute::PhysicsShapeType shapeType,
  float                 depth,
  float                 height,
  float                 length,
  float                 radius,
  float                 skinWidth,
  bool                  hasIndices,
  bool                  hasVertices,
  size_t                numPoints,
  size_t                numIndices,
  NMP::Vector3*         points,
  int32_t*              indices,
  MCOMMS::Attribute::VerticesWindingType windingType,
  NMP::Matrix34&        transform,
  float                 density,
  bool                  isConstrained,
  MCOMMS::SceneObjectID objectID,
  NMP::Matrix34&        constraintGlobalTransform,
  float                 constraintDamping,
  float                 constraintStiffness,
  NMP::Matrix34&        constraintLocalTransform,
  uint32_t              positionSolverIterationCount,
  uint32_t              velocitySolverIterationCount,
  float                 sleepThreshold,
  float                 maxAngularVelocity,
  float                 linearDamping,
  float                 angularDamping)
{
  JPH::Shape* shape = nullptr;

  switch (shapeType)
  {
  case MCOMMS::Attribute::PHYSICS_SHAPE_BOX:

  {
    JPH::BoxShapeSettings boxsettings(JPH::Vec3(depth * 0.5f, height * 0.5f, length * 0.5f));
    shape = boxsettings.Create().Get().GetPtr();
    break;
  }

  case MCOMMS::Attribute::PHYSICS_SHAPE_CYLINDER: // TODO... proper cylinder (would do but connect itself has no cylinder support so whatever)
  case MCOMMS::Attribute::PHYSICS_SHAPE_CAPSULE:

    shape = JPH::CapsuleShapeSettings(height * 0.5f, radius).Create().Get().GetPtr();
    break;

  case MCOMMS::Attribute::PHYSICS_SHAPE_SPHERE:

    shape = JPH::SphereShapeSettings(radius).Create().Get().GetPtr();
    break;

  case MCOMMS::Attribute::PHYSICS_SHAPE_MESH:

    if (isDynamic) // if the mesh is dynamic then convert it into a convex mesh
    {
      if (hasIndices && hasVertices)
      {
          JPH::Array<JPH::Vec3> joltpoints(numPoints);
          for (int i = 0; i < numPoints; i++)
              joltpoints[i] = MR::nmVector3ToJPHVec3(points[i]);

          JPH::ConvexHullShapeSettings convexmesh(joltpoints);
          JPH::ShapeSettings::ShapeResult convexresult = convexmesh.Create();
          if (convexresult.HasError())
          {
              JPH::String error = convexresult.GetError();
              NMP_ASSERT(0);
          }
          shape = convexresult.Get().GetPtr();
      }
    }
    else
    {
      if (hasIndices && hasVertices)
      {
        JPH::TriangleList list;
        for (int i = 0; i < numIndices;)
        {
            JPH::Vec3 vert1 = MR::nmVector3ToJPHVec3(points[indices[i]]);
            JPH::Vec3 vert2 = MR::nmVector3ToJPHVec3(points[indices[i + 1]]);
            JPH::Vec3 vert3 = MR::nmVector3ToJPHVec3(points[indices[i + 2]]);
            JPH::Triangle newtri(vert1, vert2, vert3);
            list.push_back(newtri);
            i += 3;
        }

        JPH::MeshShapeSettings trianglemesh(list);

        JPH::ShapeSettings::ShapeResult trianglemeshresult = trianglemesh.Create();

        // TODO if the winding type is undefined, should we create the mesh anyway?
        //if (windingType == MCOMMS::Attribute::VERTICES_WINDING_COUNTERCLOCKWISE)
        //  triangleMeshDesc.flags = physx::PxMeshFlag::eFLIPNORMALS;

        shape = trianglemeshresult.Get().GetPtr();
      }
    }
    break;

  // Physics TODO handle the other cases
  default:
    NMP_ASSERT_FAIL();
    return MR_INVALID_PHYSICS_OBJECT_ID;
  }

  JPH::Body* body = nullptr;

  if( shape )
  {
    body = createBody(
      shape,
      isDynamic,
      transform.translation(),
      transform.toQuat(),
      density,
      true,
      NULL,
      staticFriction,
      dynamicFriction,
      skinWidth,
      restitution,
      sleepThreshold,
      maxAngularVelocity,
      linearDamping,
      angularDamping);
  }

  if (!body)
      return MR_INVALID_PHYSICS_OBJECT_ID;

  JPH::BodyInterface& bodyinterface = m_physicsScene->m_joltPhysScene->GetBodyInterface();

  PhysicsUserData* userData = new PhysicsUserData(objectID);
  body->SetUserData((JPH::uint64)userData);

  bodyinterface.AddBody(body->GetID(), JPH::EActivation::Activate);

  MR::PhysicsObjectID actorID = assignPhysicsIDToBody(body);

  // Register the actor in the physics manager.
  getSceneBodies().push_back(body);

  // Create a constraint between the actor and the static world.
  if (isConstrained && isDynamic)
  {
    //physx::PxRigidDynamic* rigidDynamic = actor->is<physx::PxRigidDynamic>();
    //NMP_ASSERT(rigidDynamic);
    //
    //physx::PxTransform parentFrame = MR::nmMatrix34ToPxTransform(constraintGlobalTransform);
    //
    //NMP::Matrix34 constraintLocalTransformInv = constraintLocalTransform;
    //constraintLocalTransformInv.invert();
    //physx::PxTransform childFrame = MR::nmMatrix34ToPxTransform(constraintLocalTransformInv);
    //
    //physx::PxD6Joint *joint = PxD6JointCreate(
    //    PxGetPhysics(), 
    //    0,
    //    parentFrame,
    //    rigidDynamic, 
    //    childFrame);
    //
    //joint->setMotion(physx::PxD6Axis::eSWING1, physx::PxD6Motion::eFREE);
    //joint->setMotion(physx::PxD6Axis::eSWING2, physx::PxD6Motion::eFREE);
    //joint->setMotion(physx::PxD6Axis::eTWIST, physx::PxD6Motion::eFREE);
    //joint->setMotion(physx::PxD6Axis::eX, physx::PxD6Motion::eLOCKED);
    //joint->setMotion(physx::PxD6Axis::eY, physx::PxD6Motion::eLOCKED);
    //joint->setMotion(physx::PxD6Axis::eZ, physx::PxD6Motion::eLOCKED);
    //
    //physx::PxD6JointDrive jointDrive(constraintStiffness, constraintDamping, PX_MAX_F32, true);
    //joint->setDrive(physx::PxD6Drive::eSLERP, jointDrive);

    // Store a reference to the constraint, for deletion.
    //userData->constraint = joint;
  }

  return actorID;
}

//----------------------------------------------------------------------------------------------------------------------
void DefaultPhysicsMgr::destroyPhysicsBody(MCOMMS::SceneObjectID objectID)
{
  // destroy the physics part if there is one
  JPH::Body* foundBody = 0;
  size_t foundBodyIndex = 0;
  for (size_t i = 0; i < m_sceneBodies.size(); ++i)
  {
    if (!m_sceneBodies[i])
      continue;

    PhysicsUserData* userData = (PhysicsUserData*)m_sceneBodies[i]->GetUserData();
    if (userData && userData->isValid())
    {
      MCOMMS::SceneObjectID foundObjectID = userData->objectID;
      if (foundObjectID == objectID)
      {
        foundBody = m_sceneBodies[i];
        foundBodyIndex = i;
        break;
      }
    }
  }

  if (foundBody)
  {
    unassignPhysicsID(foundBody);

    // Destroy per-shape user data
    if (foundBody->GetShape())
    {
      JPH::Shape* shape = const_cast<JPH::Shape*>(foundBody->GetShape());
      MR::JoltPhysPerShapeData* data = MR::JoltPhysPerShapeData::getFromShape(shape);
      MR::JoltPhysPerShapeData::destroy(data, shape);
    }

    // if the object is constrained, release the physical joint
    PhysicsUserData* userData = (PhysicsUserData*)foundBody->GetUserData();
    if (userData->constraint)
    {
      userData->constraint->Release();
      userData->constraint = 0;
    }

    delete userData;
    foundBody->SetUserData(0);

    //finally delete the body
    JPH::BodyInterface& bodyinterface = m_physicsScene->m_joltPhysScene->GetBodyInterface();
    bodyinterface.DestroyBody(foundBody->GetID());

    m_sceneBodies[foundBodyIndex] = m_sceneBodies.back();
    m_sceneBodies.pop_back();
  }

}

//----------------------------------------------------------------------------------------------------------------------
bool DefaultPhysicsMgr::createConstraint(
  uint64_t            constraintGUID,
  MR::PhysicsObjectID physicsObjectID, 
  const NMP::Vector3& constraintPosition, 
  bool                lockOrientation,
  bool                constrainAtCOM)
{
    /*
  NMP_ASSERT_MSG(!getConstraint(constraintGUID), "createConstraint called for a GUID that already exists!");

  physx::PxActor *actorToConstrain = getActorByPhysicsID(physicsObjectID);
  NMP_ASSERT_MSG(actorToConstrain, "Could not find physics object of ID %i!", physicsObjectID);

  physx::PxRigidBody *bodyToConstrain = actorToConstrain->is<physx::PxRigidBody>();
  if(!bodyToConstrain)
  {
    // We can't constrain bodies if they aren't dynamic
    return false;
  }

  // This constraint object is persisted in order to make it possible to modify it later.
  void* alignedMemory = NMP::Memory::memAllocAligned(sizeof(Constraint), NMP_VECTOR_ALIGNMENT);
  Constraint *newConstraint = new(alignedMemory)Constraint();

  // If this is a constrainAtCOM type, we need to make sure we store the offset from the clicked
  // position to the COM, then create the constraint at the COM position.
  NMP::Vector3 actualConstraintPosition;
  if(constrainAtCOM)
  {
    // Get the COM position of the rigid body
    physx::PxRigidBody* rigidBody = bodyToConstrain->is<physx::PxRigidBody>();
    actualConstraintPosition = 
      MR::nmPxVec3ToVector3(rigidBody->getGlobalPose().transform(rigidBody->getCMassLocalPose().p));
    newConstraint->m_grabOffsetFromCOM = constraintPosition - actualConstraintPosition;
  }
  else
  {
    actualConstraintPosition = constraintPosition;
    newConstraint->m_grabOffsetFromCOM.setToZero();
  }

  physx::PxD6Joint *joint;
  if (lockOrientation)
  {
    joint = PxD6JointCreate(
      PxGetPhysics(), 
      0, 
      physx::PxTransform(MR::nmVector3ToPxVec3(actualConstraintPosition)),
      bodyToConstrain, 
      physx::PxTransform(
      bodyToConstrain->getGlobalPose().transformInv(MR::nmVector3ToPxVec3(actualConstraintPosition)), 
      bodyToConstrain->getGlobalPose().q.getConjugate()));
    joint->setMotion(physx::PxD6Axis::eSWING1, physx::PxD6Motion::eLOCKED);
    joint->setMotion(physx::PxD6Axis::eSWING2, physx::PxD6Motion::eLOCKED);
    joint->setMotion(physx::PxD6Axis::eTWIST, physx::PxD6Motion::eLOCKED);
  }
  else
  {
    joint = PxD6JointCreate(
      PxGetPhysics(), 
      0, 
      physx::PxTransform(MR::nmVector3ToPxVec3(actualConstraintPosition)),
      bodyToConstrain, 
      physx::PxTransform(
      bodyToConstrain->getGlobalPose().transformInv(MR::nmVector3ToPxVec3(actualConstraintPosition))));
    joint->setMotion(physx::PxD6Axis::eSWING1, physx::PxD6Motion::eFREE);
    joint->setMotion(physx::PxD6Axis::eSWING2, physx::PxD6Motion::eFREE);
    joint->setMotion(physx::PxD6Axis::eTWIST, physx::PxD6Motion::eFREE);
  }

#if 1
  // use the drive, as otherwise we don't preserve momentum when releasing 
  float dampingRatio = 1.0f;
  float spring = 1000.0f;
  float damping = 2.f*dampingRatio*sqrtf(spring); // critical damping
  physx::PxD6JointDrive drive(spring,damping,PX_MAX_F32,physx::PxD6JointDriveFlag::eACCELERATION);
  joint->setDrive(physx::PxD6Drive::eX, drive);
  joint->setDrive(physx::PxD6Drive::eY, drive);
  joint->setDrive(physx::PxD6Drive::eZ, drive);
  // Free constraint to let the drive work
  joint->setMotion(physx::PxD6Axis::eX, physx::PxD6Motion::eFREE);
  joint->setMotion(physx::PxD6Axis::eY, physx::PxD6Motion::eFREE);
  joint->setMotion(physx::PxD6Axis::eZ, physx::PxD6Motion::eFREE);
#else
  // Use a locked constraint
  joint->setMotion(physx::PxD6Axis::eX, physx::PxD6Motion::eLOCKED);
  joint->setMotion(physx::PxD6Axis::eY, physx::PxD6Motion::eLOCKED);
  joint->setMotion(physx::PxD6Axis::eZ, physx::PxD6Motion::eLOCKED);
#endif

  newConstraint->m_constrainedActor = actorToConstrain;
  newConstraint->m_jointConstraint = joint;

  newConstraint->m_isCOMConstraint = constrainAtCOM;

  m_constraintMap.insert(constraintGUID, newConstraint);

  */

  return true;
}

//----------------------------------------------------------------------------------------------------------------------
bool DefaultPhysicsMgr::destroyConstraint(uint64_t constraintGUID)
{
  Constraint* constraint = NULL;
  bool found = m_constraintMap.find(constraintGUID, &constraint);
  
  if (!found)
  {
    return false;
  }

  // Destroy the joint itself
  constraint->m_jointConstraint->Release();

  // Remove the constraint from the map
  m_constraintMap.erase(constraintGUID);

  // Delete the constraint record itself
  NMP::Memory::memFree(constraint);

  return true;
}

//----------------------------------------------------------------------------------------------------------------------
bool DefaultPhysicsMgr::moveConstraint(uint64_t constraintGUID, const NMP::Vector3 &newGrabPosition)
{
  Constraint* constraint = NULL;
  bool found = m_constraintMap.find(constraintGUID, &constraint);

  if (!found)
  {
    return false;
  }

  // Wake up the body that's connected to the constraint.
  //physx::PxRigidBody *rigidBody = constraint->m_constrainedActor->is<physx::PxRigidBody>();
  //rigidBody->addForce(physx::PxVec3(0,0,0));

  // Move the constraint to the new point in space as requested.
  // TODO: Make use of the com offset if we are a com grab!
  //constraint->m_jointConstraint->setLocalPose(physx::PxJointActorIndex::eACTOR0, physx::PxTransform(MR::nmVector3ToPxVec3(newGrabPosition), physx::PxQuat(physx::PxIdentity)));

  return true;
}


//----------------------------------------------------------------------------------------------------------------------
NMP::Vector3 DefaultPhysicsMgr::getBodyCOMPos(JPH::Body* body) const
{
  return MR::nmJPHVec3ToVector3(body->GetCenterOfMassPosition());
}

//----------------------------------------------------------------------------------------------------------------------
bool DefaultPhysicsMgr::applyForce(
  uint32_t            physicsEngineObjectID, 
  ForceMode           mode,
  const NMP::Vector3& force,
  bool                applyAtCOM,
  const NMP::Vector3& NMP_UNUSED(localSpacePosition),
  const NMP::Vector3& worldSpacePosition)
{
  JPH::Body *body = getBodyByPhysicsID(physicsEngineObjectID);
  
  NMP_ASSERT_MSG(body, "Could not find physics object of ID %i!", physicsEngineObjectID);
  if (!body)
    return false; // body not found

  if (body->IsStatic())
  {
    return false;
  }

  NMP::Vector3 forcePosition = applyAtCOM ? getBodyCOMPos(body) : worldSpacePosition;
  switch(mode)
  {
    case IPhysicsMgr::kFORCE:
      MR::addForceToBody(body, force, forcePosition);
      break; 

    case IPhysicsMgr::kIMPULSE:
      MR::addImpulseToBody(body, force, forcePosition);
      break; 

    case IPhysicsMgr::kVELOCITY_CHANGE:
      MR::addVelocityChangeToBody(body, force, forcePosition);
      break; 

    default:
      NMP_ASSERT_FAIL_MSG("Unknown force mode in apply force");
  }
  return true; 
}

//----------------------------------------------------------------------------------------------------------------------
bool DefaultPhysicsMgr::setPhysicsObjectAttribute(
  uint32_t physicsEngineObjectID,
  const MCOMMS::Attribute* physicsObjAttribute)
{
  JPH::Body *body = getBodyByPhysicsID(physicsEngineObjectID);
  NMP_ASSERT_MSG(body, "Could not find physics object of ID %i!", physicsEngineObjectID);
  if(!body)
    return false;

  switch(physicsObjAttribute->getSemantic())
  {
    // Instantaneously teleport the object to the world space transform specified in the attribute data
    case MCOMMS::Attribute::SEMANTIC_TRANSFORM:
    {
      const NMP::Matrix34* newWorldSpaceTransform = (const NMP::Matrix34*)(physicsObjAttribute->getData());
      MR::setBodyGlobalPoseTM(body, *newWorldSpaceTransform);
      MR::setBodyLinVelW(body, NMP::Vector3(NMP::Vector3::InitZero));
      MR::setBodyAngVelW(body, NMP::Vector3(NMP::Vector3::InitZero));
      return true;
    }

  default:
    break;
  }

  // attribute semantic not handled
  return false;
}

//----------------------------------------------------------------------------------------------------------------------
bool DefaultPhysicsMgr::setPhysicsEnvironmentAttribute(const MCOMMS::Attribute* physicsEnvAttribute)
{
  // We must have a physics scene to work with
  NMP_ASSERT(m_physicsScene);
  if(!m_physicsScene)
  {
    return false;
  }
  
  switch(physicsEnvAttribute->getSemantic())
  {
    // Only handling gravity related attibutes at the moment, which we pass on to updateEnvironment()
    // provided the environment has got both attribs already setup
    case MCOMMS::Attribute::SEMANTIC_GRAVITY_ENABLED:
    case MCOMMS::Attribute::SEMANTIC_GRAVITY_VECTOR:
    {
      MCOMMS::EnvironmentManagementInterface* environmentManager = MCOMMS::getRuntimeTarget()->getEnvironmentManager();
      if(environmentManager)
      {
        MCOMMS::Attribute* gravityAttr = environmentManager->getEnvironmentAttribute(MCOMMS::Attribute::SEMANTIC_GRAVITY_VECTOR);
        MCOMMS::Attribute* gravityEnabledAttr = environmentManager->getEnvironmentAttribute(MCOMMS::Attribute::SEMANTIC_GRAVITY_ENABLED);
        if(gravityAttr && gravityEnabledAttr)
        {
          updateEnvironment();
        }
      }
      break;
    }
    default:
      break;
  }

  // attribute semantic not handled
  return false;
}

//----------------------------------------------------------------------------------------------------------------------
bool DefaultPhysicsMgr::validatePluginList(const NMP::OrderedStringTable& pluginList)
{
  if (pluginList.getNumEntries() == 0)
  {
    return true;
  }

  const char* physicsPlugin = "acPluginJoltPhys_target_" NM_PLATFORM_FORMAT_STRING;
  const char* debugPhysicsPlugin = "acPluginJoltPhys_target_" NM_PLATFORM_FORMAT_STRING "_debug";

  // Physics is registered first so we expect it to be the first compiled plug-in
  const char* pluginEntry0 = pluginList.getEntryString(0);
  if (NMP_STRCMP(pluginEntry0, physicsPlugin) == 0 || NMP_STRCMP(pluginEntry0, debugPhysicsPlugin) == 0)
  {
    return true;
  }

  return false;
}
//----------------------------------------------------------------------------------------------------------------------
void DefaultPhysicsMgr::resetScene()
{
}

//----------------------------------------------------------------------------------------------------------------------
void DefaultPhysicsMgr::initializeScene()
{
  // Materials aren't released when objects are, as they may (in principle) be shared. So, clear
  // everything out here.
  clearScene();

#ifdef NM_PROFILING
  g_totalTiming = 0.0f;
  g_maxTime = 0.0f;
  g_totalSamples = 0;
#endif
}

//----------------------------------------------------------------------------------------------------------------------
void DefaultPhysicsMgr::clearScene()
{
  if (!m_physicsScene)
      return;
  JPH::BodyInterface& bodyinterface = m_physicsScene->m_joltPhysScene->GetBodyInterface();

  for (size_t i = 0; i < m_sceneBodies.size(); ++i)
  {
    if (m_sceneBodies[i])
    {
        bodyinterface.DestroyBody(m_sceneBodies[i]->GetID());
    }
  }
  m_sceneBodies.clear();

  while (!m_materials.empty())
  {
    m_materials.back()->Release();
    m_materials.pop_back();
  }

  for (size_t i = 0; i < m_joints.size(); ++i)
  {
    m_joints[i]->Release();
  }
  m_joints.clear();
}

//----------------------------------------------------------------------------------------------------------------------
void DefaultPhysicsMgr::createPhysicsRig(MR::Network *network, NMP::Vector3* initialPosition)
{
  NMP_ASSERT(network);
  NMP_ASSERT(initialPosition);

  MR::AnimRigDef* animRigDef = network->getActiveRig();
  MR::PhysicsRigDef* physicsRigDef = getActivePhysicsRigDef(network);

  if (physicsRigDef != NULL)
  {
    if (m_physicsRigType == MR::PhysicsRigJoltPhys::TYPE_ARTICULATED)
    {
      
      NMP::Memory::Resource resource = NMPMemoryAllocateFromFormat(
        MR::PhysicsRigJoltPhysRagdoll::getMemoryRequirements(physicsRigDef));
        
      NMP_ASSERT(resource.ptr);
      MR::PhysicsRigJoltPhysRagdoll* physicsRig = MR::PhysicsRigJoltPhysRagdoll::init(
        resource,
        physicsRigDef,
        getPhysicsScene(),
        animRigDef,
        getAnimToPhysicsMap(network->getNetworkDef(), network->getActiveAnimSetIndex()),
        1 << MR::GROUP_CHARACTER_PART,
        (1 << MR::GROUP_CHARACTER_CONTROLLER) | (1 << MR::GROUP_NON_COLLIDABLE) | (1 << MR::GROUP_INTERACTION_PROXY));
      physicsRig->setKinematicPos(*initialPosition);
      setPhysicsRig(network, physicsRig);

      uint32_t numParts = physicsRig->getNumParts();
      for (uint32_t i = 0; i != numParts; ++i)
      {
        MR::PhysicsRigJoltPhysRagdoll::PartJoltPhysRagdoll* part = physicsRig->getPartJoltPhysRagdoll(i);
        JPH::Body* body = part->getBody();
        assignPhysicsIDToBody(body);
      }
    }
    else
    {
      NMP::Memory::Resource resource = NMPMemoryAllocateFromFormat(
        MR::PhysicsRigJoltPhysJointed::getMemoryRequirements(physicsRigDef)); 
      NMP_ASSERT(resource.ptr);
      MR::PhysicsRigJoltPhysJointed* physicsRig = MR::PhysicsRigJoltPhysJointed::init(
        resource,
        physicsRigDef,
        getPhysicsScene(),
        animRigDef,
        getAnimToPhysicsMap(network->getNetworkDef(), network->getActiveAnimSetIndex()),
        1 << MR::GROUP_CHARACTER_PART,
        (1 << MR::GROUP_CHARACTER_CONTROLLER) | (1 << MR::GROUP_NON_COLLIDABLE) | (1 << MR::GROUP_INTERACTION_PROXY));
      physicsRig->setKinematicPos(*initialPosition);
      setPhysicsRig(network, physicsRig);

      uint32_t numParts = physicsRig->getNumParts();
      for (uint32_t i = 0; i != numParts; ++i)
      {
        MR::PhysicsRigJoltPhysJointed::PartJoltPhysJointed* part = physicsRig->getPartJoltPhysJointed(i);
        JPH::Body* body = part->getBody();
        assignPhysicsIDToBody(body);
      }
    }
  }
  else
  { // There is no physics rig in this anim set.
    setPhysicsRig(network, NULL);
  }
}

//----------------------------------------------------------------------------------------------------------------------
void DefaultPhysicsMgr::destroyPhysicsRig(MR::Network *network)
{
  NMP_ASSERT(network);

  // Check whether this network has a physics rig
  MR::PhysicsRig* physicsRigExists = getPhysicsRig(network);

  if (physicsRigExists)
  {
    MR::PhysicsRigDef* physicsRigDef = getActivePhysicsRigDef(network);
    if (physicsRigDef)
    {
      MR::PhysicsRigJoltPhys* physicsRigJoltPhys = (MR::PhysicsRigJoltPhys*) getPhysicsRig(network);

      if (physicsRigJoltPhys)
      {
        uint32_t numParts = physicsRigJoltPhys->getNumParts();
        for (uint32_t i = 0; i != numParts; ++i)
        {
          JPH::Body* body = ((MR::PhysicsRigJoltPhys::PartJoltPhys*) physicsRigJoltPhys->getPart(i))->getRigidBody();
          unassignPhysicsID(body);
        }

        physicsRigJoltPhys->term();
        NMP::Memory::memFree(physicsRigJoltPhys);
      }

      setPhysicsRig(network, NULL);
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------
void DefaultPhysicsMgr::updateConnections(
  MCOMMS::InstanceID NMP_UNUSED(instanceID),
  float              NMP_UNUSED(deltaTime))
{
  NET_LOG_ENTER_FUNC(); 
}

//----------------------------------------------------------------------------------------------------------------------
void DefaultPhysicsMgr::updatePreController(
  MCOMMS::InstanceID instanceID,
  float              deltaTime)
{
  NET_LOG_ENTER_FUNC();
  ControllerRecord* charControllerRecord = m_characterControllerManager->getControllerRecord(instanceID);

  NMP_ASSERT(m_characterControllerManager);

  if (charControllerRecord->m_network->getRootControlMethod() == MR::Network::ROOT_CONTROL_PHYSICS)
  {
    m_characterControllerManager->disableCollision(instanceID);
  }
  else
  {
    NET_LOG_MESSAGE(99, "NetworkInstanceRecord::updatePrePhysics: Updating trajectory\n");
    if (!m_characterControllerManager->getCollisionEnabled(instanceID))
    {
      m_characterControllerManager->enableCollision(instanceID);
    }

    // trajectories should be done now - get them and set in the controller
    charControllerRecord->m_deltaTranslation = charControllerRecord->m_network->getTranslationChange();
    charControllerRecord->m_deltaOrientation = charControllerRecord->m_network->getOrientationChange();

    NMP::Vector3 charDeltaTranslation = charControllerRecord->m_characterOrientation.rotateVector(charControllerRecord->m_deltaTranslation);
    charControllerRecord->m_characterOrientationOld = charControllerRecord->m_characterOrientation;
    charControllerRecord->m_characterOrientation *= charControllerRecord->m_deltaOrientation;
    charControllerRecord->m_characterOrientation.normalise();

    m_context->getNetworkInstanceManager()->findInstanceRecord(instanceID)->setRootTransform(
      NMP::Matrix34(charControllerRecord->m_characterOrientation, charControllerRecord->m_characterPosition));

    charControllerRecord->m_velocityInGravityDirectionDt = deltaTime;

#ifdef ANIM_GRAVITY
    if (charControllerRecord->m_overrideBasics.m_currentVerticalMoveState != MR::CC_STATE_VERTICAL_MOVEMENT_GRAVITY)
    {
        // Don't apply gravity to the CC but set the velocity in gravity direction to the component of the animation
        // delta in the gravity direction so that CC will inherit anim's velocity when switching back to the default
        // (gravity enabled) state.

        // Find the direction of gravity.
        NMP::Vector3 gravityDirection(getPhysicsScene()->getGravity());
        gravityDirection.fastNormalise();
        
        // Set the velocity in the gravity direction.
        charControllerRecord->m_velocityInGravityDirection =
          gravityDirection * charDeltaTranslation.dot(gravityDirection) * deltaTime;
    }
    else
    {
      // apply gravity in real world units
      MCOMMS::EnvironmentManagementInterface* environmentManager = MCOMMS::getRuntimeTarget()->getEnvironmentManager();
      NMP_ASSERT(environmentManager);

      bool gravityEnabled = true;
      MCOMMS::Attribute* gravityEnabledAttr = environmentManager->getEnvironmentAttribute(MCOMMS::Attribute::SEMANTIC_GRAVITY_ENABLED);

      if (gravityEnabledAttr != 0)
      {
        gravityEnabled = (*(int*)gravityEnabledAttr->getData() != 0);
      }
      gravityEnabled &= (charControllerRecord->m_overrideBasics.m_currentVerticalMoveState == MR::CC_STATE_VERTICAL_MOVEMENT_GRAVITY);

      if (gravityEnabled)
      {
        NMP::Vector3 gravity = getPhysicsScene()->getGravity();
        charControllerRecord->m_velocityInGravityDirection += (gravity * deltaTime);
        charDeltaTranslation += (charControllerRecord->m_velocityInGravityDirection * deltaTime);
      }
    }
#endif

    // this should overwrite our root translation via setRootFromCharacterController when the manager
    // updates anyway
    m_characterControllerManager->setRequestedMovement(instanceID, charDeltaTranslation);
  }
}

//----------------------------------------------------------------------------------------------------------------------
void DefaultPhysicsMgr::updatePrePhysics(
  MCOMMS::InstanceID instanceID, 
  float deltaTime,
  bool updatePhysicsRig)
{
  NET_LOG_ENTER_FUNC();
#if defined(NM_DEBUG)
  // For detecting memory leaks
#define NODE_MEMORY_DEBUGGINGx
#ifdef NODE_MEMORY_DEBUGGING
  ControllerRecord* ccmRecord = m_characterControllerManager->getControllerRecord(instanceID);
  ccmRecord->m_network->attribNodeAccounting(10000);
#endif // NODE_MEMORY_DEBUGGING
#endif // NM_DEBUG

  ControllerRecord *charControllerRecord = m_characterControllerManager->getControllerRecord(instanceID);

  if(updatePhysicsRig)
  {
    if (getPhysicsRig(charControllerRecord->m_network))
    {
      getPhysicsRig(charControllerRecord->m_network)->updatePrePhysics(deltaTime);
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------
void DefaultPhysicsMgr::updatePostPhysicsInit(
  MCOMMS::InstanceID instanceID, 
  float              deltaTime)
{  
  ControllerRecord *charControllerRecord = m_characterControllerManager->getControllerRecord(instanceID);

  if (getPhysicsRig(charControllerRecord->m_network))
    getPhysicsRig(charControllerRecord->m_network)->updatePostPhysics(deltaTime);
}

//----------------------------------------------------------------------------------------------------------------------
void DefaultPhysicsMgr::updatePostPhysics(
  MCOMMS::InstanceID instanceID,
  float              deltaTime,
  bool               NMP_UNUSED(updatePhysicsRig))
{
  NET_LOG_ENTER_FUNC();
  ControllerRecord* charControllerRecord = m_characterControllerManager->getControllerRecord(instanceID);

  if (charControllerRecord->m_network->getRootControlMethod() == MR::Network::ROOT_CONTROL_PHYSICS)
  {
    NET_LOG_MESSAGE(99, "NetworkInstanceRecord::updatePostPhysics: Updating trajectory");
    // Update the root translation and orientation.
    // NOTE: When using physics the root needs to be updated in updatePostPhysics so that the network
    // has the new root location, and can thus convert from world to local space. This required removing
    // setting from the root position from SceneObjectRecord::update (since that would be too late)
    charControllerRecord->m_deltaOrientation = charControllerRecord->m_network->getOrientationChange();
    charControllerRecord->m_deltaTranslation = charControllerRecord->m_network->getTranslationChange();
    NMP::Vector3 charDeltaTranslation = charControllerRecord->m_characterOrientation.rotateVector(charControllerRecord->m_deltaTranslation);
    NMP::Vector3 newPos = charControllerRecord->m_characterPosition + charDeltaTranslation;

    // make the root follow the ragdoll
    m_characterControllerManager->setPosition(instanceID, newPos);
    charControllerRecord->m_characterPosition = newPos;
    charControllerRecord->m_characterOrientationOld = charControllerRecord->m_characterOrientation;
    charControllerRecord->m_characterOrientation *= charControllerRecord->m_deltaOrientation;
    charControllerRecord->m_characterOrientation.normalise();
    charControllerRecord->m_network->updateCharacterPropertiesWorldRootTransform(
      NMP::Matrix34(charControllerRecord->m_characterOrientation, charControllerRecord->m_characterPosition),
      true);
    
    // Find the component of delta translation in the gravity direction.
    NMP::Vector3 gravity = getPhysicsScene()->getGravity();
    NMP::Vector3 gravityDirectionVector;
    gravityDirectionVector.normaliseDep(gravity);
    float deltaTranslationInGravityDirection = gravityDirectionVector.dot(charDeltaTranslation);

    // Calculate velocity in gravity direction
    if (deltaTime > 0.0f)
    {
      charControllerRecord->m_velocityInGravityDirection = gravityDirectionVector *
          (deltaTranslationInGravityDirection / deltaTime);
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------
void DefaultPhysicsMgr::setRoot(
  MCOMMS::InstanceID  instanceID,
  const NMP::Vector3& NMP_UNUSED(pos))
{
  // Set the position of the network root.  This function is called from updateControllers(), which
  // has already taken the movement requested by the animation system and filtered it through the
  // PxController's fake physics (if the controller is switched on, or just keyframed the animation along
  // if not).  It has also calculated whether the controller is on the ground or in the air, and whether
  // it has just transitioned between the two.  The ccmRecord's m_characterPosition is at this stage set
  // to point to the foot centre position as processed by the above.

  // TODO: convert to local space
  ControllerRecord* charControllerRecord = m_characterControllerManager->getControllerRecord(instanceID);
  NMP::Vector3 actualTranslation = (charControllerRecord->m_characterPosition -
                                    charControllerRecord->m_characterPositionOld);
  MCOMMS::EnvironmentManagementInterface* environmentManager = MCOMMS::getRuntimeTarget()->getEnvironmentManager();
  NMP_ASSERT(environmentManager);

  if (charControllerRecord->m_network->getRootControlMethod() != MR::Network::ROOT_CONTROL_PHYSICS)
  {
    // record the movement of the entity - but don't add on vertical velocity due to stepping.
    bool gravityEnabled = true;
    MCOMMS::Attribute* gravityEnabledAttr = environmentManager->getEnvironmentAttribute(MCOMMS::Attribute::SEMANTIC_GRAVITY_ENABLED);
    if (gravityEnabledAttr)
      gravityEnabled = (*(int*)gravityEnabledAttr->getData() != 0);
    gravityEnabled &= (charControllerRecord->m_overrideBasics.m_currentVerticalMoveState == MR::CC_STATE_VERTICAL_MOVEMENT_GRAVITY);

    // Find the component of actual translation in the gravity direction.
    NMP::Vector3 gravity = getPhysicsScene()->getGravity();
    NMP::Vector3 gravityDirectionVector;
    gravityDirectionVector.normaliseDep(gravity);
    float actualTranslationInGravityDirection = gravityDirectionVector.dot(actualTranslation);

    // Find the component of delta translation in the gravity direction.
    float deltaTranslationInGravityDirection = gravityDirectionVector.dot(charControllerRecord->m_deltaTranslation);

    if (actualTranslationInGravityDirection > deltaTranslationInGravityDirection && gravityEnabled == true)
    {
      // Find the component of delta translation in the gravity direction.
      charControllerRecord->m_velocityInGravityDirection = actualTranslation - charControllerRecord->m_deltaTranslation;
      deltaTranslationInGravityDirection = gravityDirectionVector.dot(charControllerRecord->m_velocityInGravityDirection);

      // Calculate velocity in gravity direction
      if (charControllerRecord->m_velocityInGravityDirectionDt > 0)
      {
        charControllerRecord->m_velocityInGravityDirection =
          gravityDirectionVector * (deltaTranslationInGravityDirection / charControllerRecord->m_velocityInGravityDirectionDt);
      }
    }
    else
    {
      charControllerRecord->m_velocityInGravityDirection.setToZero();
    }

    // if stepping down then zero the velocity so that we run off cliff edges starting horizontal.
    if (m_characterControllerManager->getOnGround(instanceID))
      charControllerRecord->m_velocityInGravityDirection.setToZero();
  }

  charControllerRecord->m_deltaTranslation = actualTranslation;
  NMP::Quat oldOrientInv(charControllerRecord->m_characterOrientationOld);
  oldOrientInv.conjugate();
  charControllerRecord->m_deltaTranslation = oldOrientInv.rotateVector(charControllerRecord->m_deltaTranslation);

  charControllerRecord->m_network->updateCharacterPropertiesWorldRootTransform(
    NMP::Matrix34(charControllerRecord->m_characterOrientation, charControllerRecord->m_characterPosition),
    true);
}

//----------------------------------------------------------------------------------------------------------------------
IControllerMgr* DefaultPhysicsMgr::getControllerManager()
{
  return m_characterControllerManager;
}

//----------------------------------------------------------------------------------------------------------------------
void DefaultPhysicsMgr::updatePhysicalSceneObjects()
{
  const MCOMMS::SceneObjectManagementInterface* const sceneObjectManager =
    MCOMMS::getRuntimeTarget()->getSceneObjectManager();
  NMP_ASSERT(sceneObjectManager);

  if (sceneObjectManager)
  {
    std::vector<JPH::Body*>& sceneBodies = getSceneBodies();
    size_t nBodies = sceneBodies.size();

    for (size_t i = 0 ; i < nBodies ; ++i)
    {
      JPH::Body* body = sceneBodies[i];
      if (!body)
        continue;

      if (!body->GetUserData())
        continue;

      PhysicsUserData* userData = (PhysicsUserData*)body->GetUserData();
      if (!userData->isValid())
        continue;

      const MCOMMS::SceneObjectID objectID = userData->objectID;

      // Find the scene object.
      MCOMMS::SceneObject* const sceneObect = sceneObjectManager->getSceneObject(objectID);
      if (sceneObect)
      {
        MCOMMS::Attribute* const transformAttribute = sceneObect->getAttribute(MCOMMS::Attribute::SEMANTIC_TRANSFORM);
        if (transformAttribute)
        {
          const NMP::Matrix34 transform = MR::nmJPHMat44ToNmMatrix34(body->GetWorldTransform());
          *(NMP::Matrix34*)transformAttribute->getData() = transform;
        }
      }
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------
void DefaultPhysicsMgr::updateSceneObjects(float deltaTime)
{
  m_context->getSceneObjectManager()->updateSceneObjectRecords(
    deltaTime,
    m_context->getNetworkInstanceManager(),
    getControllerManager());
}

//----------------------------------------------------------------------------------------------------------------------
void DefaultPhysicsMgr::updateControllers(float deltaTime)
{
  m_characterControllerManager->updateControllers(deltaTime);
}

namespace
{
#ifdef NM_PROFILING
//----------------------------------------------------------------------------------------------------------------------
void recursiveTraverseRecords(NMP::Profiler* profiler, const NMP::Profiler::ProfilerRecord* record, size_t depth, void (*processRecord)(NMP::Profiler*, const NMP::Profiler::ProfilerRecord*, size_t))
{
  (*processRecord)(profiler, record, depth);
  const NMP::Profiler::ProfilerRecord* end = profiler->getRecordChildrenEnd(record);
  for (const NMP::Profiler::ProfilerRecord* child = profiler->getRecordChildrenBegin(record); child != end; child = profiler->getRecordNextSibling(child))
  {
    recursiveTraverseRecords(profiler, child, depth + 1, processRecord);
  }
}

//----------------------------------------------------------------------------------------------------------------------
void printRecord(NMP::Profiler* profiler, const NMP::Profiler::ProfilerRecord* record, size_t depth)
{
  for (uint32_t tab = 1; tab < depth; ++tab)
  {
    LOG_PROFILING_MESSAGE(" | ");
  }
  if (depth)
  {
    LOG_PROFILING_MESSAGE(" |_");
  }
  else
  {
    g_maxTime = (profiler->getRecordTime(record) > g_maxTime) ? profiler->getRecordTime(record): g_maxTime;
    g_totalTiming += profiler->getRecordTime(record);
    LOG_PROFILING_MESSAGE("Average time: %6.3f ms\n", g_totalTiming/++g_totalSamples);
    LOG_PROFILING_MESSAGE("Peak time: %6.3f ms\n", g_maxTime);
  }
  LOG_PROFILING_MESSAGE("%6.3f ms - %s\n", profiler->getRecordTime(record), profiler->getRecordBlock(record)->getTag());
}

//----------------------------------------------------------------------------------------------------------------------
void gatherProfilingData(NMP::Profiler* profiler)
{
  const size_t threadCount = profiler->getNumThreads();
  for (size_t threadIndex = 0; threadIndex != threadCount; ++threadIndex)
  {
    LOG_PROFILING_MESSAGE("=================================================\n");

    const NMP::Profiler::ProfilerRecord* rootRecord = profiler->getRootRecord(threadIndex);

    if (rootRecord)
    {
      recursiveTraverseRecords(profiler, rootRecord, 0, &printRecord);
    }
  }
}
#endif // NM_PROFILING
} // anonymous namespace

//----------------------------------------------------------------------------------------------------------------------
void DefaultPhysicsMgr::update(float deltaTime)
{
  if (!m_physicsScene || !m_characterControllerManager)
  {
    // No need to update if there isn't a physics scene or controller manager.  There might not be one if the 
    // runtime target is not simulating a scene.
    return;
  }

  NM_BEGIN_PROFILING("------- Update Root -------");

  // Update all instances
  m_context->getNetworkInstanceManager()->applyAnimationSetChanges();

  PHYSICS_LOG_LINE_DIVIDE();
  NM_BEGIN_PROFILING("setNextPhysicsTimeStep");
  m_physicsScene->setNextPhysicsTimeStep(getActualTimestep(deltaTime));
  NM_END_PROFILING(); //"setNextPhysicsTimeStep"

  m_characterControllerManager->updateInstanceNetworksBegin(deltaTime);

  PHYSICS_LOG_MESSAGE("updateInstancesConnections");
  NM_BEGIN_PROFILING("updateInstancesConnections");
  m_characterControllerManager->updateInstancesConnections(deltaTime);
  NM_END_PROFILING(); //"updateInstancesConnections"

  m_characterControllerManager->updateInstanceNetworksContinue(MR::CoreTaskIDs::MR_TASKID_NETWORKUPDATECHARACTERCONTROLLER);

  PHYSICS_LOG_MESSAGE("updateInstancesPreController");
  NM_BEGIN_PROFILING("updateInstancesPreController");
  m_characterControllerManager->updateInstancesPreController(deltaTime);
  NM_END_PROFILING(); //"updateInstancesPreController"

  if (m_physicsAndCharacterControllerUpdate == MR::PHYSICS_AND_CC_UPDATE_SEPARATE)
  {
    PHYSICS_LOG_MESSAGE("updating character controllers");
    NM_BEGIN_PROFILING("updateControllers");
    m_characterControllerManager->updateControllers(deltaTime);
    NM_END_PROFILING(); //"updateControllers"
  }

  m_characterControllerManager->updateInstanceNetworksContinue(MR::CoreTaskIDs::MR_TASKID_NETWORKUPDATEPHYSICS);

  PHYSICS_LOG_MESSAGE("updateInstancesPrePhysics");
  NM_BEGIN_PROFILING("updateInstancesPrePhysics");
  m_characterControllerManager->updateInstancesPrePhysics(deltaTime);
  NM_END_PROFILING(); //"updateInstancesPrePhysics"


  if (m_physicsAndCharacterControllerUpdate != MR::PHYSICS_AND_CC_UPDATE_SEPARATE)
  {
    PHYSICS_LOG_MESSAGE("updating character controllers");
    NM_BEGIN_PROFILING("updateControllers");
    m_characterControllerManager->updateControllers(deltaTime);
    NM_END_PROFILING(); //"updateControllers"
  }

  // update scene object pre-physics
  PHYSICS_LOG_MESSAGE("Stepping Physics");
  NM_BEGIN_PROFILING("simulate");
  simulate(deltaTime);
  NM_END_PROFILING(); //"simulate"

  m_characterControllerManager->updateInstancesPostPhysicsInit(deltaTime);
  m_characterControllerManager->updateInstanceNetworksContinue(MR::CoreTaskIDs::MR_TASKID_NETWORKUPDATEROOT);

  PHYSICS_LOG_MESSAGE("updateInstancesPostPhysics");
  NM_BEGIN_PROFILING("updateInstancesPostPhysics");
  m_characterControllerManager->updateInstancesPostPhysics(deltaTime);
  NM_END_PROFILING(); //"updateInstancesPostPhysics"

  m_characterControllerManager->updateInstanceNetworksContinue(MR::TASK_ID_UNSPECIFIED);
  // we don't expect any external tasks, or any physics related tasks, to be left here
  m_characterControllerManager->updateInstanceNetworksEnd();

  PHYSICS_LOG_MESSAGE("updateSceneObjects");

  // update the physics objects
  NM_BEGIN_PROFILING("updateSceneObjectsFromPhysics");
  updatePhysicalSceneObjects();
  NM_END_PROFILING(); //"updateSceneObjectsFromPhysics"

  // Update all objects
  NM_BEGIN_PROFILING("updateSceneObjects");
  updateSceneObjects(deltaTime);
  NM_END_PROFILING(); //"updateSceneObjects"

  // Update the Connect-side representations of the character controllers
  m_characterControllerManager->updateControllerRepresentations();

  NM_END_PROFILING(); //"updateInstances"

#ifdef MR_OUTPUT_DEBUGGING
  m_characterControllerManager->sendInstanceProfileTimingDebugOutput();
#endif // MR_OUTPUT_DEBUGGING

#ifdef NM_PROFILING
  NMP::Profiler::getProfiler()->endProfilingFrame();
  gatherProfilingData(NMP::Profiler::getProfiler());
  NMP::Profiler::getProfiler()->reset();
#endif // NM_PROFILING

  ++m_frameIndex;

  return;
}

namespace
{

//----------------------------------------------------------------------------------------------------------------------
void incPhysicsAssetRefCounts(MR::NetworkDef* networkDef)
{
  uint32_t numAnimSets = networkDef->getNumAnimSets();
  for (MR::AnimSetIndex animSetIdx = 0; animSetIdx < numAnimSets; ++animSetIdx)
  {
    // Physics Rigs.
    MR::PhysicsRigDef* physicsRig = getPhysicsRigDef(networkDef, animSetIdx);
    if (physicsRig)
    {
      uint32_t refCount = MR::Manager::incObjectRefCount(physicsRig);
      NM_LOG_MESSAGE(
        RTT_LOGGER,
        RTT_MESSAGE_PRIORITY,
        "  Increasing RefCount - PhysicsRigDef AssetID: %p, RefCount: %d\n",
        MR::Manager::getInstance().getObjectIDFromObjectPtr(physicsRig),
        refCount);
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------
void decPhysicsAssetRefCounts(MR::NetworkDef* networkDef)
{
  uint32_t numAnimSets = networkDef->getNumAnimSets();
  for (MR::AnimSetIndex animSetIdx = 0; animSetIdx < numAnimSets; ++animSetIdx)
  {
    // Physics Rigs.
    MR::PhysicsRigDef* physicsRig = getPhysicsRigDef(networkDef, animSetIdx);
    if (physicsRig)
    {
      uint32_t refCount = MR::Manager::decObjectRefCount(physicsRig);
      NM_LOG_MESSAGE(
        RTT_LOGGER,
        RTT_MESSAGE_PRIORITY,
        "    Decreasing RefCount - PhysicsRigDef AssetID: %p, RefCount: %d\n",
        MR::Manager::getInstance().getObjectIDFromObjectPtr(physicsRig),
        refCount);
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------
void deleteOrphanedPhysicsAsset(MR::NetworkDef* networkDef)
{
  uint32_t numAnimSets = networkDef->getNumAnimSets();
  for (uint16_t animSetIndex = 0 ; animSetIndex < numAnimSets ; ++animSetIndex)
  {
    //----------
    // Physics Rig.
    MR::PhysicsRigDef* physicsRig = getPhysicsRigDef(networkDef, animSetIndex);
    if (physicsRig && (MR::Manager::getInstance().getObjectRefCount(physicsRig) == 0))
    {
      NM_LOG_MESSAGE(
        RTT_LOGGER,
        RTT_MESSAGE_PRIORITY,
        "  Deleting referenced PhysicsRigDef - AssetID: %p.\n",
        MR::Manager::getInstance().getObjectIDFromObjectPtr(physicsRig));
      MR::Manager::getInstance().unregisterObject(physicsRig);
      NMP::Memory::memFree(physicsRig);
    }
  }
}

} // anonymous namespace

//----------------------------------------------------------------------------------------------------------------------
void DefaultPhysicsMgr::initializePhysicsCore(uint32_t numDispatchers, MR::Dispatcher** dispatchers)
{
  // Engine specific assets
  MR::Manager::getInstance().registerAsset(MR::Manager::kAsset_PhysicsRigDef, MR::locatePhysicsRigDefJoltPhys);

  MR::initMorphemePhysics(numDispatchers, dispatchers);
}

//----------------------------------------------------------------------------------------------------------------------
void DefaultPhysicsMgr::finaliseInitPhysicsCore()
{
  MR::finaliseInitMorphemePhysics();
}

//----------------------------------------------------------------------------------------------------------------------
JPH::Body* DefaultPhysicsMgr::getBodyByPhysicsID(MR::PhysicsObjectID id) const
{
  JPH::Body* body = 0;
  m_physicsIDBodyMap.find(id, &body);
  return body;
}

//----------------------------------------------------------------------------------------------------------------------
void* DefaultPhysicsMgr::getPhysicsObjectPointerFromPhysicsID(MR::PhysicsObjectID id) const
{
  return getBodyByPhysicsID(id);
}

//----------------------------------------------------------------------------------------------------------------------
MR::PhysicsObjectID DefaultPhysicsMgr::getPhysicsObjectIDFromPhysicsObjectPointer(void* physicsObject) const
{
  return getPhysicsIDForBody((const JPH::Body*) physicsObject);
}


//----------------------------------------------------------------------------------------------------------------------
MR::PhysicsObjectID DefaultPhysicsMgr::getPhysicsIDForPart(const MR::PhysicsRig::Part* part) const
{
  JPH::Body* body = ((MR::PhysicsRigJoltPhys::PartJoltPhys*) part)->getRigidBody();
  return getPhysicsIDForBody(body);
}

//----------------------------------------------------------------------------------------------------------------------
MR::PhysicsObjectID DefaultPhysicsMgr::getPhysicsIDForBody(const JPH::Body* actor) const
{
  MR::PhysicsObjectID id = MR_INVALID_PHYSICS_OBJECT_ID;
  m_bodyPhysicsIDMap.find(const_cast<JPH::Body*>(actor), &id);
  return id;
}

//----------------------------------------------------------------------------------------------------------------------
void DefaultPhysicsMgr::resetSDKS()
{
  deleteScene();

  float assetScale = 1.0f;

  MCOMMS::RuntimeTargetInterface* runtimeTarget = MCOMMS::getRuntimeTarget();
  if (runtimeTarget)
  {
    MCOMMS::EnvironmentManagementInterface* environmentManager = runtimeTarget->getEnvironmentManager();
    MCOMMS::Attribute* assetScaleAttr = environmentManager->getEnvironmentAttribute(MCOMMS::Attribute::SEMANTIC_ASSET_SCALE);
    NMP_ASSERT(assetScaleAttr);
    if (assetScaleAttr)
    {
      assetScale = *((float*)assetScaleAttr->getData());
    }
  }
  m_toleranceScalingValue = assetScale;

  PhysicsSDK::init(m_toleranceScalingValue);

  m_maxTimeStep = 1.0f / 15.0f;

  JPH::Vec3 gravity(0, -9.81f, 0);

  JPH::PhysicsSystem* phys_system = new JPH::PhysicsSystem();

  NMP_ASSERT(phys_system);
  void* alignedMemory = NMP::Memory::memAllocAligned(sizeof(MR::PhysicsSceneJoltPhys), NMP_VECTOR_ALIGNMENT);
  m_physicsScene = new(alignedMemory) MR::PhysicsSceneJoltPhys(PhysicsSDK::sm_joltTempAllocator, PhysicsSDK::sm_joltJobSystem, phys_system);

  // create CCM and SOM
  m_characterControllerManager = new DefaultControllerMgr(this, m_context);

  resetScene();
}

//----------------------------------------------------------------------------------------------------------------------
void DefaultPhysicsMgr::onAssetScaleChanged(float assetScale)
{
  if (NMP::nmfabs(assetScale - m_toleranceScalingValue) > 0.000001f && m_sceneBodies.empty() && m_joints.empty())
  {
    resetSDKS();
  }
}

//----------------------------------------------------------------------------------------------------------------------
MR::PhysicsObjectID DefaultPhysicsMgr::assignPhysicsIDToBody(JPH::Body* body)
{
  NMP_ASSERT(body != 0);

  bool found = m_bodyPhysicsIDMap.find(body);
  if (found)
  {
    // already added to map
    return false;
  }

  // get a free id for this object
  MR::PhysicsObjectID bodyID = m_nextPhysicsObjectID++;

  bool result = m_physicsIDBodyMap.insert(bodyID, body);

  if (!result)
  {
    // insertion failed
    return MR_INVALID_PHYSICS_OBJECT_ID;
  }

  result = m_bodyPhysicsIDMap.insert(body, bodyID);

  if (!result)
  {
    // insertion failed, remove the entry from m_physicsIdBodyMap so it still contains the same
    // objects as m_bodyPhysicsIdMap.
    m_physicsIDBodyMap.erase(bodyID);
    return MR_INVALID_PHYSICS_OBJECT_ID;
  }

  return bodyID;
}

//----------------------------------------------------------------------------------------------------------------------
bool DefaultPhysicsMgr::unassignPhysicsID(JPH::Body* body)
{
  NMP_ASSERT(body != 0);

  MR::PhysicsObjectID bodyId = MR_INVALID_PHYSICS_OBJECT_ID;

  bool found = m_bodyPhysicsIDMap.find(body, &bodyId);
  if (!found)
  {
    // object was never given a physics object id.
    return false;
  }

  bool result = m_physicsIDBodyMap.erase(bodyId);
  if (!result)
  {
    // the maps got out of sync somehow
    return false;
  }

  m_bodyPhysicsIDMap.erase(body);

  return true;
}

//----------------------------------------------------------------------------------------------------------------------
void DefaultPhysicsMgr::clearAllPhysicsIDs()
{
  m_nextPhysicsObjectID = 0;
  m_physicsIDBodyMap.clear();
  m_bodyPhysicsIDMap.clear();
}

//----------------------------------------------------------------------------------------------------------------------
DefaultPhysicsMgr::Constraint* DefaultPhysicsMgr::getConstraint(uint64_t guid) const
{
  Constraint* constraint = 0;
  m_constraintMap.find(guid);
  return constraint;
}

//----------------------------------------------------------------------------------------------------------------------
void DefaultPhysicsMgr::deleteScene()
{
  clearScene();
  if (m_physicsScene)
  {
    delete m_physicsScene->m_joltPhysScene;
    m_physicsScene = 0;
  }

  delete m_characterControllerManager;
  m_characterControllerManager = 0;

  PhysicsSDK::term();
}
