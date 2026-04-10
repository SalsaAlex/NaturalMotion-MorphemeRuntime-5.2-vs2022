// Copyright (c) 2011 NaturalMotion.  All Rights Reserved.
// Not to be copied, adapted, modified, used, distributed, sold,
// licensed or commercially exploited in any manner without the
// written consent of NaturalMotion.
//
// All non public elements of this software are the confidential
// information of NaturalMotion and may not be disclosed to any
// person nor used for any purpose not expressly approved by
// NaturalMotion in writing.

//----------------------------------------------------------------------------------------------------------------------
#include "morpheme/mrManager.h"
#include "euphoria/erInteractionProxy.h"
#include "euphoria/erInteractionProxyDef.h"
#include "physics/mrPhysics.h"
#include "physics/JoltPhys/mrPhysicsRigJoltPhys.h"
#include "physics/JoltPhys/mrJoltPhysIncludes.h"


namespace ER
{

InteractionProxySetup::InteractionProxySetup(const MR::PhysicsRigJoltPhys* ownerPhysicsRig)
{
  m_ownerPhysicsRigID = ownerPhysicsRig->getRigID();
  m_mass = ownerPhysicsRig->calculateMass();
  m_initialTM = ownerPhysicsRig->getRootPartTransform();
  m_physicsScene = ownerPhysicsRig->getPhysicsSceneJoltPhys()->m_joltPhysScene;
}

//----------------------------------------------------------------------------------------------------------------------
InteractionProxySetup::InteractionProxySetup(
  float                   mass,
  const NMP::Matrix34&    initialTM,
  JPH::PhysicsSystem*         physicsScene)
{
  m_mass = mass;
  m_initialTM = initialTM;
  m_physicsScene = physicsScene;
}

//----------------------------------------------------------------------------------------------------------------------
void InteractionProxy::init(
  const InteractionProxyDef* interactionProxyDef,
  const InteractionProxySetup& interactionProxySetup)
{
  NMP_ASSERT(m_body == NULL);

  m_interactionProxyDef = interactionProxyDef;
  m_physicsScene = interactionProxySetup.m_physicsScene;

  // Create the shape
  JPH::Shape* shape = nullptr;

  // Create a basic material for the capsule
  //physx::PxMaterial* material = PxGetPhysics().createMaterial(1.0f, 1.0f, 0.0f);
  //material->setFrictionCombineMode(physx::PxCombineMode::eMULTIPLY);
  //material->setRestitutionCombineMode(physx::PxCombineMode::eMULTIPLY);

  NMP::Matrix34 m(NMP::Matrix34::kIdentity);

  switch (m_interactionProxyDef->getShapeType())
  {
  case InteractionProxyDef::CapsuleShape:
    {
      shape = new JPH::CapsuleShape(m_interactionProxyDef->getHeight() * 0.5f, m_interactionProxyDef->getRadius());
      m.fromEulerXYZ(NMP::Vector3(0.0f, NM_PI/2.f, 0.0f));
    }
    break;

  case InteractionProxyDef::SphereShape:
    {
      shape = new JPH::SphereShape(m_interactionProxyDef->getRadius());
    }
    break;

  case InteractionProxyDef::BoxShape:
    {
      shape = new JPH::BoxShape(JPH::Vec3(
          m_interactionProxyDef->getWidth() * 0.5f,
          m_interactionProxyDef->getHeight() * 0.5f,
          m_interactionProxyDef->getDepth() * 0.5f));
    }
    break;

  default:
    NMP_ASSERT_FAIL_MSG("Only capsule, sphere and box shapes are supported for the Interaction Proxy");
    return;
  }

  //shapeDesc.localPose = MR::nmMatrix34ToPxTransform(m);
  //shapeDesc.contactOffset = 0.0001f;
  //shapeDesc.restOffset = 0.0f;
  //physx::PxFilterData collisionData; // Make non-colliding
  //collisionData.word0 = 0;
  //collisionData.word1 = 0xffffffff;
  //shapeDesc.simulationFilterData = collisionData;
  //shapeDesc.setSingleMaterial(material);

  // And also a descriptor for the capsule actor
  JPH::BodyCreationSettings bodysettings;
  bodysettings.SetShape(shape);
  bodysettings.mPosition = MR::nmVector3ToJPHVec3(interactionProxySetup.m_initialTM.translation());
  bodysettings.mRotation = MR::nmQuatToJPHQuat(interactionProxySetup.m_initialTM.toQuat());
  bodysettings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
  bodysettings.mMassPropertiesOverride.mMass = interactionProxySetup.m_mass > 0.0f ? interactionProxySetup.m_mass : FLT_MAX;
  bodysettings.mMaxAngularVelocity = FLT_MAX;
  m_body = m_physicsScene->GetBodyInterface().CreateBody(bodysettings);
  if (!m_body)
  {
    return;
  }
  //m_body->setName("Interaction proxy");

  // Special query filter data for the proxy character
  //physx::PxShape* proxyShape;
  //m_actor->getShapes(&proxyShape, 1);
  //physx::PxFilterData data;
  //data.word0 = 1 << MR::GROUP_INTERACTION_PROXY;
  //// By giving it the character rig id we ensure that the character's probes don't collide with it
  //data.word2 = interactionProxySetup.m_ownerPhysicsRigID;
  //proxyShape->setQueryFilterData(data);
  //m_prevTransform = interactionProxySetup.m_initialTM;

  MR::JoltPhysPerBodyData::create(m_body);
}

//----------------------------------------------------------------------------------------------------------------------
void InteractionProxy::setRigID(int32_t ownerPhysicsRigID)
{
  //physx::PxShape* proxyShape;
  //m_actor->getShapes(&proxyShape, 1);
  //physx::PxFilterData data = proxyShape->getQueryFilterData();
  //data.word2 = ownerPhysicsRigID;
  //proxyShape->setQueryFilterData(data);
}

//----------------------------------------------------------------------------------------------------------------------
void InteractionProxy::update(
  float deltaTime, 
  const MR::Network& network, 
  MR::FrameCount frameCount, 
  bool calculateVelocities)
{
  // The body pointer can be 0 if we weren't initialised
  if (!m_body)
    return;

  NMP::Vector3 vel;
  NMP::Vector3 angVel;
  NMP::Matrix34 TM;

  NMP::DataBuffer* buffer = getNodeTransformsBuffer(
    network.getNetworkDef()->getRootNodeID(), 
    &network, 
    frameCount,
    network.getActiveAnimSetIndex());

  if (!buffer)
  {
    return;
  }

  MR::AnimRigDef* animRigDef = network.getRig(network.getActiveAnimSetIndex());

  NMP::DataBuffer* bindPose = animRigDef->getBindPose()->m_transformBuffer;

  TM = m_interactionProxyDef->getTransform();
  int32_t index = m_interactionProxyDef->getParentIndex();

  // If the channels are not available in the source transform buffer, we use the bind pose.
  NMP::Vector3 sourcePos = buffer->getUsedFlags()->isBitSet(index) ? *buffer->getChannelPos(index) : *bindPose->getChannelPos(index);
  NMP::Quat sourceQuat = buffer->getUsedFlags()->isBitSet(index) ? *buffer->getChannelQuat(index) : *bindPose->getChannelQuat(index);

  NMP::Matrix34 T(sourceQuat, sourcePos);

  TM.multiply(T);

  while ((index = (int32_t)animRigDef->getParentBoneIndex(index)) > 0)
  {
    // If the channels are not available in the source transform buffer, we use the bind pose.
    sourcePos = buffer->getUsedFlags()->isBitSet(index) ? *buffer->getChannelPos(index) : *bindPose->getChannelPos(index);
    sourceQuat = buffer->getUsedFlags()->isBitSet(index) ? *buffer->getChannelQuat(index) : *bindPose->getChannelQuat(index);

    NMP::Matrix34 bufferTM(sourceQuat, sourcePos);
    TM.multiply(bufferTM);
  }

  const NMP::Matrix34& worldRoot = network.getCharacterPropertiesWorldRootTransform();
  TM.multiply(worldRoot);
  TM.orthonormalise();

  // if not using physics, update the velocity the same way for the two animation based methods
  if (calculateVelocities && deltaTime > 0.0f)
  {
    // linear
    vel = (TM.translation() - m_prevTransform.translation()) / deltaTime;

    // angular
    NMP::Quat prevRot = m_prevTransform.toQuat();
    NMP::Quat prevInvRot = ~prevRot;
    NMP::Quat newRot = TM.toQuat();

    NMP::Quat rotation = newRot * prevInvRot;
    angVel = rotation.toRotationVector() / deltaTime;
  }
  else
  {
    vel.setToZero();
    angVel.setToZero();
  }

  updateTM(TM, vel, angVel);
  m_prevTransform = TM;
}

//----------------------------------------------------------------------------------------------------------------------
/// Update function for the manually initialised characterQueryProxy. The capsule is updated each frame directly 
/// by specifying the new world space transform, velocity and angular velocity for the capsule.
void InteractionProxy::updateTM(const NMP::Matrix34& transform, const NMP::Vector3& velocity, 
  const NMP::Vector3& angularVelocity)
{
  if (!m_body)
    return;

  JPH::BodyInterface& interface = m_physicsScene->GetBodyInterface();
  JPH::BodyID id = m_body->GetID();

  interface.SetPositionAndRotation(
      id,
      MR::nmVector3ToJPHVec3(transform.translation()),
      MR::nmQuatToJPHQuat(transform.toQuat()),
      JPH::EActivation::Activate);

  interface.SetLinearAndAngularVelocity(
      id,
      MR::nmVector3ToJPHVec3(velocity),
      MR::nmVector3ToJPHVec3(angularVelocity)
  );
}

//----------------------------------------------------------------------------------------------------------------------
/// Removes and deallocates the proxy.
void InteractionProxy::term()
{
  if (!m_body)
  {
    return;
  }
  MR::JoltPhysPerBodyData* data = MR::JoltPhysPerBodyData::getFromBody(m_body);
  MR::JoltPhysPerBodyData::destroy(data, m_body);

  //physx::PxMaterial* material[1];
  //shapes[0]->getMaterials(&material[0], 1);
  //material[0]->release();
  m_physicsScene->GetBodyInterface().RemoveBody(m_body->GetID());
  m_physicsScene->GetBodyInterface().DestroyBody(m_body->GetID());
  m_body = 0;
}

} // namespace ER

//----------------------------------------------------------------------------------------------------------------------
