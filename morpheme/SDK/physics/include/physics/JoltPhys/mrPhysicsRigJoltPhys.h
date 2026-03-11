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
#ifndef MR_PHYSICS_RIG_JOLTPHYS_H
#define MR_PHYSICS_RIG_JOLTPHYS_H
//----------------------------------------------------------------------------------------------------------------------
#include "physics/mrPhysicsRig.h"
#include "mrJoltPhysConfigure.h"
#if defined(MR_OUTPUT_DEBUGGING)
#include "sharedDefines/mPhysicsDebugInterface.h"
#endif // MR_OUTPUT_DEBUGGING
#ifdef NM_COMPILER_MSVC
  #pragma warning(disable: 4555)
#endif //NM_COMPILER_MSVC
#include "NMPlatform/NMHashMap.h"
//----------------------------------------------------------------------------------------------------------------------

namespace MR
{

class PhysicsSceneJoltPhys;

//----------------------------------------------------------------------------------------------------------------------
bool locatePhysicsRigDefJoltPhys(uint32_t assetType, void* assetMemory);

//----------------------------------------------------------------------------------------------------------------------
/// \class MR::PhysicsRigJoltPhys
/// \brief Base class for the Jolt Physics implementation of a physics rig, which can be either jointed
///        or articulation.
//----------------------------------------------------------------------------------------------------------------------
class PhysicsRigJoltPhys : public PhysicsRig
{
protected:
  /// The maximum number of shapes that will appear in a physics volume. Used to size temporary
  /// internal memory.
  enum { MAX_SHAPES_IN_VOLUME = 16 }; 
public:
  enum Type {TYPE_JOINTED, TYPE_ARTICULATED};

  class PartJoltPhys : public Part
  {
    friend class PhysicsRigJoltPhys;

  public:
    PartJoltPhys();

    void addVelocityChange(const NMP::Vector3& velChange, const NMP::Vector3& worldPos, float angularMultiplier = 1.0f) NM_OVERRIDE;
    void addImpulse(const NMP::Vector3 &impulse) NM_OVERRIDE;
    void addTorqueImpulse(const NMP::Vector3& torqueImpulse) NM_OVERRIDE;
    void addTorque(const NMP::Vector3& torque) NM_OVERRIDE;
    void addForce(const NMP::Vector3 &force) NM_OVERRIDE;
    void addLinearVelocityChange(const NMP::Vector3& velChange) NM_OVERRIDE;
    void addAngularAcceleration(const NMP::Vector3& angularAcceleration) NM_OVERRIDE;

    float getSKDeviation() const NM_OVERRIDE;
    float getSKDeviationAngle() const NM_OVERRIDE;

  protected:

    float   m_SKDeviation; ///< If SK, then this is the current position deviation from the target.
    float   m_SKDeviationAngle; ///< If SK then this is the current orientation deviation (in radians) from the target.
  };

  class JointJoltPhys : public Joint
  {
  public:

    JointJoltPhys(const PhysicsSixDOFJointDef* const def);

    PhysicsSixDOFJointModifiableLimits&       getModifiableLimits()       { return m_modifiableLimits; }
    const PhysicsSixDOFJointModifiableLimits& getModifiableLimits() const { return m_modifiableLimits; }

    void clampToLimits(NMP::Quat& orientation, float limitFrac, const NMP::Quat* origQ) const NM_OVERRIDE;
    void expandLimits(const NMP::Quat& orientation) NM_OVERRIDE;
    void scaleLimits(float scaleFactor) NM_OVERRIDE;
    void resetLimits() NM_OVERRIDE;

#if defined(MR_OUTPUT_DEBUGGING)
    virtual uint32_t serializeTxFrameData(void* outputBuffer, uint32_t outputBufferSize) const NM_OVERRIDE;
    void updateSerializeTxFrameData();
#endif // MR_OUTPUT_DEBUGGING

  protected:
#if defined(MR_OUTPUT_DEBUGGING)
    PhysicsSixDOFJointFrameData   m_serializeTxFrameData; // Copy of data to be serialized for debug render.
#endif // MR_OUTPUT_DEBUGGING

    PhysicsSixDOFJointModifiableLimits m_modifiableLimits; // Copy of joint limits that can be modified at runtime.
    const PhysicsSixDOFJointDef*    m_def;    // Pointer to jointDef (owned by parent rig).
  };

  /// Indicates if this is a jointed or articulation rig
  Type getType() const {return m_type;}

  /// Will return a pointer only if this is of type TYPE_ARTICULATED
  class PhysicsRigJoltPhysArticulation* getPhysicsRigJoltPhysArticulation();

  /// Will return a pointer only if this is of type TYPE_JOINTED
  class PhysicsRigJoltPhysJointed* getPhysicsRigJoltPhysJointed();

  /// Returns the unique identified for this rig
  int32_t getRigID() const { return m_rigID; }

  void setKinematicPos(const NMP::Vector3& pos) NM_OVERRIDE;

  float getMaxSKDeviation() const NM_OVERRIDE;

  void requestJointProjectionParameters(int iterations, float linearTolerance, float angularTolerance) NM_OVERRIDE;

  /// Get the Jolt Physics specific scene.
  PhysicsSceneJoltPhys* getPhysicsSceneJoltPhys() const { return (PhysicsSceneJoltPhys*)m_physicsScene; };

  /// This adds the mask to the filter data on all the parts
  void addQueryFilterFlagToParts(uint32_t word0, uint32_t word1, uint32_t word2, uint32_t word3);

  /// \name Gets the Jolt Physics specific part given the part index
  /// @{
  PartJoltPhys* getPartJoltPhys(uint32_t partIndex);
  const PartJoltPhys* getPartJoltPhys(uint32_t partIndex) const;
  /// @}

  void receiveImpulse(
    int32_t inputPartIndex,
    const NMP::Vector3& position,
    const NMP::Vector3& direction,
    float localMagnitude,
    float localAngularMultiplier,
    float localResponseRatio,
    float fullBodyMagnitude,
    float fullBodyAngularMultiplier,
    float fullBodyLinearMultiplier,
    float fullBodyResponseRatio,
    bool positionInWorldSpace,
    bool directionInWorldSpace,
    bool applyAsVelocityChange) NM_OVERRIDE;

  void receiveTorqueImpulse(
    int32_t inputPartIndex,
    const NMP::Vector3& direction,
    float localMagnitude,
    float localResponseRatio,
    float fullBodyMagnitude,
    float fullBodyResponseRatio,
    bool directionInWorldSpace,
    bool applyAsVelocityChange);

  virtual void setSkinWidthIncrease(uint32_t partIndex, float skinWidthIncrease) NM_OVERRIDE;

  void scaleFrictionOnPart(const int32_t partIndex, const float frictionScale) NM_OVERRIDE;

protected:
  /// This should be called internally in the pre-physics update
  void updateRegisteredJoints();

  /// Position we move the actors to when we disable the body (and make it kinematic)
  NMP::Matrix34 m_kinematicPose;

  /// The desired (in this frame) iterations for resolving joint separation. Will be reset after the physics step.
  int m_desiredJointProjectionIterations;
  /// The desired (in this frame) distance tolerance for resolving joint separation. Will be reset after the physics step.
  float m_desiredJointProjectionLinearTolerance;
  /// The desired (in this frame) angular tolerance for resolving joint separation. Will be reset after the physics step.
  float m_desiredJointProjectionAngularTolerance;

private:

  /// Indicates the Jolt Physics rig type - jointed or articulated
  Type m_type;

  /// Used to keep track of which rig instance is which, e.g. so you can detect a different
  /// character's collision but ignore your own
  int32_t m_rigID; 
};

} // namespace MR

//----------------------------------------------------------------------------------------------------------------------
#endif // MR_PHYSICS_RIG_JOLTPHYSJOINTED_H
//----------------------------------------------------------------------------------------------------------------------
