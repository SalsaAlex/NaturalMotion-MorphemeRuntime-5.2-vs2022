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
#ifndef MR_PHYSICS_RIG_JOLTPHYSJOINTED_H
#define MR_PHYSICS_RIG_JOLTPHYSJOINTED_H
//----------------------------------------------------------------------------------------------------------------------
#include "physics/JoltPhys/mrPhysicsRigJoltPhys.h"
#include "physics/JoltPhys/mrJoltPhysIncludes.h"
#include "physics/JoltPhys/mrJoltPhysConfigure.h"
//----------------------------------------------------------------------------------------------------------------------

namespace MR
{

class PhysicsRigDef;
class PhysicsScenePhysX3;
class AnimRigDef;

//----------------------------------------------------------------------------------------------------------------------
/// \class MR::PhysicsRigJoltPhysJointed
/// \brief Container for the Jolt Physics implementation of a jointed physics rig
//----------------------------------------------------------------------------------------------------------------------
class PhysicsRigJoltPhysJointed : public PhysicsRigJoltPhys
{
public:
  //--------------------------------------------------------------------------------------------------------------------
  class PartJoltPhysJointed : public PartJoltPhys
  {
    friend class PhysicsRigJoltPhysJointed;

  public:

    //-----------------------------------------------------------------------------------------------
    // The following member functions make up the implementation of the engine independent base class
    //-----------------------------------------------------------------------------------------------

    NMP::Vector3  getPosition() const NM_OVERRIDE;
    void          setPosition(const NMP::Vector3& p) NM_OVERRIDE;
    NMP::Quat     getQuaternion() const NM_OVERRIDE;
    void          setQuaternion(const NMP::Quat& q) NM_OVERRIDE;

    NMP::Matrix34 getTransform() const NM_OVERRIDE;
    void          setTransform(const NMP::Matrix34& tm) NM_OVERRIDE;
    void          moveTo(const NMP::Matrix34& t) NM_OVERRIDE;

    NMP::Vector3  getVel() const NM_OVERRIDE;
    void          setVel(const NMP::Vector3& v) NM_OVERRIDE;
    NMP::Vector3  getAngVel() const NM_OVERRIDE;
    void          setAngVel(const NMP::Vector3& r) NM_OVERRIDE;
    NMP::Vector3  getVelocityAtPoint(const NMP::Vector3& point) const NM_OVERRIDE;

    NMP::Vector3  getLinearMomentum() const NM_OVERRIDE;
    NMP::Vector3  getAngularMomentum() const NM_OVERRIDE;
    float         getMass() const NM_OVERRIDE;

    NMP::Vector3  getMassSpaceInertiaTensor() const NM_OVERRIDE;
    NMP::Matrix34 getGlobalInertiaTensor() const NM_OVERRIDE;

    NMP::Vector3  getCOMPosition() const NM_OVERRIDE;
    NMP::Matrix34 getCOMOffsetLocal() const NM_OVERRIDE;

    void          makeKinematic(bool kinematic, float massMultiplier, bool enableConstraint) NM_OVERRIDE;
    bool          isKinematic() const NM_OVERRIDE;

    void          enableCollision(bool enable) NM_OVERRIDE;
    bool          getCollisionEnabled() const NM_OVERRIDE;

    bool          storeState(MR::PhysicsSerialisationBuffer& savedState) NM_OVERRIDE;
    bool          restoreState(MR::PhysicsSerialisationBuffer& savedState) NM_OVERRIDE;

#if defined(MR_OUTPUT_DEBUGGING)
    uint32_t serializeTxPersistentData(uint16_t nameToken, uint32_t objectID, void* outputBuffer, uint32_t outputBufferSize) const NM_OVERRIDE;
    uint32_t serializeTxFrameData(void* outputBuffer, uint32_t outputBufferSize) const NM_OVERRIDE;
#endif // MR_OUTPUT_DEBUGGING

  protected:
    enum DirtyFlags
    {
      kDirty_GlobalPose = (1 << 0),
      kDirty_LinearVel = (1 << 1),
      kDirty_AngularVel = (1 << 2),
      kDirty_Collision = (1 << 3),
      kDirty_BodyFlags = (1 << 4),
      kDirty_KinematicTarget = (1 << 5),
    };

    struct CachedData
    {
      NMP::Vector3  COMPosition;
      NMP::Matrix34 COMOffsetLocal;
      NMP::Matrix34 globalPose;
      NMP::Matrix34 kinematicTarget;
      NMP::Vector3 linearVel;
      NMP::Vector3 angularVel;
      float mass;
      uint16_t bodyFlags;
      bool collisionOn;
    };

    /// \name protected as the part should only be created etc by the relevant physics rig
    /// @{
    PartJoltPhysJointed();
    PartJoltPhysJointed(const PartJoltPhysJointed& other);
    ~PartJoltPhysJointed();
    PartJoltPhysJointed& operator=(const PartJoltPhysJointed& other);
    /// @}

    // The cache is used to reduce the number of costly calls to the Jolt Physics API
    void generateCachedValues();

    // Apply the cached modified values.
    void applyModifiedValues();

    void updateCOMPosition();

    const CachedData& getCachedData() { return m_cache; }
    void setCachedData(const CachedData& cache) { m_cache = cache; }

    CachedData m_cache;
    uint32_t   m_dirtyFlags;
  };
  //--------------------------------------------------------------------------------------------------------------------

  //--------------------------------------------------------------------------------------------------------------------
  class JointJoltPhysJointed : public JointJoltPhys
  {
    friend class PhysicsRigJoltPhysJointed;

  public:
    JointJoltPhysJointed(const PhysicsSixDOFJointDef* const def);

    //-----------------------------------------------------------------------------------------------
    // The following member functions make up the implementation of the engine independent base class
    //-----------------------------------------------------------------------------------------------

#if defined(MR_OUTPUT_DEBUGGING)
    virtual uint32_t serializeTxPersistentData(
      const MR::PhysicsJointDef* jointDef,
      uint16_t stringToken,
      void* outputBuffer,
      uint32_t outputBufferSize) const NM_OVERRIDE;
#endif // MR_OUTPUT_DEBUGGING

    //-----------------------------------------------------------------------------------------------
    // The following member functions are specific to this class
    //-----------------------------------------------------------------------------------------------

    /// \name Get the joint drive properties
    /// @{
    float getMaxTwistDamping() const { return m_maxTwistDamping; }
    float getMaxTwistStrength() const { return m_maxTwistStrength; }
    float getMaxSwingDamping() const { return m_maxSwingDamping; }
    float getMaxSwingStrength() const { return m_maxSwingStrength; }
    float getMaxSlerpDamping() const { return m_maxSlerpDamping; }
    float getMaxSlerpStrength() const { return m_maxSlerpStrength; }
    /// @}

    /// Returns the joint rotation between the two parts (using the joint frames)
    NMP::Quat getRotation(const MR::PhysicsJointDef* jointDef, const NMP::Matrix34& part1TM, const NMP::Matrix34& part2TM) const;

    /// Enables/disables the joint limit
    void enableLimit(bool enable);

    virtual void writeLimits() NM_OVERRIDE;

    /// \name Set the joint drive properties
    /// @{
    void setDriveStrength(float twistStrength, float swingStrength, float slerpStrength);
    void setDriveDamping(float twistDamping, float swingDamping, float slerpDamping);
    void setDriveOrientation(const NMP::Quat &quat);
    /// @}

    // TODO: Rationalise these with the above setDriveStrength etc, if possible
    void setStrength(float strength) NM_OVERRIDE;
    float getStrength() const NM_OVERRIDE;

    void setDamping(float damping) NM_OVERRIDE;
    float getDamping() const NM_OVERRIDE;

    NMP::Quat getTargetOrientation() NM_OVERRIDE;
    void setTargetOrientation(const NMP::Quat &orientation) NM_OVERRIDE;

    /// Store the state internal and Jolt Physics (not currently implemented)
    bool storeState(MR::PhysicsSerialisationBuffer& savedState);
    /// Restore the state internal and Jolt Physics (not currently implemented)
    bool restoreState(MR::PhysicsSerialisationBuffer& savedState);

  protected:
    enum DirtyFlags
    {
      kDirty_SwingDrive = (1 << 0),
      kDirty_TwistDrive = (1 << 1),
      kDirty_SlerpDrive = (1 << 2),
      kDirty_Limits    =  (1 << 3),
      kDirty_DriveOrientation = (1 << 4)
    };

    struct CachedData
    {
      CachedData()
      {
      }

      NMP::Quat driveOrientation;
    };

    /// \name Internal functions for manipulating the cache
    /// @{
    const CachedData& getCachedData() { return m_cache; }
    void setCachedData(const CachedData& cache) { m_cache = cache; }
    /// @}

    ///
    void generateCachedValues();

    ///
    void applyModifiedValues();

    float        m_maxTwistDamping;
    float        m_maxTwistStrength;
    float        m_maxSwingDamping;
    float        m_maxSwingStrength;
    float        m_maxSlerpDamping;
    float        m_maxSlerpStrength;

    bool         m_limitsEnabled;
    CachedData   m_cache;
    uint32_t     m_dirtyFlags;
  };
  //--------------------------------------------------------------------------------------------------------------------


  /// \brief Get the memory required to create an instance from the supplied PhysicsRigDef.
  static NMP::Memory::Format getMemoryRequirements(PhysicsRigDef* phyiscsRigDef);

  //-----------------------------------------------------------------------------------------------
  // The following member functions make up the implementation of the engine independent base class
  //-----------------------------------------------------------------------------------------------

  bool term() NM_OVERRIDE;

  void updatePrePhysics(float timeStep) NM_OVERRIDE;
  void updatePostPhysics(float timeStep) NM_OVERRIDE;

  /// Get the joint orientation as a quaternion
  NMP::Quat getJointQuat(uint32_t jointIndex) NM_OVERRIDE;

  void applySoftKeyframing(
    const NMP::DataBuffer& targetBuffer,
    const NMP::DataBuffer& previousTargetBuffer,
    const NMP::DataBuffer& fallbackBuffer,
    const NMP::Matrix34&   worldRoot,
    const NMP::Matrix34&   previousWorldRoot,
    bool                   enableCollision,
    bool                   enableJointLimits,
    bool                   preserveMomentum,
    float                  externalJointCompliance,
    float                  gravityCompensationFrac,
    float                  dt,
    float                  weight,
    float                  maxAccel,
    float                  maxAngAccel,
    const PartChooser&     partChooser) NM_OVERRIDE;

  virtual void applyHardKeyframing(
    const NMP::DataBuffer& targetBuffer,
    const NMP::DataBuffer* previousTargetBuffer,
    const NMP::DataBuffer& fallbackBuffer,
    const NMP::Matrix34&   worldRoot,
    const NMP::Matrix34*   previousWorldRoot,
    bool                   enableCollision,
    float                  massMultiplier,
    bool                   enableConstraint,
    float                  dt,
    const PartChooser&     partChooser) NM_OVERRIDE;

  void applyActiveAnimation(uint32_t jointIndex, const NMP::Quat& targetQuat, bool makeChildDynamic) NM_OVERRIDE;

  void applyActiveAnimation(
    const NMP::DataBuffer& targetBuffer,
    const NMP::DataBuffer& fallbackBuffer,
    float                  strengthMultiplier,
    float                  dampingMultiplier,
    float                  internalCompliance,
    float                  externalCompliance,
    bool                   enableJointLimits,
    const JointChooser&    jointChooser,
    float                  limitClampFraction) NM_OVERRIDE;

  /// Support for sleeping control
  void disableSleeping() NM_OVERRIDE;
  void reenableSleeping() NM_OVERRIDE {};

  /// \n\me These accessors are helpers to make client code that uses the Jolt Physics rig clearer
  /// @{
  PartJoltPhysJointed *getPartJoltPhysJointed(uint32_t index) {return (PartJoltPhysJointed*)getPart(index);};
  const PartJoltPhysJointed *getPartJoltPhysJointed(uint32_t index) const {return (PartJoltPhysJointed*)getPart(index);};
  JointJoltPhysJointed *getJointJoltPhysJointed(uint32_t index) {return (JointJoltPhysJointed*)getJoint(index);};
  const JointJoltPhysJointed *getJointJoltPhysJointed(uint32_t index) const {return (JointJoltPhysJointed*)getJoint(index);};
  /// @}

protected:
  //-----------------------------------------------------------------------------------------------
  // The following member functions make up the implementation of the engine independent base class
  //-----------------------------------------------------------------------------------------------
  void makeKinematic(bool moveToKinematicPos) NM_OVERRIDE;
  void makeDynamic() NM_OVERRIDE;
  void addToScene() NM_OVERRIDE;
  void removeFromScene() NM_OVERRIDE;
  void restoreAllJointDrivesToDefault() NM_OVERRIDE;

  /// Physics rig constructor - only called internally
  PhysicsRigJoltPhysJointed(PhysicsSceneJoltPhys*physicsScene);

  /// Generate a cache of body and joint values to reduce the number of costly calls to the Jolt Physics API
  void generateCachedValues();

  /// Apply the cached modified values in one shot.
  void applyModifiedValues();

  /// Bit mask using MR::GameGroup, possibly some game-specific bits too, indicating the type that this rig claims
  /// to be. Would normally be at least 1 << MR::GameGroup::GROUP_CHARACTER_PART
  int32_t             m_collisionTypeMask;
  /// Bit mask indicating what this character doesn't collide with. 
  /// Will normally be at least (1 << GROUP_NON_COLLIDABLE) | (GROUP_CHARACTER_CONTROLLER)
  int32_t             m_collisionIgnoreMask;

  static const float s_limitContactAngle;// this is like a limit "skin width"
};

} // namespace MR

//----------------------------------------------------------------------------------------------------------------------
#endif // MR_PHYSICS_RIG_JOLTPHYSJOINTED_H
//----------------------------------------------------------------------------------------------------------------------
