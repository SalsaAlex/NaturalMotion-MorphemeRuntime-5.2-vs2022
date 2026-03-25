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
#ifndef MR_PHYSICS_DRIVER_DATA_JOLTPHYS_H
#define MR_PHYSICS_DRIVER_DATA_JOLTPHYS_H
//----------------------------------------------------------------------------------------------------------------------
#include "physics/mrPhysicsRigDef.h"
#include "physics/JoltPhys/mrJoltPhys.h"
//----------------------------------------------------------------------------------------------------------------------

namespace MR
{
//----------------------------------------------------------------------------------------------------------------------
// PhysicsShapeDriverDataJoltPhys
//----------------------------------------------------------------------------------------------------------------------
struct PhysicsShapeDriverDataJoltPhys : public PhysicsDriverData
{
  void locate();
  void dislocate();

  float m_restOffset;
  float m_contactOffset;
};

enum JP_frictioncombinemode
{
	eAVERAGE = 0,
	eMIN,
	eMAX,
	eMULTIPLY
};

//----------------------------------------------------------------------------------------------------------------------
// PhysicsMaterialDriverDataJoltPhys
//----------------------------------------------------------------------------------------------------------------------
struct PhysicsMaterialDriverDataJoltPhys : public PhysicsDriverData
{
  void locate();
  void dislocate();

  float                       m_dynamicFriction;
  JP_frictioncombinemode	  m_frictionCombineMode;
  JP_frictioncombinemode	  m_restitutionCombineMode;
  uint32_t                    m_disableStrongFriction;
};

//----------------------------------------------------------------------------------------------------------------------
// PhysicsActorDriverDataJoltPhys
//----------------------------------------------------------------------------------------------------------------------
struct PhysicsActorDriverDataJoltPhys : public PhysicsDriverData
{
  void locate();
  void dislocate();

  float m_maxContactOffsetIncrease;
};

//----------------------------------------------------------------------------------------------------------------------
// PhysicsBodyDriverDataJoltPhys
//----------------------------------------------------------------------------------------------------------------------
struct PhysicsBodyDriverDataJoltPhys : public PhysicsDriverData
{
  void locate();
  void dislocate();

  float     m_maxAngularVelocity;
  float     m_inertiaSphericalisation;
  uint32_t  m_positionSolverIterationCount;
  uint32_t  m_velocitySolverIterationCount;
};

//----------------------------------------------------------------------------------------------------------------------
// PhysicsJointDriverDataJoltPhys
//----------------------------------------------------------------------------------------------------------------------
struct PhysicsJointDriverDataJoltPhys : public PhysicsDriverData
{
  void locate();
  void dislocate();

  void endianSwap();

  // For the twist/swing drive when jointed
  float m_twistDriveDamping;
  float m_twistDriveSpring;
  float m_swingDriveDamping;
  float m_swingDriveSpring;

  // For the slerp drive when jointed
  float m_slerpDriveDamping;
  float m_slerpDriveSpring;

  // For the drive when articulated
  float m_articulationSpring;
  float m_articulationDamping;

  float m_driveStrengthScale;
  float m_driveDampingScale;
  float m_driveMinDampingScale;
  float m_driveCompensationScale;

  uint32_t m_useSlerpDrive;
  uint32_t m_useAccelerationSprings;
};

//----------------------------------------------------------------------------------------------------------------------
/// \brief Locates all Jolt Physics driver data for a PhysicsRigDef, this must be called after PhysicsRigDef::locate.
//----------------------------------------------------------------------------------------------------------------------
bool locateDriverDataJoltPhys(PhysicsRigDef* physicsRigDef);

//----------------------------------------------------------------------------------------------------------------------
/// \brief Dislocates all Jolt Physics driver data for a PhysicsRigDef, this must be called before PhysicsRigDef::dislocate.
//----------------------------------------------------------------------------------------------------------------------
bool dislocateDriverDataJoltPhys(PhysicsRigDef* physicsRigDef);

//----------------------------------------------------------------------------------------------------------------------
// Inline implementations
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
// PhysicsShapeDriverDataJoltPhys
//----------------------------------------------------------------------------------------------------------------------
inline void PhysicsShapeDriverDataJoltPhys::locate()
{
  NMP::endianSwap(m_restOffset);
  NMP::endianSwap(m_contactOffset);
}

//----------------------------------------------------------------------------------------------------------------------
inline void PhysicsShapeDriverDataJoltPhys::dislocate()
{
  NMP::endianSwap(m_restOffset);
  NMP::endianSwap(m_contactOffset);
}

//----------------------------------------------------------------------------------------------------------------------
// PhysicsMaterialDriverDataJoltPhys
//----------------------------------------------------------------------------------------------------------------------
inline void PhysicsMaterialDriverDataJoltPhys::locate()
{
  NMP::endianSwap(m_dynamicFriction);
  NMP::endianSwap(m_frictionCombineMode);
  NMP::endianSwap(m_restitutionCombineMode);
  NMP::endianSwap(m_disableStrongFriction);
}

//----------------------------------------------------------------------------------------------------------------------
inline void PhysicsMaterialDriverDataJoltPhys::dislocate()
{
  NMP::endianSwap(m_dynamicFriction);
  NMP::endianSwap(m_frictionCombineMode);
  NMP::endianSwap(m_restitutionCombineMode);
  NMP::endianSwap(m_disableStrongFriction);
}

//----------------------------------------------------------------------------------------------------------------------
// PhysicsActorDriverDataJoltPhys
//----------------------------------------------------------------------------------------------------------------------
inline void PhysicsActorDriverDataJoltPhys::locate()
{
  NMP::endianSwap(m_maxContactOffsetIncrease);
}

//----------------------------------------------------------------------------------------------------------------------
inline void PhysicsActorDriverDataJoltPhys::dislocate()
{
  NMP::endianSwap(m_maxContactOffsetIncrease);
}

//----------------------------------------------------------------------------------------------------------------------
// PhysicsBodyDriverDataJoltPhys
//----------------------------------------------------------------------------------------------------------------------
inline void PhysicsBodyDriverDataJoltPhys::locate()
{
  NMP::endianSwap(m_maxAngularVelocity);
  NMP::endianSwap(m_inertiaSphericalisation);
  NMP::endianSwap(m_positionSolverIterationCount);
  NMP::endianSwap(m_velocitySolverIterationCount);
}

//----------------------------------------------------------------------------------------------------------------------
inline void PhysicsBodyDriverDataJoltPhys::dislocate()
{
  NMP::endianSwap(m_maxAngularVelocity);
  NMP::endianSwap(m_inertiaSphericalisation);
  NMP::endianSwap(m_positionSolverIterationCount);
  NMP::endianSwap(m_velocitySolverIterationCount);
}

//----------------------------------------------------------------------------------------------------------------------
// PhysicsJointDriverDataJoltPhys
//----------------------------------------------------------------------------------------------------------------------
inline void PhysicsJointDriverDataJoltPhys::locate()
{
  endianSwap();
}

//----------------------------------------------------------------------------------------------------------------------
inline void PhysicsJointDriverDataJoltPhys::dislocate()
{
  endianSwap();
}

//----------------------------------------------------------------------------------------------------------------------
inline void PhysicsJointDriverDataJoltPhys::endianSwap()
{
  NMP::endianSwap(m_twistDriveDamping);
  NMP::endianSwap(m_twistDriveSpring);

  NMP::endianSwap(m_swingDriveDamping);
  NMP::endianSwap(m_swingDriveSpring);

  NMP::endianSwap(m_slerpDriveDamping);
  NMP::endianSwap(m_slerpDriveSpring);

  NMP::endianSwap(m_articulationSpring);
  NMP::endianSwap(m_articulationDamping);

  NMP::endianSwap(m_driveStrengthScale);
  NMP::endianSwap(m_driveDampingScale);
  NMP::endianSwap(m_driveMinDampingScale);
  NMP::endianSwap(m_driveCompensationScale);

  NMP::endianSwap(m_useSlerpDrive);
  NMP::endianSwap(m_useAccelerationSprings);
}

} // namespace MR

//----------------------------------------------------------------------------------------------------------------------
#endif // MR_PHYSICS_DRIVER_DATA_JOLTPHYS_H
//----------------------------------------------------------------------------------------------------------------------
