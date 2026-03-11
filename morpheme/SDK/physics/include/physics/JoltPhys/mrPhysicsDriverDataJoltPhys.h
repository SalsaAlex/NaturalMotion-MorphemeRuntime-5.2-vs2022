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
};

//----------------------------------------------------------------------------------------------------------------------
// PhysicsMaterialDriverDataJoltPhys
//----------------------------------------------------------------------------------------------------------------------
struct PhysicsMaterialDriverDataJoltPhys : public PhysicsDriverData
{
  void locate();
  void dislocate();
};

//----------------------------------------------------------------------------------------------------------------------
// PhysicsActorDriverDataJoltPhys
//----------------------------------------------------------------------------------------------------------------------
struct PhysicsActorDriverDataJoltPhys : public PhysicsDriverData
{
  void locate();
  void dislocate();
};

//----------------------------------------------------------------------------------------------------------------------
// PhysicsBodyDriverDataJoltPhys
//----------------------------------------------------------------------------------------------------------------------
struct PhysicsBodyDriverDataJoltPhys : public PhysicsDriverData
{
  void locate();
  void dislocate();
};

//----------------------------------------------------------------------------------------------------------------------
// PhysicsJointDriverDataJoltPhys
//----------------------------------------------------------------------------------------------------------------------
struct PhysicsJointDriverDataJoltPhys : public PhysicsDriverData
{
  void locate();
  void dislocate();

  void endianSwap();
};

//----------------------------------------------------------------------------------------------------------------------
/// \brief Locates all JoltPhys driver data for a PhysicsRigDef, this must be called after PhysicsRigDef::locate.
//----------------------------------------------------------------------------------------------------------------------
bool locateDriverDataJoltPhys(PhysicsRigDef* physicsRigDef);

//----------------------------------------------------------------------------------------------------------------------
/// \brief Dislocates all JoltPhys driver data for a PhysicsRigDef, this must be called before PhysicsRigDef::dislocate.
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
}

//----------------------------------------------------------------------------------------------------------------------
inline void PhysicsShapeDriverDataJoltPhys::dislocate()
{
}

//----------------------------------------------------------------------------------------------------------------------
// PhysicsMaterialDriverDataJoltPhys
//----------------------------------------------------------------------------------------------------------------------
inline void PhysicsMaterialDriverDataJoltPhys::locate()
{
}

//----------------------------------------------------------------------------------------------------------------------
inline void PhysicsMaterialDriverDataJoltPhys::dislocate()
{
}

//----------------------------------------------------------------------------------------------------------------------
// PhysicsActorDriverDataJoltPhys
//----------------------------------------------------------------------------------------------------------------------
inline void PhysicsActorDriverDataJoltPhys::locate()
{
}

//----------------------------------------------------------------------------------------------------------------------
inline void PhysicsActorDriverDataJoltPhys::dislocate()
{
}

//----------------------------------------------------------------------------------------------------------------------
// PhysicsBodyDriverDataJoltPhys
//----------------------------------------------------------------------------------------------------------------------
inline void PhysicsBodyDriverDataJoltPhys::locate()
{
}

//----------------------------------------------------------------------------------------------------------------------
inline void PhysicsBodyDriverDataJoltPhys::dislocate()
{
}

//----------------------------------------------------------------------------------------------------------------------
// PhysicsJointDriverDataJoltPhys
//----------------------------------------------------------------------------------------------------------------------
inline void PhysicsJointDriverDataJoltPhys::locate()
{
}

//----------------------------------------------------------------------------------------------------------------------
inline void PhysicsJointDriverDataJoltPhys::dislocate()
{
}

//----------------------------------------------------------------------------------------------------------------------
inline void PhysicsJointDriverDataJoltPhys::endianSwap()
{
}

} // namespace MR

//----------------------------------------------------------------------------------------------------------------------
#endif // MR_PHYSICS_DRIVER_DATA_JOLTPHYS_H
//----------------------------------------------------------------------------------------------------------------------
