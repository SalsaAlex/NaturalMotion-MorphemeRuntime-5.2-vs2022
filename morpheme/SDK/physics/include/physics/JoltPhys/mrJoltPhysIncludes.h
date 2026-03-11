// Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
// Not to be copied, adapted, modified, used, distributed, sold,
// licensed or commercially exploited in any manner without the
// written consent of NaturalMotion.
//
// All non public elements of this software are the confidential
// information of NaturalMotion and may not be disclosed to any
// person nor used for any purpose not expressly approved by
// NaturalMotion in writing.

#ifndef NM_JOLTPHYSINCLUDES_H
#define NM_JOLTPHYSINCLUDES_H

#include "mrJoltPhysConfigure.h"
#include "NMPlatform/NMPlatform.h"

#include <extern/Jolt/Jolt.h>
#include <extern/Jolt/RegisterTypes.h>
#include <extern/Jolt/Core/Factory.h>
#include <extern/Jolt/Core/TempAllocator.h>
#include <extern/Jolt/Core/JobSystemThreadPool.h>
#include <extern/Jolt/Physics/PhysicsSettings.h>
#include <extern/Jolt/Physics/PhysicsSystem.h>
#include <extern/Jolt/Physics/Collision/EstimateCollisionResponse.h>
#include <extern/Jolt/Physics/Collision/Shape/BoxShape.h>
#include <extern/Jolt/Physics/Collision/Shape/SphereShape.h>
#include <extern/Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <extern/Jolt/Physics/Collision/Shape/MeshShape.h>
#include <extern/Jolt/Physics/Collision/RayCast.h>
#include <extern/Jolt/Physics/Collision/AABoxCast.h>
#include <extern/Jolt/Physics/Collision/CastResult.h>
#include <extern/Jolt/Physics/Collision/ShapeCast.h>
#include <extern/Jolt/Physics/Body/BodyCreationSettings.h>
#include <extern/Jolt/Physics/Body/BodyActivationListener.h>

// NM include
#include "mrJoltPhysDeprecated.h"

#endif // NM_JOLTPHYSINCLUDES_H

