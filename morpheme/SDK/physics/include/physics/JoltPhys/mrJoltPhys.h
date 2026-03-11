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
#ifndef MR_PHYSX_H
#define MR_PHYSX_H

// Define this to enable attempted recovery from PhysX making the character explode. Simply resets
// the parts to the origin! Just do it on PC so consoles remain uncluttered for profiling.
#if defined(NM_HOST_WIN32) || defined(NM_HOST_WIN64)
  #define RECOVER_FROM_ERRORS
#endif

#define USE_PHYSX_SWEEPS_FOR_CHARACTERx // See MORPH-9039

#include "NMPlatform/NMPlatform.h"
#include "NMPlatform/NMVector3.h"
#include "NMPlatform/NMMatrix34.h"
#include "NMPlatform/NMQuat.h"
#include "mrJoltPhysIncludes.h"

namespace MR
{

//----------------------------------------------------------------------------------------------------------------------
static inline JPH::Vec3 nmVector3ToJPHVec3(const NMP::Vector3& v)
{
  return JPH::Vec3(v.x, v.y, v.z);
}

//----------------------------------------------------------------------------------------------------------------------
static inline NMP::Vector3 nmJPHVec3ToVector3(const JPH::Vec3& v)
{
  return NMP::Vector3(v.GetX(), v.GetY(), v.GetZ());
}

//----------------------------------------------------------------------------------------------------------------------
static inline JPH::Quat nmQuatToJPHQuat(const NMP::Quat& q)
{
  return JPH::Quat(q.x, q.y, q.z, q.w);
}

//----------------------------------------------------------------------------------------------------------------------
static inline NMP::Quat nmJPHQuatToQuat(const JPH::Quat& q)
{
  return NMP::Quat(q.GetX(), q.GetY(), q.GetZ(), q.GetW());
}
//----------------------------------------------------------------------------------------------------------------------
static inline NMP::Matrix34 nmJPHMat44ToNmMatrix34(const JPH::Mat44& nxT)
{
  NMP::Quat quat = nmJPHQuatToQuat(nxT.GetQuaternion());
  NMP::Matrix34 m;
  m.fromQuat(quat);
  m.translation().set(nmJPHVec3ToVector3(nxT.GetTranslation()));
  return m;
}
//----------------------------------------------------------------------------------------------------------------------
static inline JPH::Mat44 nmMatrix34ToPxTransform(const NMP::Matrix34& m)
{
  NMP::Quat quat = m.toQuat();
  JPH::Mat44 mat = JPH::Mat44::sRotationTranslation(nmQuatToJPHQuat(quat), nmVector3ToJPHVec3(m.translation()));
  return mat;
}

}

//----------------------------------------------------------------------------------------------------------------------
#endif // MR_PHYSX_H
//----------------------------------------------------------------------------------------------------------------------
