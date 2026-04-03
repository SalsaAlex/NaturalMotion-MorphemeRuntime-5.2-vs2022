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
#ifndef MR_JOLTPHYS_H
#define MR_JOLTPHYS_H

// Define this to enable attempted recovery from PhysX making the character explode. Simply resets
// the parts to the origin! Just do it on PC so consoles remain uncluttered for profiling.
#if defined(NM_HOST_WIN32) || defined(NM_HOST_WIN64)
  #define RECOVER_FROM_ERRORS
#endif

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
static inline NMP::Matrix34 nmJPHMat44ToNmMatrix34(const JPH::Mat44 nxM)
{
  NMP::Quat quat = nmJPHQuatToQuat(nxM.GetQuaternion());
  NMP::Matrix34 m = NMP::Matrix34Identity();
  m.fromQuat(quat);
  m.translation().set(nmJPHVec3ToVector3(nxM.GetTranslation()));
  return m;
}
//----------------------------------------------------------------------------------------------------------------------
static inline const JPH::Mat44 nmMatrix34ToJPHMat44(const NMP::Matrix34& m)
{
  JPH::Quat quat = nmQuatToJPHQuat(m.toQuat());
  JPH::Vec3 pos = nmVector3ToJPHVec3(m.translation());
  JPH::Mat44 jm = JPH::Mat44::sRotationTranslation(quat, pos);
  return jm;
}

static inline const JPH::Vec3 getBodyMassSpaceInertiaTensor(const JPH::Body* body)
{
	return body->GetMotionProperties()->GetLocalSpaceInverseInertia().GetDiagonal3().Reciprocal();
}

static inline const JPH::Mat44 getBodyCenterMassLocalPose(const JPH::Body* body)
{
	JPH::Mat44 trans = JPH::Mat44::sIdentity();
	trans.SetTranslation(body->GetShape()->GetCenterOfMass());
	return trans;
}

}

//----------------------------------------------------------------------------------------------------------------------
#endif // MR_JOLTPHYS_H
//----------------------------------------------------------------------------------------------------------------------
