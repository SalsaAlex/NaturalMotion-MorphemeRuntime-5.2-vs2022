// Copyright (c) 2011 NaturalMotion.  All Rights Reserved.
// Not to be copied, adapted, modified, used, distributed, sold,
// licensed or commercially exploited in any manner without the
// written consent of NaturalMotion.
//
// All non public elements of this software are the confidential
// information of NaturalMotion and may not be disclosed to any
// person nor used for any purpose not expressly approved by
// NaturalMotion in writing.

#include "euphoria/erLimbDef.h"
#include "euphoria/erBodyDef.h"
#include "euphoria/erDebugDraw.h"

namespace ER
{
/// Hands:
///  - Orientation: x out of the back of the hands, y along the fingers
///  - Location: middle of the hand, on the palm surface
///
///  Feet:
///  - Orientation: x out the top of the feet, y along the foot
///  - Location: middle of the foot support area, on the sole surface
///
///  Head:
///  - Orientation: x facing forward, y up
///  - Location: middle of the head
///
///  Spine:
///  - Orientation: x facing forward, y up
///  - Location: middle of the spine part
///
///  Pelvis:
///  - Orientation: x facing forward, y up
///  - Location: middle of the pelvis part

//----------------------------------------------------------------------------------------------------------------------
// Reach limit implementation
//----------------------------------------------------------------------------------------------------------------------
void ReachLimit::draw(MR::InstanceDebugInterface* MR_OUTPUT_DEBUG_ARG(pDebugDrawInst), float MR_OUTPUT_DEBUG_ARG(coneAngle), const NMP::Colour& MR_OUTPUT_DEBUG_ARG(colour)) const
{
#if defined(MR_OUTPUT_DEBUGGING)

  // by default draw with the given reach limit angle
  if (coneAngle == 0.0f)
  {
    coneAngle = m_angle;
  }

  MR_DEBUG_DRAW_CONE_AND_DIAL(
    pDebugDrawInst,
    m_transform.translation(),
    m_transform.zAxis(),
    m_angle,
    m_distance,
    m_transform.zAxis(),
    colour );

#endif // defined(MR_OUTPUT_DEBUGGING)
}

//----------------------------------------------------------------------------------------------------------------------
void ReachLimit::locate()
{
  NMP::endianSwap(m_transform);
  NMP::endianSwap(m_angle);
  NMP::endianSwap(m_distance);
}

//----------------------------------------------------------------------------------------------------------------------
void ReachLimit::dislocate()
{
  NMP::endianSwap(m_transform);
  NMP::endianSwap(m_angle);
  NMP::endianSwap(m_distance);
}

void printboolarray(bool* vals, int count, const char* name)
{
    NMP_STDOUT("%s\n", name);
    for (int i = 0; i < count; i++)
        NMP_STDOUT(" %s\n ", vals[i] ? "true" : "false");
    NMP_STDOUT("\n");
}
void printbool(bool val, const char* name)
{
    NMP_STDOUT("%s : %s\n", name, val ? "true" : "false");
}

void printint(int val, const char* name)
{
    NMP_STDOUT("%s : %d\n", name, val);
}

void printfloat(float val, const char* name)
{
    NMP_STDOUT("%s : %f\n", name, val);
}

void printfloatarray(float* array, int count, const char* name)
{
    NMP_STDOUT("%s\n", name);
    for (int i = 0; i < count; i++)
        NMP_STDOUT(" %f ", array[i]);
    NMP_STDOUT("\n");
}

void printquatarray(NMP::Quat* array, int count, const char* name)
{
    NMP_STDOUT("%s\n", name);
    for (int i = 0; i < count; i++)
        NMP_STDOUT(" %f, %f, %f, %f ", array[i].x, array[i].y, array[i].z, array[i].w );
    NMP_STDOUT("\n");
}

void printmat3x4(NMP::Matrix34 marray, const char* name)
{
    NMP_STDOUT("%s\n", name);

        NMP::Vector3 column1 = marray.r[0];
        NMP::Vector3 column2 = marray.r[1];
        NMP::Vector3 column3 = marray.r[2];
        NMP::Vector3 column4 = marray.r[3];

        NMP_STDOUT(" [ %f, %f, %f ]\n ", column1.x, column1.y, column1.z);
        NMP_STDOUT(" [ %f, %f, %f ]\n ", column2.x, column2.y, column2.z);
        NMP_STDOUT(" [ %f, %f, %f ]\n ", column3.x, column3.y, column3.z);
        NMP_STDOUT(" [ %f, %f, %f ]\n ", column4.x, column4.y, column4.z);

    NMP_STDOUT("\n");
}

#define PRINTFLOAT(x) printfloat(x, #x)
#define PRINTFLOATARRAY(x, count) printfloatarray(x, count, #x)
#define PRINTINT(x) printint(x, #x)
#define PRINTBOOL(x) printbool(x, #x)
#define PRINTBOOLARRAY(x, count) printboolarray(x, count, #x)
#define PRINTMAT3X4(x) printmat3x4(x, #x)
#define PRINTQUATARRAY(x, count) printquatarray(x, count, #x)

//----------------------------------------------------------------------------------------------------------------------
// LimbDef implementation
//----------------------------------------------------------------------------------------------------------------------
void LimbDef::locate()
{
  NMP::endianSwap(m_type);

  NMP::endianSwap(m_rootIndex);
  NMP::endianSwap(m_endIndex);
  NMP::endianSwap(m_baseIndex);
  NMP::endianSwap(m_midIndex);
  NMP::endianSwap(m_numPartsBeforeBase);
  NMP::endianSwap(m_isRootLimb);

  m_reachLimit.locate();

  NMP::endianSwap(m_endOffset);
  NMP::endianSwap(m_rootOffset);
  
  NMP::endianSwap(m_defaultPoseEndRelativeToRoot);
  NMP::endianSwap(m_defaultPoseEndQuatRelativeToRoot);
  NMP::endianSwap(m_defaultPoseLimbLength);
  NMP::endianSwap(m_zeroPoseEndRelativeToRoot);

  NMP::endianSwap(m_hamstring.m_stiffness);
  NMP::endianSwap(m_hamstring.m_thresholdAngle);
  NMP::endianSwap(m_hamstring.m_enabled);
  NMP::endianSwap(m_numJointsInChain);

  uint32_t numBendScaleWeights = m_numJointsInChain + 1;
  REFIX_SWAP_PTR(NMP::Vector3, m_hamstring.m_bendScaleWeights);
  if (m_hamstring.m_enabled)
  {
    NMP::endianSwapArray((float*) m_hamstring.m_bendScaleWeights, numBendScaleWeights * 4);
  }

  REFIX_SWAP_PTR(int, m_physicsRigJointIndices);
  for (uint32_t i = 0; i < m_numJointsInChain; ++i)
  {
    NMP::endianSwap(m_physicsRigJointIndices[i]);
  }

  NMP::endianSwap(m_totalNumParts);
  REFIX_SWAP_PTR(int, m_physicsRigPartIndices);
  for (uint32_t i = 0; i < m_totalNumParts; ++i)
  {
    NMP::endianSwap(m_physicsRigPartIndices[i]);
  }

  NMP::endianSwap(m_numPartsInChain);

  REFIX_SWAP_PTR(char, m_name);

  // IK attrs
  NMP::endianSwap(m_guidePoseWeight);
  NMP::endianSwap(m_neutralPoseWeight);

  REFIX_SWAP_PTR(bool, m_activeGuidePoseJoints);
  REFIX_SWAP_PTR(float, m_positionWeights);
  REFIX_SWAP_PTR(float, m_orientationWeights);
  REFIX_SWAP_PTR(NMP::Quat, m_zeroSwivelPoseQuats);
  REFIX_SWAP_PTR(NMP::Quat, m_negativeSwivelPoseQuats);
  REFIX_SWAP_PTR(NMP::Quat, m_positiveSwivelPoseQuats);

  NMP::endianSwapArray(m_orientationWeights, m_numPartsInChain);

  NMP::endianSwapArray(m_positionWeights, m_numPartsInChain);

  NMP::endianSwapArray(m_zeroSwivelPoseQuats, m_numJointsInChain);
  NMP::endianSwapArray(m_negativeSwivelPoseQuats, m_numJointsInChain);
  NMP::endianSwapArray(m_positiveSwivelPoseQuats, m_numJointsInChain);



  NMP_STDOUT("limb name : %s\n", m_name);
  NMP_STDOUT("limb type : %d\n", m_type);

  PRINTINT(m_rootIndex);
  PRINTINT(m_endIndex);
  PRINTINT(m_baseIndex);
  PRINTINT(m_midIndex);
  PRINTINT(m_numPartsBeforeBase);
  PRINTBOOL(m_isRootLimb);


  PRINTMAT3X4(m_endOffset);
  PRINTMAT3X4(m_rootOffset);

  PRINTMAT3X4(m_defaultPoseEndRelativeToRoot);
  PRINTFLOATARRAY(m_defaultPoseEndQuatRelativeToRoot.getPtr(), 4);
  PRINTFLOAT(m_defaultPoseLimbLength);
  PRINTMAT3X4(m_zeroPoseEndRelativeToRoot);

  PRINTFLOAT(m_hamstring.m_stiffness);
  PRINTFLOAT(m_hamstring.m_thresholdAngle);
  PRINTBOOL(m_hamstring.m_enabled);
  PRINTINT(m_numJointsInChain);

  for (uint32_t i = 0; i < m_numJointsInChain; ++i)
      PRINTINT(m_physicsRigJointIndices[i]);

  for (uint32_t i = 0; i < m_totalNumParts; ++i)
      PRINTINT(m_physicsRigPartIndices[i]);

  PRINTINT(m_numPartsInChain);

  PRINTFLOAT(m_guidePoseWeight);
  PRINTFLOAT(m_neutralPoseWeight);

  PRINTBOOLARRAY(m_activeGuidePoseJoints, m_numJointsInChain);

  PRINTFLOATARRAY(m_orientationWeights, m_numPartsInChain);

  PRINTFLOATARRAY(m_positionWeights, m_numPartsInChain);

  PRINTQUATARRAY(m_zeroSwivelPoseQuats, m_numJointsInChain);
  PRINTQUATARRAY(m_negativeSwivelPoseQuats, m_numJointsInChain);
  PRINTQUATARRAY(m_positiveSwivelPoseQuats, m_numJointsInChain);
}

//----------------------------------------------------------------------------------------------------------------------
void LimbDef::dislocate()
{
  // IK attrs
  NMP::endianSwap(m_guidePoseWeight);
  NMP::endianSwap(m_neutralPoseWeight);

  NMP::endianSwapArray(m_orientationWeights, m_numPartsInChain);
  NMP::endianSwapArray(m_positionWeights, m_numPartsInChain);
  NMP::endianSwapArray(m_zeroSwivelPoseQuats, m_numJointsInChain);
  NMP::endianSwapArray(m_negativeSwivelPoseQuats, m_numJointsInChain);
  NMP::endianSwapArray(m_positiveSwivelPoseQuats, m_numJointsInChain);

  UNFIX_SWAP_PTR(bool, m_activeGuidePoseJoints);
  UNFIX_SWAP_PTR(float, m_positionWeights);
  UNFIX_SWAP_PTR(float, m_orientationWeights);
  UNFIX_SWAP_PTR(NMP::Quat, m_zeroSwivelPoseQuats);
  UNFIX_SWAP_PTR(NMP::Quat, m_negativeSwivelPoseQuats);
  UNFIX_SWAP_PTR(NMP::Quat, m_positiveSwivelPoseQuats);

  NMP::endianSwap(m_type);
  NMP::endianSwap(m_rootIndex);
  NMP::endianSwap(m_endIndex);
  NMP::endianSwap(m_baseIndex);
  NMP::endianSwap(m_midIndex);
  NMP::endianSwap(m_numPartsBeforeBase);
  NMP::endianSwap(m_isRootLimb);

  m_reachLimit.dislocate();

  NMP::endianSwap(m_endOffset);
  NMP::endianSwap(m_rootOffset);

  NMP::endianSwap(m_defaultPoseEndRelativeToRoot);
  NMP::endianSwap(m_defaultPoseEndQuatRelativeToRoot);
  NMP::endianSwap(m_defaultPoseLimbLength);
  NMP::endianSwap(m_zeroPoseEndRelativeToRoot);

  NMP::endianSwap(m_hamstring.m_stiffness);
  NMP::endianSwap(m_hamstring.m_thresholdAngle);
  NMP::endianSwap(m_hamstring.m_enabled);

  uint32_t numBendScaleWeights = m_numJointsInChain + 1;
  if (m_hamstring.m_enabled)
  {
    NMP::endianSwapArray((float*) m_hamstring.m_bendScaleWeights, numBendScaleWeights * 4);
    UNFIX_SWAP_PTR(NMP::Vector3, m_hamstring.m_bendScaleWeights);
  }

  for (uint32_t i = 0; i < m_numJointsInChain; ++i)
  {
    NMP::endianSwap(m_physicsRigJointIndices[i]);
  }

  UNFIX_SWAP_PTR(int, m_physicsRigJointIndices);
  NMP::endianSwap(m_numJointsInChain);

  for (uint32_t i = 0; i < m_totalNumParts; ++i)
  {
    NMP::endianSwap(m_physicsRigPartIndices[i]);
  }

  UNFIX_SWAP_PTR(int, m_physicsRigPartIndices);
  NMP::endianSwap(m_totalNumParts);

  NMP::endianSwap(m_numPartsInChain);

  UNFIX_SWAP_PTR(char, m_name);

  // Null the pointer to ensure binary invariance
  m_bodyDef = NULL;
}

//----------------------------------------------------------------------------------------------------------------------
NMP::Matrix34 LimbDef::getRootToEndTransform(const NMP::Matrix34* TMs) const
{
  // The root and end parts in world space after the offset
  NMP::Matrix34 r, e;
  r.multiply(m_rootOffset, TMs[m_rootIndex]);
  e.multiply(m_endOffset,  TMs[m_endIndex]);
  // want e in r's space
  r.invertFast();
  NMP::Matrix34 result = e * r;
  result.orthonormalise(); // Needed to prevent euphoria validation failing
  return result;
}

}
