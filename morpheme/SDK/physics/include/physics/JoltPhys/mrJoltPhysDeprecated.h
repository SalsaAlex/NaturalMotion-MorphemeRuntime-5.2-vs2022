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
#ifndef MR_PHYSX3_DEPRECATED_H
#define MR_PHYSX3_DEPRECATED_H
//----------------------------------------------------------------------------------------------------------------------
#include <vector>
#include "mrJoltPhysIncludes.h"
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
// empty for now
//----------------------------------------------------------------------------------------------------------------------


#ifdef NM_COMPILER_SNC
#pragma diag_push
#pragma diag_suppress=1700 // class "PxArticulationLinkDesc" has virtual functions but non-virtual destructor
#endif

#ifdef NM_COMPILER_SNC
#pragma diag_pop
#endif

#endif // MR_JOLTPHYS_DEPRECATED_H
