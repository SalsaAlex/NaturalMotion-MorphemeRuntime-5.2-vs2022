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
#ifndef MR_PHYSICS_UTILS_JOLTPHYS_H
#define MR_PHYSICS_UTILS_JOLTPHYS_H
//----------------------------------------------------------------------------------------------------------------------
#include "NMPlatform/NMPlatform.h"
#include "NMPlatform/NMVector3.h"
#include "NMPlatform/NMMatrix34.h"
#include "NMPlatform/NMQuat.h"
#include "NMPlatform/NMHashMap.h"
#include "physics/JoltPhys/mrJoltPhys.h"
#include "mrJoltPhysIncludes.h"

namespace MR
{

    struct CastData
    {
        struct perbodydata //expand as we go on
        {
            JPH::Body* body;
            JPH::Vec3 normal;
            JPH::Vec3 contactpoint; //worldspace. on the surface of the shape being swept
            JPH::SubShapeID subshapeid;
            float fraction; //fraction of the trace
        };
        std::vector<perbodydata> hits;
    };


    //sweep axis aligned shape through world
    CastData SweepAAShapeVsWorld(JPH::PhysicsSystem* scene, JPH::Shape* shape, JPH::Vec3 pos, JPH::Vec3 dir, float dist, uint32_t ignoremask = 0);

    //sweep axis aligned shape against body
    CastData SweepAAShapeVsBody(JPH::PhysicsSystem* scene, JPH::Shape* shape, JPH::Body* body, JPH::Vec3 pos, JPH::Vec3 dir, float dist, uint32_t ignoremask = 0);

    //sweep ray through world
    CastData SweepRayVsWorld(JPH::PhysicsSystem* scene, JPH::Vec3 pos, JPH::Vec3 dir, float dist, uint32_t ignoremask = 0);

    class JPH_nocollide_entry
    {
    public:
        void AddBody(JPH::BodyCreationSettings* body) { m_bodies.push_back(body); }
        std::vector<JPH::BodyCreationSettings*> m_bodies;
        bool m_enable;
    };

    class JPH_nocollidegroup
    {
    public:

        JPH_nocollide_entry& MakeEntry() { return m_collidegroups.emplace_back(JPH_nocollide_entry()); }
        std::vector<JPH_nocollide_entry> m_collidegroups;
        uint32_t m_numbodies; //total number of unique bodies
    };

    JPH::CollisionGroup CreateNoCollideGroup(const JPH_nocollidegroup& group);

} // namespace MR

#endif // MR_PHYSICS_UTILS_JOLTPHYS_h