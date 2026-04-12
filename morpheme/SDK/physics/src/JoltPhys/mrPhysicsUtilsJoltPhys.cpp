// Copyright (c) 2009 NaturalMotion.  All Rights Reserved.
// Not to be copied, adapted, modified, used, distributed, sold,
// licensed or commercially exploited in any manner without the
// written consent of NaturalMotion.  
//
// All non public elements of this software are the confidential
// information of NaturalMotion and may not be disclosed to any
// person nor used for any purpose not expressly approved by
// NaturalMotion in writing.

//----------------------------------------------------------------------------------------------------------------------
#include "mrJoltPhys.h"
#include "mrPhysicsUtilsJoltPhys.h"


namespace MR
{

class NMCastShapeCollector : public JPH::CastShapeCollector
{
public:
    NMCastShapeCollector(JPH::PhysicsSystem* scene, bool closest, uint32_t ignoremask = 0)
        : m_pScene(scene), m_ignoreMask(ignoremask), m_bClosest(closest) {
    }

    void AddHit(const ResultType& inResult) NM_OVERRIDE
    {
        CastData::perbodydata entry;
        entry.body = m_pScene->GetBodyLockInterface().TryGetBody(inResult.mBodyID2);
        entry.normal = -inResult.mPenetrationAxis.Normalized();
        entry.contactpoint = inResult.mContactPointOn1;
        entry.fraction = inResult.mFraction;

        if (m_ignoreMask & (1 << entry.body->GetObjectLayer()))
            return;

        if (!m_vCastResult.hits.empty())
        {
            if (m_bClosest && (entry.fraction > m_vCastResult.hits[0].fraction))
                return;
        }

        m_vCastResult.hits.push_back(entry);
    }

    CastData m_vCastResult;
private:
    JPH::PhysicsSystem* m_pScene;
    bool m_bClosest; //return only the closest hit
    uint32_t m_ignoreMask; //1 << MR::NMPhysLayers::
};

class NMCastRayCollector : public JPH::CastRayCollector
{
public:
    NMCastRayCollector(JPH::PhysicsSystem* scene, bool closest, JPH::RayCast ray, uint32_t ignoremask = 0)
        : m_pScene(scene), m_ignoreMask(ignoremask), m_bClosest(closest), m_joltRay(ray) {
    }

    void AddHit(const ResultType& inResult) NM_OVERRIDE
    {
        CastData::perbodydata entry;
        entry.body = m_pScene->GetBodyLockInterface().TryGetBody(inResult.mBodyID);
        entry.normal = entry.body->GetShape()->GetSurfaceNormal(inResult.mSubShapeID2, m_joltRay.GetPointOnRay(inResult.mFraction));
        entry.normal = entry.body->GetWorldTransform() * entry.normal;
        entry.contactpoint = m_joltRay.GetPointOnRay(inResult.mFraction);
        entry.fraction = inResult.mFraction;

        if (m_ignoreMask & (1 << entry.body->GetObjectLayer()))
            return;

        if (m_bClosest && (entry.fraction > m_vCastResult.hits[0].fraction))
            return;

        m_vCastResult.hits.push_back(entry);
    }

    CastData m_vCastResult;
private:
    JPH::PhysicsSystem* m_pScene;
    bool m_bClosest; //return only the closest hit
    JPH::RayCast m_joltRay;
    uint32_t m_ignoreMask; //1 << MR::NMPhysLayers::
};


//sweep axis aligned shape through world
CastData SweepAAShapeVsWorld(JPH::PhysicsSystem* scene, JPH::Shape* shape, JPH::Vec3 pos, JPH::Vec3 dir, float dist, uint32_t ignoremask)
{
    JPH::RShapeCast start(shape,
        JPH::Vec3::sOne(),
        JPH::Mat44::sRotationTranslation(JPH::Quat(0, 0, 0, 1), pos),
        dir);
    JPH::ShapeCastSettings sweepsettings;
    NMCastShapeCollector collector(scene, true, ignoremask);

    scene->GetNarrowPhaseQuery().CastShape(
        start,
        sweepsettings,
        JPH::Vec3::sOne(),
        collector
    );

    return collector.m_vCastResult;
}

//sweep axis aligned shape against body
CastData SweepAAShapeVsBody(JPH::PhysicsSystem* scene, JPH::Shape* shape, JPH::Body* body, JPH::Vec3 pos, JPH::Vec3 dir, float dist, uint32_t ignoremask)
{
    JPH::ShapeCast start(shape,
        JPH::Vec3::sOne(),
        JPH::Mat44::sRotationTranslation(JPH::Quat(0, 0, 0, 1), pos),
        dir);
    JPH::ShapeCastSettings sweepsettings;
    NMCastShapeCollector collector(scene, true, ignoremask);

    JPH::CollisionDispatch::sCastShapeVsShapeWorldSpace(
        start,
        sweepsettings,
        body->GetShape(),
        JPH::Vec3::sOne(),
        JPH::ShapeFilter(),
        body->GetWorldTransform(),
        JPH::SubShapeIDCreator(),
        JPH::SubShapeIDCreator(),
        collector
    );

    return collector.m_vCastResult;
}

//sweep ray through world
CastData SweepRayVsWorld(JPH::PhysicsSystem* scene, JPH::Vec3 pos, JPH::Vec3 dir, float dist, uint32_t ignoremask)
{
    JPH::RRayCast start(pos, dir);
    JPH::RayCastResult sweepresult;
    NMCastRayCollector collector(scene, true, JPH::RayCast(pos, dir), ignoremask);

    scene->GetNarrowPhaseQuery().CastRay(
        start,
        sweepresult
    );

    return collector.m_vCastResult;
}

JPH::CollisionGroup CreateNoCollideGroup(const JPH_nocollidegroup& group)
{
    static uint32_t collisiongroup_id = 0;
    static uint32_t collisionsubgroup_id = 0;
    JPH::Ref<JPH::GroupFilterTable> filtertable = new JPH::GroupFilterTable(group.m_numbodies);

    //yuck these for loops
    for (int i = 0; i < group.m_collidegroups.size(); i++)
    {
        const JPH_nocollide_entry& entry = group.m_collidegroups[i];
        for (int j = 0; j < entry.m_bodies.size(); j++)
        {
            entry.m_bodies[j]->mCollisionGroup.SetSubGroupID(j);
            entry.m_bodies[j]->mCollisionGroup.SetGroupFilter(filtertable);
            for (int k = 0; k < entry.m_bodies.size(); k++)
            {
                if (k == j)
                    continue;
                if(entry.m_enable)
                    filtertable->DisableCollision(j, k);

            }
        }
    }
    
    return JPH::CollisionGroup(filtertable, collisiongroup_id++, collisionsubgroup_id++);
}

} // namespace MR