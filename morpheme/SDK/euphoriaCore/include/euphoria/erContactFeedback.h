// Copyright (c) 2011 NaturalMotion.  All Rights Reserved.
// Not to be copied, adapted, modified, used, distributed, sold,
// licensed or commercially exploited in any manner without the
// written consent of NaturalMotion.  
//
// All non public elements of this software are the confidential
// information of NaturalMotion and may not be disclosed to any
// person nor used for any purpose not expressly approved by
// NaturalMotion in writing.

#ifndef NM_CONTACTFEEDBACK_H
#define NM_CONTACTFEEDBACK_H
#include "mrJoltPhys.h"
#include "physics/mrPhysicsScene.h"

namespace ER
{
//----------------------------------------------------------------------------------------------------------------------
/// Base class so that users can register their own handler for collisions that are reported to and
/// registered by Euphoria.
//----------------------------------------------------------------------------------------------------------------------
class UserContactHandler
{
public:

  virtual ~UserContactHandler() {}

  /// This will be called when there is an active contact.
  virtual void onContact(
	const JPH::Body& inBody1, const JPH::Body& inBody2r) = 0;
};

// This class allows euphoria to access contact forces resulting from character collisions
class ContactFeedback : public JPH::ContactListener
{
public:
  static void initialise(MR::PhysicsScene* physicsScene);
  static void deinitialise(MR::PhysicsScene* physicsScene);

  static void setUserContactHandler(UserContactHandler* handler);

  // Debug Draw
  static bool getDrawContactsFlag();
  static void setDrawContactsFlag(bool drawEnabled);
  static bool getDrawDetailedContactsFlag();
  static void setDrawDetailedContactsFlag(bool drawEnabled);

protected:
  UserContactHandler* m_userContactHandler;

protected:
	// See: ContactListener
	JPH::ValidateResult	OnContactValidate(const JPH::Body& inBody1, const JPH::Body& inBody2,
		JPH::RVec3Arg inBaseOffset, const JPH::CollideShapeResult& inCollisionResult)
		NM_OVERRIDE;

	void OnContactAdded(const JPH::Body& inBody1,
			const JPH::Body& inBody2, const JPH::ContactManifold& inManifold,
			JPH::ContactSettings& ioSettings) NM_OVERRIDE;

	void OnContactPersisted(const JPH::Body& inBody1,
		const JPH::Body& inBody2, const JPH::ContactManifold& inManifold,
		JPH::ContactSettings& ioSettings) NM_OVERRIDE;

	void OnContactRemoved(const JPH::SubShapeIDPair& inSubShapePair) NM_OVERRIDE;
}

#endif // NM_CONTACTFEEDBACK_H
