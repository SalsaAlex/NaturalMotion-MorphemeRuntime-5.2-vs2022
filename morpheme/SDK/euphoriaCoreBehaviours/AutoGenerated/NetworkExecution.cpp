/*
 * Copyright (c) 2026 NaturalMotion Ltd. All rights reserved.
 *
 * Not to be copied, adapted, modified, used, distributed, sold,
 * licensed or commercially exploited in any manner without the
 * written consent of NaturalMotion.
 *
 * All non public elements of this software are the confidential
 * information of NaturalMotion and may not be disclosed to any
 * person nor used for any purpose not expressly approved by
 * NaturalMotion in writing.
 *
 */

//----------------------------------------------------------------------------------------------------------------------
//                                  This file is auto-generated
//----------------------------------------------------------------------------------------------------------------------

#include "NetworkDescriptor.h"
#include "euphoria/erModule.h"
#include "NMPlatform/NMTimer.h"

#include "AimBehaviourInterface.h"
#include "AnimateBehaviourInterface.h"
#include "ArmsBraceBehaviourInterface.h"
#include "ArmsPlacementBehaviourInterface.h"
#include "ArmsWindmillBehaviourInterface.h"
#include "BalanceBehaviourInterface.h"
#include "BalancePoserBehaviourInterface.h"
#include "CharacteristicsBehaviourInterface.h"
#include "EuphoriaRagdollBehaviourInterface.h"
#include "EyesBehaviourInterface.h"
#include "FreeFallBehaviourInterface.h"
#include "HazardAwarenessBehaviourInterface.h"
#include "HeadAvoidBehaviourInterface.h"
#include "HeadDodgeBehaviourInterface.h"
#include "HoldActionBehaviourInterface.h"
#include "HoldBehaviourInterface.h"
#include "IdleAwakeBehaviourInterface.h"
#include "LegsPedalBehaviourInterface.h"
#include "LookBehaviourInterface.h"
#include "ObserveBehaviourInterface.h"
#include "PropertiesBehaviourInterface.h"
#include "ProtectBehaviourInterface.h"
#include "ReachForBodyBehaviourInterface.h"
#include "ReachForWorldBehaviourInterface.h"
#include "ShieldActionBehaviourInterface.h"
#include "ShieldBehaviourInterface.h"
#include "SitBehaviourInterface.h"
#include "SittingBodyBalance.h"
#include "UserHazardBehaviourInterface.h"
#include "WritheBehaviourInterface.h"
#include "Arm.h"
#include "ArmAim.h"
#include "ArmBrace.h"
#include "ArmGrab.h"
#include "ArmHold.h"
#include "ArmHoldingSupport.h"
#include "ArmPlacement.h"
#include "ArmPose.h"
#include "ArmReachForBodyPart.h"
#include "ArmReachForWorld.h"
#include "ArmReachReaction.h"
#include "ArmSittingStep.h"
#include "ArmSpin.h"
#include "ArmStandingSupport.h"
#include "ArmStep.h"
#include "ArmSwing.h"
#include "ArmWrithe.h"
#include "BalanceAssistant.h"
#include "BalanceBehaviourFeedback.h"
#include "BalanceManagement.h"
#include "BalancePoser.h"
#include "BodyBalance.h"
#include "BodyFrame.h"
#include "BodySection.h"
#include "BraceChooser.h"
#include "EnvironmentAwareness.h"
#include "FreeFallManagement.h"
#include "Grab.h"
#include "GrabDetection.h"
#include "HazardManagement.h"
#include "HazardResponse.h"
#include "Head.h"
#include "HeadAim.h"
#include "HeadAvoid.h"
#include "HeadDodge.h"
#include "HeadEyes.h"
#include "HeadPoint.h"
#include "HeadPose.h"
#include "HeadSupport.h"
#include "ImpactPredictor.h"
#include "Leg.h"
#include "LegBrace.h"
#include "LegPose.h"
#include "LegReachReaction.h"
#include "LegSittingSupport.h"
#include "LegSpin.h"
#include "LegStandingSupport.h"
#include "LegStep.h"
#include "LegSwing.h"
#include "LegWrithe.h"
#include "MyNetwork.h"
#include "PositionCore.h"
#include "RandomLook.h"
#include "ReachForBody.h"
#include "RotateCore.h"
#include "SceneProbes.h"
#include "ShieldManagement.h"
#include "Spine.h"
#include "SpineAim.h"
#include "SpineControl.h"
#include "SpinePose.h"
#include "SpineReachReaction.h"
#include "SpineSupport.h"
#include "SpineWrithe.h"
#include "StaticBalance.h"
#include "SteppingBalance.h"
#include "SupportPolygon.h"
//----------------------------------------------------------------------------------------------------------------------
#define TIMETYPE float
TIMETYPE g_profilingTimerFraction = TIMETYPE(1.0); // amount to update the timings by each update - for a rolling average
TIMETYPE g_profilingTotalTimerFraction = TIMETYPE(1.0); // amount to update the total timings by each update - for a rolling average
namespace NM_BEHAVIOUR_LIB_NAMESPACE
{

void MyNetwork::executeNetworkUpdate(float timeStep)
{
  {
    CharacteristicsBehaviourInterface* module_characteristicsBehaviourInterface = (CharacteristicsBehaviourInterface*)getModule(NetworkManifest::characteristicsBehaviourInterface);
    if (isEnabled(NetworkManifest::characteristicsBehaviourInterface))
    {
      module_characteristicsBehaviourInterface->update(timeStep);
    }
    PropertiesBehaviourInterface* module_propertiesBehaviourInterface = (PropertiesBehaviourInterface*)getModule(NetworkManifest::propertiesBehaviourInterface);
    if (isEnabled(NetworkManifest::propertiesBehaviourInterface))
    {
      module_propertiesBehaviourInterface->update(timeStep);
    }
    EuphoriaRagdollBehaviourInterface* module_euphoriaRagdollBehaviourInterface = (EuphoriaRagdollBehaviourInterface*)getModule(NetworkManifest::euphoriaRagdollBehaviourInterface);
    if (isEnabled(NetworkManifest::euphoriaRagdollBehaviourInterface))
    {
      module_euphoriaRagdollBehaviourInterface->update(timeStep);
    }
    ArmsBraceBehaviourInterface* module_armsBraceBehaviourInterface = (ArmsBraceBehaviourInterface*)getModule(NetworkManifest::armsBraceBehaviourInterface);
    if (isEnabled(NetworkManifest::armsBraceBehaviourInterface))
    {
      module_armsBraceBehaviourInterface->update(timeStep);
    }
    ArmsPlacementBehaviourInterface* module_armsPlacementBehaviourInterface = (ArmsPlacementBehaviourInterface*)getModule(NetworkManifest::armsPlacementBehaviourInterface);
    if (isEnabled(NetworkManifest::armsPlacementBehaviourInterface))
    {
      module_armsPlacementBehaviourInterface->update(timeStep);
    }
    AimBehaviourInterface* module_aimBehaviourInterface = (AimBehaviourInterface*)getModule(NetworkManifest::aimBehaviourInterface);
    if (isEnabled(NetworkManifest::aimBehaviourInterface))
    {
      module_aimBehaviourInterface->update(timeStep);
    }
    HeadDodgeBehaviourInterface* module_headDodgeBehaviourInterface = (HeadDodgeBehaviourInterface*)getModule(NetworkManifest::headDodgeBehaviourInterface);
    if (isEnabled(NetworkManifest::headDodgeBehaviourInterface))
    {
      module_headDodgeBehaviourInterface->update(timeStep);
    }
    ReachForBodyBehaviourInterface* module_reachForBodyBehaviourInterface = (ReachForBodyBehaviourInterface*)getModule(NetworkManifest::reachForBodyBehaviourInterface);
    if (isEnabled(NetworkManifest::reachForBodyBehaviourInterface))
    {
      module_reachForBodyBehaviourInterface->update(timeStep);
    }
    ReachForWorldBehaviourInterface* module_reachForWorldBehaviourInterface = (ReachForWorldBehaviourInterface*)getModule(NetworkManifest::reachForWorldBehaviourInterface);
    if (isEnabled(NetworkManifest::reachForWorldBehaviourInterface))
    {
      module_reachForWorldBehaviourInterface->update(timeStep);
    }
    AnimateBehaviourInterface* module_animateBehaviourInterface = (AnimateBehaviourInterface*)getModule(NetworkManifest::animateBehaviourInterface);
    if (isEnabled(NetworkManifest::animateBehaviourInterface))
    {
      module_animateBehaviourInterface->update(timeStep);
    }
    BalancePoserBehaviourInterface* module_balancePoserBehaviourInterface = (BalancePoserBehaviourInterface*)getModule(NetworkManifest::balancePoserBehaviourInterface);
    if (isEnabled(NetworkManifest::balancePoserBehaviourInterface))
    {
      module_balancePoserBehaviourInterface->update(timeStep);
    }
    ProtectBehaviourInterface* module_protectBehaviourInterface = (ProtectBehaviourInterface*)getModule(NetworkManifest::protectBehaviourInterface);
    if (isEnabled(NetworkManifest::protectBehaviourInterface))
    {
      module_protectBehaviourInterface->update(timeStep);
    }
    HazardAwarenessBehaviourInterface* module_hazardAwarenessBehaviourInterface = (HazardAwarenessBehaviourInterface*)getModule(NetworkManifest::hazardAwarenessBehaviourInterface);
    if (isEnabled(NetworkManifest::hazardAwarenessBehaviourInterface))
    {
      module_hazardAwarenessBehaviourInterface->update(timeStep);
    }
    UserHazardBehaviourInterface* module_userHazardBehaviourInterface = (UserHazardBehaviourInterface*)getModule(NetworkManifest::userHazardBehaviourInterface);
    if (isEnabled(NetworkManifest::userHazardBehaviourInterface))
    {
      module_userHazardBehaviourInterface->update(timeStep);
    }
    ObserveBehaviourInterface* module_observeBehaviourInterface = (ObserveBehaviourInterface*)getModule(NetworkManifest::observeBehaviourInterface);
    if (isEnabled(NetworkManifest::observeBehaviourInterface))
    {
      module_observeBehaviourInterface->update(timeStep);
    }
    IdleAwakeBehaviourInterface* module_idleAwakeBehaviourInterface = (IdleAwakeBehaviourInterface*)getModule(NetworkManifest::idleAwakeBehaviourInterface);
    if (isEnabled(NetworkManifest::idleAwakeBehaviourInterface))
    {
      module_idleAwakeBehaviourInterface->update(timeStep);
    }
    LookBehaviourInterface* module_lookBehaviourInterface = (LookBehaviourInterface*)getModule(NetworkManifest::lookBehaviourInterface);
    if (isEnabled(NetworkManifest::lookBehaviourInterface))
    {
      module_lookBehaviourInterface->update(timeStep);
    }
    HeadAvoidBehaviourInterface* module_headAvoidBehaviourInterface = (HeadAvoidBehaviourInterface*)getModule(NetworkManifest::headAvoidBehaviourInterface);
    if (isEnabled(NetworkManifest::headAvoidBehaviourInterface))
    {
      module_headAvoidBehaviourInterface->update(timeStep);
    }
    ShieldBehaviourInterface* module_shieldBehaviourInterface = (ShieldBehaviourInterface*)getModule(NetworkManifest::shieldBehaviourInterface);
    if (isEnabled(NetworkManifest::shieldBehaviourInterface))
    {
      module_shieldBehaviourInterface->update(timeStep);
    }
    HoldBehaviourInterface* module_holdBehaviourInterface = (HoldBehaviourInterface*)getModule(NetworkManifest::holdBehaviourInterface);
    if (isEnabled(NetworkManifest::holdBehaviourInterface))
    {
      module_holdBehaviourInterface->update(timeStep);
    }
    HoldActionBehaviourInterface* module_holdActionBehaviourInterface = (HoldActionBehaviourInterface*)getModule(NetworkManifest::holdActionBehaviourInterface);
    if (isEnabled(NetworkManifest::holdActionBehaviourInterface))
    {
      module_holdActionBehaviourInterface->update(timeStep);
    }
    FreeFallBehaviourInterface* module_freeFallBehaviourInterface = (FreeFallBehaviourInterface*)getModule(NetworkManifest::freeFallBehaviourInterface);
    if (isEnabled(NetworkManifest::freeFallBehaviourInterface))
    {
      module_freeFallBehaviourInterface->update(timeStep);
    }
    ArmsWindmillBehaviourInterface* module_armsWindmillBehaviourInterface = (ArmsWindmillBehaviourInterface*)getModule(NetworkManifest::armsWindmillBehaviourInterface);
    if (isEnabled(NetworkManifest::armsWindmillBehaviourInterface))
    {
      module_armsWindmillBehaviourInterface->update(timeStep);
    }
    LegsPedalBehaviourInterface* module_legsPedalBehaviourInterface = (LegsPedalBehaviourInterface*)getModule(NetworkManifest::legsPedalBehaviourInterface);
    if (isEnabled(NetworkManifest::legsPedalBehaviourInterface))
    {
      module_legsPedalBehaviourInterface->update(timeStep);
    }
    ShieldActionBehaviourInterface* module_shieldActionBehaviourInterface = (ShieldActionBehaviourInterface*)getModule(NetworkManifest::shieldActionBehaviourInterface);
    if (isEnabled(NetworkManifest::shieldActionBehaviourInterface))
    {
      module_shieldActionBehaviourInterface->update(timeStep);
    }
    SitBehaviourInterface* module_sitBehaviourInterface = (SitBehaviourInterface*)getModule(NetworkManifest::sitBehaviourInterface);
    if (isEnabled(NetworkManifest::sitBehaviourInterface))
    {
      module_sitBehaviourInterface->update(timeStep);
    }
    WritheBehaviourInterface* module_writheBehaviourInterface = (WritheBehaviourInterface*)getModule(NetworkManifest::writheBehaviourInterface);
    if (isEnabled(NetworkManifest::writheBehaviourInterface))
    {
      module_writheBehaviourInterface->update(timeStep);
    }
    // SYNC
    BalanceBehaviourInterface* module_balanceBehaviourInterface = (BalanceBehaviourInterface*)getModule(NetworkManifest::balanceBehaviourInterface);
    if (isEnabled(NetworkManifest::balanceBehaviourInterface))
    {
      module_balanceBehaviourInterface->combineInputs();
      module_balanceBehaviourInterface->update(timeStep);
      module_balanceBehaviourInterface->combineOutputs();
    }
    RandomLook* module_randomLook = (RandomLook*)getModule(NetworkManifest::randomLook);
    if (isEnabled(NetworkManifest::randomLook))
    {
      module_randomLook->combineInputs();
      module_randomLook->update(timeStep);
    }
    HazardManagement* module_hazardManagement = (HazardManagement*)getModule(NetworkManifest::hazardManagement);
    if (isEnabled(NetworkManifest::hazardManagement))
    {
      {
        GrabDetection* module_hazardManagement_grabDetection = (GrabDetection*)getModule(NetworkManifest::hazardManagement_grabDetection);
        if (isEnabled(NetworkManifest::hazardManagement_grabDetection))
        {
          module_hazardManagement_grabDetection->combineInputs();
          module_hazardManagement_grabDetection->update(timeStep);
        }
        HazardResponse* module_hazardManagement_hazardResponse = (HazardResponse*)getModule(NetworkManifest::hazardManagement_hazardResponse);
        if (isEnabled(NetworkManifest::hazardManagement_hazardResponse))
        {
          module_hazardManagement_hazardResponse->combineInputs();
          module_hazardManagement_hazardResponse->update(timeStep);
        }
        FreeFallManagement* module_hazardManagement_freeFallManagement = (FreeFallManagement*)getModule(NetworkManifest::hazardManagement_freeFallManagement);
        if (isEnabled(NetworkManifest::hazardManagement_freeFallManagement))
        {
          module_hazardManagement_freeFallManagement->combineInputs();
          module_hazardManagement_freeFallManagement->update(timeStep);
        }
        ImpactPredictor* module_hazardManagement_chestImpactPredictor = (ImpactPredictor*)getModule(NetworkManifest::hazardManagement_chestImpactPredictor);
        if (isEnabled(NetworkManifest::hazardManagement_chestImpactPredictor))
        {
          module_hazardManagement_chestImpactPredictor->combineInputs();
          module_hazardManagement_chestImpactPredictor->update(timeStep);
        }
        // SYNC
        Grab* module_hazardManagement_grab = (Grab*)getModule(NetworkManifest::hazardManagement_grab);
        if (isEnabled(NetworkManifest::hazardManagement_grab))
        {
          module_hazardManagement_grab->combineInputs();
          module_hazardManagement_grab->update(timeStep);
        }
        ShieldManagement* module_hazardManagement_shieldManagement = (ShieldManagement*)getModule(NetworkManifest::hazardManagement_shieldManagement);
        if (isEnabled(NetworkManifest::hazardManagement_shieldManagement))
        {
          module_hazardManagement_shieldManagement->combineInputs();
          module_hazardManagement_shieldManagement->update(timeStep);
        }
      }
      module_hazardManagement->combineOutputs();
    }
    // SYNC
    BalanceManagement* module_balanceManagement = (BalanceManagement*)getModule(NetworkManifest::balanceManagement);
    if (isEnabled(NetworkManifest::balanceManagement))
    {
      module_balanceManagement->combineInputs();
      {
        StaticBalance* module_balanceManagement_staticBalance = (StaticBalance*)getModule(NetworkManifest::balanceManagement_staticBalance);
        if (isEnabled(NetworkManifest::balanceManagement_staticBalance))
        {
          module_balanceManagement_staticBalance->combineInputs();
          module_balanceManagement_staticBalance->update(timeStep);
          module_balanceManagement_staticBalance->combineOutputs();
        }
        SteppingBalance* module_balanceManagement_steppingBalance = (SteppingBalance*)getModule(NetworkManifest::balanceManagement_steppingBalance);
        if (isEnabled(NetworkManifest::balanceManagement_steppingBalance))
        {
          module_balanceManagement_steppingBalance->combineInputs();
          module_balanceManagement_steppingBalance->update(timeStep);
        }
        // SYNC
        BalancePoser* module_balanceManagement_balancePoser = (BalancePoser*)getModule(NetworkManifest::balanceManagement_balancePoser);
        if (isEnabled(NetworkManifest::balanceManagement_balancePoser))
        {
          module_balanceManagement_balancePoser->combineInputs();
          module_balanceManagement_balancePoser->update(timeStep);
        }
      }
      module_balanceManagement->combineOutputs();
    }
    EnvironmentAwareness* module_environmentAwareness = (EnvironmentAwareness*)getModule(NetworkManifest::environmentAwareness);
    if (isEnabled(NetworkManifest::environmentAwareness))
    {
      module_environmentAwareness->combineInputs();
      module_environmentAwareness->update(timeStep);
    }
    // SYNC
    BodyFrame* module_bodyFrame = (BodyFrame*)getModule(NetworkManifest::bodyFrame);
    if (isEnabled(NetworkManifest::bodyFrame))
    {
      module_bodyFrame->combineInputs();
      {
        SupportPolygon* module_bodyFrame_supportPolygon = (SupportPolygon*)getModule(NetworkManifest::bodyFrame_supportPolygon);
        if (isEnabled(NetworkManifest::bodyFrame_supportPolygon))
        {
          module_bodyFrame_supportPolygon->combineInputs();
          module_bodyFrame_supportPolygon->update(timeStep);
        }
        SupportPolygon* module_bodyFrame_sittingSupportPolygon = (SupportPolygon*)getModule(NetworkManifest::bodyFrame_sittingSupportPolygon);
        if (isEnabled(NetworkManifest::bodyFrame_sittingSupportPolygon))
        {
          module_bodyFrame_sittingSupportPolygon->combineInputs();
          module_bodyFrame_sittingSupportPolygon->update(timeStep);
        }
        ReachForBody* module_bodyFrame_reachForBody = (ReachForBody*)getModule(NetworkManifest::bodyFrame_reachForBody);
        if (isEnabled(NetworkManifest::bodyFrame_reachForBody))
        {
          module_bodyFrame_reachForBody->combineInputs();
          module_bodyFrame_reachForBody->update(timeStep);
        }
        // SYNC
        BodyBalance* module_bodyFrame_bodyBalance = (BodyBalance*)getModule(NetworkManifest::bodyFrame_bodyBalance);
        if (isEnabled(NetworkManifest::bodyFrame_bodyBalance))
        {
          module_bodyFrame_bodyBalance->combineInputs();
          module_bodyFrame_bodyBalance->update(timeStep);
        }
        SittingBodyBalance* module_bodyFrame_sittingBodyBalance = (SittingBodyBalance*)getModule(NetworkManifest::bodyFrame_sittingBodyBalance);
        if (isEnabled(NetworkManifest::bodyFrame_sittingBodyBalance))
        {
          module_bodyFrame_sittingBodyBalance->combineInputs();
          module_bodyFrame_sittingBodyBalance->update(timeStep);
        }
        // SYNC
        BalanceAssistant* module_bodyFrame_balanceAssistant = (BalanceAssistant*)getModule(NetworkManifest::bodyFrame_balanceAssistant);
        if (isEnabled(NetworkManifest::bodyFrame_balanceAssistant))
        {
          module_bodyFrame_balanceAssistant->combineInputs();
          module_bodyFrame_balanceAssistant->update(timeStep);
        }
      }
      module_bodyFrame->combineOutputs();
    }
    // SYNC
    BodySection* module_upperBody = (BodySection*)getModule(NetworkManifest::upperBody);
    if (isEnabled(NetworkManifest::upperBody))
    {
      module_upperBody->combineInputs();
      {
        RotateCore* module_upperBody_rotate = (RotateCore*)getModule(NetworkManifest::upperBody_rotate);
        if (isEnabled(NetworkManifest::upperBody_rotate))
        {
          module_upperBody_rotate->combineInputs();
          module_upperBody_rotate->update(timeStep);
        }
        PositionCore* module_upperBody_position = (PositionCore*)getModule(NetworkManifest::upperBody_position);
        if (isEnabled(NetworkManifest::upperBody_position))
        {
          module_upperBody_position->combineInputs();
          module_upperBody_position->update(timeStep);
        }
        BraceChooser* module_upperBody_braceChooser = (BraceChooser*)getModule(NetworkManifest::upperBody_braceChooser);
        if (isEnabled(NetworkManifest::upperBody_braceChooser))
        {
          module_upperBody_braceChooser->update(timeStep);
        }
      }
      module_upperBody->combineOutputs();
    }
    BodySection* module_lowerBody = (BodySection*)getModule(NetworkManifest::lowerBody);
    if (isEnabled(NetworkManifest::lowerBody))
    {
      module_lowerBody->combineInputs();
      {
        RotateCore* module_lowerBody_rotate = (RotateCore*)getModule(NetworkManifest::lowerBody_rotate);
        if (isEnabled(NetworkManifest::lowerBody_rotate))
        {
          module_lowerBody_rotate->combineInputs();
          module_lowerBody_rotate->update(timeStep);
        }
        PositionCore* module_lowerBody_position = (PositionCore*)getModule(NetworkManifest::lowerBody_position);
        if (isEnabled(NetworkManifest::lowerBody_position))
        {
          module_lowerBody_position->combineInputs();
          module_lowerBody_position->update(timeStep);
        }
        BraceChooser* module_lowerBody_braceChooser = (BraceChooser*)getModule(NetworkManifest::lowerBody_braceChooser);
        if (isEnabled(NetworkManifest::lowerBody_braceChooser))
        {
          module_lowerBody_braceChooser->update(timeStep);
        }
      }
      module_lowerBody->combineOutputs();
    }
    // SYNC
    Leg* module_legs_0 = (Leg*)getModule(NetworkManifest::legs_0);
    if (isEnabled(NetworkManifest::legs_0))
    {
      module_legs_0->combineInputs();
      module_legs_0->update(timeStep);
      {
        LegBrace* module_legs_0_brace = (LegBrace*)getModule(NetworkManifest::legs_0_brace);
        if (isEnabled(NetworkManifest::legs_0_brace))
        {
          module_legs_0_brace->combineInputs();
          module_legs_0_brace->update(timeStep);
        }
        LegStandingSupport* module_legs_0_standingSupport = (LegStandingSupport*)getModule(NetworkManifest::legs_0_standingSupport);
        if (isEnabled(NetworkManifest::legs_0_standingSupport))
        {
          module_legs_0_standingSupport->combineInputs();
          module_legs_0_standingSupport->update(timeStep);
        }
        LegSittingSupport* module_legs_0_sittingSupport = (LegSittingSupport*)getModule(NetworkManifest::legs_0_sittingSupport);
        if (isEnabled(NetworkManifest::legs_0_sittingSupport))
        {
          module_legs_0_sittingSupport->combineInputs();
          module_legs_0_sittingSupport->update(timeStep);
        }
        LegStep* module_legs_0_step = (LegStep*)getModule(NetworkManifest::legs_0_step);
        if (isEnabled(NetworkManifest::legs_0_step))
        {
          module_legs_0_step->combineInputs();
          module_legs_0_step->update(timeStep);
        }
        LegSwing* module_legs_0_swing = (LegSwing*)getModule(NetworkManifest::legs_0_swing);
        if (isEnabled(NetworkManifest::legs_0_swing))
        {
          module_legs_0_swing->combineInputs();
          module_legs_0_swing->update(timeStep);
        }
        LegPose* module_legs_0_pose = (LegPose*)getModule(NetworkManifest::legs_0_pose);
        if (isEnabled(NetworkManifest::legs_0_pose))
        {
          module_legs_0_pose->combineInputs();
          module_legs_0_pose->update(timeStep);
        }
        LegSpin* module_legs_0_spin = (LegSpin*)getModule(NetworkManifest::legs_0_spin);
        if (isEnabled(NetworkManifest::legs_0_spin))
        {
          module_legs_0_spin->combineInputs();
          module_legs_0_spin->update(timeStep);
        }
        LegReachReaction* module_legs_0_reachReaction = (LegReachReaction*)getModule(NetworkManifest::legs_0_reachReaction);
        if (isEnabled(NetworkManifest::legs_0_reachReaction))
        {
          module_legs_0_reachReaction->combineInputs();
          module_legs_0_reachReaction->update(timeStep);
        }
        LegWrithe* module_legs_0_writhe = (LegWrithe*)getModule(NetworkManifest::legs_0_writhe);
        if (isEnabled(NetworkManifest::legs_0_writhe))
        {
          module_legs_0_writhe->combineInputs();
          module_legs_0_writhe->update(timeStep);
        }
      }
      module_legs_0->combineOutputs();
    }
    Leg* module_legs_1 = (Leg*)getModule(NetworkManifest::legs_1);
    if (isEnabled(NetworkManifest::legs_1))
    {
      module_legs_1->combineInputs();
      module_legs_1->update(timeStep);
      {
        LegBrace* module_legs_1_brace = (LegBrace*)getModule(NetworkManifest::legs_1_brace);
        if (isEnabled(NetworkManifest::legs_1_brace))
        {
          module_legs_1_brace->combineInputs();
          module_legs_1_brace->update(timeStep);
        }
        LegStandingSupport* module_legs_1_standingSupport = (LegStandingSupport*)getModule(NetworkManifest::legs_1_standingSupport);
        if (isEnabled(NetworkManifest::legs_1_standingSupport))
        {
          module_legs_1_standingSupport->combineInputs();
          module_legs_1_standingSupport->update(timeStep);
        }
        LegSittingSupport* module_legs_1_sittingSupport = (LegSittingSupport*)getModule(NetworkManifest::legs_1_sittingSupport);
        if (isEnabled(NetworkManifest::legs_1_sittingSupport))
        {
          module_legs_1_sittingSupport->combineInputs();
          module_legs_1_sittingSupport->update(timeStep);
        }
        LegStep* module_legs_1_step = (LegStep*)getModule(NetworkManifest::legs_1_step);
        if (isEnabled(NetworkManifest::legs_1_step))
        {
          module_legs_1_step->combineInputs();
          module_legs_1_step->update(timeStep);
        }
        LegSwing* module_legs_1_swing = (LegSwing*)getModule(NetworkManifest::legs_1_swing);
        if (isEnabled(NetworkManifest::legs_1_swing))
        {
          module_legs_1_swing->combineInputs();
          module_legs_1_swing->update(timeStep);
        }
        LegPose* module_legs_1_pose = (LegPose*)getModule(NetworkManifest::legs_1_pose);
        if (isEnabled(NetworkManifest::legs_1_pose))
        {
          module_legs_1_pose->combineInputs();
          module_legs_1_pose->update(timeStep);
        }
        LegSpin* module_legs_1_spin = (LegSpin*)getModule(NetworkManifest::legs_1_spin);
        if (isEnabled(NetworkManifest::legs_1_spin))
        {
          module_legs_1_spin->combineInputs();
          module_legs_1_spin->update(timeStep);
        }
        LegReachReaction* module_legs_1_reachReaction = (LegReachReaction*)getModule(NetworkManifest::legs_1_reachReaction);
        if (isEnabled(NetworkManifest::legs_1_reachReaction))
        {
          module_legs_1_reachReaction->combineInputs();
          module_legs_1_reachReaction->update(timeStep);
        }
        LegWrithe* module_legs_1_writhe = (LegWrithe*)getModule(NetworkManifest::legs_1_writhe);
        if (isEnabled(NetworkManifest::legs_1_writhe))
        {
          module_legs_1_writhe->combineInputs();
          module_legs_1_writhe->update(timeStep);
        }
      }
      module_legs_1->combineOutputs();
    }
    Spine* module_spines_0 = (Spine*)getModule(NetworkManifest::spines_0);
    if (isEnabled(NetworkManifest::spines_0))
    {
      module_spines_0->combineInputs();
      module_spines_0->update(timeStep);
      {
        SpineSupport* module_spines_0_support = (SpineSupport*)getModule(NetworkManifest::spines_0_support);
        if (isEnabled(NetworkManifest::spines_0_support))
        {
          module_spines_0_support->combineInputs();
          module_spines_0_support->update(timeStep);
        }
        SpinePose* module_spines_0_pose = (SpinePose*)getModule(NetworkManifest::spines_0_pose);
        if (isEnabled(NetworkManifest::spines_0_pose))
        {
          module_spines_0_pose->combineInputs();
          module_spines_0_pose->update(timeStep);
        }
        SpineReachReaction* module_spines_0_reachReaction = (SpineReachReaction*)getModule(NetworkManifest::spines_0_reachReaction);
        if (isEnabled(NetworkManifest::spines_0_reachReaction))
        {
          module_spines_0_reachReaction->combineInputs();
          module_spines_0_reachReaction->update(timeStep);
        }
        SpineWrithe* module_spines_0_writhe = (SpineWrithe*)getModule(NetworkManifest::spines_0_writhe);
        if (isEnabled(NetworkManifest::spines_0_writhe))
        {
          module_spines_0_writhe->combineInputs();
          module_spines_0_writhe->update(timeStep);
        }
        SpineAim* module_spines_0_aim = (SpineAim*)getModule(NetworkManifest::spines_0_aim);
        if (isEnabled(NetworkManifest::spines_0_aim))
        {
          module_spines_0_aim->combineInputs();
          module_spines_0_aim->update(timeStep);
        }
        // SYNC
        SpineControl* module_spines_0_control = (SpineControl*)getModule(NetworkManifest::spines_0_control);
        if (isEnabled(NetworkManifest::spines_0_control))
        {
          module_spines_0_control->combineInputs();
          module_spines_0_control->update(timeStep);
        }
      }
      module_spines_0->combineOutputs();
    }
    // SYNC
    Arm* module_arms_0 = (Arm*)getModule(NetworkManifest::arms_0);
    if (isEnabled(NetworkManifest::arms_0))
    {
      module_arms_0->combineInputs();
      module_arms_0->update(timeStep);
      {
        ArmGrab* module_arms_0_grab = (ArmGrab*)getModule(NetworkManifest::arms_0_grab);
        if (isEnabled(NetworkManifest::arms_0_grab))
        {
          module_arms_0_grab->combineInputs();
          module_arms_0_grab->update(timeStep);
        }
        ArmAim* module_arms_0_aim = (ArmAim*)getModule(NetworkManifest::arms_0_aim);
        if (isEnabled(NetworkManifest::arms_0_aim))
        {
          module_arms_0_aim->combineInputs();
          module_arms_0_aim->update(timeStep);
        }
        ArmBrace* module_arms_0_brace = (ArmBrace*)getModule(NetworkManifest::arms_0_brace);
        if (isEnabled(NetworkManifest::arms_0_brace))
        {
          module_arms_0_brace->combineInputs();
          module_arms_0_brace->update(timeStep);
        }
        ArmSittingStep* module_arms_0_sittingStep = (ArmSittingStep*)getModule(NetworkManifest::arms_0_sittingStep);
        if (isEnabled(NetworkManifest::arms_0_sittingStep))
        {
          module_arms_0_sittingStep->combineInputs();
          module_arms_0_sittingStep->update(timeStep);
        }
        ArmStep* module_arms_0_step = (ArmStep*)getModule(NetworkManifest::arms_0_step);
        if (isEnabled(NetworkManifest::arms_0_step))
        {
          module_arms_0_step->combineInputs();
          module_arms_0_step->update(timeStep);
        }
        ArmSpin* module_arms_0_spin = (ArmSpin*)getModule(NetworkManifest::arms_0_spin);
        if (isEnabled(NetworkManifest::arms_0_spin))
        {
          module_arms_0_spin->combineInputs();
          module_arms_0_spin->update(timeStep);
        }
        ArmSwing* module_arms_0_swing = (ArmSwing*)getModule(NetworkManifest::arms_0_swing);
        if (isEnabled(NetworkManifest::arms_0_swing))
        {
          module_arms_0_swing->combineInputs();
          module_arms_0_swing->update(timeStep);
        }
        ArmReachForBodyPart* module_arms_0_reachForBodyPart = (ArmReachForBodyPart*)getModule(NetworkManifest::arms_0_reachForBodyPart);
        if (isEnabled(NetworkManifest::arms_0_reachForBodyPart))
        {
          module_arms_0_reachForBodyPart->combineInputs();
          module_arms_0_reachForBodyPart->update(timeStep);
        }
        ArmPlacement* module_arms_0_placement = (ArmPlacement*)getModule(NetworkManifest::arms_0_placement);
        if (isEnabled(NetworkManifest::arms_0_placement))
        {
          module_arms_0_placement->combineInputs();
          module_arms_0_placement->update(timeStep);
        }
        ArmPose* module_arms_0_pose = (ArmPose*)getModule(NetworkManifest::arms_0_pose);
        if (isEnabled(NetworkManifest::arms_0_pose))
        {
          module_arms_0_pose->combineInputs();
          module_arms_0_pose->update(timeStep);
        }
        ArmReachReaction* module_arms_0_reachReaction = (ArmReachReaction*)getModule(NetworkManifest::arms_0_reachReaction);
        if (isEnabled(NetworkManifest::arms_0_reachReaction))
        {
          module_arms_0_reachReaction->combineInputs();
          module_arms_0_reachReaction->update(timeStep);
        }
        ArmWrithe* module_arms_0_writhe = (ArmWrithe*)getModule(NetworkManifest::arms_0_writhe);
        if (isEnabled(NetworkManifest::arms_0_writhe))
        {
          module_arms_0_writhe->combineInputs();
          module_arms_0_writhe->update(timeStep);
        }
        // SYNC
        ArmHold* module_arms_0_hold = (ArmHold*)getModule(NetworkManifest::arms_0_hold);
        if (isEnabled(NetworkManifest::arms_0_hold))
        {
          module_arms_0_hold->combineInputs();
          module_arms_0_hold->update(timeStep);
        }
        ArmStandingSupport* module_arms_0_standingSupport = (ArmStandingSupport*)getModule(NetworkManifest::arms_0_standingSupport);
        if (isEnabled(NetworkManifest::arms_0_standingSupport))
        {
          module_arms_0_standingSupport->combineInputs();
          module_arms_0_standingSupport->update(timeStep);
        }
        ArmReachForWorld* module_arms_0_reachForWorld = (ArmReachForWorld*)getModule(NetworkManifest::arms_0_reachForWorld);
        if (isEnabled(NetworkManifest::arms_0_reachForWorld))
        {
          module_arms_0_reachForWorld->combineInputs();
          module_arms_0_reachForWorld->update(timeStep);
        }
        // SYNC
        ArmHoldingSupport* module_arms_0_holdingSupport = (ArmHoldingSupport*)getModule(NetworkManifest::arms_0_holdingSupport);
        if (isEnabled(NetworkManifest::arms_0_holdingSupport))
        {
          module_arms_0_holdingSupport->combineInputs();
          module_arms_0_holdingSupport->update(timeStep);
        }
      }
      module_arms_0->combineOutputs();
    }
    Arm* module_arms_1 = (Arm*)getModule(NetworkManifest::arms_1);
    if (isEnabled(NetworkManifest::arms_1))
    {
      module_arms_1->combineInputs();
      module_arms_1->update(timeStep);
      {
        ArmGrab* module_arms_1_grab = (ArmGrab*)getModule(NetworkManifest::arms_1_grab);
        if (isEnabled(NetworkManifest::arms_1_grab))
        {
          module_arms_1_grab->combineInputs();
          module_arms_1_grab->update(timeStep);
        }
        ArmAim* module_arms_1_aim = (ArmAim*)getModule(NetworkManifest::arms_1_aim);
        if (isEnabled(NetworkManifest::arms_1_aim))
        {
          module_arms_1_aim->combineInputs();
          module_arms_1_aim->update(timeStep);
        }
        ArmBrace* module_arms_1_brace = (ArmBrace*)getModule(NetworkManifest::arms_1_brace);
        if (isEnabled(NetworkManifest::arms_1_brace))
        {
          module_arms_1_brace->combineInputs();
          module_arms_1_brace->update(timeStep);
        }
        ArmSittingStep* module_arms_1_sittingStep = (ArmSittingStep*)getModule(NetworkManifest::arms_1_sittingStep);
        if (isEnabled(NetworkManifest::arms_1_sittingStep))
        {
          module_arms_1_sittingStep->combineInputs();
          module_arms_1_sittingStep->update(timeStep);
        }
        ArmStep* module_arms_1_step = (ArmStep*)getModule(NetworkManifest::arms_1_step);
        if (isEnabled(NetworkManifest::arms_1_step))
        {
          module_arms_1_step->combineInputs();
          module_arms_1_step->update(timeStep);
        }
        ArmSpin* module_arms_1_spin = (ArmSpin*)getModule(NetworkManifest::arms_1_spin);
        if (isEnabled(NetworkManifest::arms_1_spin))
        {
          module_arms_1_spin->combineInputs();
          module_arms_1_spin->update(timeStep);
        }
        ArmSwing* module_arms_1_swing = (ArmSwing*)getModule(NetworkManifest::arms_1_swing);
        if (isEnabled(NetworkManifest::arms_1_swing))
        {
          module_arms_1_swing->combineInputs();
          module_arms_1_swing->update(timeStep);
        }
        ArmReachForBodyPart* module_arms_1_reachForBodyPart = (ArmReachForBodyPart*)getModule(NetworkManifest::arms_1_reachForBodyPart);
        if (isEnabled(NetworkManifest::arms_1_reachForBodyPart))
        {
          module_arms_1_reachForBodyPart->combineInputs();
          module_arms_1_reachForBodyPart->update(timeStep);
        }
        ArmPlacement* module_arms_1_placement = (ArmPlacement*)getModule(NetworkManifest::arms_1_placement);
        if (isEnabled(NetworkManifest::arms_1_placement))
        {
          module_arms_1_placement->combineInputs();
          module_arms_1_placement->update(timeStep);
        }
        ArmPose* module_arms_1_pose = (ArmPose*)getModule(NetworkManifest::arms_1_pose);
        if (isEnabled(NetworkManifest::arms_1_pose))
        {
          module_arms_1_pose->combineInputs();
          module_arms_1_pose->update(timeStep);
        }
        ArmReachReaction* module_arms_1_reachReaction = (ArmReachReaction*)getModule(NetworkManifest::arms_1_reachReaction);
        if (isEnabled(NetworkManifest::arms_1_reachReaction))
        {
          module_arms_1_reachReaction->combineInputs();
          module_arms_1_reachReaction->update(timeStep);
        }
        ArmWrithe* module_arms_1_writhe = (ArmWrithe*)getModule(NetworkManifest::arms_1_writhe);
        if (isEnabled(NetworkManifest::arms_1_writhe))
        {
          module_arms_1_writhe->combineInputs();
          module_arms_1_writhe->update(timeStep);
        }
        // SYNC
        ArmHold* module_arms_1_hold = (ArmHold*)getModule(NetworkManifest::arms_1_hold);
        if (isEnabled(NetworkManifest::arms_1_hold))
        {
          module_arms_1_hold->combineInputs();
          module_arms_1_hold->update(timeStep);
        }
        ArmStandingSupport* module_arms_1_standingSupport = (ArmStandingSupport*)getModule(NetworkManifest::arms_1_standingSupport);
        if (isEnabled(NetworkManifest::arms_1_standingSupport))
        {
          module_arms_1_standingSupport->combineInputs();
          module_arms_1_standingSupport->update(timeStep);
        }
        ArmReachForWorld* module_arms_1_reachForWorld = (ArmReachForWorld*)getModule(NetworkManifest::arms_1_reachForWorld);
        if (isEnabled(NetworkManifest::arms_1_reachForWorld))
        {
          module_arms_1_reachForWorld->combineInputs();
          module_arms_1_reachForWorld->update(timeStep);
        }
        // SYNC
        ArmHoldingSupport* module_arms_1_holdingSupport = (ArmHoldingSupport*)getModule(NetworkManifest::arms_1_holdingSupport);
        if (isEnabled(NetworkManifest::arms_1_holdingSupport))
        {
          module_arms_1_holdingSupport->combineInputs();
          module_arms_1_holdingSupport->update(timeStep);
        }
      }
      module_arms_1->combineOutputs();
    }
    Head* module_heads_0 = (Head*)getModule(NetworkManifest::heads_0);
    if (isEnabled(NetworkManifest::heads_0))
    {
      module_heads_0->combineInputs();
      module_heads_0->update(timeStep);
      {
        HeadEyes* module_heads_0_eyes = (HeadEyes*)getModule(NetworkManifest::heads_0_eyes);
        if (isEnabled(NetworkManifest::heads_0_eyes))
        {
          module_heads_0_eyes->combineInputs();
          module_heads_0_eyes->update(timeStep);
        }
        HeadAvoid* module_heads_0_avoid = (HeadAvoid*)getModule(NetworkManifest::heads_0_avoid);
        if (isEnabled(NetworkManifest::heads_0_avoid))
        {
          module_heads_0_avoid->combineInputs();
          module_heads_0_avoid->update(timeStep);
        }
        HeadDodge* module_heads_0_dodge = (HeadDodge*)getModule(NetworkManifest::heads_0_dodge);
        if (isEnabled(NetworkManifest::heads_0_dodge))
        {
          module_heads_0_dodge->combineInputs();
          module_heads_0_dodge->update(timeStep);
        }
        HeadAim* module_heads_0_aim = (HeadAim*)getModule(NetworkManifest::heads_0_aim);
        if (isEnabled(NetworkManifest::heads_0_aim))
        {
          module_heads_0_aim->combineInputs();
          module_heads_0_aim->update(timeStep);
        }
        HeadSupport* module_heads_0_support = (HeadSupport*)getModule(NetworkManifest::heads_0_support);
        if (isEnabled(NetworkManifest::heads_0_support))
        {
          module_heads_0_support->combineInputs();
          module_heads_0_support->update(timeStep);
        }
        HeadPose* module_heads_0_pose = (HeadPose*)getModule(NetworkManifest::heads_0_pose);
        if (isEnabled(NetworkManifest::heads_0_pose))
        {
          module_heads_0_pose->combineInputs();
          module_heads_0_pose->update(timeStep);
        }
        // SYNC
        HeadPoint* module_heads_0_point = (HeadPoint*)getModule(NetworkManifest::heads_0_point);
        if (isEnabled(NetworkManifest::heads_0_point))
        {
          module_heads_0_point->combineInputs();
          module_heads_0_point->update(timeStep);
        }
      }
      module_heads_0->combineOutputs();
    }
    // SYNC
    SceneProbes* module_sceneProbes = (SceneProbes*)getModule(NetworkManifest::sceneProbes);
    if (isEnabled(NetworkManifest::sceneProbes))
    {
      module_sceneProbes->combineInputs();
      module_sceneProbes->update(timeStep);
    }
  }
}
//----------------------------------------------------------------------------------------------------------------------
void MyNetwork::executeNetworkFeedback(float timeStep)
{
  {
    AnimateBehaviourInterface* module_animateBehaviourInterface = (AnimateBehaviourInterface*)getModule(NetworkManifest::animateBehaviourInterface);
    if (isEnabled(NetworkManifest::animateBehaviourInterface))
    {
      module_animateBehaviourInterface->feedback(timeStep);
    }
    FreeFallBehaviourInterface* module_freeFallBehaviourInterface = (FreeFallBehaviourInterface*)getModule(NetworkManifest::freeFallBehaviourInterface);
    if (isEnabled(NetworkManifest::freeFallBehaviourInterface))
    {
      module_freeFallBehaviourInterface->combineFeedbackInputs();
      module_freeFallBehaviourInterface->feedback(timeStep);
    }
    Arm* module_arms_0 = (Arm*)getModule(NetworkManifest::arms_0);
    if (isEnabled(NetworkManifest::arms_0))
    {
      module_arms_0->feedback(timeStep);
      {
        ArmHold* module_arms_0_hold = (ArmHold*)getModule(NetworkManifest::arms_0_hold);
        if (isEnabled(NetworkManifest::arms_0_hold))
        {
          module_arms_0_hold->feedback(timeStep);
        }
        ArmBrace* module_arms_0_brace = (ArmBrace*)getModule(NetworkManifest::arms_0_brace);
        if (isEnabled(NetworkManifest::arms_0_brace))
        {
          module_arms_0_brace->feedback(timeStep);
        }
        ArmStandingSupport* module_arms_0_standingSupport = (ArmStandingSupport*)getModule(NetworkManifest::arms_0_standingSupport);
        if (isEnabled(NetworkManifest::arms_0_standingSupport))
        {
          module_arms_0_standingSupport->feedback(timeStep);
        }
        ArmHoldingSupport* module_arms_0_holdingSupport = (ArmHoldingSupport*)getModule(NetworkManifest::arms_0_holdingSupport);
        if (isEnabled(NetworkManifest::arms_0_holdingSupport))
        {
          module_arms_0_holdingSupport->feedback(timeStep);
        }
        ArmSittingStep* module_arms_0_sittingStep = (ArmSittingStep*)getModule(NetworkManifest::arms_0_sittingStep);
        if (isEnabled(NetworkManifest::arms_0_sittingStep))
        {
          module_arms_0_sittingStep->combineFeedbackInputs();
        }
        ArmStep* module_arms_0_step = (ArmStep*)getModule(NetworkManifest::arms_0_step);
        if (isEnabled(NetworkManifest::arms_0_step))
        {
          module_arms_0_step->feedback(timeStep);
        }
        ArmSpin* module_arms_0_spin = (ArmSpin*)getModule(NetworkManifest::arms_0_spin);
        if (isEnabled(NetworkManifest::arms_0_spin))
        {
          module_arms_0_spin->feedback(timeStep);
        }
        ArmReachForBodyPart* module_arms_0_reachForBodyPart = (ArmReachForBodyPart*)getModule(NetworkManifest::arms_0_reachForBodyPart);
        if (isEnabled(NetworkManifest::arms_0_reachForBodyPart))
        {
          module_arms_0_reachForBodyPart->combineFeedbackInputs();
          module_arms_0_reachForBodyPart->feedback(timeStep);
        }
        ArmReachForWorld* module_arms_0_reachForWorld = (ArmReachForWorld*)getModule(NetworkManifest::arms_0_reachForWorld);
        if (isEnabled(NetworkManifest::arms_0_reachForWorld))
        {
          module_arms_0_reachForWorld->feedback(timeStep);
        }
        // SYNC
        ArmGrab* module_arms_0_grab = (ArmGrab*)getModule(NetworkManifest::arms_0_grab);
        if (isEnabled(NetworkManifest::arms_0_grab))
        {
          module_arms_0_grab->combineFeedbackInputs();
          module_arms_0_grab->feedback(timeStep);
        }
      }
      module_arms_0->combineFeedbackOutputs();
    }
    Arm* module_arms_1 = (Arm*)getModule(NetworkManifest::arms_1);
    if (isEnabled(NetworkManifest::arms_1))
    {
      module_arms_1->feedback(timeStep);
      {
        ArmHold* module_arms_1_hold = (ArmHold*)getModule(NetworkManifest::arms_1_hold);
        if (isEnabled(NetworkManifest::arms_1_hold))
        {
          module_arms_1_hold->feedback(timeStep);
        }
        ArmBrace* module_arms_1_brace = (ArmBrace*)getModule(NetworkManifest::arms_1_brace);
        if (isEnabled(NetworkManifest::arms_1_brace))
        {
          module_arms_1_brace->feedback(timeStep);
        }
        ArmStandingSupport* module_arms_1_standingSupport = (ArmStandingSupport*)getModule(NetworkManifest::arms_1_standingSupport);
        if (isEnabled(NetworkManifest::arms_1_standingSupport))
        {
          module_arms_1_standingSupport->feedback(timeStep);
        }
        ArmHoldingSupport* module_arms_1_holdingSupport = (ArmHoldingSupport*)getModule(NetworkManifest::arms_1_holdingSupport);
        if (isEnabled(NetworkManifest::arms_1_holdingSupport))
        {
          module_arms_1_holdingSupport->feedback(timeStep);
        }
        ArmSittingStep* module_arms_1_sittingStep = (ArmSittingStep*)getModule(NetworkManifest::arms_1_sittingStep);
        if (isEnabled(NetworkManifest::arms_1_sittingStep))
        {
          module_arms_1_sittingStep->combineFeedbackInputs();
        }
        ArmStep* module_arms_1_step = (ArmStep*)getModule(NetworkManifest::arms_1_step);
        if (isEnabled(NetworkManifest::arms_1_step))
        {
          module_arms_1_step->feedback(timeStep);
        }
        ArmSpin* module_arms_1_spin = (ArmSpin*)getModule(NetworkManifest::arms_1_spin);
        if (isEnabled(NetworkManifest::arms_1_spin))
        {
          module_arms_1_spin->feedback(timeStep);
        }
        ArmReachForBodyPart* module_arms_1_reachForBodyPart = (ArmReachForBodyPart*)getModule(NetworkManifest::arms_1_reachForBodyPart);
        if (isEnabled(NetworkManifest::arms_1_reachForBodyPart))
        {
          module_arms_1_reachForBodyPart->combineFeedbackInputs();
          module_arms_1_reachForBodyPart->feedback(timeStep);
        }
        ArmReachForWorld* module_arms_1_reachForWorld = (ArmReachForWorld*)getModule(NetworkManifest::arms_1_reachForWorld);
        if (isEnabled(NetworkManifest::arms_1_reachForWorld))
        {
          module_arms_1_reachForWorld->feedback(timeStep);
        }
        // SYNC
        ArmGrab* module_arms_1_grab = (ArmGrab*)getModule(NetworkManifest::arms_1_grab);
        if (isEnabled(NetworkManifest::arms_1_grab))
        {
          module_arms_1_grab->combineFeedbackInputs();
          module_arms_1_grab->feedback(timeStep);
        }
      }
      module_arms_1->combineFeedbackOutputs();
    }
    Head* module_heads_0 = (Head*)getModule(NetworkManifest::heads_0);
    if (isEnabled(NetworkManifest::heads_0))
    {
      module_heads_0->feedback(timeStep);
      {
        HeadEyes* module_heads_0_eyes = (HeadEyes*)getModule(NetworkManifest::heads_0_eyes);
        if (isEnabled(NetworkManifest::heads_0_eyes))
        {
          module_heads_0_eyes->feedback(timeStep);
        }
        HeadAvoid* module_heads_0_avoid = (HeadAvoid*)getModule(NetworkManifest::heads_0_avoid);
        if (isEnabled(NetworkManifest::heads_0_avoid))
        {
          module_heads_0_avoid->feedback(timeStep);
        }
        HeadDodge* module_heads_0_dodge = (HeadDodge*)getModule(NetworkManifest::heads_0_dodge);
        if (isEnabled(NetworkManifest::heads_0_dodge))
        {
          module_heads_0_dodge->feedback(timeStep);
        }
        HeadPoint* module_heads_0_point = (HeadPoint*)getModule(NetworkManifest::heads_0_point);
        if (isEnabled(NetworkManifest::heads_0_point))
        {
          module_heads_0_point->feedback(timeStep);
        }
      }
    }
    Leg* module_legs_0 = (Leg*)getModule(NetworkManifest::legs_0);
    if (isEnabled(NetworkManifest::legs_0))
    {
      module_legs_0->feedback(timeStep);
      {
        LegBrace* module_legs_0_brace = (LegBrace*)getModule(NetworkManifest::legs_0_brace);
        if (isEnabled(NetworkManifest::legs_0_brace))
        {
          module_legs_0_brace->feedback(timeStep);
        }
        LegStandingSupport* module_legs_0_standingSupport = (LegStandingSupport*)getModule(NetworkManifest::legs_0_standingSupport);
        if (isEnabled(NetworkManifest::legs_0_standingSupport))
        {
          module_legs_0_standingSupport->feedback(timeStep);
        }
        LegSittingSupport* module_legs_0_sittingSupport = (LegSittingSupport*)getModule(NetworkManifest::legs_0_sittingSupport);
        if (isEnabled(NetworkManifest::legs_0_sittingSupport))
        {
          module_legs_0_sittingSupport->feedback(timeStep);
        }
        LegStep* module_legs_0_step = (LegStep*)getModule(NetworkManifest::legs_0_step);
        if (isEnabled(NetworkManifest::legs_0_step))
        {
          module_legs_0_step->feedback(timeStep);
        }
        LegSpin* module_legs_0_spin = (LegSpin*)getModule(NetworkManifest::legs_0_spin);
        if (isEnabled(NetworkManifest::legs_0_spin))
        {
          module_legs_0_spin->feedback(timeStep);
        }
      }
      module_legs_0->combineFeedbackOutputs();
    }
    Leg* module_legs_1 = (Leg*)getModule(NetworkManifest::legs_1);
    if (isEnabled(NetworkManifest::legs_1))
    {
      module_legs_1->feedback(timeStep);
      {
        LegBrace* module_legs_1_brace = (LegBrace*)getModule(NetworkManifest::legs_1_brace);
        if (isEnabled(NetworkManifest::legs_1_brace))
        {
          module_legs_1_brace->feedback(timeStep);
        }
        LegStandingSupport* module_legs_1_standingSupport = (LegStandingSupport*)getModule(NetworkManifest::legs_1_standingSupport);
        if (isEnabled(NetworkManifest::legs_1_standingSupport))
        {
          module_legs_1_standingSupport->feedback(timeStep);
        }
        LegSittingSupport* module_legs_1_sittingSupport = (LegSittingSupport*)getModule(NetworkManifest::legs_1_sittingSupport);
        if (isEnabled(NetworkManifest::legs_1_sittingSupport))
        {
          module_legs_1_sittingSupport->feedback(timeStep);
        }
        LegStep* module_legs_1_step = (LegStep*)getModule(NetworkManifest::legs_1_step);
        if (isEnabled(NetworkManifest::legs_1_step))
        {
          module_legs_1_step->feedback(timeStep);
        }
        LegSpin* module_legs_1_spin = (LegSpin*)getModule(NetworkManifest::legs_1_spin);
        if (isEnabled(NetworkManifest::legs_1_spin))
        {
          module_legs_1_spin->feedback(timeStep);
        }
      }
      module_legs_1->combineFeedbackOutputs();
    }
    Spine* module_spines_0 = (Spine*)getModule(NetworkManifest::spines_0);
    if (isEnabled(NetworkManifest::spines_0))
    {
      module_spines_0->feedback(timeStep);
      {
        SpineSupport* module_spines_0_support = (SpineSupport*)getModule(NetworkManifest::spines_0_support);
        if (isEnabled(NetworkManifest::spines_0_support))
        {
          module_spines_0_support->combineFeedbackInputs();
          module_spines_0_support->feedback(timeStep);
        }
      }
    }
    SceneProbes* module_sceneProbes = (SceneProbes*)getModule(NetworkManifest::sceneProbes);
    if (isEnabled(NetworkManifest::sceneProbes))
    {
      module_sceneProbes->feedback(timeStep);
    }
    BalanceBehaviourFeedback* module_balanceBehaviourFeedback = (BalanceBehaviourFeedback*)getModule(NetworkManifest::balanceBehaviourFeedback);
    if (isEnabled(NetworkManifest::balanceBehaviourFeedback))
    {
      module_balanceBehaviourFeedback->feedback(timeStep);
    }
    // SYNC
    EyesBehaviourInterface* module_eyesBehaviourInterface = (EyesBehaviourInterface*)getModule(NetworkManifest::eyesBehaviourInterface);
    if (isEnabled(NetworkManifest::eyesBehaviourInterface))
    {
      module_eyesBehaviourInterface->combineFeedbackInputs();
      module_eyesBehaviourInterface->feedback(timeStep);
    }
    AimBehaviourInterface* module_aimBehaviourInterface = (AimBehaviourInterface*)getModule(NetworkManifest::aimBehaviourInterface);
    if (isEnabled(NetworkManifest::aimBehaviourInterface))
    {
      module_aimBehaviourInterface->combineFeedbackInputs();
      module_aimBehaviourInterface->feedback(timeStep);
    }
    ReachForWorldBehaviourInterface* module_reachForWorldBehaviourInterface = (ReachForWorldBehaviourInterface*)getModule(NetworkManifest::reachForWorldBehaviourInterface);
    if (isEnabled(NetworkManifest::reachForWorldBehaviourInterface))
    {
      module_reachForWorldBehaviourInterface->combineFeedbackInputs();
      module_reachForWorldBehaviourInterface->feedback(timeStep);
    }
    RandomLook* module_randomLook = (RandomLook*)getModule(NetworkManifest::randomLook);
    if (isEnabled(NetworkManifest::randomLook))
    {
      module_randomLook->combineFeedbackInputs();
    }
    EnvironmentAwareness* module_environmentAwareness = (EnvironmentAwareness*)getModule(NetworkManifest::environmentAwareness);
    if (isEnabled(NetworkManifest::environmentAwareness))
    {
      module_environmentAwareness->combineFeedbackInputs();
      module_environmentAwareness->feedback(timeStep);
    }
    BodyFrame* module_bodyFrame = (BodyFrame*)getModule(NetworkManifest::bodyFrame);
    if (isEnabled(NetworkManifest::bodyFrame))
    {
      module_bodyFrame->combineFeedbackInputs();
      module_bodyFrame->feedback(timeStep);
      {
        SupportPolygon* module_bodyFrame_supportPolygon = (SupportPolygon*)getModule(NetworkManifest::bodyFrame_supportPolygon);
        if (isEnabled(NetworkManifest::bodyFrame_supportPolygon))
        {
          module_bodyFrame_supportPolygon->combineFeedbackInputs();
        }
        SupportPolygon* module_bodyFrame_sittingSupportPolygon = (SupportPolygon*)getModule(NetworkManifest::bodyFrame_sittingSupportPolygon);
        if (isEnabled(NetworkManifest::bodyFrame_sittingSupportPolygon))
        {
          module_bodyFrame_sittingSupportPolygon->combineFeedbackInputs();
        }
        BodyBalance* module_bodyFrame_bodyBalance = (BodyBalance*)getModule(NetworkManifest::bodyFrame_bodyBalance);
        if (isEnabled(NetworkManifest::bodyFrame_bodyBalance))
        {
          module_bodyFrame_bodyBalance->combineFeedbackInputs();
          module_bodyFrame_bodyBalance->feedback(timeStep);
          module_bodyFrame_bodyBalance->combineFeedbackOutputs();
        }
        ReachForBody* module_bodyFrame_reachForBody = (ReachForBody*)getModule(NetworkManifest::bodyFrame_reachForBody);
        if (isEnabled(NetworkManifest::bodyFrame_reachForBody))
        {
          module_bodyFrame_reachForBody->combineFeedbackInputs();
          module_bodyFrame_reachForBody->feedback(timeStep);
        }
        BalanceAssistant* module_bodyFrame_balanceAssistant = (BalanceAssistant*)getModule(NetworkManifest::bodyFrame_balanceAssistant);
        if (isEnabled(NetworkManifest::bodyFrame_balanceAssistant))
        {
          module_bodyFrame_balanceAssistant->combineFeedbackInputs();
        }
        // SYNC
        SittingBodyBalance* module_bodyFrame_sittingBodyBalance = (SittingBodyBalance*)getModule(NetworkManifest::bodyFrame_sittingBodyBalance);
        if (isEnabled(NetworkManifest::bodyFrame_sittingBodyBalance))
        {
          module_bodyFrame_sittingBodyBalance->combineFeedbackInputs();
          module_bodyFrame_sittingBodyBalance->feedback(timeStep);
        }
      }
      module_bodyFrame->combineFeedbackOutputs();
    }
    BodySection* module_upperBody = (BodySection*)getModule(NetworkManifest::upperBody);
    if (isEnabled(NetworkManifest::upperBody))
    {
      module_upperBody->combineFeedbackInputs();
      {
        RotateCore* module_upperBody_rotate = (RotateCore*)getModule(NetworkManifest::upperBody_rotate);
        if (isEnabled(NetworkManifest::upperBody_rotate))
        {
          module_upperBody_rotate->combineFeedbackInputs();
          module_upperBody_rotate->feedback(timeStep);
        }
        PositionCore* module_upperBody_position = (PositionCore*)getModule(NetworkManifest::upperBody_position);
        if (isEnabled(NetworkManifest::upperBody_position))
        {
          module_upperBody_position->combineFeedbackInputs();
          module_upperBody_position->feedback(timeStep);
        }
        BraceChooser* module_upperBody_braceChooser = (BraceChooser*)getModule(NetworkManifest::upperBody_braceChooser);
        if (isEnabled(NetworkManifest::upperBody_braceChooser))
        {
          module_upperBody_braceChooser->combineFeedbackInputs();
        }
      }
      module_upperBody->combineFeedbackOutputs();
    }
    BodySection* module_lowerBody = (BodySection*)getModule(NetworkManifest::lowerBody);
    if (isEnabled(NetworkManifest::lowerBody))
    {
      module_lowerBody->combineFeedbackInputs();
      {
        RotateCore* module_lowerBody_rotate = (RotateCore*)getModule(NetworkManifest::lowerBody_rotate);
        if (isEnabled(NetworkManifest::lowerBody_rotate))
        {
          module_lowerBody_rotate->combineFeedbackInputs();
          module_lowerBody_rotate->feedback(timeStep);
        }
        PositionCore* module_lowerBody_position = (PositionCore*)getModule(NetworkManifest::lowerBody_position);
        if (isEnabled(NetworkManifest::lowerBody_position))
        {
          module_lowerBody_position->combineFeedbackInputs();
          module_lowerBody_position->feedback(timeStep);
        }
        BraceChooser* module_lowerBody_braceChooser = (BraceChooser*)getModule(NetworkManifest::lowerBody_braceChooser);
        if (isEnabled(NetworkManifest::lowerBody_braceChooser))
        {
          module_lowerBody_braceChooser->combineFeedbackInputs();
        }
      }
      module_lowerBody->combineFeedbackOutputs();
    }
    // SYNC
    ReachForBodyBehaviourInterface* module_reachForBodyBehaviourInterface = (ReachForBodyBehaviourInterface*)getModule(NetworkManifest::reachForBodyBehaviourInterface);
    if (isEnabled(NetworkManifest::reachForBodyBehaviourInterface))
    {
      module_reachForBodyBehaviourInterface->combineFeedbackInputs();
      module_reachForBodyBehaviourInterface->feedback(timeStep);
    }
    BalancePoserBehaviourInterface* module_balancePoserBehaviourInterface = (BalancePoserBehaviourInterface*)getModule(NetworkManifest::balancePoserBehaviourInterface);
    if (isEnabled(NetworkManifest::balancePoserBehaviourInterface))
    {
      module_balancePoserBehaviourInterface->combineFeedbackInputs();
    }
    ObserveBehaviourInterface* module_observeBehaviourInterface = (ObserveBehaviourInterface*)getModule(NetworkManifest::observeBehaviourInterface);
    if (isEnabled(NetworkManifest::observeBehaviourInterface))
    {
      module_observeBehaviourInterface->combineFeedbackInputs();
      module_observeBehaviourInterface->feedback(timeStep);
    }
    IdleAwakeBehaviourInterface* module_idleAwakeBehaviourInterface = (IdleAwakeBehaviourInterface*)getModule(NetworkManifest::idleAwakeBehaviourInterface);
    if (isEnabled(NetworkManifest::idleAwakeBehaviourInterface))
    {
      module_idleAwakeBehaviourInterface->combineFeedbackInputs();
      module_idleAwakeBehaviourInterface->feedback(timeStep);
    }
    SitBehaviourInterface* module_sitBehaviourInterface = (SitBehaviourInterface*)getModule(NetworkManifest::sitBehaviourInterface);
    if (isEnabled(NetworkManifest::sitBehaviourInterface))
    {
      module_sitBehaviourInterface->combineFeedbackInputs();
      module_sitBehaviourInterface->feedback(timeStep);
      module_sitBehaviourInterface->combineFeedbackOutputs();
    }
    HazardManagement* module_hazardManagement = (HazardManagement*)getModule(NetworkManifest::hazardManagement);
    if (isEnabled(NetworkManifest::hazardManagement))
    {
      module_hazardManagement->combineFeedbackInputs();
      {
        ImpactPredictor* module_hazardManagement_chestImpactPredictor = (ImpactPredictor*)getModule(NetworkManifest::hazardManagement_chestImpactPredictor);
        if (isEnabled(NetworkManifest::hazardManagement_chestImpactPredictor))
        {
          module_hazardManagement_chestImpactPredictor->combineFeedbackInputs();
          module_hazardManagement_chestImpactPredictor->feedback(timeStep);
        }
        ShieldManagement* module_hazardManagement_shieldManagement = (ShieldManagement*)getModule(NetworkManifest::hazardManagement_shieldManagement);
        if (isEnabled(NetworkManifest::hazardManagement_shieldManagement))
        {
          module_hazardManagement_shieldManagement->combineFeedbackInputs();
          module_hazardManagement_shieldManagement->feedback(timeStep);
        }
        // SYNC
        Grab* module_hazardManagement_grab = (Grab*)getModule(NetworkManifest::hazardManagement_grab);
        if (isEnabled(NetworkManifest::hazardManagement_grab))
        {
          module_hazardManagement_grab->combineFeedbackInputs();
          module_hazardManagement_grab->feedback(timeStep);
        }
        HazardResponse* module_hazardManagement_hazardResponse = (HazardResponse*)getModule(NetworkManifest::hazardManagement_hazardResponse);
        if (isEnabled(NetworkManifest::hazardManagement_hazardResponse))
        {
          module_hazardManagement_hazardResponse->combineFeedbackInputs();
        }
        FreeFallManagement* module_hazardManagement_freeFallManagement = (FreeFallManagement*)getModule(NetworkManifest::hazardManagement_freeFallManagement);
        if (isEnabled(NetworkManifest::hazardManagement_freeFallManagement))
        {
          module_hazardManagement_freeFallManagement->combineFeedbackInputs();
        }
        // SYNC
        GrabDetection* module_hazardManagement_grabDetection = (GrabDetection*)getModule(NetworkManifest::hazardManagement_grabDetection);
        if (isEnabled(NetworkManifest::hazardManagement_grabDetection))
        {
          module_hazardManagement_grabDetection->combineFeedbackInputs();
          module_hazardManagement_grabDetection->feedback(timeStep);
        }
      }
    }
    BalanceManagement* module_balanceManagement = (BalanceManagement*)getModule(NetworkManifest::balanceManagement);
    if (isEnabled(NetworkManifest::balanceManagement))
    {
      module_balanceManagement->combineFeedbackInputs();
      {
        StaticBalance* module_balanceManagement_staticBalance = (StaticBalance*)getModule(NetworkManifest::balanceManagement_staticBalance);
        if (isEnabled(NetworkManifest::balanceManagement_staticBalance))
        {
          module_balanceManagement_staticBalance->combineFeedbackInputs();
        }
        SteppingBalance* module_balanceManagement_steppingBalance = (SteppingBalance*)getModule(NetworkManifest::balanceManagement_steppingBalance);
        if (isEnabled(NetworkManifest::balanceManagement_steppingBalance))
        {
          module_balanceManagement_steppingBalance->combineFeedbackInputs();
          module_balanceManagement_steppingBalance->feedback(timeStep);
        }
      }
    }
    // SYNC
    BalanceBehaviourInterface* module_balanceBehaviourInterface = (BalanceBehaviourInterface*)getModule(NetworkManifest::balanceBehaviourInterface);
    if (isEnabled(NetworkManifest::balanceBehaviourInterface))
    {
      module_balanceBehaviourInterface->combineFeedbackInputs();
      module_balanceBehaviourInterface->feedback(timeStep);
    }
    HazardAwarenessBehaviourInterface* module_hazardAwarenessBehaviourInterface = (HazardAwarenessBehaviourInterface*)getModule(NetworkManifest::hazardAwarenessBehaviourInterface);
    if (isEnabled(NetworkManifest::hazardAwarenessBehaviourInterface))
    {
      module_hazardAwarenessBehaviourInterface->combineFeedbackInputs();
      module_hazardAwarenessBehaviourInterface->feedback(timeStep);
      module_hazardAwarenessBehaviourInterface->combineFeedbackOutputs();
    }
    UserHazardBehaviourInterface* module_userHazardBehaviourInterface = (UserHazardBehaviourInterface*)getModule(NetworkManifest::userHazardBehaviourInterface);
    if (isEnabled(NetworkManifest::userHazardBehaviourInterface))
    {
      module_userHazardBehaviourInterface->combineFeedbackInputs();
    }
    ShieldBehaviourInterface* module_shieldBehaviourInterface = (ShieldBehaviourInterface*)getModule(NetworkManifest::shieldBehaviourInterface);
    if (isEnabled(NetworkManifest::shieldBehaviourInterface))
    {
      module_shieldBehaviourInterface->combineFeedbackInputs();
      module_shieldBehaviourInterface->feedback(timeStep);
    }
    HoldBehaviourInterface* module_holdBehaviourInterface = (HoldBehaviourInterface*)getModule(NetworkManifest::holdBehaviourInterface);
    if (isEnabled(NetworkManifest::holdBehaviourInterface))
    {
      module_holdBehaviourInterface->combineFeedbackInputs();
      module_holdBehaviourInterface->feedback(timeStep);
    }
    HoldActionBehaviourInterface* module_holdActionBehaviourInterface = (HoldActionBehaviourInterface*)getModule(NetworkManifest::holdActionBehaviourInterface);
    if (isEnabled(NetworkManifest::holdActionBehaviourInterface))
    {
      module_holdActionBehaviourInterface->combineFeedbackInputs();
      module_holdActionBehaviourInterface->feedback(timeStep);
    }
    ShieldActionBehaviourInterface* module_shieldActionBehaviourInterface = (ShieldActionBehaviourInterface*)getModule(NetworkManifest::shieldActionBehaviourInterface);
    if (isEnabled(NetworkManifest::shieldActionBehaviourInterface))
    {
      module_shieldActionBehaviourInterface->combineFeedbackInputs();
      module_shieldActionBehaviourInterface->feedback(timeStep);
    }
  }
}

} // namespace NM_BEHAVIOUR_LIB_NAMESPACE

