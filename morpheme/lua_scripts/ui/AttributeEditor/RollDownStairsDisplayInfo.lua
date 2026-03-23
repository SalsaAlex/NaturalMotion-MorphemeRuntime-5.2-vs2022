------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"

------------------------------------------------------------------------------------------------------------------------
-- Add a rollDownStairsDisplayInfoSection
-- Used by OperatorRollDownStairs
------------------------------------------------------------------------------------------------------------------------
attributeEditor.rollDownStairsDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.rollDownStairsDisplayInfoSection")

  -- Parameters
  --
  local rollup = rollContainer:addRollup{ label = "Parameters", flags = "mainSection", name = "rollDownStairsDisplayInfoSection_parameters"}
  local rollPanel = rollup:getPanel()
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    rollPanel:addHSpacer(6)
    rollPanel:setBorder(1)
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      rollPanel:setFlexGridColumnExpandable(2)

      -- Reference Part Index
      --
      attributeEditor.addAttributeLabel(rollPanel, "ReferencePartIndex", selection, "ReferencePartIndex")
      attributeEditor.addAttributeWidget(rollPanel, "ReferencePartIndex", selection)

    rollPanel:endSizer()
  rollPanel:endSizer()

  -- Is Rolling When
  --
  local rollup = rollContainer:addRollup{ label = "Is Rolling When", flags = "mainSection", name = "rollDownStairsDisplayInfoSection_isRollingWhen"}
  attributeEditor.bindHelpToWidget(rollup:getHeader(), "Determines when the character is considered to be Rolling")

  local rollPanel = rollup:getPanel()
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    rollPanel:addHSpacer(6)
    rollPanel:setBorder(1)
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      rollPanel:setFlexGridColumnExpandable(2)

      -- Speed Greater Than
      --
      attributeEditor.addAttributeLabel(rollPanel, "Velocity Ratio Threshold", selection, "MinMotionRatioToBeConsideredRolling")
      attributeEditor.addAttributeWidget(rollPanel, "MinMotionRatioToBeConsideredRolling", selection)

    rollPanel:endSizer()
  rollPanel:endSizer()

  -- Crossfade from Push to Pose
  --
  local rollup = rollContainer:addRollup{ label = "Crossfade from Push to Pose", flags = "mainSection", name = "rollDownStairsDisplayInfoSection_crossfadeFromPushToPose"}
  attributeEditor.bindHelpToWidget(rollup:getHeader(), "The behaviour applies a Push force at low speed and a Pose at high speed. Linear interpolation from Push To Pose occurs when either the speed down stairs or the angular speed is in the range described below. Only the parameter producing the highest weight is used." )

  local rollPanel = rollup:getPanel()
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    rollPanel:addHSpacer(6)
    rollPanel:setBorder(1)
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      rollPanel:setFlexGridColumnExpandable(2)

      -- Speed down stairs greater than
      --
      attributeEditor.addAttributeLabel(rollPanel, "Speed Down Slope Min", selection, "MinTangentialVelocityToApplyRollPose")
      attributeEditor.addAttributeWidget(rollPanel, "MinTangentialVelocityToApplyRollPose", selection)

      -- Speed down stairs less than
      --
      attributeEditor.addAttributeLabel(rollPanel, "Speed Down Slope Max", selection, "MaxTangentialVelocityToApplyRollPose")
      attributeEditor.addAttributeWidget(rollPanel, "MaxTangentialVelocityToApplyRollPose", selection)

      -- Angular speed greater than
      --
      attributeEditor.addAttributeLabel(rollPanel, "Angular Speed Min", selection, "MinAngularVelocityToApplyRollPose")
      attributeEditor.addAttributeWidget(rollPanel, "MinAngularVelocityToApplyRollPose", selection)

      -- Angular speed less than
      --
      attributeEditor.addAttributeLabel(rollPanel, "Angular Speed Max", selection, "MaxAngularVelocityToApplyRollPose")
      attributeEditor.addAttributeWidget(rollPanel, "MaxAngularVelocityToApplyRollPose", selection)

    rollPanel:endSizer()
  rollPanel:endSizer()

  -- Blend In Pose
  --
  local rollup = rollContainer:addRollup{ label = "Blend In Pose", flags = "mainSection", name = "rollDownStairsDisplayInfoSection_blendInPose"}
  attributeEditor.bindHelpToWidget(rollup:getHeader(), "The angle between the reference parts forward direction and the slope normal determines the blend weight of the Pose. That weight scales linearly from 0 to 1 as the angle ranges from min to max. This can be used to tuck in limbs as the character rolls onto its front ( entering the roll ) and let them lose when rolling onto its back ( exiting )");

  local rollPanel = rollup:getPanel()
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    rollPanel:addHSpacer(6)
    rollPanel:setBorder(1)
    rollPanel:beginFlexGridSizer{ cols = 3, flags = "expand", proportion = 1 }
      rollPanel:setFlexGridColumnExpandable(1)

      rollPanel:addStaticText{ text = "" }
      rollPanel:addStaticText{ text = "Min" }
      rollPanel:addStaticText{ text = "Max" }

      -- Angle To Slope Normal On Enter
      --
      attributeEditor.addStaticTextWithHelp(rollPanel, "Rolling From Back To Front", "The range of angles between the reference part and the slope normal in which the Pose blend occurs on rolling from back to front. In degrees." )
      attributeEditor.addAttributeWidget(rollPanel, "MinEnterAngleWithNormalToApplyRollPose", selection)
      attributeEditor.addAttributeWidget(rollPanel, "MaxEnterAngleWithNormalToApplyRollPose", selection)

      -- Angle To Slope Normal On Exit
      --
      attributeEditor.addStaticTextWithHelp(rollPanel, "Rolling From Front To Back", "The range of angles between the reference part and the slope normal in which the Pose blend occurs on rolling from front to back. In degrees." )
      attributeEditor.addAttributeWidget(rollPanel, "MinExitAngleWithNormalToApplyRollPose", selection)
      attributeEditor.addAttributeWidget(rollPanel, "MaxExitAngleWithNormalToApplyRollPose", selection)

    rollPanel:endSizer()
  rollPanel:endSizer()

  -- Input Defaults
  --
  local rollup = rollContainer:addRollup{ label = "Input Defaults", flags = "mainSection", name = "rollDownStairsDisplayInfoSection_inputDefaults"}
  attributeEditor.bindHelpToWidget(rollup:getHeader(), "Default values used by unconnected input pins.");

  local rollPanel = rollup:getPanel()
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    rollPanel:addHSpacer(6)
    rollPanel:setBorder(1)
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      rollPanel:setFlexGridColumnExpandable(2)

      -- Max Push
      --
      attributeEditor.addAttributeLabel(rollPanel, "Max Push", selection, "PushAccelerationMaxMagnitude")
      attributeEditor.addAttributeWidget(rollPanel, "PushAccelerationMaxMagnitude", selection)

    rollPanel:endSizer()
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.rollDownStairsDisplayInfoSection")
end
