------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2011 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"

------------------------------------------------------------------------------------------------------------------------
-- create a per animation set joint properties section
------------------------------------------------------------------------------------------------------------------------
local addJointPropertiesDisplayInfoSection = function(panel, selection, attributes, set)
  attributeEditor.logEnterFunc("addJointPropertiesDisplayInfoSection")

  local strengthMultiplierWidget, dampingMultiplierWidget

  attributeEditor.log("panel:beginVSizer")
  panel:beginVSizer{ flags = "expand" }

    -- Enable Joint Limits checkbox
    panel:beginHSizer{ flags = "expand" }
      panel:addHSpacer(6)
      local EnableJointLimitsLabel = attributeEditor.addAttributeLabel(panel, "Enable Limits", selection, "EnableJointLimits")
      local EnableJointLimitsCheckbox = panel:addCheckBox{ flags = "expand" }
      bindWidgetToAttribute(EnableJointLimitsCheckbox, selection, "EnableJointLimits", set)
      attributeEditor.bindAttributeHelpToWidget(EnableJointLimitsCheckbox, selection, "EnableJointLimits")
    panel:endSizer()

    panel:addVSpacer(3)

    attributeEditor.log("panel:beginFlexGridSizer")
    panel:beginFlexGridSizer{ cols = 3, flags = "expand", proportion = 1 }
      panel:setFlexGridColumnExpandable(3)

      panel:addHSpacer(6)
      attributeEditor.addAttributeLabel(panel, "Strength", selection, "PoseStrengthMultiplier")
      strengthMultiplierWidget = attributeEditor.addAttributeWidget(panel, "PoseStrengthMultiplier", selection, set)
      strengthMultiplierWidget:enable( true )

      panel:addHSpacer(6)
      attributeEditor.addAttributeLabel(panel, "Damping", selection, "DampingMultiplier")
      dampingMultiplierWidget = attributeEditor.addAttributeWidget(panel, "DampingMultiplier", selection, set)
      dampingMultiplierWidget:enable( true )

    attributeEditor.log("panel:endSizer")
    panel:endSizer()

  attributeEditor.log("panel:endSizer")
  panel:endSizer()

  -- Create a change context to watch the EnableJointLimits
  local changeContext = attributeEditor.createChangeContext()
  changeContext:setObjects(selection)
  changeContext:addAttributeChangeEvent("EnableJointLimits")

  attributeEditor.logExitFunc("addJointPropertiesDisplayInfoSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Add's a display info section for per anim set joint properties attributes.
-- Used by SoftKeyFrameActiveAnimation.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.jointPropertiesDisplayInfoSection = function(panel, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.jointPropertiesDisplayInfoSection")

  attributeEditor.log("rollContainter:addRollup")
  local rollup = panel:addRollup{ label = displayInfo.title, flags = "mainSection", name = "jointPropertiesDisplayInfoSection" }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginVSizer")
  rollPanel:beginVSizer{ flags = "expand" }

    attributeEditor.addAnimationSetWidget(rollPanel, displayInfo.usedAttributes, selection, addJointPropertiesDisplayInfoSection)

  attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.jointPropertiesDisplayInfoSection")
end