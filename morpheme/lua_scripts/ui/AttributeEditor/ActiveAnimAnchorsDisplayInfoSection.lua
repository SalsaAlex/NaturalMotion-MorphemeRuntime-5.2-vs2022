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
-- Add's a display info section containing anchor attributes.
-- Use by SoftKeyFrame and HardKeyFrame.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.activeAnimAnchorsDisplayInfoSection = function(rollContainer, displayInfo, selection)

  attributeEditor.logEnterFunc("attributeEditor.activeAnimAnchorsDisplayInfoSection")

  attributeEditor.log("rollContainer:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "activeAnimAnchorsDisplayInfoSection" }
  local rollPanel = rollup:getPanel()
 -- rollup:expand(false)

  attributeEditor.log("rollPanel:beginHSizer")
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    rollPanel:addHSpacer(6)
    rollPanel:setBorder(1)

    local useAsKeyframeAnchorHelp = getAttributeHelpText(selection[1], "UseAsKeyframeAnchor")

    rollPanel:addStaticText{
      text = "HK/SK Anchor Output",
      onMouseEnter = function()
        attributeEditor.setHelpText(useAsKeyframeAnchorHelp)
      end,
      onMouseLeave = function()
        attributeEditor.clearHelpText()
      end
    }

    local UseAsKeyframeAnchorComboBox = attributeEditor.addBoolAttributeCombo{
      panel = rollPanel,
      attribute = "UseAsKeyframeAnchor",
      objects = selection,
      trueValue = "Source",
      falseValue = "Bind Pose",
      helpText = useAsKeyframeAnchorHelp
    }

    attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  -- disable controls in reference networks
  for i, object in ipairs(selection) do
    if isReferenced(object) then
      UseAsKeyframeAnchorComboBox:enable(false)
      break
    end
  end

  attributeEditor.logExitFunc("attributeEditor.activeAnimAnchorsDisplayInfoSection")

end

