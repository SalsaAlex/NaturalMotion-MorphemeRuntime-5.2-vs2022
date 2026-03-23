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
-- Add's a display info section containing character controller scale attributes.
-- Use by SoftKeyFrame and HardKeyFrame.
------------------------------------------------------------------------------------------------------------------------
local addControllerScaleDisplayInfoSection = function(panel, selection, attributes, set)

  attributeEditor.logEnterFunc("attributeEditor.addControllerScaleDisplayInfoSection")

  attributeEditor.log("panel:beginVSizer")
  panel:beginVSizer{ flags = "expand" }

    attributeEditor.log("panel:beginFlexGridSizer")
    panel:beginFlexGridSizer{ cols = 3, flags = "expand", proportion = 1 }
      panel:setFlexGridColumnExpandable(3)

      panel:addHSpacer(6)
      attributeEditor.addAttributeLabel(panel, "Radius", selection, "ControllerRadiusFraction")
      radiusFractionWidget = attributeEditor.addAttributeWidget(panel, "ControllerRadiusFraction", selection, set)
      radiusFractionWidget:enable( true )

      panel:addHSpacer(6)
      attributeEditor.addAttributeLabel(panel, "Height", selection, "ControllerHeightFraction")
      heightFractionWidget = attributeEditor.addAttributeWidget(panel, "ControllerHeightFraction", selection, set)
      heightFractionWidget:enable( true )

    attributeEditor.log("panel:endSizer")
    panel:endSizer()

  attributeEditor.log("panel:endSizer")
  panel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.addControllerScaleDisplayInfoSection")

end

attributeEditor.controllerScaleDisplayInfoSection = function(rollContainer, displayInfo, selection)

  attributeEditor.logEnterFunc("attributeEditor.controllerScaleDisplayInfoSection")

  attributeEditor.log("rollContainer:addRollup")
 
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "controllerScaleDisplayInfoSection" }
  local rollPanel = rollup:getPanel()

    attributeEditor.addAnimationSetWidget(rollPanel, displayInfo.usedAttributes, selection, addControllerScaleDisplayInfoSection)

    attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.controllerScaleDisplayInfoSection")
end
