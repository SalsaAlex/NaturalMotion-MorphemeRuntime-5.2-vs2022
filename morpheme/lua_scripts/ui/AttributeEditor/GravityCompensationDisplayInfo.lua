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
-- Add's a display info section containing gravity compensation attributes.
-- Use by SoftKeyFrame.
------------------------------------------------------------------------------------------------------------------------
local addGravityCompensationDisplayInfoSection = function(panel, selection, attributes, set)
  attributeEditor.logEnterFunc("attributeEditor.addGravityCompensationDisplayInfoSection")

  attributeEditor.log("panel:beginVSizer")
  panel:beginVSizer{ flags = "expand" }

    attributeEditor.log("panel:beginFlexGridSizer")
    panel:beginFlexGridSizer{ cols = 3, flags = "expand", proportion = 1 }
      panel:setFlexGridColumnExpandable(3)

      panel:addHSpacer(6)
      attributeEditor.addAttributeLabel(panel, "", selection, "GravityCompensation")
      radiusFractionWidget = attributeEditor.addAttributeWidget(panel, "GravityCompensation", selection, set)
      radiusFractionWidget:enable( true )

    attributeEditor.log("panel:endSizer")
    panel:endSizer()

  attributeEditor.log("panel:endSizer")
  panel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.addGravityCompensationDisplayInfoSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Add's a display info section containing physX 3 joint compliance attributes.for all anim sets.
-- Used by SoftKeyFrame and HardKeyFrame.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.gravityCompensationDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.gravityCompensationDisplayInfoSection")

  attributeEditor.log("rollContainer:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "gravityCompensationDisplayInfoSection" }
  local rollPanel = rollup:getPanel()
  
    attributeEditor.addAnimationSetWidget(rollPanel, displayInfo.usedAttributes, selection, addGravityCompensationDisplayInfoSection)
  
    attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.gravityCompensationDisplayInfoSection")
end
