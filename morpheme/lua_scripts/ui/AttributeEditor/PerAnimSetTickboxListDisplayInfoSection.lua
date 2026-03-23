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
-- Adds a rollup for boolean attributes.
------------------------------------------------------------------------------------------------------------------------
local addPerAnimSetTickboxListDisplayInfoSection = function(panel, selection, attributes, set)
  attributeEditor.logEnterFunc("attributeEditor.addPerAnimSetTickboxListDisplayInfoSection")
  
  attributeEditor.log("panel:beginVSizer")
  panel:beginVSizer{ flags = "expand" }

    attributeEditor.log("panel:beginFlexGridSizer")
    panel:beginFlexGridSizer{ cols = 3, flags = "expand", proportion = 1 }
      panel:setFlexGridColumnExpandable(3)

      for _, attributeName in ipairs(attributes) do
        -- Create checkbox widget for each attribute.
        panel:addHSpacer(6)
        local name = utils.getDisplayString(getAttributeDisplayName(selection[1], attributeName))
        local label = attributeEditor.addAttributeLabel(panel, name, selection, attributeName)
        local checkBox = panel:addCheckBox{ flags = "expand" }
        bindWidgetToAttribute(checkBox, selection, attributeName, set)
        attributeEditor.bindAttributeHelpToWidget(checkBox, selection, attributeName)
      end

    attributeEditor.log("panel:endSizer")
    panel:endSizer()

  attributeEditor.log("panel:endSizer")
  panel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.addPerAnimSetTickboxListDisplayInfoSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Adds a display info section for per-anim-set boolean attributes.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.perAnimSetTickboxListDisplayInfoSection = function(rollContainer, displayInfo, selection)

  attributeEditor.logEnterFunc("attributeEditor.perAnimSetTickboxListDisplayInfoSection")

  attributeEditor.log("rollContainer:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "perAnimSetTickboxListDisplayInfoSection" }
  local rollPanel = rollup:getPanel()
  
    attributeEditor.addAnimationSetWidget(rollPanel, displayInfo.usedAttributes, selection, addPerAnimSetTickboxListDisplayInfoSection)
  
    attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.perAnimSetTickboxListDisplayInfoSection")
end
