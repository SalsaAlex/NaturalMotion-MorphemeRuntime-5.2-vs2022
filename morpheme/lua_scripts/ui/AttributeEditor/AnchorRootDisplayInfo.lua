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
local addAnchorRootDisplayInfoSection = function(panel, selection, attributes, set)
  attributeEditor.logEnterFunc("attributeEditor.addAnchorRootDisplayInfoSection")

  attributeEditor.log("panel:beginVSizer")
  panel:beginVSizer{ flags = "expand" }

    attributeEditor.log("panel:beginFlexGridSizer")
    panel:beginFlexGridSizer{ cols = 3, flags = "expand", proportion = 1 }
      panel:setFlexGridColumnExpandable(2)
      panel:setFlexGridColumnExpandable(3)

      local useRootAsAnchorHelp = getAttributeHelpText(selection[1], "UseRootAsAnchor")

      panel:addHSpacer(6)
      attributeEditor.addAttributeLabel(panel, "Method", selection, "UseRootAsAnchor")

      attributeWidget = attributeEditor.addBoolAttributeCombo{
        panel = panel,
        flags = expand,
        proportion = 1,
        attribute = "UseRootAsAnchor",
        objects = selection,
        trueValue ="Character (world space)",
        falseValue = "Default",
        helpText = useRootAsAnchorHelp
      }

      panel:addVSpacer(3)
      
      attributeWidget:enable( true )

    attributeEditor.log("panel:endSizer")
    panel:endSizer()

  attributeEditor.log("panel:endSizer")
  panel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.addAnchorRootDisplayInfoSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Add's a display info section containing physX 3 joint compliance attributes.for all anim sets.
-- Used by SoftKeyFrame and HardKeyFrame.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.anchorRootDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.anchorRootDisplayInfoSection")

  attributeEditor.log("rollContainer:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "anchorRootDisplayInfoSection" }
  local rollPanel = rollup:getPanel()
  
    attributeEditor.addAnimationSetWidget(rollPanel, displayInfo.usedAttributes, selection, addAnchorRootDisplayInfoSection)
  
    attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.anchorRootDisplayInfoSection")
end
