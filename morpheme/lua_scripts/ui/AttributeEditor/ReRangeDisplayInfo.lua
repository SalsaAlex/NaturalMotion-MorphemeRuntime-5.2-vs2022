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
-- Add a reRangeDisplayInfoSection
-- Used by OperatorFallOverWall
------------------------------------------------------------------------------------------------------------------------
attributeEditor.reRangeDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.reRangeDisplayInfoSection")

  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", 
  name = "reRangeDisplayInfoSection"  }
  local rollPanel = rollup:getPanel()

  -- first add the ui for the section
  attributeEditor.log("rollPanel:beginHSizer")
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }

    rollPanel:addHSpacer(6)
    rollPanel:setBorder(1)

    attributeEditor.log("rollPanel:beginFlexGridSizer")
    rollPanel:beginFlexGridSizer{ cols = 3, flags = "expand", proportion = 1 }

      rollPanel:setFlexGridColumnExpandable(1)
      rollPanel:setFlexGridColumnExpandable(3)

      -- col 1
      rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
        rollPanel:addStretchSpacer{ proportion = 1 }
        rollPanel:addStaticText{ text = "Range In" }:setFont("bold")
        rollPanel:addStretchSpacer{ proportion = 1 }
      rollPanel:endSizer()
     
      -- col 2
      rollPanel:addHSpacer(8)
      
      -- col 3
      rollPanel:beginHSizer{ flags = "expand"}
        rollPanel:addStretchSpacer{ proportion = 1 }
        rollPanel:addStaticText{ text = "Range Out" }:setFont("bold")
        rollPanel:addStretchSpacer{ proportion = 1 }
      rollPanel:endSizer()
     
      -- col 1
      rollPanel:beginHSizer{ flags = "expand"}
        attributeEditor.addAttributeWidget(rollPanel, "InputRange1", selection, set)
        rollPanel:addStaticText{ text = ".." }
        attributeEditor.addAttributeWidget(rollPanel, "InputRange2", selection, set)
      rollPanel:endSizer()
      
      -- col 2
      rollPanel:addHSpacer(8)

      -- col 3
      rollPanel:beginHSizer{ flags = "expand"}
        attributeEditor.addAttributeWidget(rollPanel, "OutputRange1", selection, set)
        rollPanel:addStaticText{ text = ".." }
        attributeEditor.addAttributeWidget(rollPanel, "OutputRange2", selection, set)
      rollPanel:endSizer()

      attributeEditor.log("rollPanel:endSizer")
    rollPanel:endSizer()

    attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.reRangeDisplayInfoSection")
end
