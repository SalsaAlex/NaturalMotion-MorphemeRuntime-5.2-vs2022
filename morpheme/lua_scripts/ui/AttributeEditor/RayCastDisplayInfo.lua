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
-- Add's a rayCastDisplayInfoSection
-- Used by OperatorRayCast
------------------------------------------------------------------------------------------------------------------------
attributeEditor.rayCastDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.rayCastDisplayInfoSection")

  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "rayCastDisplayInfoSection"  }
  local rollPanel = rollup:getPanel()

  -- first add the ui for the section
  attributeEditor.log("rollPanel:beginHSizer")
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }

    rollPanel:addHSpacer(6)
    rollPanel:setBorder(1)

    attributeEditor.log("rollPanel:beginFlexGridSizer")
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }

      rollPanel:setFlexGridColumnExpandable(2)

      rollPanel:addStaticText{
        text = "Start",
        onMouseEnter = function()
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "RayStartX"))
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      local rayStartAttrs = { "RayStartX", "RayStartY", "RayStartZ" }
      attributeEditor.addVectorAttributeWidget(rollPanel, rayStartAttrs, selection)

      rollPanel:addStaticText{
        text = "Delta",
        onMouseEnter = function()
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "RayDeltaX"))
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      local rayDeltaAttrs = { "RayDeltaX", "RayDeltaY", "RayDeltaZ" }
      attributeEditor.addVectorAttributeWidget(rollPanel, rayDeltaAttrs, selection)

      attributeEditor.log("rollPanel:endSizer")
    rollPanel:endSizer()

    attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.rayCastDisplayInfoSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Add's a rayCastDisplayInfo
-- Used by RayHit
------------------------------------------------------------------------------------------------------------------------
attributeEditor.rayCastDisplayInfo = function(rollPanel, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.rayCastDisplayInfo")

  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
  rollPanel:addStaticText{
    text = "Offset",
    onMouseEnter = function()
      attributeEditor.setHelpText(getAttributeHelpText(selection[1], "RayStartX"))
    end,
    onMouseLeave = function()
      attributeEditor.clearHelpText()
    end
  }

  local rayStartAttrs = { "RayStartX", "RayStartY", "RayStartZ" }
  attributeEditor.addVectorAttributeWidget(rollPanel, rayStartAttrs, selection)

  rollPanel:endSizer()
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
  rollPanel:addStaticText{
    text = "Vector",
    onMouseEnter = function()
      attributeEditor.setHelpText(getAttributeHelpText(selection[1], "RayDeltaX"))
    end,
    onMouseLeave = function()
      attributeEditor.clearHelpText()
    end
  }

  local rayDeltaAttrs = { "RayDeltaX", "RayDeltaY", "RayDeltaZ" }
  attributeEditor.addVectorAttributeWidget(rollPanel, rayDeltaAttrs, selection)

  rollPanel:endSizer()
  attributeEditor.logExitFunc("attributeEditor.rayCastDisplayInfo")
end

