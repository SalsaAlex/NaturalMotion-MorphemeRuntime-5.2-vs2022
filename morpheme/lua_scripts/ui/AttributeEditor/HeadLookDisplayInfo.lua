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
-- Add's a headLookDisplayInfoSection.
-- Used by HeadLook.
------------------------------------------------------------------------------------------------------------------------
local perAnimSetHeadLookDisplayInfoSection = function(panel, selection, attributes, set)
  attributeEditor.logEnterFunc("perAnimSetHeadLookDisplayInfoSection")

  local pointingVectorLabel = nil
  local pointingVectorWidget = { }
  local effectorOffsetLabel = nil
  local effectorOffsetWidget = { }
  local preserveDirectionLabel = nil
  local perserveDirectionWidget = { }

  -- first add the ui for the section
  attributeEditor.log("panel:beginHSizer")
  panel:beginHSizer{ flags = "expand", proportion = 1 }
    panel:addHSpacer(6)
    panel:setBorder(1)

    attributeEditor.log("panel:beginFlexGridSizer")
    panel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      panel:setFlexGridColumnExpandable(2)

      pointingVectorLabel = panel:addStaticText{
        text = "Vector",
        onMouseEnter = function()
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "PointingVectorX"))
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      local pointingAttrs = { "PointingVectorX", "PointingVectorY", "PointingVectorZ" }
      pointingVectorWidget = attributeEditor.addVectorAttributeWidget(panel, pointingAttrs, selection, set)

      effectorOffsetLabel = panel:addStaticText{
        text = "Offset",
        onMouseEnter = function()
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "EndEffectorOffsetX"))
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      local effectorAttrs = { "EndEffectorOffsetX", "EndEffectorOffsetY", "EndEffectorOffsetZ" }
      effectorOffsetWidget = attributeEditor.addVectorAttributeWidget(panel, effectorAttrs, selection, set)

      attributeEditor.log("panel:endSizer")
    panel:endSizer()

    attributeEditor.log("panel:endSizer")
  panel:endSizer()

  attributeEditor.logExitFunc("perAnimSetHeadLookDisplayInfoSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Adds a headLookDisplayInfoSection.
-- Used by HeadLook.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.headLookDisplayInfoSection = function(panel, displayInfo, selection)
  local attrs = checkSetOrderAndBuildAttributeList(selection, displayInfo.usedAttributes)

  local rollup = panel:addRollup{
    label = displayInfo.title,
    flags = "mainSection",
    name = string.format("headLookDisplayInfoSection")
  }
  local rollPanel = rollup:getPanel()

  rollPanel:beginVSizer{ flags = "expand" }
    rollPanel:addAnimationSetWidget{
      attributes = attrs,
      displayFunc = perAnimSetHeadLookDisplayInfoSection,
      flags = "expand",
      proportion = 1,
    }
  rollPanel:endSizer()
end
