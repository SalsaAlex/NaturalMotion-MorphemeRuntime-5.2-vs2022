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
-- Add's a HitModeDisplayInfo
-- Used by RayHit
------------------------------------------------------------------------------------------------------------------------
attributeEditor.HitModeDisplayInfo = function(rollPanel, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.HitModeDisplayInfo")

  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }

  local HitModeHelpText = getAttributeHelpText(selection[1], "HitMode")

  rollPanel:addStaticText{
    text = "Mode",
    onMouseEnter = function()
      attributeEditor.setHelpText(HitModeHelpText)
    end,
    onMouseLeave = function()
      attributeEditor.clearHelpText()
    end
  }

  attributeEditor.log("attributeEditor.addIntAttributeCombo")
  attributeEditor.addIntAttributeCombo{
    panel = rollPanel,
    objects = selection,
    attribute = "HitMode",
    values = { [0] = "On Miss", [1] = "On Hit", [2] = "On Hit Moving", [3] = "On Hit Stationary" },
    helpText = HitModeHelpText
  }
  rollPanel:endSizer()
  attributeEditor.logExitFunc("attributeEditor.HitModeDisplayInfo")
end

