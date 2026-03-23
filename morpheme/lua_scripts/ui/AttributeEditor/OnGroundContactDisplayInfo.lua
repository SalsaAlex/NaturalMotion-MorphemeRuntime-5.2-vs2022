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
-- Add's a OnGroundContactInfoSection
-- Used by GroundContact Condition
------------------------------------------------------------------------------------------------------------------------
attributeEditor.OnGroundContactInfoSection = function(rollPanel, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.OnGroundContactInfoSection")

  rollPanel:beginVSizer{ flags = "expand", proportion = 0 }
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }

  local onGroundHelpText = getAttributeHelpText(selection[1], "OnGround")
  local triggerTimeHelpText = getAttributeHelpText(selection[1], "TriggerTime")

  local triggerTime = { }
  for i, object in ipairs(selection) do
    table.insert(triggerTime, string.format("%s.TriggerTime", object))
  end

  attributeEditor.log("rollPanel:addAttributeWidget")
  rollPanel:addAttributeWidget{
    flags = "expand",
    proportion = 0.5,
    attributes = triggerTime,
    onMouseEnter = function()
      attributeEditor.setHelpText(triggerTimeHelpText)
    end,
    onMouseLeave = function()
      attributeEditor.clearHelpText()
    end,
  }

  attributeEditor.log("attributeEditor.addStaticText")
  local spacingText = rollPanel:addStaticText{
    text = "seconds"
  }

  attributeEditor.log("attributeEditor.addBoolAttributeCombo")
  attributeEditor.addBoolAttributeCombo{
    panel = rollPanel,
    objects = selection,
    attribute = "OnGround",
    falseValue = "In Air",
    trueValue ="On Ground",
    helpText = onGroundHelpText,
  }

  rollPanel:endSizer()
  rollPanel:endSizer()
  attributeEditor.logExitFunc("attributeEditor.OnGroundContactInfoSection")
end

