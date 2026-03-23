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
-- Add's a spaceDisplayInfo
-- Used by RayHit
------------------------------------------------------------------------------------------------------------------------

-- the names that appear in the style popup
local kWorld = "World"
local kLocal = "Local"

-- the style popup
local spaceItems = {
  kLocal,
  kWorld
}

local spaceHelpText =
[[
Coordinate Space
Ray direction is specified in local space.
Ray direction in specified in world space.
]]

attributeEditor.spaceDisplayInfo = function(rollPanel, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.spaceDisplayInfo")

  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }

  rollPanel:addStaticText{
    text = "Space",
    onMouseEnter = function()
      attributeEditor.setHelpText(spaceHelpText)
    end,
    onMouseLeave = function()
      attributeEditor.clearHelpText()
    end
  }

  local spaceItemsWidget = attributeEditor.addBoolAttributeCombo{
    panel = rollPanel,
    objects = selection,
    attribute = "UseLocalOrientation",
    trueValue = kLocal,
    falseValue = kWorld,
    helpText = spaceHelpText
  }

  rollPanel:endSizer()

  -- these prevent callbacks firing off other callbacks causing infinite loops
  local enableContextEvents = true
  local enableUISetAttribute = true

  -- check if the selection contains any referenced objects
  local hasReference = containsReference(selection)

  if hasReference then
    spaceItemsWidget:enable(false)
  end

  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.logExitFunc("attributeEditor.spaceDisplayInfo")
end

