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
-- Add's a display info section containing blend weights.
-- Use by BlendN, BlendNMatchEvents.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.vector3ToFloatDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.vector3ToFloatDisplayInfoSection")

  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{
    label = displayInfo.title,
    flags = "mainSection",
    name = "blendTimeStretchDisplayInfoSection"
  }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginVSizer")
  rollPanel:beginVSizer{ flags = "expand" }
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 0 }
      rollPanel:setFlexGridColumnExpandable(2)

      attributeEditor.addAttributeLabel(rollPanel, "Mode", selection, "OutputValue")
      attributeEditor.addIntAttributeCombo{
        panel = rollPanel,
        objects = selection,
        attribute = "OutputValue",
        values = { "X value", "Y value", "Z value", },
        order = { "X value", "Y value", "Z value", },
      }
    rollPanel:endSizer()

  attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.vector3ToFloatDisplayInfoSection")
end