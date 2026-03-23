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
attributeEditor.operatorVector3AngleDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.operatorVector3AngleDisplayInfoSection")

  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{
    label = displayInfo.title,
    flags = "mainSection",
    name = "operatorVector3AngleDisplayInfoSection"
  }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginVSizer")
  rollPanel:beginVSizer{ flags = "expand" }
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 0 }
      rollPanel:setFlexGridColumnExpandable(2)
      
      -- AxisType -------------------------------------------------------
      local attribute = "Axis"
      local dataTypeItems = { [0] = "Shortest angle", [1] = "X Axis", [2] = "Y Axis", [3] = "Z Axis" }
      attributeEditor.addAttributeLabel(rollPanel, "Axis", selection, attribute)
      local dataTypeComboBox = attributeEditor.addIntAttributeCombo{
        panel = rollPanel,
        flags = "expand",
        objects = selection,
        proportion = 1,
        values = dataTypeItems,
        attribute = attribute,
        helpText = getAttributeHelpText(selection[1], attribute)
      }
      
    rollPanel:endSizer()

  attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.operatorVector3AngleDisplayInfoSection")
end