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
-- Adds an OperatorVector3OneInputArithmetic display info section.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.operatorVector3OneInputArithmeticDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.operatorVector3OneInputArithmeticDisplayInfoSection")

  -- first add the ui for the section
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "operatorVector3OneInputArithmeticDisplayInfoSection" }
  local rollPanel = rollup:getPanel()

  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    rollPanel:addHSpacer(8)
    rollPanel:setBorder(1)

    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      rollPanel:setFlexGridColumnExpandable(2)

      -- Constant -------------------------------------------------------
      local attribute = "ConstantValueX"
      local initialLabel = attributeEditor.addAttributeLabel(rollPanel, "Value", selection, attribute)
      local initalValueWidgets = attributeEditor.addVectorAttributeWidget(rollPanel, {"ConstantValueX", "ConstantValueY", "ConstantValueZ"}, selection)

      -- Operation -------------------------------------------------------
      local attribute = "Operation"
      local kOperatorItems = 
      { 
        "*", 
        "+", 
        "/", 
        "-", 
        "min", 
        "max"
      }
      attributeEditor.addAttributeLabel(rollPanel, "Operation", selection, attribute)
      local operatorComboBox = attributeEditor.addStringAttributeCombo{
        panel = rollPanel,
        flags = "expand",
        objects = selection,
        proportion = 1,
        values = kOperatorItems,
        attribute = attribute,
        helpText = getAttributeHelpText(selection[1], attribute)
      }

    rollPanel:endSizer()
  rollPanel:endSizer()

  -- these prevent callbacks firing off other callbacks causing infinite loops
  local enableContextEvents = true
  local enableUISetAttribute = true

  -- Ensure update editor
  attributeEditor.onFlowEdgeCreateDestroy = refreshUI

  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.logExitFunc("attributeEditor.operatorVector3OneInputArithmeticDisplayInfoSection")
end

