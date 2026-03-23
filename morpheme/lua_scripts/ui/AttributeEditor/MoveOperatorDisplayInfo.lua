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
attributeEditor.moveOperatorControllerDisplayInfoSection = function(panel, displayInfo, selection)

  local rollup = panel:addRollup{
    label = displayInfo.title,
    flags = "mainSection",
    name = string.format("moveOperatorControllerDisplayInfoSection")
  }
  local rollPanel = rollup:getPanel()

  rollPanel:beginVSizer{ flags = "expand" }
    rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
      rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
        rollPanel:setFlexGridColumnExpandable(2)
        
        -- MoveController
        attributeEditor.addAttributeLabel(rollPanel, "Controller ID", selection, "MoveControllerID")
        local controlerWidget = rollPanel:addFloatSpinner{ flags = "expand", proportion = 1, min = 0, max = 3 }
        attributeEditor.bindHelpToWidget(controlerWidget, getAttributeHelpText(selection[1], "MoveControllerID"))
        bindWidgetToAttribute(controlerWidget, selection, "MoveControllerID")

      rollPanel:endSizer()
    rollPanel:endSizer()
  rollPanel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
attributeEditor.moveOperatorRotationOffsetsDisplayInfoSection = function(panel, displayInfo, selection)
  local enableRigMatchingWidget = nil
  local animSetControls = { }
  
  ----------------------------------------------------------------------------------------------------------------------
  local perAnimSetSection = function(panel, selection, attributes, set)
    panel:beginHSizer{ flags = "expand", proportion = 0 }
      panel:setBorder(2)
      panel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
        panel:setFlexGridColumnExpandable(2)

        local endJointNameLabel = attributeEditor.addAttributeLabel(panel, "End Effector", selection, "EndJointName")
        local endJointNameWidget = attributeEditor.addAttributeWidget(panel, "EndJointName", selection, set)

        local rigRotationAttrsLabel = attributeEditor.addAttributeLabel(panel, "Character", selection, "RigRotationX")
        local rigRotationAttrs = { "RigRotationX",  "RigRotationY", "RigRotationZ" }
        local rigRotationAttrsWidget = attributeEditor.addVectorAttributeWidget(panel, rigRotationAttrs, selection, set)

        local controllerAttrsLabel = attributeEditor.addAttributeLabel(panel, "Controller", selection, "ControllerRotationX")
        local controllerAttrs = { "ControllerRotationX",  "ControllerRotationY", "ControllerRotationZ" }
        local controllerAttrsWidget = attributeEditor.addVectorAttributeWidget(panel, controllerAttrs, selection, set)

        table.insert(animSetControls, endJointNameLabel)
        table.insert(animSetControls, endJointNameWidget)
        table.insert(animSetControls, rigRotationAttrsLabel)
        table.insert(animSetControls, rigRotationAttrsWidget)
        table.insert(animSetControls, controllerAttrsLabel)
        table.insert(animSetControls, controllerAttrsWidget)
      panel:endSizer()
    panel:endSizer()
  end

  ----------------------------------------------------------------------------------------------------------------------
  local updateUI = function()
    local enableRigMatching = getCommonAttributeValue(selection, "EnableRigMatching")
    local enableWidget = not containsReference(selection) and (enableRigMatching ~= nil) and enableRigMatching
    for _, widget in ipairs(animSetControls) do
      widget:enable(enableWidget)
    end
  end
  
  ----------------------------------------------------------------------------------------------------------------------
  local rollup = panel:addRollup{
    label = displayInfo.title,
    flags = "mainSection",
    name = string.format("moveOperatorRotationOffsetsDisplayInfoSection")
  }
  local rollPanel = rollup:getPanel()

  rollPanel:beginVSizer{ flags = "expand" }
    rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
      rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
        rollPanel:setFlexGridColumnExpandable(2)
        
        -- EnableRigMatching
        local enableRigMatchingHelpText = getAttributeHelpText(selection[1], "EnableRigMatching");
        attributeEditor.addStaticTextWithHelp(rollPanel, "Reference Frame", enableRigMatchingHelpText)
        enableRigMatchingWidget = attributeEditor.addBoolAttributeCombo{
          panel = rollPanel,
          objects = selection,
          attribute = "EnableRigMatching",
          falseValue = "World Space",
          trueValue = "Character Space",
          helpText = enableRigMatchingHelpText
        }
      rollPanel:endSizer()
    rollPanel:endSizer()

    local attrs = checkSetOrderAndBuildAttributeList(selection, 
      { "EndJointName", 
        "RigRotationX",  "RigRotationY", "RigRotationZ", 
        "ControllerRotationX", "ControllerRotationY", "ControllerRotationZ"})
       
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 0 }
      rollPanel:setFlexGridColumnExpandable(2)
      rollPanel:addHSpacer(6)
      rollPanel:addAnimationSetWidget{
        attributes = attrs,
        displayFunc = perAnimSetSection,
        flags = "expand",
        proportion = 1,
      }
    rollPanel:endSizer()

  rollPanel:endSizer()
  
  updateUI()

  -- create the change context
  local context = attributeEditor.createChangeContext()
  context:setObjects(selection)
  context:addAttributeChangeEvent("EnableRigMatching")
  context:setAttributeChangedHandler(updateUI)
end