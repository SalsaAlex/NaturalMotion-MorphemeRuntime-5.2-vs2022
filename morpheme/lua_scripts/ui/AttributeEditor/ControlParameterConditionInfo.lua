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
-- Add's a controlParameterConditionInfoSection.
-- Used by ControlParameterConditions.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.controlParameterConditionInfoSection = function(rollPanel, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.controlParameterConditionInfoSection")

  local controlParameterAttrPaths = { }
  local controlParameterTriggerValues = { }
  for i, object in ipairs(selection) do
    table.insert(controlParameterAttrPaths, string.format("%s.ControlParameter", object))
    table.insert(controlParameterTriggerValues, string.format("%s.TriggerValue", object))
  end

  rollPanel:beginFlexGridSizer{ cols = 3, flags = "expand", proportion = 0 }
    rollPanel:setFlexGridColumnExpandable(1)
    rollPanel:setFlexGridColumnExpandable(3)
    

    local controlParameterBox = rollPanel:addAttributeWidget{
      attributes = controlParameterAttrPaths,
      flags = "expand",
      proportion = 1,
    }

    local operatorItems = { ">", "<", ">=", "<=" }
    local operatorComboBox = rollPanel:addComboBox{
      flags = "expand",
      proportion = 0,
      size = { width = 45, height = 0 },
      items = operatorItems,
    }

    if containsReference(selection) then
      operatorComboBox:enable(false)
    end

    local triggerValueParameterBox = rollPanel:addAttributeWidget{
      attributes = controlParameterTriggerValues,
      flags = "expand",
      proportion = 1,
    }

  rollPanel:endSizer()

  ----------------------------------------------------------------------------------------------------------------------
  -- this function sets the attributes when the combo box selection is changed
  ----------------------------------------------------------------------------------------------------------------------
  operatorComboBox:setOnChanged(
    function(self)
      attributeEditor.logEnterFunc("operatorComboBox:setOnChanged")
        local selectedItem = self:getSelectedItem()
        if string.len(selectedItem) > 0 then
          setCommonAttributeValue(selection, "Comparison", selectedItem)
        end
      attributeEditor.logExitFunc("operatorComboBox:setOnChanged")
    end
  )

  ----------------------------------------------------------------------------------------------------------------------
  -- this data change context ensures the ui reflects any changes that happen via script, or through undo redo.
  ----------------------------------------------------------------------------------------------------------------------
  local changeContext = attributeEditor.createChangeContext()

  changeContext:setObjects(selection)
  changeContext:addAttributeChangeEvent("Comparison")

  local syncInterface = function()
    local attrValue = getCommonAttributeValue(selection, "Comparison")
    if attrValue ~= nil then
      operatorComboBox:setIsIndeterminate(false)
      operatorComboBox:setSelectedItem(attrValue)
    else
      operatorComboBox:setIsIndeterminate(true)
    end
  end

  changeContext:setAttributeChangedHandler(
    function(object, attr)
      attributeEditor.logEnterFunc("changeContext attributeChangedHandler")
      syncInterface()
      attributeEditor.logExitFunc("changeContext attributeChangedHandler")
    end
  )

  syncInterface()

  changeContext:setAttributeChangedHandler(
    function(object, attr)
      attributeEditor.logEnterFunc("changeContext attributeChangedHandler")
      syncInterface()
      attributeEditor.logExitFunc("changeContext attributeChangedHandler")
    end
  )

  syncInterface()

  attributeEditor.logExitFunc("attributeEditor.controlParameterConditionInfoSection")
end

