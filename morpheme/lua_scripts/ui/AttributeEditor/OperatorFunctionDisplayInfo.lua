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
-- Adds an operatorFunction display info section.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.operatorFunctionDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.operatorFunctionDisplayInfoSection")

  -- check if the selection contains any referenced objects
  local hasReference = containsReference(selection)

  -- first add the ui for the section
  attributeEditor.log("rollContainer:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "operatorFunctionDisplayInfoSection"  }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginHSizer")
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    rollPanel:addHSpacer(8)
    rollPanel:setBorder(1)

    local operationHelp = getAttributeHelpText(selection[1], "Operation")

    local operatorLabel = rollPanel:addStaticText{
      text = "Operation",
      onMouseEnter = function()
        attributeEditor.setHelpText(operationHelp)
      end,
      onMouseLeave = function()
        attributeEditor.clearHelpText()
      end
    }

    local operatorItems = { "sin", "cos", "tan", "exp", "log", "sqrt", "abs", "asin", "acos" }

    local operatorComboBox = rollPanel:addComboBox{
      flags = "expand",
      proportion = 1,
      items = operatorItems,
      onMouseEnter = function()
        attributeEditor.setHelpText(operationHelp)
      end,
      onMouseLeave = function()
        attributeEditor.clearHelpText()
      end
    }

    attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  if hasReference then
    operatorComboBox:enable(false)
  end

  -- these prevent callbacks firing off other callbacks causing infinite loops
  local enableContextEvents = true
  local enableUISetAttribute = true

  ----------------------------------------------------------------------------------------------------------------------
  -- this function syncs the ui with the current attribute values
  ----------------------------------------------------------------------------------------------------------------------
  local syncUIWithAttributes = function()
    attributeEditor.logEnterFunc("syncUIWithAttributes")

    local operatorValue = getCommonAttributeValue(selection, "Operation")

    operatorComboBox:setItems(operatorItems)
    if operatorValue == nil then
      attributeEditor.log("not all objects have the same values, clearing current selection")
      operatorComboBox:addItem("")
      operatorComboBox:setSelectedItem("")
    else
      attributeEditor.log("all objects have the same values, setting selection index")
      operatorComboBox:setSelectedItem(operatorValue)
    end

    attributeEditor.logExitFunc("syncUIWithAttributes")
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- this function sets the attributes when the combo box selection is changed
  ----------------------------------------------------------------------------------------------------------------------
  operatorComboBox:setOnChanged(
    function(self)
      attributeEditor.logEnterFunc("operatorComboBox:setOnChanged")

      if enableUISetAttribute then
        -- prevent the change context callbacks from firing off
        enableContextEvents = false

        local selectedItem = self:getSelectedItem()

        -- the combo box may contain an empty selection string which means
        -- not all attributes in the selection are the same, if the empty string
        -- is selected then don't change any attributes
        if string.len(selectedItem) > 0 then
          -- if the empty selection wasn't selected then reset the selectable items
          -- and select the previous selection
          self:setItems(operatorItems)
          self:setSelectedItem(selectedItem)

          -- set the values of the attribues based on the selected item
          setCommonAttributeValue(selection, "Operation", selectedItem)
        end

        enableContextEvents = true
      end

      attributeEditor.logExitFunc("operatorComboBox:setOnChanged")
    end
  )

  ----------------------------------------------------------------------------------------------------------------------
  -- this data change context ensures that any changes that happen to selected
  -- attributes through script or undo redo are reflected in the custom ui.
  attributeEditor.log("creating data change context")
  local operatorContext = attributeEditor.createChangeContext()

  operatorContext:setObjects(selection)
  operatorContext:addAttributeChangeEvent("Operation")

  ----------------------------------------------------------------------------------------------------------------------
  -- this function syncs the ui with the attribute values when they are changed
  -- via script or through undo and redo
  ----------------------------------------------------------------------------------------------------------------------
  operatorContext:setAttributeChangedHandler(
    function(object, attr)
      attributeEditor.logEnterFunc("operatorContext attributeChangedHandler")

      if enableContextEvents then
        enableUISetAttribute = false
        syncUIWithAttributes()
        enableUISetAttribute = true
      end

      attributeEditor.logExitFunc("operatorContext attributeChangedHandler")
    end
  )

  -- set the initial state of the axis combo box
  syncUIWithAttributes()

  ----------------------------------------------------------------------------------------------------------------------

  attributeEditor.logExitFunc("attributeEditor.operatorFunctionDisplayInfoSection")
end

