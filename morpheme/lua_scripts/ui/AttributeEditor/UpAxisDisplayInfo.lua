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
-- Add's a upAxisDisplayInfoSection.
-- Used by LockFoot.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.upAxisDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.upAxisDisplayInfoSection")

  -- check if the selection contains any referenced objects
  local hasReference = containsReference(selection)

  -- add the ui for the section
  attributeEditor.log("rollContainer:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "upAxisDisplayInfoSection" }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginHSizer")
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    rollPanel:addHSpacer(6)
    rollPanel:setBorder(1)

    local axisHelp = getAttributeHelpText(selection[1], "UpAxisX")
    local axisItems = { "X Axis", "Y Axis", "Z Axis", "" }

    local axisComboBox = rollPanel:addComboBox{
      name = "AxisBox",
      flags = "expand",
      proportion = 1,
      items = axisItems,
      onMouseEnter = function()
        attributeEditor.setHelpText(axisHelp)
      end,
      onMouseLeave = function()
        attributeEditor.clearHelpText()
      end
    }

    attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  if hasReference then
    axisComboBox:enable(false)
  end

  -- these prevent callbacks firing off other callbacks causing infinite loops
  local enableContextEvents = true
  local enableUISetAttribute = true

  ----------------------------------------------------------------------------------------------------------------------
  -- this function syncs the axis combo box with the current attribute values
  ----------------------------------------------------------------------------------------------------------------------
  local syncAxisUIWithValue = function()
    attributeEditor.logEnterFunc("syncAxisUIWithValue")

    local upAxisXValue = getCommonAttributeValue(selection, "UpAxisX")
    local upAxisYValue = getCommonAttributeValue(selection, "UpAxisY")
    local upAxisZValue = getCommonAttributeValue(selection, "UpAxisZ")

    if upAxisXValue == nil or upAxisYValue == nil or upAxisZValue == nil then
      attributeEditor.log("not all objects have the same values, clearing current selection")
      axisComboBox:setSelectedIndex(4)
    else
      attributeEditor.log("all objects have the same values, setting selection index")
      if upAxisYValue then
        attributeEditor.log("setting selection to \"%s\"", axisItems[2])
        axisComboBox:setSelectedIndex(2)
      elseif upAxisZValue then
        attributeEditor.log("setting selection to \"%s\"", axisItems[3])
        axisComboBox:setSelectedIndex(3)
      elseif upAxisXValue then
        attributeEditor.log("setting selection to \"%s\"", axisItems[1])
        axisComboBox:setSelectedIndex(1)
      else
        attributeEditor.log("clearing current selection")
        axisComboBox:setSelectedIndex(4)
      end
    end

    attributeEditor.logExitFunc("syncAxisUIWithValue")
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- this function sets the attributes when the combo box selection is changed
  ----------------------------------------------------------------------------------------------------------------------
  axisComboBox:setOnChanged(
    function(self)
      attributeEditor.logEnterFunc("axisComboBox:setOnChanged")

      if enableUISetAttribute then
        -- prevent the change context callbacks from firing off
        enableContextEvents = false

        -- set the values of the attribues base on the selected item
        local upAxisXValue = false
        local upAxisYValue = false
        local upAxisZValue = false

        local selectedItem = self:getSelectedItem()
        if selectedItem == axisItems[1] then
          upAxisXValue = true
        elseif selectedItem == axisItems[2] then
          upAxisYValue = true
        elseif selectedItem == axisItems[3] then
          upAxisZValue = true
        end

        setCommonAttributeValue(selection, "UpAxisX", upAxisXValue)
        setCommonAttributeValue(selection, "UpAxisY", upAxisYValue)
        setCommonAttributeValue(selection, "UpAxisZ", upAxisZValue)

        enableContextEvents = true
      end

      attributeEditor.logExitFunc("axisComboBox:setOnChanged")
    end
  )

  ----------------------------------------------------------------------------------------------------------------------
  -- this data change context ensures the ui reflects any changes that happen to selected
  -- up axis attributes through script or undo redo are reflected in the custom ui.
  attributeEditor.log("creating data change context")
  local axisContext = attributeEditor.createChangeContext()

  axisContext:setObjects(selection)
  axisContext:addAttributeChangeEvent("UpAxisX")
  axisContext:addAttributeChangeEvent("UpAxisY")
  axisContext:addAttributeChangeEvent("UpAxisZ")

  ----------------------------------------------------------------------------------------------------------------------
  -- this function syncs the axis combo box with the attribute values when they are changed
  -- via script or through undo and redo
  ----------------------------------------------------------------------------------------------------------------------
  axisContext:setAttributeChangedHandler(
    function(object, attr)
      attributeEditor.logEnterFunc("axisContext attributeChangedHandler")

      if enableContextEvents then
        enableUISetAttribute = false
        syncAxisUIWithValue()
        enableUISetAttribute = true
      end

      attributeEditor.logExitFunc("axisContext attributeChangedHandler")
    end
  )

  -- set the initial state of the axis combo box
  syncAxisUIWithValue()

  ----------------------------------------------------------------------------------------------------------------------

  attributeEditor.logExitFunc("attributeEditor.upAxisDisplayInfoSection")
end

