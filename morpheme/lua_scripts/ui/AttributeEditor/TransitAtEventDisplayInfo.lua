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
-- Add's a transitAtEventDisplayInfoSection.
-- Used by TransitAtEvent.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.transitAtEventDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.transitAtEventDisplayInfoSection")

  -- check if the selection contains any referenced objects
  local hasReference = containsReference(selection)

  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "transitAtEventDisplayInfoSection" }
  local rollPanel = rollup:getPanel()

  -- first add the ui for the section
  attributeEditor.log("rollPanel:beginHSizer")
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    rollPanel:addHSpacer(6)
    rollPanel:setBorder(1)

    attributeEditor.log("rollPanel:beginFlexGridSizer")
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      rollPanel:setFlexGridColumnExpandable(2)

      local destinationStartEventIndexAttrPaths = { }
      local destinationStartEventFractionAttrPaths = { }
      local startFromSetStartEventFractionInDestEventAttrPaths = { }

      for i, object in ipairs(selection) do
        table.insert(destinationStartEventIndexAttrPaths, string.format("%s.DestinationStartEventIndex", object))
        table.insert(destinationStartEventFractionAttrPaths, string.format("%s.DestinationStartEventFraction", object))
        table.insert(startFromSetStartEventFractionInDestEventAttrPaths, string.format("%s.StartFromSetStartEventFractionInDestEvent", object))
      end

local styleHelpText =
[[
Event index is the index of a synchronisation event within the destination that defines the playback position on transition start.
Fraction works in conjunction with the event index to allow the accurate specification of the start position in the destination.
]]

       -- the names that appear in the style popup
      local eventIndexName = "Event index"
      local eventIndexAndFractionName = "Event index and fraction"

      -- the style popup
      local styleItems = {
        eventIndexName,
        eventIndexAndFractionName
       }

      rollPanel:addStaticText{
        text = "Style",
        onMouseEnter = function()
          attributeEditor.setHelpText(styleHelpText)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      local styleComboBox = attributeEditor.addBoolAttributeCombo{
        panel = rollPanel,
        objects = selection,
        attribute = "StartFromSetStartEventFractionInDestEvent",
        trueValue = "eventIndexAndFractionName",
        falseValue ="eventIndexName",
        helpText = styleHelpText
      }

      rollPanel:addStaticText{
        text = "Index",
        onMouseEnter = function()
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "DestinationStartEventIndex"))
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      local indexWidget = rollPanel:addAttributeWidget{
        attributes = destinationStartEventIndexAttrPaths,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function(self)
          attributeEditor.setHelpText(self:getAttributeHelp())
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      rollPanel:addStaticText{
        text = "Fraction",
        onMouseEnter = function()
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "DestinationStartEventFraction"))
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      local fractionWidget = rollPanel:addAttributeWidget{
        attributes = destinationStartEventFractionAttrPaths,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function(self)
          attributeEditor.setHelpText(self:getAttributeHelp())
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      attributeEditor.log("rollPanel:endSizer")
    rollPanel:endSizer()

    attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  if hasReference then
    styleComboBox:enable(false)
  end

  -- these prevent callbacks firing off other callbacks causing infinite loops
  local enableContextEvents = true
  local enableUISetAttribute = true

  ----------------------------------------------------------------------------------------------------------------------
  -- enable DestinationStartEventFraction based on the value of StartFromSetStartEventFractionInDestEvent
  ----------------------------------------------------------------------------------------------------------------------
  local syncUIWithAttributes = function()
    attributeEditor.logEnterFunc("syncUIWithAttributes")

    local startFromSetStartEventFractionInDestEvent = getCommonAttributeValue(selection, "StartFromSetStartEventFractionInDestEvent")

    styleComboBox:setItems(styleItems)
    if startFromSetStartEventFractionInDestEvent ~= nil then
      if not startFromSetStartEventFractionInDestEvent then
        attributeEditor.log("all objects have the same value, setting selection to \"%s\"", styleItems[1])
        styleComboBox:setSelectedIndex(1)
        fractionWidget:enable(false)
      else
        attributeEditor.log("all objects have the same value, setting selection to \"%s\"", styleItems[2])
        styleComboBox:setSelectedIndex(2)
        fractionWidget:enable(not hasReference)
      end
    else
      attributeEditor.log("not all objects have the same value, clearing current selection index")
      styleComboBox:addItem("")
      styleComboBox:setSelectedIndex(3)
    end

    attributeEditor.logExitFunc("syncUIWithAttributes")
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- this function sets the attributes when the style combo box selection is changed
  ----------------------------------------------------------------------------------------------------------------------
  styleComboBox:setOnChanged(
    function(self)
      attributeEditor.logEnterFunc("styleComboBox:setOnChanged")

      if enableUISetAttribute then
        -- prevent the change context callbacks from firing off
        enableContextEvents = false

        local selectedItem = self:getSelectedItem()
        attributeEditor.log("selected item is \"%s\"", selectedItem)

        -- the combo box may contain an empty selection string which means
        -- not all attributes in the selection are the same, if the empty string
        -- is selected then don't change any attributes
        if string.len(selectedItem) > 0 then

          -- if the empty selection wasn't selected then reset the selectable items
          -- and select the previous selection
          self:setItems(styleItems)
          self:setSelectedItem(selectedItem)

          -- set the values of the attribues base on the selected item
          local startFromSetStartEventFractionInDestEvent = false
          if selectedItem == styleItems[2] then
            startFromSetStartEventFractionInDestEvent = true
          end

          setCommonAttributeValue(selection, "StartFromSetStartEventFractionInDestEvent", startFromSetStartEventFractionInDestEvent)

          fractionWidget:enable(startFromSetStartEventFractionInDestEvent and not hasReference)
        end

        enableContextEvents = true
      end

      attributeEditor.logExitFunc("styleComboBox:setOnChanged")
    end
  )

  ----------------------------------------------------------------------------------------------------------------------
  -- this data change context ensures the ui reflects any changes that happen via script,
  -- or through undo redo, are reflected in the custom ui.
  local changeContext = attributeEditor.createChangeContext()
  changeContext:setObjects(selection)
  changeContext:addAttributeChangeEvent("StartFromSetStartEventFractionInDestEvent")

  ------------------------------------------------------------------------------------------------------------------------
  -- this function is called whenever StartFromSetStartEventFractionInDestEvent is changed via script, or through undo redo.
  -- enable DestinationStartEventFraction based on the value of StartFromSetStartEventFractionInDestEvent.
  ------------------------------------------------------------------------------------------------------------------------
  changeContext:setAttributeChangedHandler(
    function(object, attr)
      attributeEditor.logEnterFunc("changeContext attributeChangedHandler")

      if enableContextEvents then
        enableUISetAttribute = false
        syncUIWithAttributes()
        enableUISetAttribute = true
      end

      attributeEditor.logExitFunc("changeContext attributeChangedHandler")
    end
  )

  -- set the initial state of the ui
  syncUIWithAttributes()

  ------------------------------------------------------------------------------------------------------------------------

  attributeEditor.logExitFunc("attributeEditor.transitAtEventDisplayInfoSection")
end

