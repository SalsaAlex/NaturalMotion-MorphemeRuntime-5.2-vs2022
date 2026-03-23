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
-- Adds an OperatorRandomFloat display info section.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.operatorRandomFloatRangeDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.operatorRandomFloatRangeDisplayInfoSection")

    -- check if the selection contains any referenced objects
  local hasReference = containsReference(selection)

    -- first add the ui for the section
  attributeEditor.log("rollContainer:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "operatorRandomFloatRangeDisplayInfoSection"  }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginHSizer")
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    rollPanel:addHSpacer(6)
    rollPanel:setBorder(1)

    local minHelp = getAttributeHelpText(selection[1], "Min")

    local minLabel = rollPanel:addStaticText{
      text = "Min",
      onMouseEnter = function() attributeEditor.setHelpText(minHelp) end,
      onMouseLeave = function() attributeEditor.clearHelpText() end
    }

    local minAttrPaths = { }
    for j, object in pairs(selection) do
      local attrPath = string.format("%s.Min", object)
      table.insert(minAttrPaths, attrPath)
    end

    local minWidget = rollPanel:addAttributeWidget{
      attributes = minAttrPaths,
      flags = "expand",
      proportion = 1,
      onMouseEnter = function() attributeEditor.setHelpText(minHelp) end,
      onMouseLeave = function() attributeEditor.clearHelpText() end
    }

    local maxHelp = getAttributeHelpText(selection[1], "Max")

    local maxLabel = rollPanel:addStaticText{
      text = "Max",
      onMouseEnter = function() attributeEditor.setHelpText(maxHelp) end,
      onMouseLeave = function() attributeEditor.clearHelpText() end
    }

    local maxAttrPaths = { }
    for j, object in pairs(selection) do
      local attrPath = string.format("%s.Max", object)
      table.insert(maxAttrPaths, attrPath)
    end

    local maxWidget = rollPanel:addAttributeWidget{
      attributes = maxAttrPaths,
      flags = "expand",
      proportion = 1,
      onMouseEnter = function() attributeEditor.setHelpText(maxHelp) end,
      onMouseLeave = function() attributeEditor.clearHelpText() end
    }

  attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  if hasReference then
    minLabel:enable(false)
    minWidget:enable(false)
    maxLabel:enable(false)
    maxWidget:enable(false)
  end

  ----------------------------------------------------------------------------------------------------------------------

  attributeEditor.logExitFunc("attributeEditor.operatorRandomFloatRangeDisplayInfoSection")
end

attributeEditor.operatorRandomFloatSeedDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.operatorRandomFloatSeedDisplayInfoSection")

  -- check if the selection contains any referenced objects
  local hasReference = containsReference(selection)

  -- first add the ui for the section
  attributeEditor.log("rollContainer:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "operatorRandomFloatSeedDisplayInfoSection"  }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginHSizer")
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    rollPanel:addHSpacer(6)
    rollPanel:setBorder(1)

    attributeEditor.log("rollPanel:beginFlexGridSizer")
    rollPanel:beginFlexGridSizer{ cols = 3, flags = "expand", proportion = 1 }
      rollPanel:setFlexGridColumnExpandable(3)
      local generateSeedHelp = getAttributeHelpText(selection[1], "GenerateSeed")

      local generateSeedLabel = rollPanel:addStaticText{
        text = "Mode",
        onMouseEnter = function() attributeEditor.setHelpText(generateSeedHelp) end,
        onMouseLeave = function() attributeEditor.clearHelpText() end
      }

      local generateSeedItems = { "Generated", "User Specified" }

      local generateSeedComboBox = rollPanel:addComboBox{
        flags = "expand",
        proportion = 1,
        items = generateSeedItems,
        onMouseEnter = function() attributeEditor.setHelpText(generateSeedHelp) end,
        onMouseLeave = function() attributeEditor.clearHelpText() end
      }

      local seedLabel = rollPanel:addStaticText{
        text = ""
      }

      local seedHelp = getAttributeHelpText(selection[1], "Seed")

      local seedLabel = rollPanel:addStaticText{
        text = "Seed",
        onMouseEnter = function() attributeEditor.setHelpText(seedHelp) end,
        onMouseLeave = function() attributeEditor.clearHelpText() end
      }

      local attrPaths = { }
      for j, object in pairs(selection) do
        local attrPath = string.format("%s.Seed", object)
        table.insert(attrPaths, attrPath)
      end

      local seedWidget = rollPanel:addAttributeWidget{
        attributes = attrPaths,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function() attributeEditor.setHelpText(seedHelp) end,
        onMouseLeave = function() attributeEditor.clearHelpText() end
      }
      
      local seedButtonHelp = "Auto generate a new seed."
      local seedButton = rollPanel:addButton{
        label = "New seed",
        onClick = function() setCommonAttributeValue(selection, "Seed", math.random(1000000), set) end,
        onMouseEnter = function() attributeEditor.setHelpText(seedButtonHelp) end,
        onMouseLeave = function() attributeEditor.clearHelpText() end,
        size = { width = 74 }
      }

    attributeEditor.log("rollPanel:endFlexGridSizer")
    rollPanel:endSizer()

  attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  if hasReference then
    generateSeedLabel:enable(false)
    generateSeedComboBox:enable(false)
    seedLabel:enable(false)
    seedWidget:enable(false)
    seedButton:enable(false)
  end

  -- these prevent callbacks firing off other callbacks causing infinite loops
  local enableContextEvents = true
  local enableUISetAttribute = true

  ----------------------------------------------------------------------------------------------------------------------
  -- this function syncs the ui with the current attribute values
  ----------------------------------------------------------------------------------------------------------------------
  local syncUIWithAttributes = function()
    attributeEditor.logEnterFunc("syncUIWithAttributes")

    local generateSeedValue = getCommonAttributeValue(selection, "GenerateSeed")
    local enableSeedWidget = false

    generateSeedComboBox:setItems(generateSeedItems)
    if generateSeedValue == nil then
      attributeEditor.log("not all objects have the same values, clearing current selection")
      generateSeedComboBox:addItem("")
      generateSeedComboBox:setSelectedItem("")
    else
      attributeEditor.log("all objects have the same values, setting selection index")
      generateSeedComboBox:setSelectedItem(generateSeedValue)

      if generateSeedValue == generateSeedItems[2] then
        enableSeedWidget = true
      end

    end

    seedLabel:enable(enableSeedWidget)
    seedWidget:enable(enableSeedWidget)
    seedButton:enable(enableSeedWidget)

    attributeEditor.logExitFunc("syncUIWithAttributes")
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- this function sets the attributes when the combo box selection is changed
  ----------------------------------------------------------------------------------------------------------------------
  generateSeedComboBox:setOnChanged(
    function(self)
      attributeEditor.logEnterFunc("generateSeedComboBox:setOnChanged")

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
          self:setItems(generateSeedItems)
          self:setSelectedItem(selectedItem)

          -- set the values of the attribues based on the selected item
          setCommonAttributeValue(selection, "GenerateSeed", selectedItem)
        end

        -- update the seed widget
        local enableSeedWidget = selectedItem == generateSeedItems[2]
        seedLabel:enable(enableSeedWidget)
        seedWidget:enable(enableSeedWidget)
        seedButton:enable(enableSeedWidget)

        enableContextEvents = true
      end

      attributeEditor.logExitFunc("generateSeedComboBox:setOnChanged")
    end
  )

  ----------------------------------------------------------------------------------------------------------------------
  -- this data change context ensures that any changes that happen to selected
  -- attributes through script or undo redo are reflected in the custom ui.
  attributeEditor.log("creating data change context")
  local seedContext = attributeEditor.createChangeContext()

  seedContext:setObjects(selection)
  seedContext:addAttributeChangeEvent("GenerateSeed")
  --seedContext:addAttributeChangeEvent("Seed")

  ----------------------------------------------------------------------------------------------------------------------
  -- this function syncs the ui with the attribute values when they are changed
  -- via script or through undo and redo
  ----------------------------------------------------------------------------------------------------------------------
  seedContext:setAttributeChangedHandler(
    function(object, attr)
      attributeEditor.logEnterFunc("seedContext attributeChangedHandler")

      if enableContextEvents then
        enableUISetAttribute = false
        syncUIWithAttributes()
        enableUISetAttribute = true
      end

      attributeEditor.logExitFunc("seedContext attributeChangedHandler")
    end
  )

  -- set the initial state of the UI
  syncUIWithAttributes()

  ----------------------------------------------------------------------------------------------------------------------

  attributeEditor.logExitFunc("attributeEditor.operatorRandomFloatSeedDisplayInfoSection")
end

attributeEditor.operatorRandomFloatDurationDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.operatorRandomFloatDurationDisplayInfoSection")

  -- check if the selection contains any referenced objects
  local hasReference = containsReference(selection)

  -- first add the ui for the section
  attributeEditor.log("rollContainer:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "operatorRandomFloatDurationDisplayInfoSection"  }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginHSizer")
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    rollPanel:addHSpacer(6)
    rollPanel:setBorder(1)

    attributeEditor.log("rollPanel:beginFlexGridSizer")
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      rollPanel:setFlexGridColumnExpandable(2)
      local durationModeHelp = getAttributeHelpText(selection[1], "DurationMode")

      local durationModeLabel = rollPanel:addStaticText{
        text = "Mode",
        onMouseEnter = function() attributeEditor.setHelpText(durationModeHelp) end,
        onMouseLeave = function() attributeEditor.clearHelpText() end
      }

      local durationModeItems = { "Every Update", "Specify" }

      local durationModeComboBox = rollPanel:addComboBox{
        flags = "expand",
        proportion = 1,
        items = durationModeItems,
        onMouseEnter = function() attributeEditor.setHelpText(durationModeHelp) end,
        onMouseLeave = function() attributeEditor.clearHelpText() end
      }

      local intervalHelp = getAttributeHelpText(selection[1], "Interval")

      local intervalLabel = rollPanel:addStaticText{
        text = "Interval",
        onMouseEnter = function() attributeEditor.setHelpText(intervalHelp) end,
        onMouseLeave = function() attributeEditor.clearHelpText() end
      }

      local attrPaths = { }
      for j, object in pairs(selection) do
        local attrPath = string.format("%s.Interval", object)
        table.insert(attrPaths, attrPath)
      end

      local intervalWidget = rollPanel:addAttributeWidget{
        attributes = attrPaths,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function() attributeEditor.setHelpText(seedHelp) end,
        onMouseLeave = function() attributeEditor.clearHelpText() end
      }

    attributeEditor.log("rollPanel:endFlexGridSizer")
    rollPanel:endSizer()

  attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  if hasReference then
    durationModeLabel:enable(false)
    durationModeComboBox:enable(false)
    intervalLabel:enable(false)
    intervalWidget:enable(false)
  end

  -- these prevent callbacks firing off other callbacks causing infinite loops
  local enableContextEvents = true
  local enableUISetAttribute = true

  ----------------------------------------------------------------------------------------------------------------------
  -- this function syncs the ui with the current attribute values
  ----------------------------------------------------------------------------------------------------------------------
  local syncUIWithAttributes = function()
    attributeEditor.logEnterFunc("syncUIWithAttributes")

    local durationModeValue = getCommonAttributeValue(selection, "DurationMode")
    local enableIntervalWidget = false

    durationModeComboBox:setItems(durationModeItems)
    if durationModeValue == nil then
      attributeEditor.log("not all objects have the same values, clearing current selection")
      durationModeComboBox:addItem("")
      durationModeComboBox:setSelectedItem("")
    else
      attributeEditor.log("all objects have the same values, setting selection index")
      durationModeComboBox:setSelectedItem(durationModeValue)

      if durationModeValue == durationModeItems[2] then
        enableIntervalWidget = true
      end

    end

    intervalLabel:enable(enableIntervalWidget)
    intervalWidget:enable(enableIntervalWidget)

    attributeEditor.logExitFunc("syncUIWithAttributes")
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- this function sets the attributes when the combo box selection is changed
  ----------------------------------------------------------------------------------------------------------------------
  durationModeComboBox:setOnChanged(
    function(self)
      attributeEditor.logEnterFunc("durationModeComboBox:setOnChanged")

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
          self:setItems(durationModeItems)
          self:setSelectedItem(selectedItem)

          -- set the values of the attribues based on the selected item
          setCommonAttributeValue(selection, "DurationMode", selectedItem)
        end

        -- update the seed widget
        local enableIntervalWidget = selectedItem == durationModeItems[2]
        intervalLabel:enable(enableIntervalWidget)
        intervalWidget:enable(enableIntervalWidget)

        enableContextEvents = true
      end

      attributeEditor.logExitFunc("durationModeComboBox:setOnChanged")
    end
  )

  ----------------------------------------------------------------------------------------------------------------------
  -- this data change context ensures that any changes that happen to selected
  -- attributes through script or undo redo are reflected in the custom ui.
  attributeEditor.log("creating data change context")
  local durationContext = attributeEditor.createChangeContext()

  durationContext:setObjects(selection)
  durationContext:addAttributeChangeEvent("DurationMode")

  ----------------------------------------------------------------------------------------------------------------------
  -- this function syncs the ui with the attribute values when they are changed
  -- via script or through undo and redo
  ----------------------------------------------------------------------------------------------------------------------
  durationContext:setAttributeChangedHandler(
    function(object, attr)
      attributeEditor.logEnterFunc("durationContext attributeChangedHandler")

      if enableContextEvents then
        enableUISetAttribute = false
        syncUIWithAttributes()
        enableUISetAttribute = true
      end

      attributeEditor.logExitFunc("durationContext attributeChangedHandler")
    end
  )

  -- set the initial state of the UI
  syncUIWithAttributes()

  ----------------------------------------------------------------------------------------------------------------------

  attributeEditor.logExitFunc("attributeEditor.operatorRandomFloatDurationDisplayInfoSection")
end
