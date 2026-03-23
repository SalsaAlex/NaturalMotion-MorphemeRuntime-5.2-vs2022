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
-- Adds format specific controls to the animation format panel
------------------------------------------------------------------------------------------------------------------------
local addFormatSpecificPanel = function(panel, selection, attribute, set)
  attributeEditor.logEnterFunc("addFormatSpecificPanel")

  local formatValue = getCommonSubAttributeValue(selection, attribute, "format", set)
  local format = animfmt.get(formatValue)

  if type(format.addFormatOptionsPanel) == "function" and type(format.updateFormatOptionsPanel) == "function" then
    panel:clear()
    panel:beginVSizer{ flags = "expand" }
  
      local getSelectionCount = function()
        return table.getn(selection)
      end

      local getOptionsTable = function(index)
        local take = getAttribute(selection[index], attribute, set)
        return animfmt.parseOptions(take.options)
      end

      local setOptionsTable = function(index, options)
        local take = getAttribute(selection[index], attribute, set)
        take.options = animfmt.compileOptions(options)
        setAttribute(string.format("%s.%s", selection[index], attribute), take, set)
      end

      attributeEditor.log("creating shouldEnableControls callback function for custom animation format panel")

      -- check if there is a referenced object within the selection
      local hasReference = containsReference(selection)

      local shouldEnableControls = nil
      if hasReference then
        -- if the selection contains a referenced object then the controls should never be enabled.
        shouldEnableControls = function()
          return false
        end
      else
        -- if there is no referenced object then check the attribute isn't synch'd with it's animation set.
        shouldEnableControls = function()
          return not getCommonSubAttributeValue(selection, attribute, "syncwithset", set)
        end
      end

      format.addFormatOptionsPanel(panel, getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)

    panel:endSizer()
    
    format.updateFormatOptionsPanel(panel, getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)

    -- create the change context to update the animation format specific panel
    local formatOptionsChangeContext = attributeEditor.createChangeContext()

    formatOptionsChangeContext:setObjects(selection)
    formatOptionsChangeContext:addAttributeChangeEvent(attribute)

    formatOptionsChangeContext:setAttributeChangedHandler(
      function(object, attr)
        if formatValue == getCommonSubAttributeValue(selection, attribute, "format", set) then
          format.updateFormatOptionsPanel(panel, getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
        end
      end
    )
  end

  attributeEditor.logExitFunc("addFormatSpecificPanel")
end

------------------------------------------------------------------------------------------------------------------------
-- add's the animation format panel containing a format combo box and also calls the
-- function to add format specific controls
------------------------------------------------------------------------------------------------------------------------
local addAnimationFormatOptionsPanel = function(panel, selection, attribute, set)
  attributeEditor.logEnterFunc("addAnimationFormatOptionsPanel")

  if not panel then
    return
  end

  panel:setBorder(1)

  attributeEditor.log("building animation format options ui")
  panel:beginHSizer{ flags = "expand", proportion = 1 }
    panel:addHSpacer(5)
    panel:beginVSizer{ flags = "expand", proportion = 1 }

      panel:addVSpacer(3)
      panel:beginHSizer{ flags = "expand", proportion = 1 }
        local items = { }
        for i, value in ipairs(animfmt.ls()) do
          table.insert(items, value.format)
        end
        table.insert(items, "")

        local formatComboHelp = "Compression format."

        local formatLabel = panel:addStaticText{
          text = "Format",
          name = "FormatTitle",
          onMouseEnter = function()
            attributeEditor.setHelpText(formatComboHelp)
          end,
          onMouseLeave = function()
            attributeEditor.clearHelpText()
          end
        }

        local formatCombo = panel:addComboBox{
          flags = "expand",
          name = "FormatComboBox",
          proportion = 1,
          items = items,
          onMouseEnter = function()
            attributeEditor.setHelpText(formatComboHelp)
          end,
          onMouseLeave = function()
            attributeEditor.clearHelpText()
          end
        }

      panel:endSizer()

      -- add any format specific controls
      local formatSpecificPanel = panel:addPanel{ flags = "expand", name = "FormatSpecificPanel" }
      addFormatSpecificPanel(formatSpecificPanel, selection, attribute, set)

    panel:endSizer()
  panel:endSizer()

  -- check if the selection contains any referenced objects
  local hasReference = containsReference(selection)

  ----------------------------------------------------------------------------------------------------------------------
  -- set the combo box on changed function
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.log("setting the format combo box change function")
  formatCombo:setOnChanged(
    function(self)
      local currentFormat = getCommonSubAttributeValue(selection, attribute, "format", set)
      local newFormat = self:getSelectedItem()

      if currentFormat == newFormat then
        return
      end
      
      -- wrap all the changes in an undo block
      attributeEditor.suspendUpdates()
      
      undoBlock(
        function()
          local take = { format = newFormat }
          for i, object in ipairs(selection) do
            setAttribute(string.format("%s.%s", object, attribute), take, set)
          end

          addFormatSpecificPanel(formatSpecificPanel, selection, attribute, set)
        end
      )
      
      attributeEditor.resumeUpdates()
    end
  )

  ----------------------------------------------------------------------------------------------------------------------
  -- syncs the ui with the current attribute values
  ----------------------------------------------------------------------------------------------------------------------
  local updateAnimationFormatOptionsPanel = function()
    attributeEditor.logEnterFunc("updateAnimationFormatOptionsPanel")

    attributeEditor.log("syncing for set \"%s\"", set)

    local syncWithSetValue = getCommonSubAttributeValue(selection, attribute, "syncwithset", set)
    local formatValue = getCommonSubAttributeValue(selection, attribute, "format", set)

    local enable = false
    if type(syncWithSetValue) == "boolean" then
      enable = not syncWithSetValue and not hasReference
      attributeEditor.log("sync with set is set")
    end

    -- set the enable state of the format combo and resampling check box
    attributeEditor.log("setting the enable state of the ui")
    formatLabel:enable(enable)
    formatCombo:enable(enable)

    -- set the value for the format combo
    if formatValue and type(formatValue) == "string" then
      formatCombo:setSelectedItem(formatValue)
    else
      formatCombo:setSelectedItem("")
    end

    panel:rebuild()

    attributeEditor.logExitFunc("updateAnimationFormatOptionsPanel")
  end

  attributeEditor.log("adding \"%s\" change context for current selection", attribute)
  local animTakeContext = attributeEditor.createChangeContext()

  animTakeContext:setObjects(selection)
  animTakeContext:addAttributeChangeEvent(attribute)

  ----------------------------------------------------------------------------------------------------------------------
  -- watch the take attribute for changes
  ----------------------------------------------------------------------------------------------------------------------
  animTakeContext:setAttributeChangedHandler(
    function(object, attr)
      attributeEditor.logEnterFunc("animTakeContext attributeChangedHandler")
      updateAnimationFormatOptionsPanel()
      attributeEditor.logExitFunc("animTakeContext attributeChangedHandler")
    end
  )

  updateAnimationFormatOptionsPanel()

  attributeEditor.logExitFunc("addAnimationFormatOptionsPanel")
end

------------------------------------------------------------------------------------------------------------------------
-- get Take attribute help string
------------------------------------------------------------------------------------------------------------------------
local getTakeAttrHelpString = function(takeAttrName)
  if takeAttrName == "File" then
    return [[
The File and Take attributes specify a unique animation take from a particular file.
These attributes will be filled in automatically when you create this node by dragging an animation from the animation browser into the network panel.
]]
  elseif takeAttrName == "Take" then
    return [[
The File and Take attributes specify a unique animation take from a particular file.
These attributes will be filled in automatically when you create this node by dragging an animation from the animation browser into the network panel.
]]
  elseif takeAttrName == "SyncTrack" then
    return [[
The sync track specifies which track to use in the event matching system of morpheme.
By default a track called 'Footsteps' is used for event synchronization, but any track can be selected as the output sync track for the animation.
]]
  elseif takeAttrName == "SyncWithSet" then
    return [[
Turn off to override the default compression format and options.
]]
  end

  return ""
end

------------------------------------------------------------------------------------------------------------------------
-- Called once per animation set to add the actual AnimationTake attribute widget and to build the
-- custom format panel for the widget.
------------------------------------------------------------------------------------------------------------------------
local animationSetDisplayFunction = function(panel, selection, attributes, set)
  local attribute = attributes[1]

  panel:beginVSizer{ flags = "expand" }

  local attrs = { }
  for i, object in ipairs(selection) do
    local current = string.format("%s.%s", object, attribute)
    table.insert(attrs, current)
    attributeEditor.log("adding \"%s\" to attribute list", current)
  end

  local widget = panel:addAttributeWidget{
    attributes = attrs,
    flags = "expand",
    proportion = 1,
    set = set,
    onMouseEnter = function(self, takeAttrName)
      attributeEditor.setHelpText(getTakeAttrHelpString(takeAttrName))
    end,
    onMouseLeave = function()
      attributeEditor.clearHelpText()
    end
  }

  local customFormatPanel = widget:getChild("CustomFormatPanel")
  if customFormatPanel then
    addAnimationFormatOptionsPanel(customFormatPanel, selection, attribute, set)
  end

  panel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- Add's a display info section for animation take attributes.
-- Used by AnimWithEvents.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.animationTakeDisplayInfoSection = function(panel, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.animationTakeDisplayInfoSection")

  if table.getn(displayInfo.usedAttributes) ~= 1 then
    attributeEditor.log("more than one attribute specified to animationTakeDisplayInfoSection")
    attributeEditor.logEnterFunc("attributeEditor.animationTakeDisplayInfoSection")
    return
  end

  local attribute = displayInfo.usedAttributes[1]
  local attrs = checkSetOrderAndBuildAttributeList(selection, attribute)

  attributeEditor.log("rollContainter:addRollup")
  local rollup = panel:addRollup{ label = displayInfo.title, flags = "mainSection", name = "animationTakeDisplayInfoSection" }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginVSizer")
  rollPanel:beginVSizer{ flags = "expand" }

    attributeEditor.log("rollPanel:addAttributeWidget")
    rollPanel:addAnimationSetWidget{
      attributes = attrs,
      displayFunc = animationSetDisplayFunction,
      flags = "expand",
      proportion = 1,
    }

  attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.animationTakeDisplayInfoSection")
end