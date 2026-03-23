------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- separates an animData object's attributes into a list of standard
-- and a list of dynamic attributes and returns both lists
local separateDynamicAttributes = function(animData)
  local attributes = anim.listAttributes(animData)
  local hiddenAttributes = {
    "Guid"
  }

  local standardAttributes = { }
  local dynamicAttributes = { }

  for i, attr in attributes do
    local hidden = false
    for j, hiddenAttr in hiddenAttributes do
      if attr == hiddenAttr then
        hidden = true
        break
      end
    end

    if not hidden then
      if anim.isAttributeDynamic(animData .. "." .. attr) then
        table.insert(dynamicAttributes, attr)
      else
        table.insert(standardAttributes, attr)
      end
    end
  end

  return standardAttributes, dynamicAttributes
end

------------------------------------------------------------------------------------------------------------------------
local thisDialogChanged = false

------------------------------------------------------------------------------------------------------------------------
-- this function sets an attribute by calling func with the arguments passed in
-- but by also setting the thisDialogChanged flag temporarily to true it prevents
-- the onMarkupSelectionChanged callback from causing a rebuild which is not necessary
-- example control dialog changed function:
-- function(self)
--   local newValue = self:getValue()
--   setWithNoRebuild(anim.setAttribute, attribute, newValue)
--   -- after the set update the control value again checking that
--   -- the displayed value is in sync with the actual value.
--   self:setValue(anim.getAttribute(attribute))
-- end
setWithNoRebuild = function(func, ...)
  thisDialogChanged = true
  func(unpack(arg))
  thisDialogChanged = false
end

------------------------------------------------------------------------------------------------------------------------
-- adds an attribute name text and a text box with slider to edit the float attribute value
local addDefaultFloatAttributeSlider = function(panel, attribute, displayName)
  local staticText = panel:addStaticText{
    text = string.format("%s:", displayName),
    flags = "right"
  }

  local textBox = panel:addFloatSlider{
    min = 0, max = anim.getTakeDuration(anim.getSelectedTakeInAssetManager()),
    value = anim.getAttribute(attribute),
    flags = "expand",
    onChanged = function(self)
      local newValue = self:getValue()
      setWithNoRebuild(anim.setAttribute, attribute, newValue)
      self:setValue(anim.getAttribute(attribute))
    end
  }

  return staticText, textBox
end

------------------------------------------------------------------------------------------------------------------------
-- adds an attribute name text and a text box to edit the float attribute value
-- assumes this is called within the scope of a flex grid sizer
local addDefaultFloatAttribute = function(panel, attribute, displayName)
  local staticText = panel:addStaticText{
    text = string.format("%s:", displayName),
    flags = "right"
  }

  local value = anim.getAttribute(attribute)
  local textValue = tostring(value)
  local textBox = panel:addTextBox{
    value = textValue,
    flags = "expand;numeric",
    onEnter = function(self)
      local newTextValue = self:getValue()
      local newValue = tonumber(newTextValue)
      setWithNoRebuild(anim.setAttribute, attribute, newValue)
      local value = anim.getAttribute(attribute)
      local textValue = utils.numberToString(value, 0, 3)
      self:setValue(textValue)
    end
  }

  return staticText, textBox
end

------------------------------------------------------------------------------------------------------------------------
-- adds an attribute name text and a text box to edit the int attribute value
-- assumes this is called within the scope of a flex grid sizer
local addDefaultIntAttribute = function(panel, attribute, displayName)
  local staticText = panel:addStaticText{
    text = string.format("%s:", displayName),
    flags = "right"
  }

  local value = anim.getAttribute(attribute)
  local textValue = tostring(value)
  local textBox = panel:addTextBox{
    value = textValue,
    flags = "expand;numeric",
    onEnter = function(self)
      local newTextValue = self:getValue()
      local newValue = tonumber(newTextValue)
      setWithNoRebuild(anim.setAttribute, attribute, newValue)
      local value = anim.getAttribute(attribute)
      local textValue = tostring(value)
      self:setValue(textValue)
    end
  }

  return staticText, textBox
end

------------------------------------------------------------------------------------------------------------------------
-- adds an attribute name text and a text box to edit the bool attribute value
-- assumes this is called within the scope of a flex grid sizer
local addDefaultBoolAttribute = function(panel, attribute, displayName)
  local staticText = panel:addStaticText{
    text = string.format("%s:", displayName),
    flags = "right"
  }

  local checkBox = panel:addCheckBox{
    checked = anim.getAttribute(attribute),
    onChanged = function(self)
      local newValue = self:getValue()
      setWithNoRebuild(anim.setAttribute, attribute, newValue)
      self:setValue(anim.getAttribute(attribute))
    end
  }

  return staticText, checkBox
end

------------------------------------------------------------------------------------------------------------------------
-- adds an attribute name text and a text box to edit the string attribute value
-- assumes this is called within the scope of a flex grid sizer
local addDefaultStringAttribute = function(panel, attribute, displayName)
  local staticText = panel:addStaticText{
    text = string.format("%s:", displayName),
    flags = "right"
  }

  local textBox = panel:addTextBox{
    value = tostring(anim.getAttribute(attribute)),
    flags = "expand",
    onEnter = function(self)
      local newValue = self:getValue()
      setWithNoRebuild(anim.setAttribute, attribute, newValue)
      self:setValue(anim.getAttribute(attribute))
    end
  }

  return staticText, textBox
end

------------------------------------------------------------------------------------------------------------------------
-- adds a named section of attributes to the animation data attribute editor
-- assumes this is called within the scope of a flex grid sizer
local addAnimAttributeSection = function(panel, parent, attributes, dynamicSection)
  local attributeCount = table.getn(attributes)

  local flexGridCols = 2

  if attributeCount > 0 then
    if dynamicSection then
      panel:addStaticText{
        text = "Custom Properties",
        font = "large",
        flags = "expand"
      }

      flexGridCols = 3

      panel:addVSpacer(2)
    end

    panel:beginFlexGridSizer{
      cols = flexGridCols,
      rows = attributeCount,
      flags = "expand"
    }

    panel:setFlexGridColumnExpandable(2)

    for i, attr in attributes do
      local fullAttrPath = string.format("%s.%s", parent, attr)
      local type = anim.getAttributeType(fullAttrPath)

      if type == "float" then
        addDefaultFloatAttribute(panel, fullAttrPath, attr)
      elseif type == "int" then
        addDefaultIntAttribute(panel, fullAttrPath, attr)
      elseif type == "bool" then
        addDefaultBoolAttribute(panel, fullAttrPath, attr)
      elseif type == "string" then
        addDefaultStringAttribute(panel, fullAttrPath, attr)
      end

      if dynamicSection then
        panel:addButton{
          label = "Remove",
          onClick = function(self)
            anim.deleteAttribute(fullAttrPath)
          end
        }
      end
    end

    panel:endSizer()
  end

  panel:beginHSizer{ flags = "expand" }

    panel:addStretchSpacer{ proportion = 1 }

    if dynamicSection then
      panel:addVSpacer(2)

      panel:addButton{
        label = "Add Attribute",
        flags = "right",
        onClick = function()
          showAddAnimAttributeDialog(parent)
        end
      }
    end

  panel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- adds a section of attributes specialised for duration events
local addDurationEventAttributeSection = function(panel, event)
  panel:beginFlexGridSizer{
    cols = 2,
    rows = 3,
    flags = "expand",
  }

  local round = function(number)
    return math.floor(number + 0.5)
  end

  panel:setFlexGridColumnExpandable(2)

  local timePositionAttr = string.format("%s.TimePosition", event)
  local timeDurationAttr = string.format("%s.TimeDuration", event)
  local userDataAttr = string.format("%s.UserData", event)

  local takeLength = anim.getTakeDuration(anim.getSelectedTakeInAssetManager())

  local timelineDisplaysFrames = preferences.get("TimeDisplayUnits") == "Frames"
  local framerate = 0
  if timelineDisplaysFrames then
    framerate = preferences.get("TimeDisplayFramerate")
  end

  -- start position
  local startPositionSlider = nil
  if timelineDisplaysFrames then
    panel:addStaticText{
      text = "Start Position:",
      flags = "right"
    }
    startPositionSlider = panel:addIntSlider{
      min = 0, max = framerate * takeLength,
      value = round(anim.getAttribute(timePositionAttr) * framerate),
      flags = "expand"
    }
  else
    _, startPositionSlider = addDefaultFloatAttributeSlider(panel, timePositionAttr, "Start Position")
  end

  -- end position
  panel:addStaticText{
    text = "End Position:",
    flags = "right"
  }

  -- as the event only has position and duration we have to combine these to set/get the end position.
  local endPositionSlider = nil
  if timelineDisplaysFrames then

    endPositionSlider = panel:addIntSlider{
      min = 0, max = framerate * takeLength,
      value = round((anim.getAttribute(timePositionAttr) + anim.getAttribute(timeDurationAttr)) * framerate),
      flags = "expand"
    }
  else
    endPositionSlider = panel:addFloatSlider{
      min = 0, max = takeLength,
      value = anim.getAttribute(timePositionAttr) + anim.getAttribute(timeDurationAttr),
      flags = "expand"
    }
  end

  -- also add duration to give the user the option of editing both
  local durationFloatSlider = nil
  if timelineDisplaysFrames then
    panel:addStaticText{
      text = "Duration:",
      flags = "right"
    }

    durationFloatSlider = panel:addIntSlider{
      min = 0, max = framerate * takeLength,
      value = round(anim.getAttribute(timeDurationAttr) * framerate),
      flags = "expand"
    }
  else
    _, durationFloatSlider = addDefaultFloatAttributeSlider(panel, timeDurationAttr, "Duration")
  end

  if timelineDisplaysFrames then
    durationFloatSlider:setOnChanged(
      function(self)
        local newValue = self:getValue() / framerate
        setWithNoRebuild(anim.setAttribute, timeDurationAttr, newValue)
        local duration = anim.getAttribute(timeDurationAttr)
        self:setValue(round(duration * framerate))
        local endValue = anim.getAttribute(timePositionAttr) + duration
        if endValue > takeLength then
          endValue = endValue - takeLength
        end
        endPositionSlider:setValue(round(endValue * framerate))
      end
    )
  else
    durationFloatSlider:setOnChanged(
      function(self)
        local newValue = self:getValue()
        setWithNoRebuild(anim.setAttribute, timeDurationAttr, newValue)
        local duration = anim.getAttribute(timeDurationAttr)
        self:setValue(duration)
        local endValue = anim.getAttribute(timePositionAttr) + duration
        if endValue > takeLength then
          endValue = endValue - takeLength
        end
        endPositionSlider:setValue(endValue)
      end
    )
  end

  if timelineDisplaysFrames then
    endPositionSlider:setOnChanged(
      function(self)
        local newValue = self:getValue() / framerate - anim.getAttribute(timePositionAttr)
        if newValue < 0 then
          newValue = newValue + takeLength
        end
        setWithNoRebuild(anim.setAttribute, timeDurationAttr, newValue)
        local duration = anim.getAttribute(timeDurationAttr)
        durationFloatSlider:setValue(round(duration * framerate))
        newValue = (anim.getAttribute(timePositionAttr) + duration)
        if newValue > takeLength then
          newValue = newValue - takeLength
        end
        self:setValue(round(newValue * framerate))
      end
    )
  else
    endPositionSlider:setOnChanged(
      function(self)
        local newValue = self:getValue() - anim.getAttribute(timePositionAttr)
        if newValue < 0 then
          newValue = newValue + takeLength
        end
        setWithNoRebuild(anim.setAttribute, timeDurationAttr, newValue)
        local duration = anim.getAttribute(timeDurationAttr)
        durationFloatSlider:setValue(duration)
        newValue = anim.getAttribute(timePositionAttr) + duration
        if newValue > takeLength then
          newValue = newValue - takeLength
        end
        self:setValue(newValue)
      end
    )
  end

  if timelineDisplaysFrames then
    startPositionSlider:setOnChanged(
      function(self)
        local newValue = self:getValue() / framerate
        setWithNoRebuild(anim.setAttribute, timePositionAttr, newValue)
        local duration = endPositionSlider:getValue() / framerate - newValue
        if duration < 0 then
          duration = duration + takeLength
        end
        setWithNoRebuild(anim.setAttribute, timeDurationAttr, duration)
        durationFloatSlider:setValue(round(duration * framerate))
      end
    )
  else
    startPositionSlider:setOnChanged(
      function(self)
        local newValue = self:getValue()
        setWithNoRebuild(anim.setAttribute, timePositionAttr, newValue)
        local duration = endPositionSlider:getValue() - newValue
        if duration < 0 then
          duration = duration + takeLength
        end
        setWithNoRebuild(anim.setAttribute, timeDurationAttr, duration)
        durationFloatSlider:setValue(duration)
      end
    )
  end

  addDefaultIntAttribute(panel, userDataAttr, "User Data")

  panel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- adds a section of attributes specialised for tick events
local addTickEventAttributeSection = function(panel, event)
  panel:beginFlexGridSizer{
    cols = 2,
    rows = 2,
    flags = "expand"
  }

  local round = function(number)
    return math.floor(number + 0.5)
  end

  panel:setFlexGridColumnExpandable(2)

  local timelineDisplaysFrames = preferences.get("TimeDisplayUnits") == "Frames"
  local framerate = 0
  if timelineDisplaysFrames then
    framerate = preferences.get("TimeDisplayFramerate")
  end

  local timePositionAttr = string.format("%s.TimePosition", event)
  local userDataAttr = string.format("%s.UserData", event)

  local takeLength = anim.getTakeDuration(anim.getSelectedTakeInAssetManager())

  -- start position
  if timelineDisplaysFrames then
    panel:addStaticText{
      text = "Time:",
      flags = "right"
    }
    
    startPositionSlider = panel:addIntSlider{
      min = 0, max = framerate * takeLength,
      value = round(anim.getAttribute(timePositionAttr) * framerate),
      flags = "expand",
      onChanged = function(self)

        local newValue = self:getValue() / framerate
        setWithNoRebuild(anim.setAttribute, timePositionAttr, newValue)
        self:setValue(round(anim.getAttribute(timePositionAttr) * framerate))
      end
    }

  else
    addDefaultFloatAttributeSlider(panel, timePositionAttr, "Position")
  end

  addDefaultIntAttribute(panel, userDataAttr, "User Data")

  panel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- adds an event to the editor panel
local addEventToEditor = function(panel, event)
  -- get a list of standard and dynamic attributes.
  local standardAttributes, dynamicAttributes = separateDynamicAttributes(event)

  local track = anim.getParent(event)
  local eventType = anim.getAttribute(string.format("%s.%s", track, "EventType"))

  if eventType == "Duration" then
    -- add standard attributes
    addDurationEventAttributeSection(panel, event)
  elseif eventType == "Tick" then
    -- add standard attributes
    addTickEventAttributeSection(panel, event)
  else
    -- add standard attributes
    addAnimAttributeSection(panel, event, standardAttributes)
  end

  panel:addVSpacer(5)

  -- add dynamic attributes
  addAnimAttributeSection(panel, event, dynamicAttributes, true)
end

------------------------------------------------------------------------------------------------------------------------
-- adds a track to the editor panel
local addTrackToEditor = function(panel, track)
  -- get a list of standard and dynamic attributes.
  local standardAttributes, dynamicAttributes = separateDynamicAttributes(track)

  -- add standard track attributes
    panel:beginFlexGridSizer{
    cols = 2,
    rows = 2,
    flags = "expand"
  }

  panel:setFlexGridColumnExpandable(2)

  local trackNameAttr = string.format("%s.TrackName", track)
  local eventTypeAttr = string.format("%s.EventType", track)
  local userDataAttr = string.format("%s.UserData", track)

  -- renaming a track requires a rebuild of the editor
  panel:addStaticText{ text = "Name:", flags = "right" }
  panel:addTextBox{
    value = tostring(anim.getAttribute(trackNameAttr)),
    flags = "expand",
    onEnter = function(self)
      local newValue = self:getValue()
      panel:addIdleCallback(
        function()
          -- setAttribute will generate a mcMarkupSelectionChange event,
          -- which in turn will call rebuildAnimPropertiesEditor
          anim.setAttribute(trackNameAttr, newValue)
        end
      )
    end
  }

  local eventTypeLabel, eventTypeText = addDefaultStringAttribute(panel, eventTypeAttr, "Event Type")
  eventTypeText:enable(false)

  addDefaultIntAttribute(panel, userDataAttr, "User Data")

  panel:endSizer()

  panel:addVSpacer(5)

  -- add dynamic attributes
  addAnimAttributeSection(panel, track, dynamicAttributes, true)
end

------------------------------------------------------------------------------------------------------------------------
-- rebuilds the anim properties editor
local rebuildAnimPropertiesEditor = function(force)
  -- this dialog caused the rebuild so do nothing.
  if thisDialogChanged then
    return
  end

  local dlg = ui.getWindow("AnimPropertiesEditor")

  if dlg and (dlg:isShown() or force) then
    local eventTimelineSelection = anim.ls("selection")

    local tracks = { }
    local events = { }
    for i, object in eventTimelineSelection do
      local type = anim.getType(object)

      if type == "track" then
        table.insert(tracks, object)
      elseif type == "event" then
        table.insert(events, object)
      end
    end

    if table.getn(events) == 0 and table.getn(tracks) == 0 then
      dlg:hide()
      return
    end

    dlg:clear()

    dlg:beginVSizer{
      flags = "expand",
      proportion = 1
    }

      if table.getn(events) > 0 then
        dlg:addStaticText{
          text = "Event",
          font = "bold",
          flags = "expand"
        }
        dlg:addVSpacer(3)
        -- display the first selected event if there is one
        addEventToEditor(dlg, events[1])
      elseif table.getn(tracks) > 0 then
        dlg:addStaticText{
          text = "Track",
          font = "bold",
          flags = "expand"
        }
        dlg:addVSpacer(3)
        -- display the first selected track if there are no events selected
        addTrackToEditor(dlg, tracks[1])
      end

    dlg:endSizer()

    dlg:rebuild()
  end
end

------------------------------------------------------------------------------------------------------------------------
-- show the animation data properties editor
showAnimPropertiesEditor = function()
  local dlg = ui.getWindow("AnimPropertiesEditor")

  if not dlg then
    dlg = ui.createModelessDialog{
      name = "AnimPropertiesEditor",
      caption = "Markup Properties",
      resize = false
    }
  end

  rebuildAnimPropertiesEditor(true)

  dlg:show()
end

------------------------------------------------------------------------------------------------------------------------
-- hide the animation data properties editor
hideAnimPropertiesEditor = function()
  local dlg = ui.getWindow("AnimPropertiesEditor")

  if dlg then
    dlg:hide()
  end
end

------------------------------------------------------------------------------------------------------------------------
if not animationMarkupHandlersRegistered then
  registerEventHandler(
    "mcMarkupSelectionChange",
    function()
      rebuildAnimPropertiesEditor(false)
    end
  )

  animationMarkupHandlersRegistered = true
end
