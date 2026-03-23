------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- Adds a FractionThroughDurationEventInfoSection
-- Used by FractionThroughDurationEvent
------------------------------------------------------------------------------------------------------------------------
attributeEditor.FractionThroughDurationEventInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.FractionThroughDurationEventInfoSection")

  attributeEditor.log("rollContainter:addRollup")
    -- first add the ui for the section
  attributeEditor.log("rollPanel:beginHSizer")
    local testValue = { }
    local eventTrackUserID = { }
    local useEventTrackUserID = { }
    local eventUserID = { }
    local useEventUserID = { }
    for i, object in ipairs(selection) do
      table.insert(testValue, string.format("%s.TestValue", object))
      table.insert(eventTrackUserID, string.format("%s.EventTrackUserTypeID", object))
      table.insert(useEventTrackUserID, string.format("%s.UseEventTrackUserTypeID", object))
      table.insert(eventUserID, string.format("%s.EventUserTypeID", object))
      table.insert(useEventUserID, string.format("%s.UseEventUserTypeID", object))
    end
    attributeEditor.log("rollContainer:addAttributeWidget")
    rollContainer:addHSpacer(6)

    ------------------------
    rollContainer:beginHSizer{ flags = "expand", proportion = 0 }
      local HelpText = getAttributeHelpText(selection[1], "UseEventTrackUserTypeID")
      attributeEditor.addBoolAttributeCombo{
          panel = rollContainer,
          objects = selection,
          attribute = "UseEventTrackUserTypeID",
          trueValue = "Use Track user ID",
          falseValue = "Don't use Track user ID",
          helpText = HelpText,
          flags = "expand",
          proportion = 1
        }
    rollContainer:endSizer()

    ------------------------
    rollContainer:beginHSizer{ flags = "expand", proportion = 1 }
      rollContainer:addStaticText{
          text = "Track user ID",
          onMouseEnter = function()
            attributeEditor.setHelpText(getAttributeHelpText(selection[1], "EventTrackUserTypeID"))
          end,
          onMouseLeave = function()
            attributeEditor.clearHelpText()
          end
        }

      local trackUserIDWdgt = rollContainer:addAttributeWidget{
        attributes = eventTrackUserID,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function()
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "EventTrackUserTypeID"))
        end,
        onMouseLeave = function()
         attributeEditor.clearHelpText()
        end
      }
    rollContainer:endSizer();

    ------------------------
    rollContainer:beginHSizer{ flags = "expand", proportion = 0 }
      local HelpText = getAttributeHelpText(selection[1], "UseEventUserTypeID")
      attributeEditor.addBoolAttributeCombo{
          panel = rollContainer,
          objects = selection,
          attribute = "UseEventUserTypeID",
          trueValue = "Use Event user ID",
          falseValue = "Don't use Event user ID",
          helpText = HelpText,
          flags = "expand",
          proportion = 1
        }
    rollContainer:endSizer()

    ------------------------
    rollContainer:beginHSizer{ flags = "expand", proportion = 1 }
      rollContainer:addStaticText{
          text = "Event user ID",
          onMouseEnter = function()
            attributeEditor.setHelpText(getAttributeHelpText(selection[1], "EventUserTypeID"))
          end,
          onMouseLeave = function()
            attributeEditor.clearHelpText()
          end
        }

      local eventUserIDWdgt = rollContainer:addAttributeWidget{
        attributes = eventUserID,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function()
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "EventUserTypeID"))
        end,
        onMouseLeave = function()
         attributeEditor.clearHelpText()
        end
      }
    rollContainer:endSizer();

    ------------------------
    rollContainer:beginHSizer{ flags = "expand", proportion = 1 }
      rollContainer:addStaticText{
          text = "Fraction ",
          onMouseEnter = function()
            attributeEditor.setHelpText(getAttributeHelpText(selection[1], "TestValue"))
          end,
          onMouseLeave = function()
            attributeEditor.clearHelpText()
          end
        }

      local valueSlider = rollContainer:addFloatSlider{
        value = 1 - getAttribute(selection[1] .. ".TestValue"),
        min = 0,
        max = 1,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function()
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "TestValue"))
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      if containsReference(selection) then
        valueSlider:enable(false)
      end
    rollContainer:endSizer()

    valueSlider:setOnChanged(
      function(self)
        local sliderValue = self:getValue()
        local attrValue = 1 - sliderValue
        setCommonAttributeValue(selection, "TestValue", attrValue)
      end
    )

 ----------------------------------------------------------------------------------------------------------------------
  -- this data change context ensures the ui reflects any changes that happen via script, or through undo redo.
  ----------------------------------------------------------------------------------------------------------------------
  local changeContext = attributeEditor.createChangeContext()

  changeContext:setObjects(selection)
  changeContext:addAttributeChangeEvent("UseEventUserTypeID")
  changeContext:addAttributeChangeEvent("UseEventTrackUserTypeID")
  changeContext:addAttributeChangeEvent("TestValue")

  local syncInterface = function()
    local enableEventID = getCommonAttributeValue(selection, "UseEventUserTypeID");
    if enableEventID ~= nil then
      eventUserIDWdgt:enable(enableEventID and not hasReference)
    else
      eventUserIDWdgt:enable(false)
    end

    local enableTrackID = getCommonAttributeValue(selection, "UseEventTrackUserTypeID")
    if enableTrackID ~= nil then
      trackUserIDWdgt:enable(enableTrackID and not hasReference)
    else
      trackUserIDWdgt:enable(false)
    end

    local attrValue = getCommonAttributeValue(selection, "TestValue")
    if attrValue ~= nil then
      local sliderValue = tonumber(string.format("%.6f", 1 - attrValue))
      valueSlider:setIsIndeterminate(false)
      valueSlider:setValue(sliderValue)
    else
      valueSlider:setIsIndeterminate(true)
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

  attributeEditor.logExitFunc("attributeEditor.FractionThroughDurationEventInfoSection")
end

