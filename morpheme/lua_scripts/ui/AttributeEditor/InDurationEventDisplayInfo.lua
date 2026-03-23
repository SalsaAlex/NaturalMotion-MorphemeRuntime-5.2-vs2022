------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- Adds a InDurationEventInfoSection
-- Used by InDurationEvent
------------------------------------------------------------------------------------------------------------------------
attributeEditor.InDurationEventDisplayInfo = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.InDurationEventInfoSection")

  local hasReference = containsReference(selection)

  attributeEditor.log("rollContainter:addRollup")
    -- first add the ui for the section
  attributeEditor.log("rollPanel:beginHSizer")
  local eventTrackUserID = { }
  local useEventTrackUserID = { }
  local eventUserID = { }
  local useEventUserID = { }
  for i, object in ipairs(selection) do
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
        falseValue = "Dont use Track user ID",
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
        falseValue = "Dont use Event user ID",
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
  rollContainer:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 0 }
  rollContainer:setFlexGridColumnExpandable(2)
    attributeEditor.addAttributeLabel(rollContainer, getAttributeDisplayName(selection[1], "OnNotSet") , selection, "OnNotSet")
    attributeEditor.addAttributeWidget(rollContainer, "OnNotSet", selection)
  rollContainer:endSizer()

  local syncInputFields = function()
    attributeEditor.logEnterFunc("enabledContext attributeChangedHandler")

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

    attributeEditor.logExitFunc("enableRequirements")
  end

  -- this data change context ensures the ui reflects any changes that happen to selected
  -- blend with event nodes through script or undo redo are reflected in the custom ui.
  attributeEditor.log("creating data change context")
  local enabledContext = attributeEditor.createChangeContext()

  enabledContext:setObjects(selection)
  enabledContext:addAttributeChangeEvent("UseEventUserTypeID")
  enabledContext:addAttributeChangeEvent("UseEventTrackUserTypeID")
  enabledContext:setAttributeChangedHandler(
    function(object, attr)
      syncInputFields()
    end
  )

  syncInputFields()

  attributeEditor.logExitFunc("attributeEditor.InDurationEventDisplayInfo")
end

