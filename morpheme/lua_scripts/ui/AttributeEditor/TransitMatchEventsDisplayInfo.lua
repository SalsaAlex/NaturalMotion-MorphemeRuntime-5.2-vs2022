------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"

local styleHelpText = [[
Choose between Sequence Offset and an Index for destination start events.
]]
local offsetHelpText = [[
Offset the destination synchronisation event track in relation to the source synchronisation event track.
]]
local indexHelpText = [[
Force a particular event from the destination synchronisation event track to be the destination start event (irrespective of the source start event).
]]

------------------------------------------------------------------------------------------------------------------------
-- Add's a transitMatchEventsDisplayInfoSection.
-- Used by TransitMatchEvents.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.transitMatchEventsDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.transitMatchEventsDisplayInfoSection")

  local eventIndexWidget = nil
  local useEventIndexWidget = nil
  local indexLabel = nil;
  local styleComboBox = nil;

  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "transitMatchEventsDisplayInfoSection" }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginVSizer")
  rollPanel:beginVSizer{ flags = "expand", proportion = 1 }
    rollPanel:beginHSizer{ flags = "expand" }
      local sequenceOffsetAttrPaths = { }
      local eventIndexAttrPaths = { }
      local useEventIndexAttrPaths = { }

       -- the names that appear in the style popup
      local kSequenceOffsetName = "Sequence Offset"
      local kIndexName = "Index"

      -- the style popup
      local styleItems = {
        kSequenceOffsetName,
        kIndexName
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

      attributeEditor.addBoolAttributeCombo{
        panel = rollPanel,
        objects = selection,
        attribute = "UsingDestStartEventIndex",
        falseValue = "Sequence Offset",
        trueValue ="Index",
        helpText = styleHelpText,
      }
    rollPanel:endSizer()

    local destEventStartEventPanel = rollPanel:addPanel{ flags = "expand", proportion = 1 }
    destEventStartEventPanel:setBorder(0)

    attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  local rebuildUI = function()
    destEventStartEventPanel:clear()
    destEventStartEventPanel:setBorder(0)
    destEventStartEventPanel:beginHSizer{ flags = "expand", proportion = 1 }
    if (getAttribute(selection[1] .. ".UsingDestStartEventIndex") == true)then
      indexLabel = destEventStartEventPanel:addStaticText{
        text = "Index",
        onMouseEnter = function()
          attributeEditor.setHelpText(indexHelpText)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      destEventStartEventPanel:addHSpacer(2)

      local destStartEventIndexAttr = { }
      for i, object in ipairs(selection) do
        table.insert(destStartEventIndexAttr, string.format("%s.DestStartEventIndex", object))
      end

      eventIndexWidget = destEventStartEventPanel:addAttributeWidget{
        attributes = destStartEventIndexAttr,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function()
          attributeEditor.setHelpText(indexHelpText)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }
    else
      indexLabel = destEventStartEventPanel:addStaticText{
        text = "Offset",
        onMouseEnter = function()
          attributeEditor.setHelpText(offsetHelpText)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      destEventStartEventPanel:addHSpacer(4)

      local destStartEventOffsetAttr = { }
      for i, object in ipairs(selection) do
        table.insert(destStartEventOffsetAttr, string.format("%s.DestEventSequenceOffset", object))
      end

      eventOffsetWidget = destEventStartEventPanel:addAttributeWidget{
        attributes = destStartEventOffsetAttr,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function()
          attributeEditor.setHelpText(offsetHelpText)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }
    end
    destEventStartEventPanel:endSizer()
  end

  local changeContext = attributeEditor.createChangeContext()

  changeContext:setObjects(selection)
  changeContext:addAttributeChangeEvent("UsingDestStartEventIndex")
  changeContext:setAttributeChangedHandler(
    function(object, attr)
      rebuildUI()
    end
  )

  rebuildUI()

  attributeEditor.logExitFunc("attributeEditor.transitMatchEventsDisplayInfoSection")
end

