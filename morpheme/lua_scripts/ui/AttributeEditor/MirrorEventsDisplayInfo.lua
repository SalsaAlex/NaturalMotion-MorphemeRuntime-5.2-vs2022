------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"

local eventPassThroughHelpText = "\"Mirror\" the events in the animation or \"Pass through\" the events unchanged."
local eventOffsetHelpText = "The offset (first event) from which the mirrored animation will start."

------------------------------------------------------------------------------------------------------------------------
-- Add's a mirrorEventsDisplayInfoSection
------------------------------------------------------------------------------------------------------------------------
attributeEditor.mirrorEventsDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.mirrorEventsDisplayInfoSection")

  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "mirrorEvents" }
  local rollPanel = rollup:getPanel()

  -- first add the ui for the section
  attributeEditor.log("rollPanel:beginHSizer")
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    rollPanel:addHSpacer(6)
    rollPanel:setBorder(1)

    attributeEditor.log("rollPanel:beginFlexGridSizer")
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      rollPanel:setFlexGridColumnExpandable(2)

      attributeEditor.addStaticTextWithHelp(rollPanel, "Mode", eventPassThroughHelpText)
      attributeEditor.log("rollPanel:addStaticText")

      local planeVectorWidget = nil
      local eventPassThroughCombo = nil

      eventPassThroughCombo = attributeEditor.addBoolAttributeCombo{
        panel = rollPanel,
        objects = selection,
        attribute = "EventPassThrough",
        trueValue = "Pass through",
        falseValue ="Mirror",
        helpText = eventPassThroughHelpText
      }

      local attrPaths = { }
      for j, object in pairs(selection) do
        local attrPath = string.format("%s.EventOffset", object)
        table.insert(attrPaths, attrPath)
      end

      attributeEditor.log("rollPanel:addStaticText")
      local offsetLabel = attributeEditor.addStaticTextWithHelp(rollPanel, "Offset", eventOffsetHelpText)

      attributeEditor.log("rollPanel:addAttributeWidget")
      local offsetWidget = rollPanel:addAttributeWidget{
        attributes = attrPaths,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function() attributeEditor.setHelpText(eventOffsetHelpText) end,
        onMouseLeave = function() attributeEditor.clearHelpText() end
      }

    attributeEditor.log("rollPanel:endFlexGridSizer")
    rollPanel:endSizer()

  attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.mirrorEventsDisplayInfoSection")

  local syncInterface = function()
    attributeEditor.logEnterFunc("syncInterface")
      local enableOffsets = false
      for j, object in pairs(selection) do
        local attrPath = string.format("%s.EventPassThrough", object)
        enableOffsets = enableOffsetWidget or (not getAttribute(attrPath))
      end
      offsetWidget:enable(enableOffsets)
      offsetLabel:enable(enableOffsets)
   attributeEditor.logExitFunc("syncInterface")
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- this data change context ensures the ui reflects any changes that happen via script, or through undo redo.
  ----------------------------------------------------------------------------------------------------------------------

  local changeContext = attributeEditor.createChangeContext()

  changeContext:setObjects(selection)
  changeContext:addAttributeChangeEvent("EventPassThrough")

  changeContext:setAttributeChangedHandler(
    function(object, attr)
      attributeEditor.logEnterFunc("changeContext attributeChangedHandler")
      syncInterface()
      attributeEditor.logExitFunc("changeContext attributeChangedHandler")
    end
  )

  syncInterface()
end

