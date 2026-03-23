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
Choose between merging and blending duration events.  Merging duration events will output all events from all sources as a combined event track. Blending duration events will output only events that can be blended together as a combined event track. The conditions for whether an event can be successfully blended together are set using the options in the Blend Requirements Section.
]]

local spacingHelpText = [[
Ignoring spacing will blend events together regardless of their position and duration.
On overlap spacing will blend events together only if they match all other criteria and are overlapping.
Within range spacing will blend events together if they are less than 1.0 apart.
]]

------------------------------------------------------------------------------------------------------------------------
-- Add's a display info section for blend duration event attributes.
-- Used by Blend2MatchEvents, BlendNMatchEvents and TransitMatchEvents.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.blendDurationEventDisplayInfo = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.blendDurationEventDisplayInfo")

  local hasReference = containsReference(selection)

  -- first add the ui for the section
  attributeEditor.log("rollContainer:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "blendDurationEventDisplayInfo" }
  rollup:expand(false)

  local rollPanel = rollup:getPanel()

  rollPanel:beginVSizer{ flags = "expand", proportion = 1 }
    rollPanel:beginHSizer{ flags = "expand", proportion = 0 }

      local styleText = rollPanel:addStaticText{
        text = "Style:",
        onMouseEnter = function()
          attributeEditor.setHelpText(styleHelpText)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      local styleCombo = attributeEditor.addBoolAttributeCombo{
        panel = rollPanel,
        objects = selection,
        attribute = "DurationEventBlendPassThrough",
        falseValue = "Blend",
        trueValue ="Merge",
        helpText = styleHelpText,
      }

    rollPanel:endSizer()
    rollPanel:addVSpacer(5)
    rollPanel:beginVSizer{ label = "Blend Requirements", flags = "expand;group", proportion = 1 }

      rollPanel:addVSpacer(4)

      attributeEditor.log("rollPanel:beginFlexGridSizer")
      rollPanel:beginFlexGridSizer{ cols = 2, rows = 2, flags = "expand" }
        rollPanel:setFlexGridColumnExpandable(2)

        attributeEditor.log("attributeEditor.addStaticTextWithHelp")
        local ignoreEventOrderHelpText = getAttributeHelpText(selection[1], "DurationEventBlendIgnoreEventOrder")
        local ignoreEventOrderText = attributeEditor.addStaticTextWithHelp(rollPanel, "Ignore Event Order", ignoreEventOrderHelpText)

        -- same user data can be added as a standard attribute widget.
        local attrPaths = { }
        for j, object in pairs(selection) do
          table.insert(attrPaths, string.format("%s.DurationEventBlendIgnoreEventOrder", object))
        end

        attributeEditor.log("rollPanel:addAttributeWidget")
        local ignoreEventOrderWidget = rollPanel:addAttributeWidget{
          attributes = attrPaths,
          flags = "expand",
          proportion = 1,
          onMouseEnter = function()
            attributeEditor.setHelpText(ignoreEventOrderHelpText)
          end,
          onMouseLeave = function()
            attributeEditor.clearHelpText()
          end
        }

        attributeEditor.log("attributeEditor.addStaticTextWithHelp")
        local sameUserDataHelp = getAttributeHelpText(selection[1], "DurationEventBlendSameUserData")
        local sameUserDataText = attributeEditor.addStaticTextWithHelp(rollPanel, "Same User Data", sameUserDataHelp)

        -- same user data can be added as a standard attribute widget.
        local attrPaths = { }
        for j, object in pairs(selection) do
          table.insert(attrPaths, string.format("%s.DurationEventBlendSameUserData", object))
        end

        attributeEditor.log("rollPanel:addAttributeWidget")
        local sameUserDataWidget = rollPanel:addAttributeWidget{
          attributes = attrPaths,
          flags = "expand",
          proportion = 1,
          onMouseEnter = function()
            attributeEditor.setHelpText(sameUserDataHelp)
          end,
          onMouseLeave = function()
            attributeEditor.clearHelpText()
          end
        }

      attributeEditor.log("rollPanel:endFlexGridSizer")
      rollPanel:endSizer()

      rollPanel:beginHSizer{ flags = "expand", proportion = 1 }

        local spacingItems = {
          "Ignore",
          "On Overlap",
          "Within Range"
        }

        local spacingText = attributeEditor.addStaticTextWithHelp(rollPanel, "Spacing", spacingHelpText)

        local spacingComboBox = attributeEditor.addCustomComboBox{
          panel = rollPanel,
          objects = selection,
          attributes = { "DurationEventBlendOnOverlap", "DurationEventBlendWithinRange" },
          helpText = spacingHelpText,
          values = {
            ["Ignore"] = function(selection)
              for i, object in pairs(selection) do
                setAttribute(string.format("%s.DurationEventBlendOnOverlap", object), false)
                setAttribute(string.format("%s.DurationEventBlendWithinRange", object), false)
              end
            end,
            ["On Overlap"] = function(selection)
              for i, object in pairs(selection) do
                setAttribute(string.format("%s.DurationEventBlendOnOverlap", object), true)
                setAttribute(string.format("%s.DurationEventBlendWithinRange", object), false)
              end
            end,
            ["Within Range"] = function(selection)
              for i, object in pairs(selection) do
                setAttribute(string.format("%s.DurationEventBlendOnOverlap", object), false)
                setAttribute(string.format("%s.DurationEventBlendWithinRange", object), true)
              end
            end,
          },
          syncValueWithUI = function(combo, selection)
            local onOverlapValue = getCommonAttributeValue(selection, "DurationEventBlendOnOverlap")
            local withinRangeValue = getCommonAttributeValue(selection, "DurationEventBlendWithinRange")

            if onOverlapValue == nil or withinRangeValue == nil then
              attributeEditor.log("not all objects have the same value, clearing current selection index")
              combo:addItem("")
              combo:setSelectedIndex(4)
            elseif not onOverlapValue and not withinRangeValue then
              attributeEditor.log("all objects have the same value, setting selection to \"%s\"", spacingItems[1])
              combo:setSelectedIndex(3)
            elseif onOverlapValue or not withinRangeValue then
              attributeEditor.log("all objects have the same value, setting selection to \"%s\"", spacingItems[1])
              combo:setSelectedIndex(2)
            else
              attributeEditor.log("all objects have the same value, setting selection to \"%s\"", spacingItems[2])
              combo:setSelectedIndex(1)
            end
          end
        }

      rollPanel:endSizer()
    rollPanel:endSizer()
  rollPanel:endSizer()

  -- syncs the enable disable state of the blend requirements controls with the blend style combo box.
  local syncBlendUI = function()
    local enableDurationEventBlending = true

    if attributeExists(selection[1], "TimeStretchMode") then
      local timeStretchMode = getCommonAttributeValue(selection, "TimeStretchMode")
      if not timeStretchMode or timeStretchMode == 0 then
        enableDurationEventBlending = false
      end
    end

    local enableBlendRequirements = false
    if enableDurationEventBlending then
      if getCommonAttributeValue(selection, "DurationEventBlendPassThrough") == false then
        enableBlendRequirements = true
        attributeEditor.log("enabling requirements")
      else
        attributeEditor.log("disabling requirements")
      end
    end

    attributeEditor.logEnterFunc("enabledContext attributeChangedHandler")

    styleText:enable(enableDurationEventBlending)
    styleCombo:enable(enableDurationEventBlending and not hasReference)

    ignoreEventOrderText:enable(enableBlendRequirements)
    ignoreEventOrderWidget:enable(enableBlendRequirements and not hasReference)

    sameUserDataText:enable(enableBlendRequirements)
    sameUserDataWidget:enable(enableBlendRequirements and not hasReference)

    spacingText:enable(enableBlendRequirements)
    spacingComboBox:enable(enableBlendRequirements and not hasReference)

    attributeEditor.logExitFunc("enableRequirements")
  end

  -- this data change context ensures the ui reflects any changes that happen to selected
  -- blend with event nodes through script or undo redo are reflected in the custom ui.
  attributeEditor.log("creating data change context")
  local enabledContext = attributeEditor.createChangeContext()

  enabledContext:setObjects(selection)
  if attributeExists(selection[1], "TimeStretchMode") then
    enabledContext:addAttributeChangeEvent("TimeStretchMode")
  end
  enabledContext:addAttributeChangeEvent("DurationEventBlendPassThrough")
  enabledContext:setAttributeChangedHandler(
    function(object, attr)
      syncBlendUI()
    end
  )

  syncBlendUI()

  attributeEditor.logExitFunc("attributeEditor.blendDurationEventDisplayInfo")
end