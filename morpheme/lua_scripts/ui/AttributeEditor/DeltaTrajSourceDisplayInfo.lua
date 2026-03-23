------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"

local sourceHelpText = [[
Choose where we source delta trajectory from.
]]

------------------------------------------------------------------------------------------------------------------------
-- Add's a display info section for DeltaTrajSource attribute.
-- Used by TransitMatchEvents
------------------------------------------------------------------------------------------------------------------------
attributeEditor.DeltaTrajSourceDisplayInfoSection = function(rollContainer, displayInfo, selection)
  if hasTransitionCategory(selection, "euphoria") then
    return
  end
  
  attributeEditor.logEnterFunc("attributeEditor.DeltaTrajSourceDisplayInfoSection")

  local hasReference = containsReference(selection)

  -- first add the ui for the section
  attributeEditor.log("rollContainer:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "DeltaTrajSourceDisplayInfoSection" }
  local rollPanel = rollup:getPanel()
  rollup:expand(false)

  attributeEditor.log("rollPanel:beginHSizer")
  rollPanel:beginVSizer{ flags = "expand", proportion = 1 }
    rollPanel:addHSpacer(6)
    rollPanel:setBorder(1)

    attributeEditor.log("rollPanel:beginFlexGridSizer")
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      rollPanel:setFlexGridColumnExpandable(2)

        local sourceItems = {
          "Destination",
          "Source",
          "Blend"
        }

        rollPanel:addStaticText{
          text = "Source:",
          onMouseEnter = function()
            attributeEditor.setHelpText(sourceHelpText)
          end,
          onMouseLeave = function()
            attributeEditor.clearHelpText()
          end
        }

        local spacingComboBox = attributeEditor.addCustomComboBox{
          panel = rollPanel,
          objects = selection,
          attributes = { "DeltaTrajSource" },
          helpText = sourceHelpText,
          values = {
            ["Blend"] = function(selection)
              for i, object in pairs(selection) do
                setAttribute(string.format("%s.DeltaTrajSource", object), 3)
              end
            end,
            ["Source"] = function(selection)
              for i, object in pairs(selection) do
                setAttribute(string.format("%s.DeltaTrajSource", object), 2)
              end
            end,
            ["Destination"] = function(selection)
              for i, object in pairs(selection) do
                setAttribute(string.format("%s.DeltaTrajSource", object), 1)
              end
            end,
          },

          syncValueWithUI = function(combo, selection)
            local sourceValue = getCommonAttributeValue(selection, "DeltaTrajSource")

            if sourceValue == nil then
              attributeEditor.log("not all objects have the same value, clearing current selection index")
              combo:addItem("")
              combo:setSelectedIndex(4)
            elseif sourceValue < 4 then
              attributeEditor.log("all objects have the same value, setting selection to \"%s\"", sourceItems[sourceValue])
              combo:setSelectedIndex(sourceValue)
            else
              attributeEditor.log("all objects have the same value, setting selection to \"%s\"", sourceItems[3])
              combo:setSelectedIndex(3)
            end
          end
        }

      attributeEditor.log("rollPanel:endSizer")
    rollPanel:endSizer()

    attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.DeltaTrajSourceDisplayInfoSection")
end

