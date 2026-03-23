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
-- Add's a controlParameterConditionInRangeInfoSection.
-- Used by ControlParameterConditions.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.controlParameterConditionInRangeInfoSection = function(rollPanel, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.controlParameterConditionInRangeInfoSection")
  rollPanel:beginVSizer{ flags = "expand", proportion = 0 }
    local controlParameterAttrPaths = { }
    local controlParameterLowerTestValues = { }
    local controlParameterUpperTestValues = { }
    local controlParameterNotInRange = { }
    for i, object in ipairs(selection) do
      table.insert(controlParameterAttrPaths, string.format("%s.ControlParameter", object))
      table.insert(controlParameterLowerTestValues, string.format("%s.LowerTestValue", object))
      table.insert(controlParameterUpperTestValues, string.format("%s.UpperTestValue", object))
      table.insert(controlParameterNotInRange, string.format("%s.NotInRange", object))
    end

    local cparamHelpText = getAttributeHelpText(selection[1], "ControlParameter")
    local lowerValueHelpText = getAttributeHelpText(selection[1], "LowerTestValue")
    local upperValueHelpText = getAttributeHelpText(selection[1], "UpperTestValue")
    local notInRangeHelpText = getAttributeHelpText(selection[1], "NotInRange")

    rollPanel:beginHSizer{ flags = "expand", proportion = 1 }

      local lowerValue = rollPanel:addAttributeWidget{
        attributes = controlParameterLowerTestValues,
        flags = "expand",
        proportion = 1,
         onMouseEnter = function()
          attributeEditor.setHelpText(lowerValueHelpText)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }
      rollPanel:addStaticText{ text = "<=" }
      local controlParameterBox = rollPanel:addAttributeWidget{
        attributes = controlParameterAttrPaths,
        flags = "expand",
        proportion = 2,
         onMouseEnter = function()
          attributeEditor.setHelpText(cparamHelpText)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }
      rollPanel:addStaticText{ text = "<=" }
      local upperValue = rollPanel:addAttributeWidget{
        attributes = controlParameterUpperTestValues,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function()
          attributeEditor.setHelpText(upperValueHelpText)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

    rollPanel:endSizer()
    rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
      attributeEditor.addStaticTextWithHelp(rollPanel, "Not In Range", notInRangeHelpText)
      local boolValue = rollPanel:addAttributeWidget{
        attributes = controlParameterNotInRange
      }
    rollPanel:endSizer()
    ------------------------------------------------------------------------------------------------------------------------
  rollPanel:endSizer()
  attributeEditor.logExitFunc("attributeEditor.controlParameterConditionInRangeInfoSection")
end

