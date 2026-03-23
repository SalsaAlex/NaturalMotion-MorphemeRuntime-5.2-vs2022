------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"

local evaluateHelpText = [[
Evaluation of weight pin.
The value of the weight pin is only evaluated when the current input animation has finished so the animation only changes at the end of playback.
The value of the weight pin is evaluated every frame potentially switching which input is used every frame.
]]

local SourceSelectionHelpText = [[
This defines when the node will switch to a different input when the input blend weight value lies between the specified source weights.
]]

------------------------------------------------------------------------------------------------------------------------
-- Add's a switchDisplayInfoSection.
-- Used by Switch and SwitchMatchEvents.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.switchDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.switchDisplayInfoSection")

  local styleComboBox = nil;

  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "switchDisplayInfoSection" }
  local rollPanel = rollup:getPanel()

  -- first add the ui for the section
  attributeEditor.log("rollPanel:beginHSizer")
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    rollPanel:addHSpacer(6)
    rollPanel:setBorder(1)

    attributeEditor.log("rollPanel:beginFlexGridSizer")
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      rollPanel:setFlexGridColumnExpandable(2)

      rollPanel:addStaticText{
        text = "Evaluate",
        onMouseEnter = function()
          attributeEditor.setHelpText(evaluateHelpText)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      evaluateComboBox = attributeEditor.addBoolAttributeCombo{
        panel = rollPanel,
        objects = selection,
        attribute = "EvalEveryFrame",
        trueValue = "Every Frame",
        falseValue = "End of Animation",
        helpText = evaluateHelpText
      }
      
      rollPanel:addStaticText{
        text = "Input Selection Method",
        onMouseEnter = function()
          attributeEditor.setHelpText(SourceSelectionHelpText)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      attributeEditor.addIntAttributeCombo{
        panel = rollPanel,
        objects = selection,
        attribute = "InputSelectionMethod",
        values = { [0] = "Closest", [1] = "Floor", [2] = "Ceiling", },
        helpText = SourceSelectionHelpText,
      }

      attributeEditor.log("rollPanel:endSizer")
    rollPanel:endSizer()

    attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  ------------------------------------------------------------------------------------------------------------------------

  attributeEditor.logExitFunc("attributeEditor.switchDisplayInfoSection")
end

