------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"

local perAnimSetHeadPropertiesDisplayInfoSection = function(panel, selection, attributes, set)
  attributeEditor.logEnterFunc("perAnimSetHeadPropertiesDisplayInfoSection")
    panel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      panel:setFlexGridColumnExpandable(2)

      attributeEditor.addAttributeLabel(panel, "Bias", selection, "Bias")
      local biasWidget = panel:addFloatSlider{ flags = "expand", proportion = 1, min = -1, max = 1 }

      attributeEditor.bindHelpToWidget(biasWidget, getAttributeHelpText(selection[1], "Bias"))
      bindWidgetToAttribute(biasWidget, selection, "Bias", set)

  panel:endSizer()
  attributeEditor.logExitFunc("perAnimSetHeadPropertiesDisplayInfoSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Adds a headlookPropertiesDisplayInfoSection.
-- Used by HeadLook IK.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.headlookPropertiesDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.headlookPropertiesDisplayInfoSection")

  local targetSpaceHelpText = [[The coordinate frame of the target control parameter, which can either be World Space or Character Space.]]

  -- add the ui for the section
  attributeEditor.log("rollContainer:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "headlookPropertiesDisplayInfoSection" }
  local rollPanel = rollup:getPanel()

  -- first add the ui for the section
  attributeEditor.log("rollPanel:beginHSizer")
  rollPanel:beginVSizer{ flags = "expand", proportion = 1 }
    rollPanel:beginHSizer{ flags = "expand", proportion = 0 }
      rollPanel:addHSpacer(6)
      rollPanel:setBorder(1)

      attributeEditor.log("rollPanel:beginFlexGridSizer")
      rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
        rollPanel:setFlexGridColumnExpandable(2)

        -- Target Space
        attributeEditor.addAttributeLabel(rollPanel, "Target Frame", selection, "WorldSpaceTarget")
        attributeEditor.addBoolAttributeCombo{
          panel = rollPanel,
          objects = selection,
          attribute = "WorldSpaceTarget",
          trueValue = "World Space",
          falseValue = "Character Space",
          helpText = targetSpaceHelpText
        }

        -- Keep Upright
        attributeEditor.addAttributeLabel(rollPanel, "Keep Upright", selection, "KeepUpright")
        attributeEditor.addAttributeWidget(rollPanel, "KeepUpright", selection)

        -- Apply Joint Limits
        attributeEditor.addAttributeLabel(rollPanel, "Apply Joint Limits", selection, "ApplyJointLimits")
        attributeEditor.addAttributeWidget(rollPanel, "ApplyJointLimits", selection)

        -- Update Frame
        attributeEditor.addAttributeLabel(rollPanel, "Update Frame", selection, "UpdateTargetByDeltas")
        attributeEditor.addBoolAttributeCombo{
          panel = rollPanel,
          objects = selection,
          attribute = "UpdateTargetByDeltas",
          trueValue = "Previous",
          falseValue ="Current",
          helpText = getAttributeHelpText(selection[1], "UpdateTargetByDeltas")
        }

        attributeEditor.log("rollPanel:endSizer")
      rollPanel:endSizer()

      attributeEditor.log("rollPanel:endSizer")
    rollPanel:endSizer()

    rollPanel:beginHSizer{ flags = "expand", proportion = 0 }
      rollPanel:addHSpacer(6)
      rollPanel:setBorder(1)
      local attrs = checkSetOrderAndBuildAttributeList(selection, { "Bias" })
        rollPanel:addAnimationSetWidget{
        attributes = attrs,
        displayFunc = perAnimSetHeadPropertiesDisplayInfoSection,
        flags = "expand",
        proportion = 1,
      }
   rollPanel:endSizer()

 rollPanel:endSizer()

  ----------------------------------------------------------------------------------------------------------------------

  attributeEditor.logExitFunc("attributeEditor.headlookPropertiesDisplayInfoSection")
end

