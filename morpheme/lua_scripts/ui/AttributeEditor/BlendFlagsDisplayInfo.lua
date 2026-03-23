------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2011 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"

------------------------------------------------------------------------------------------------------------------------
-- Add's a display info section containing blend flags.
-- Used by Blend2, BlendN, featherBlend, blendNxM.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.blendFlagsDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.blendFlagsDisplayInfoSection")

  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "blendFlagsDisplayInfoSection" }
  local rollPanel = rollup:getPanel()

      attributeEditor.log("rollPanel:beginFlexGridSizer")
      rollPanel:beginFlexGridSizer{ cols = 2, rows = 3, flags = "expand" }
        rollPanel:setFlexGridColumnExpandable(2)

        local trueText = "Always blend"
        local falseText = "Optimise"

        -- Always Blend Trajectory And Transforms ------------------------------------------------------------------
        attributeEditor.log("attributeEditor.addStaticTextWithHelp")
        local blendTrajectoryAndTransformsHelpText = getAttributeHelpText(selection[1], "AlwaysBlendTrajectoryAndTransforms")
        local blendTrajectoryAndTransformsText = attributeEditor.addStaticTextWithHelp(rollPanel, "Trajectory and transforms", blendTrajectoryAndTransformsHelpText)

        local blendTrajectoryAndTransformsWidget = attributeEditor.addBoolAttributeCombo{
          panel = rollPanel,
          objects = selection,
          attribute = "AlwaysBlendTrajectoryAndTransforms",
          helpText = blendTrajectoryAndTransformsHelpText,
          trueValue = trueText,
          falseValue = falseText,
        }

        -- Always Blend Events --------------------------------------------------------------------------
        attributeEditor.log("attributeEditor.addStaticTextWithHelp")
        local blendEventsHelp = getAttributeHelpText(selection[1], "AlwaysBlendEvents")
        local blendEventsText = attributeEditor.addStaticTextWithHelp(rollPanel, "Events", blendEventsHelp)

        local blendEventsWidget = attributeEditor.addBoolAttributeCombo{
          panel = rollPanel,
          objects = selection,
          attribute = "AlwaysBlendEvents",
          helpText = blendEventsHelp,
          trueValue = trueText,
          falseValue = falseText,
        }

      attributeEditor.log("rollPanel:endFlexGridSizer")
      rollPanel:endSizer()


  attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  -- syncs the enable disable state of the blend event flag.
  local syncBlendUI = function()
    local enableTrajectoryAndTransformFlag = true
    local enableEventFlag = true

    if attributeExists(selection[1], "PassThroughMode") then
      local passThroughMode = getCommonAttributeValue(selection, "PassThroughMode")
      if  passThroughMode ~= 2 then
        enableTrajectoryAndTransformFlag = false
        enableEventFlag = false
      end
    end

    if attributeExists(selection[1], "TimeStretchMode") then
      local timeStretchMode = getCommonAttributeValue(selection, "TimeStretchMode")
      if not timeStretchMode or timeStretchMode == 0 then
        enableEventFlag = false
      end
    end

    attributeEditor.logEnterFunc("enabledContext attributeChangedHandler")

    --
    blendTrajectoryAndTransformsText:enable(enableTrajectoryAndTransformFlag)
    blendTrajectoryAndTransformsWidget:enable(enableTrajectoryAndTransformFlag and not hasReference)
    --
    blendEventsText:enable(enableEventFlag)
    blendEventsWidget:enable(enableEventFlag and not hasReference)

    attributeEditor.logExitFunc("enabledContext")

  end

  -- this data change context ensures the ui reflects any changes that happen to selected
  -- blend with event nodes through script or undo redo are reflected in the custom ui.
  attributeEditor.log("creating data change context")
  local enabledContext = attributeEditor.createChangeContext()

  enabledContext:setObjects(selection)
  if attributeExists(selection[1], "TimeStretchMode") then
    enabledContext:addAttributeChangeEvent("TimeStretchMode")
  end
  if attributeExists(selection[1], "PassThroughMode") then
    enabledContext:addAttributeChangeEvent("PassThroughMode")
  end
  if attributeExists(selection[1], "PositionBlendMode") then
    enabledContext:addAttributeChangeEvent("PositionBlendMode")
  end
  if attributeExists(selection[1], "RotationBlendMode") then
    enabledContext:addAttributeChangeEvent("RotationBlendMode")
  end
  enabledContext:setAttributeChangedHandler(
    function(object, attr)
      syncBlendUI()
    end
  )

  syncBlendUI()

  attributeEditor.logExitFunc("attributeEditor.blendFlagsDisplayInfoSection")
end
