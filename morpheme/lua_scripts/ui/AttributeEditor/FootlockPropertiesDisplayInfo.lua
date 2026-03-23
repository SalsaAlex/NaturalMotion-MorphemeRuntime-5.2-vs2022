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
-- Add's a footlockPropertiesDisplayInfoSection.
-- Used by LockFoot.
------------------------------------------------------------------------------------------------------------------------
local perAnimSetFootlockPropertiesDisplayInfoSection = function(panel, selection, attributes, set)

  attributeEditor.logEnterFunc("perAnimSetFootlockPropertiesDisplayInfoSection")

  local dampingWidget = nil
  local maxExtensionWidget = nil
  local toleranceWidget = nil

  -- first add the ui for the section
  attributeEditor.log("panel:beginHSizer")
  panel:beginHSizer{ flags = "expand", proportion = 1 }
    panel:addHSpacer(6)
    panel:setBorder(1)

    attributeEditor.log("panel:beginFlexGridSizer")
    panel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      panel:setFlexGridColumnExpandable(2)

      -- Damping
      panel:addStaticText{
        text = "Damping",
        onMouseEnter = function()
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "CatchUpSpeedFactor"))
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      dampingWidget = panel:addFloatSlider{
        flags = "expand",
        proportion = 1,
        onMouseEnter = function(self)
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "CatchUpSpeedFactor"))
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

     -- Tolerance
      panel:addStaticText{
        text = "Tolerance",
        onMouseEnter = function()
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "SnapToSourceDistance"))
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      toleranceWidget = panel:addFloatSlider{
        flags = "expand",
        proportion = 1,
        onMouseEnter = function(self)
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "SnapToSourceDistance"))
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

     -- Max Extension
      panel:addStaticText{
        text = "Max Extension",
        onMouseEnter = function()
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "StraightestLegFactor"))
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      maxExtensionWidget = panel:addIntSlider{
        flags = "expand",
        proportion = 1,
        onMouseEnter = function(self)
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "StraightestLegFactor"))
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      attributeEditor.log("panel:endSizer")
    panel:endSizer()

    attributeEditor.log("panel:endSizer")
  panel:endSizer()

   -- check if the selection contains any referenced objects
  local hasReference = containsReference(selection)

  if hasReference then
    dampingWidget:enable(false)
    toleranceWidget:enable(false)
    maxExtensionWidget:enable(false)
  end

  -- these prevent callbacks firing off other callbacks causing infinite loops
  local enableContextEvents = true
  local enableUISetAttribute = true

  ----------------------------------------------------------------------------------------------------------------------
  -- this data change context ensures the ui reflects any changes that happen to selected
  -- up axis attributes through script or undo redo are reflected in the custom ui.
  attributeEditor.log("creating data change context")
  local changeContext = attributeEditor.createChangeContext()

  changeContext:setObjects(selection)
  changeContext:addAttributeChangeEvent("CatchUpSpeedFactor")
  changeContext:addAttributeChangeEvent("StraightestLegFactor")
  changeContext:addAttributeChangeEvent("SnapToSourceDistance")

  ----------------------------------------------------------------------------------------------------------------------
  -- this function syncs the ui with the current attribute values
  ----------------------------------------------------------------------------------------------------------------------
  local syncUIWithAttributes = function()
    attributeEditor.logEnterFunc("syncUIWithAttributes")

    local catchUpSpeedFactor = getCommonAttributeValue(selection, "CatchUpSpeedFactor", set)
    if catchUpSpeedFactor ~= nil then
      -- calculate damping value rounded to 6 deimal places
      local damping = tonumber(string.format("%.6f", 1 - catchUpSpeedFactor))
      dampingWidget:setIsIndeterminate(false)
      dampingWidget:setValue(damping)
    else
      dampingWidget:setIsIndeterminate(true)
    end

    local straightestLegFactor = getCommonAttributeValue(selection, "StraightestLegFactor", set)
    if straightestLegFactor ~= nil then
      -- calculate extension value as a percentage
      local maxExtension = math.floor(100 * straightestLegFactor + 0.5)
      maxExtensionWidget:setIsIndeterminate(false)
      maxExtensionWidget:setValue(maxExtension)
    else
      maxExtensionWidget:setIsIndeterminate(true)
    end

    local snapToSourceDistance = getCommonAttributeValue(selection, "SnapToSourceDistance", set)
    if snapToSourceDistance ~= nil then
      toleranceWidget:setIsIndeterminate(false)
      toleranceWidget:setValue(snapToSourceDistance)
    else
      toleranceWidget:setIsIndeterminate(true)
    end

    attributeEditor.logExitFunc("syncUIWithAttributes")
  end

  dampingWidget:setOnChanged(
    function(self)
      attributeEditor.logEnterFunc("dampingWidget:setOnChanged")

      if enableUISetAttribute then
        enableContextEvents = false -- prevent the change context callbacks from firing off
        local value = self:getValue();
        setCommonAttributeValue(selection, "CatchUpSpeedFactor", 1 - value, set);
        syncUIWithAttributes();
        enableContextEvents = true
      end

      attributeEditor.logExitFunc("dampingWidget:setOnChanged")
    end
   )

  toleranceWidget:setOnChanged(
    function(self)
      attributeEditor.logEnterFunc("toleranceWidget:setOnChanged")

      if enableUISetAttribute then
        enableContextEvents = false -- prevent the change context callbacks from firing off
        local value = self:getValue();
        setCommonAttributeValue(selection, "SnapToSourceDistance", value, set);
        syncUIWithAttributes();
        enableContextEvents = true
      end

      attributeEditor.logExitFunc("toleranceWidget:setOnChanged")
    end
   )

  maxExtensionWidget:setOnChanged(
    function(self)
      attributeEditor.logEnterFunc("maxExtensionWidget:setOnChanged")

      if enableUISetAttribute then
        enableContextEvents = false -- prevent the change context callbacks from firing off
        local value = self:getValue();
        setCommonAttributeValue(selection, "StraightestLegFactor", value / 100, set);
        syncUIWithAttributes();
        enableContextEvents = true
      end

      attributeEditor.logExitFunc("maxExtensionWidget:setOnChanged")
    end
   )

  ----------------------------------------------------------------------------------------------------------------------
  -- this function syncs the UI with the attribute values when they are changed
  -- via script or through undo and redo
  ----------------------------------------------------------------------------------------------------------------------
  changeContext:setAttributeChangedHandler(
    function(object, attr)
      attributeEditor.logEnterFunc("changeContext attributeChangedHandler")

      if enableContextEvents then
        enableUISetAttribute = false
        syncUIWithAttributes()
        enableUISetAttribute = true
      end

      attributeEditor.logExitFunc("changeContext attributeChangedHandler")
    end
  )

  -- set the initial state of the ui
  syncUIWithAttributes()

  ----------------------------------------------------------------------------------------------------------------------

  attributeEditor.logExitFunc("perAnimSetFootlockPropertiesDisplayInfoSection")
end

attributeEditor.footlockPropertiesDisplayInfoSection = function(panel, displayInfo, selection)
  local attrs = checkSetOrderAndBuildAttributeList(selection, displayInfo.usedAttributes)

  local rollup = panel:addRollup{
    label = displayInfo.title,
    flags = "mainSection",
    name = string.format("footlockPropertiesDisplayInfoSection")
  }
  local rollPanel = rollup:getPanel()

  rollPanel:beginVSizer{ flags = "expand" }
    rollPanel:addAnimationSetWidget{
      attributes = attrs,
      displayFunc = perAnimSetFootlockPropertiesDisplayInfoSection,
      flags = "expand",
      proportion = 1,
    }
  rollPanel:endSizer()
end

