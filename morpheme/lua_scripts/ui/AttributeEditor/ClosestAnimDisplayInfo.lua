------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"

local positionInfluenceHelpText = [[
The percentage weighting to give position over orientation for each joint when comparing the input animations to the network output.
]]

local orientationInfluenceHelpText = [[
The percentage weighting to give orientation over position for each joint when comparing the input animations to the network output.
]]

------------------------------------------------------------------------------------------------------------------------
-- Add's a display info section for the Matching method attributes of the ClosestAnimNode
-- This is only used by the ClosestAnimNode.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.closestAnimMatchingSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.closestAnimMatchingSection")

  -- first add the ui for the section
  attributeEditor.log("rollContainer:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "closestAnimMatchingSection" }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollContainer:beginVSizer")
  rollPanel:beginVSizer{ flags = "expand", proportion = 1 }

    attributeEditor.log("rollContainer:beginFlexGridSizer")
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand" }

      -- Attribute paths
      local precomputeSourcesOfflineAttrPaths = { }

      for i, object in ipairs(selection) do
        table.insert(precomputeSourcesOfflineAttrPaths, string.format("%s.PrecomputeSourcesOffline", object))
      end

      -- Add the pre-compute sources offline bool combo box
      rollPanel:addStaticText{
        text = "Source Evaluation Method",
        onMouseEnter = function()
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "PrecomputeSourcesOffline"))
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      updateFrameWidget = attributeEditor.addBoolAttributeCombo{
        panel = rollPanel,
        objects = selection,
        attribute = "PrecomputeSourcesOffline",
        trueValue = "Offline",
        falseValue ="Online",
        helpText = getAttributeHelpText(selection[1], "PrecomputeSourcesOffline")
      }

      attributeEditor.log("rollContainer:endSizer")
      rollPanel:endSizer()

    attributeEditor.log("rollContainer:endSizer")
    rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.closestAnimMatchingSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Add's a display info section for the influence attributes of the ClosestAnimNode
-- This is only used by the ClosestAnimNode.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.closestAnimInfluenceSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.closestAnimInfluenceSection")

  -- first add the ui for the section
  attributeEditor.log("rollContainer:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "closestAnimInfluenceSection" }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollContainer:beginVSizer")
  rollPanel:beginVSizer{ flags = "expand", proportion = 1 }

    attributeEditor.log("rollContainer:beginFlexGridSizer")
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand" }

      ------------------------------------
      -- Influence between position and orientation
      
      -- add the position slider
      attributeEditor.log("rollContainer:addStaticText")
      rollPanel:addStaticText{
        text = "Position (%)",
        onMouseEnter = function()
          attributeEditor.setHelpText(positionInfluenceHelpText)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      attributeEditor.log("rollContainer:addFloatSlider")
      local positionFloatSlider = rollPanel:addFloatSlider{
        name = "PositionSlider",
        min = 0, value = 50, max = 100,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function(self)
          attributeEditor.setHelpText(positionInfluenceHelpText)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      -- add the orientation slider
      attributeEditor.log("rollContainer:addStaticText")
      rollPanel:addStaticText{
        text = "Orientation (%)",
        onMouseEnter = function()
          attributeEditor.setHelpText(orientationInfluenceHelpText)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      attributeEditor.log("rollContainer:addFloatSlider")
      local orientationFloatSlider = rollPanel:addFloatSlider{
        name = "OrientationSlider",
        min = 0, value = 50, max = 100,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function(self)
          attributeEditor.setHelpText(orientationInfluenceHelpText)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      ------------------------------------
      -- Use velocity
      local useVelocityAttrPaths = { }

      for i, object in ipairs(selection) do
        table.insert(useVelocityAttrPaths, string.format("%s.UseVelocity", object))
      end

      -- add the use velocity check box
      rollPanel:addStaticText{
        text = "Use Velocity",
        onMouseEnter = function()
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "UseVelocity"))
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      attributeEditor.log("rollPanel:addAttributeWidget")
      rollPanel:addAttributeWidget{
        attributes = useVelocityAttrPaths,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function(self)
          attributeEditor.setHelpText(self:getAttributeHelp())
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

    attributeEditor.log("rollContainer:endSizer")
    rollPanel:endSizer()

  attributeEditor.log("rollContainer:endSizer")
  rollPanel:endSizer()

  if containsReference(selection) then
    orientationFloatSlider:enable(false)
    positionFloatSlider:enable(false)
  end

  -- Position and orientation influence syncing functions
  local syncInfluencePositionAndOrientationWithValue = function()
    attributeEditor.logEnterFunc("syncInfluencePositionAndOrientationWithValue")

    local value = getCommonAttributeValue(selection, "InfluenceBetweenPositionAndOrientation")
    if value then
      positionFloatSlider:setValue((1 - value) * 100)
      orientationFloatSlider:setValue(value * 100)
    else
      positionFloatSlider:setIsIndeterminate(true)
      orientationFloatSlider:setIsIndeterminate(true)
    end

    attributeEditor.logExitFunc("syncInfluencePositionAndOrientationWithValue")
  end

  positionFloatSlider:setOnChanged(
    function(self)
      attributeEditor.logEnterFunc("positionFloatSlider:onChanged")

      local value = self:getValue()
      value = value / 100
      value = 1 - value

      attributeEditor.log("setting InfluenceBetweenPositionAndOrientation to %f", value)
      setCommonAttributeValue(selection, "InfluenceBetweenPositionAndOrientation", value)

      attributeEditor.logExitFunc("positionFloatSlider:onChanged")
    end
  )

  orientationFloatSlider:setOnChanged(
    function(self)
      attributeEditor.logEnterFunc("orientationFloatSlider:onChanged")

      local value = self:getValue()
      value = value / 100

      attributeEditor.log("setting InfluenceBetweenPositionAndOrientation to %f", value)
      setCommonAttributeValue(selection, "InfluenceBetweenPositionAndOrientation", value)

      attributeEditor.logExitFunc("orientationFloatSlider:onChanged")
    end
  )

  orientationFloatSlider:setOnChanging(
    function(self)
      attributeEditor.logEnterFunc("orientationFloatSlider:onChanging")
      positionFloatSlider:setValue(100 - self:getValue())
      attributeEditor.logExitFunc("orientationFloatSlider:onChanging")
    end
  )

  positionFloatSlider:setOnChanging(
    function(self)
      attributeEditor.logEnterFunc("positionFloatSlider:onChanging")
      orientationFloatSlider:setValue(100 - self:getValue())
      attributeEditor.logExitFunc("positionFloatSlider:onChanging")
    end
  )

  -- this data change context ensures the ui reflects any changes that happen to selected
  -- blend with event nodes through script or undo redo are reflected in the custom ui.
  attributeEditor.log("creating data change context")
  local influenceContext = attributeEditor.createChangeContext()

  influenceContext:setObjects(selection)
  influenceContext:addAttributeChangeEvent("InfluenceBetweenPositionAndOrientation")

  ----------------------------------------------------------------------------------------------------------------------
  -- this function syncs the style combo box with the attribute values when they are changed
  -- via script or through undo and redo
  ----------------------------------------------------------------------------------------------------------------------
  influenceContext:setAttributeChangedHandler(
    function(object, attr)
      attributeEditor.logEnterFunc("influenceContext:attributeChangedHandler")

      syncInfluencePositionAndOrientationWithValue()

      attributeEditor.logExitFunc("influenceContext:attributeChangedHandler")
    end
  )

  -- sync the sliders with the actual value
  syncInfluencePositionAndOrientationWithValue()

  attributeEditor.logExitFunc("attributeEditor.closestAnimInfluenceSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Add's a display info section for the Root Rotation Blending attributes of the ClosestAnimNode
-- This is only used by the ClosestAnimNode.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.closestAnimRootRotationSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.closestAnimRootRotationSection")

  -- first add the ui for the section
  attributeEditor.log("rollContainer:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "closestAnimRootRotationSection" }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollContainer:beginVSizer")
  rollPanel:beginVSizer{ flags = "expand", proportion = 1 }

    attributeEditor.log("rollContainer:beginFlexGridSizer")
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand" }

      -- Attribute paths
      local useRootRotationBlendingAttrPaths = { }

      for i, object in ipairs(selection) do
        table.insert(useRootRotationBlendingAttrPaths, string.format("%s.UseRootRotationBlending", object))
      end

      -- Add the use root rotation blending bool combo box
      rollPanel:addStaticText{
        text = "Blend Type",
        onMouseEnter = function()
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "UseRootRotationBlending"))
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      updateFrameWidget = attributeEditor.addBoolAttributeCombo{
        panel = rollPanel,
        objects = selection,
        attribute = "UseRootRotationBlending",
        trueValue = "Character Root",
        falseValue ="Trajectory",
        helpText = getAttributeHelpText(selection[1], "UseRootRotationBlending")
      }

      -- add the fraction through source slider
      attributeEditor.log("rollContainer:addStaticText")
      rollPanel:addStaticText{
        text = "Blend Duration",
        onMouseEnter = function()
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "BlendDuration"))
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      attributeEditor.log("rollContainer:addFloatSlider")
      local blendDurationFloatSlider = rollPanel:addFloatSlider{
        name = "BlendDurationFloatSlider",
        min = 0, value = 0.25, max = 1,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function(self)
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "BlendDuration"))
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      if containsReference(selection) then
        blendDurationFloatSlider:enable(false)
      end

      -- add the max root rotation angle slider
      attributeEditor.log("rollContainer:addStaticText")
      rollPanel:addStaticText{
        text = "Match Tolerance",
        onMouseEnter = function()
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "MatchTolerance"))
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      attributeEditor.log("rollContainer:addFloatSlider")
      local matchToleranceFloatSlider = rollPanel:addFloatSlider{
        name = "MatchToleranceSlider",
        min = 0, value = 180, max = 180,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function(self)
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "MatchTolerance"))
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      if containsReference(selection) then
        matchToleranceFloatSlider:enable(false)
      end

      attributeEditor.log("rollContainer:endSizer")
      rollPanel:endSizer()

    attributeEditor.log("rollContainer:endSizer")
    rollPanel:endSizer()

    -- Fraction through source syncing functions
    blendDurationFloatSlider:setOnChanged(
      function(self)
        for i, object in ipairs(selection) do
          setAttribute(string.format("%s.BlendDuration", object), self:getValue())
        end
      end
    )

    local syncFractionThroughSourceWithValue = function()
      blendDurationFloatSlider:setValue(getAttribute(selection[1] .. ".BlendDuration"))
    end

    -- Max root rotation angle syncing functions
    matchToleranceFloatSlider:setOnChanged(
      function(self)
        for i, object in ipairs(selection) do
          setAttribute(string.format("%s.MatchTolerance", object), self:getValue())
        end
      end
    )

    local syncMaxRootRotationAngleWithValue = function()
      matchToleranceFloatSlider:setValue(getAttribute(selection[1] .. ".MatchTolerance"))
    end

    ----------------------------------------------------------------------------------------------------------------------
    -- enable blendDurationFloatSlider based on the value of UseRootRotationBlending
    -- enable matchToleranceFloatSlider based on the value of UseRootRotationBlending
    ----------------------------------------------------------------------------------------------------------------------
    local syncRootRotationInterfaceFromAttribute = function()
      attributeEditor.logEnterFunc("syncRootRotationInterfaceFromAttribute")

      local enableRootRotationAttributes = getCommonAttributeValue(selection, "UseRootRotationBlending")

      if enableRootRotationAttributes ~= nil then
        blendDurationFloatSlider:enable(enableRootRotationAttributes and not hasReference)
        matchToleranceFloatSlider:enable(enableRootRotationAttributes and not hasReference)
      end

      attributeEditor.logExitFunc("syncRootRotationInterfaceFromAttribute")
    end

    -- this data change context ensures the ui reflects any changes that happen to selected
    -- blend with event nodes through script or undo redo are reflected in the custom ui.
    attributeEditor.log("creating data change context")
    local RootRotationContext = attributeEditor.createChangeContext()

    RootRotationContext:setObjects(selection)
    RootRotationContext:addAttributeChangeEvent("UseRootRotationBlending")
    RootRotationContext:addAttributeChangeEvent("BlendDuration")
    RootRotationContext:addAttributeChangeEvent("MatchTolerance")

    ----------------------------------------------------------------------------------------------------------------------
    -- this function syncs the style combo box with the attribute values when they are changed
    -- via script or through undo and redo
    ----------------------------------------------------------------------------------------------------------------------
    RootRotationContext:setAttributeChangedHandler(
      function(object, attr)
        attributeEditor.logEnterFunc("RootRotationContext:attributeChangedHandler")

        syncFractionThroughSourceWithValue()
        syncMaxRootRotationAngleWithValue()
        syncRootRotationInterfaceFromAttribute()

        attributeEditor.logExitFunc("RootRotationContext:attributeChangedHandler")
      end
    )

    -- sync the sliders with the actual value
    syncFractionThroughSourceWithValue()
    syncMaxRootRotationAngleWithValue()
    syncRootRotationInterfaceFromAttribute()

  attributeEditor.logExitFunc("attributeEditor.closestAnimRootRotationSection")
end
