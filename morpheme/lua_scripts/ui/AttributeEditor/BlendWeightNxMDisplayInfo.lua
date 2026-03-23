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
-- functions used to add an automatic weights distribution section
-- Use by blendWeightNxMDisplayInfoSection.
------------------------------------------------------------------------------------------------------------------------

-- test for approximate equality
local equal = function(val1, val2)
  local tolerance = 0.0001
  return val1 > (val2 - tolerance) and val1 < (val2 + tolerance)
end

-- set a selection of objects to evenly distributed
local evenlyDistributeWeightsX = function(selection)
  for i, object in ipairs(selection) do
    local xValue = getAttribute(object .. ".XValue")
    local attrsX = getAttribute(object .. ".BlendWeightsX")
    local currWeightX = 0

    for id = 1, xValue do
      if (attrsX[id] ~= nil and not equal(attrsX[id], currWeightX)) then
        attrsX[id] = currWeightX
      end
      currWeightX = id / (xValue - 1)
    end
    setAttribute(object .. ".BlendWeightsX", attrsX)
  end
end

-- set a selection of objects to evenly distributed
local evenlyDistributeWeightsY = function(selection)
  for i, object in ipairs(selection) do
    local yValue      = getAttribute(object .. ".YValue")
    local attrsY      = getAttribute(object .. ".BlendWeightsY")
    local currWeightY = 0
    
    for id = 1, yValue do
      if (attrsY[id] ~= nil and not equal(attrsY[id], currWeightY)) then
        attrsY[id] = currWeightY
      end
      currWeightY = id / (yValue - 1)
    end
    setAttribute(object .. ".BlendWeightsY", attrsY)
  end
end

-- test if a specific object IS evenly distributed
local isEvenlyDistributed = function(object)
  local xValue      = getAttribute(object .. ".XValue")
  local yValue      = getAttribute(object .. ".YValue")
  local auto        = true
  local attrsX      = getAttribute(object .. ".BlendWeightsX")
  local currWeightX = 0

  for id = 1, xValue do
    if (attrsX[id] ~= nil and not equal(attrsX[id], currWeightX)) then
      auto = false
    end
    currWeightX = id / (xValue - 1)
  end

  if (auto == true) then
    local attrsY      = getAttribute(object .. ".BlendWeightsY")
    local currWeightY = 0
    for id = 1, yValue do
      if (attrsY[id] ~= nil and not equal(attrsY[id], currWeightY)) then
        auto = false
      end
      currWeightY = id / (yValue - 1)
    end
  end

  return auto
end

-- enable of disable the auto weights button as required
local setAutoWeights = function(selection, autoWeights)
  local shouldEnable = false
  if not containsReference(selection) then
    for i, object in ipairs(selection) do
      if (not isEvenlyDistributed(object)) then
        shouldEnable = true
      end
    end
  end
  autoWeights:enable(shouldEnable)
end

-- set evenly distributed weights in a given NxM node
-- This function is used after collapsing a network into a NxM node
attributeEditor.evenlyDistributeWeights = function(object)
  local xValue      = getAttribute(object .. ".XValue")
  local yValue      = getAttribute(object .. ".YValue")
  local attrsX      = getAttribute(object .. ".BlendWeightsX")
  local attrsY      = getAttribute(object .. ".BlendWeightsY")
  local currWeightX = 0
  local currWeightY = 0
  -- Weight X
  for id = 1, xValue do
    if (attrsX[id] ~= nil and not equal(attrsX[id], currWeightX)) then
      attrsX[id] = currWeightX
    end
    currWeightX = id / (xValue - 1)
  end
  setAttribute(object .. ".BlendWeightsX", attrsX)
  -- Weight Y
  for id = 1, yValue do
    if (attrsY[id] ~= nil and not equal(attrsY[id], currWeightY)) then
      attrsY[id] = currWeightY
    end
    currWeightY = id / (yValue - 1)
  end
  setAttribute(object .. ".BlendWeightsY", attrsY)
end

------------------------------------------------------------------------------------------------------------------------
-- Add's a display info section containing blend weights and  X & Y values.
-- Use by BlendNxM, BlendNxMMatchEvents.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.blendWeightNxMDisplayInfoSection = function(rollContainer, displayInfo, selection)

  attributeEditor.logEnterFunc("attributeEditor.blendWeightNxMDisplayInfoSection")

  attributeEditor.log("rollContainter:addRollup")
  local rollup    = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "blendWeightNxMDisplayInfoSection1" }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginVSizer")
  rollPanel:beginVSizer{ flags = "expand" }

  local showLabels = true
  if table.getn(displayInfo.usedAttributes) == 1 then
    showLabels = false
  end

  local attrsX = { }
  local attrsY = { }
  local XValue = { }
  local YValue = { }
  for i, object in ipairs(selection) do
    local current = string.format("%s.BlendWeightsX", object)
    table.insert(attrsX, current)
    attributeEditor.log("adding \"%s\" to attribute list", current)

    current = string.format("%s.BlendWeightsY", object)
    table.insert(attrsY, current)
    attributeEditor.log("adding \"%s\" to attribute list", current)

    current = string.format("%s.XValue", object)
    table.insert(XValue, current)
    attributeEditor.log("adding \"%s\" to attribute list", current)

    current = string.format("%s.YValue", object)
    table.insert(YValue, current)
    attributeEditor.log("adding \"%s\" to attribute list", current)
  end

  -- Blend Weights container -------------------------------------------------------------
  local weightXLabels =
  {
    "SourceX 0", "SourceX 1", "SourceX 2", "SourceX 3", "SourceX 4",
  }

  local weightYLabels =
  {
    "SourceY 0", "SourceY 1", "SourceY 2", "SourceY 3", "SourceY 4",
  }

  attributeEditor.addAttributeLabel(rollPanel, "Weight X", selection, "BlendWeightsX")
  attributeEditor.log("rollPanel:addAttributeWidget")
  rollPanel:addAttributeWidget{
    attributes          = attrsX,
    flags               = "expand",
    proportion          = 1,
    labels              = weightXLabels,
    displayCount        = getAttribute(XValue[1]),
    showAddRemove       = false,
    showAttributeLabels = showLabels
  }

  attributeEditor.addAttributeLabel(rollPanel, "Weight Y", selection, "BlendWeightsY")
  attributeEditor.log("rollPanel:addAttributeWidget")
  rollPanel:addAttributeWidget{
    attributes          = attrsY,
    flags               = "expand",
    proportion          = 1,
    labels              = weightYLabels,
    displayCount        = getAttribute(YValue[1]),
    showAddRemove       = false,
    showAttributeLabels = showLabels
  }

  local autoWeights = rollPanel:addButton{
    flags           = "expand",
    proportion      = 0,
    label           = "Auto Weights",
    onClick         = function()
      evenlyDistributeWeightsX(selection)
      evenlyDistributeWeightsY(selection)
    end,
    onMouseEnter    = function()
      attributeEditor.setHelpText("Space weight values evenly between 0 - 1 automatically. This will enable automatically recalulating weights when new connections are made or broken.")
    end,
    onMouseLeave    = function()
      attributeEditor.clearHelpText()
    end
  }

  setAutoWeights(selection, autoWeights)

  local changeContext = attributeEditor.createChangeContext()
  changeContext:setObjects(selection)
  changeContext:addAttributeChangeEvent("BlendWeightsX")
  changeContext:setAttributeChangedHandler(
    function(object, attr)
      if (attr == "BlendWeightsX") then
        setAutoWeights({ object }, autoWeights)
      else
        local editor = attributeEditor.editorWindow
        editor:addIdleCallback(
          function()
            -- Rebuild the weights, will also redraw the list
            rebuildBlendWeights(object)
          end
        )
      end
    end
  )

  local changeContext = attributeEditor.createChangeContext()
  changeContext:setObjects(selection)
  changeContext:addAttributeChangeEvent("BlendWeightsY")
  changeContext:setAttributeChangedHandler(
    function(object, attr)
      if (attr == "BlendWeightsY") then
        setAutoWeights({ object }, autoWeights)
      else
        local editor = attributeEditor.editorWindow
        editor:addIdleCallback(
          function()
            -- Rebuild the weights, will also redraw the list
            rebuildBlendWeights(object)
          end
        )
      end
    end
  )

  -- X & Y Values container -------------------------------------------------------------
  attributeEditor.log("rollContainter:addRollup")
  local rollup2    = rollContainer:addRollup{ label = "X & Y Values", flags = "mainSection", name = "blendWeightNxMDisplayInfoSection2" }
  local rollPanel2 = rollup2:getPanel()

  attributeEditor.log("rollPanel2:beginVSizer")
  rollPanel2:beginVSizer{ flags = "expand" }

    attributeEditor.log("rollPanel2:addAttributeWidget")
    attributeEditor.addAttributeLabel(rollPanel2, "X Value", selection, "XValue")
    rollPanel2:addAttributeWidget{
      attributes          = XValue,
      flags               = "expand",
      proportion          = 1,
      label               = "X value",
      displayCount        = 1,
      min                 = 2,
      max                 = 5,
      showAddRemove       = false,
      showAttributeLabels = true
    }

    attributeEditor.log("rollPanel2:addAttributeWidget")
    attributeEditor.addAttributeLabel(rollPanel2, "Y Value", selection, "YValue")
    rollPanel2:addAttributeWidget{
      attributes          = YValue,
      flags               = "expand",
      proportion          = 1,
      label               = "Y value",
      displayCount        = 1,
      min                 = 2,
      max                 = 5,
      showAddRemove       = false,
      showAttributeLabels = true
    }

    local changeContext = attributeEditor.createChangeContext()
    changeContext:setObjects(selection)
    changeContext:addAttributeChangeEvent("XValue")
    changeContext:setAttributeChangedHandler(
      function(object, attr)
        if (attr == "XValue") then
          local editor = attributeEditor.editorWindow
          editor:addIdleCallback(
            function()
              -- Redraw the list
              evenlyDistributeWeightsX(selection)
              safefunc(attributeEditor.onSelectionChange)
            end
          )
        else
          local editor = attributeEditor.editorWindow
          editor:addIdleCallback(
            function()
              -- Rebuild the weights, will also redraw the list
              rebuildBlendWeights(object)
            end
          )
        end
      end
    )

    local changeContext = attributeEditor.createChangeContext()
    changeContext:setObjects(selection)
    changeContext:addAttributeChangeEvent("YValue")
    changeContext:setAttributeChangedHandler(
      function(object, attr)
        if (attr == "YValue") then
          local editor = attributeEditor.editorWindow
          editor:addIdleCallback(
            function()
              -- Redraw the list
              evenlyDistributeWeightsY(selection)
              safefunc(attributeEditor.onSelectionChange)
            end
          )
        else
          local editor = attributeEditor.editorWindow
          editor:addIdleCallback(
            function()
              -- Rebuild the weights, will also redraw the list
              rebuildBlendWeights(object)
            end
          )
        end
      end
    )

  attributeEditor.log("rollPanel2:endSizer")
  rollPanel2:endSizer()

  attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.blendWeightNxMDisplayInfoSection")
end

