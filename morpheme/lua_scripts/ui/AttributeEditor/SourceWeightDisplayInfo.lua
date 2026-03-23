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
-- Add's a display info section containing blend weights.
-- Use by BlendN.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.sourceWeightDisplayInfoSection = function(rollContainer, displayInfo, selection)

  local sourceWeightsWidget = nil
  local sourceWeightDistributionCombo = nil

  ----------------------------------------------------------------------------------------------------------------------
  -- Returns a table of the set of labels for the common set of pins, or nil if no common set
  ----------------------------------------------------------------------------------------------------------------------
  local calcCommonWeightLabels = function()
  
    local isFirstObjectInSelection = true
    local commonWeightCount
    local commonWrapping
    
    for i, object in ipairs(selection) do
    
      local wrapping = getAttribute(object,"WrapWeights")
      local weightCount = 0
      for pinIndex = 0, 10 do
        local pinName = string.format("%s.Source%d", object, pinIndex)
        if isConnected{ SourcePin = pinName, ResolveReferences = false } then
          weightCount = weightCount + 1
        else
          break
        end
      end

      if isFirstObjectInSelection then 
        commonWeightCount = weightCount
        commonWrapping = wrapping
        isFirstObjectInSelection = false
      end
      
      if commonWeightCount == weightCount and commonWrapping == wrapping then
        -- This node is in common with all other nodes in the selection (ie same number of elements)
      else
        -- not enough in common so we can't display the weights
        return nil
      end

    end

    local weightLabels = { }
    for i = 1, commonWeightCount do
      table.insert(weightLabels, string.format("Source %d", i - 1))
    end

    if commonWrapping then    
      table.insert(weightLabels, "Wrap to 1st")
    end

    return weightLabels
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- Build the user interface and set the parameters correctly
  ----------------------------------------------------------------------------------------------------------------------
  local buildUI = function()

    local distributionType = getCommonAttributeValue(selection, "SourceWeightDistribution")
    sourceWeightDistributionCombo:setIsIndeterminate(true)
    local enableStates = {}
    if distributionType ~= nil then
      -- We have a common distribution type
      sourceWeightDistributionCombo:setIsIndeterminate(false)

      if distributionType == 0 then
        sourceWeightDistributionCombo:setSelectedItem("Custom")
      elseif distributionType == 1 then
        sourceWeightDistributionCombo:setSelectedItem("Linear")
      elseif distributionType == 2 then
        sourceWeightDistributionCombo:setSelectedItem("Integer")
      else
        assert(false)
      end
      
      -- Check to see if there are a common number of connections for the selection
      local labels = calcCommonWeightLabels()
      if labels == nil then
        -- no common values so don't bother showing
        sourceWeightsWidget:setShown(false)
        
      else
        -- The selection has enough in common with each other
        local elementCount = table.getn(labels)
    
        if distributionType == 0 then
          -- Custom: all elements enabled
          for i = 1, elementCount do
            enableStates[i] = true
          end
        else
          -- Most elements disabled gfor Linear and Integer
          for i = 1, elementCount do
            enableStates[i] = false
          end
          -- The first element will be enabled for linear and integer
          enableStates[1] = true
          
          if (distributionType == 1) then
            -- For linear distributions the las element is enabled
            enableStates[elementCount] = true
          end
        end
      
        -- Update the UI for the weights
        sourceWeightsWidget:enableItems(enableStates)
        sourceWeightsWidget:setLabels(labels)
        sourceWeightsWidget:setMaxDisplayedItemCount(elementCount)
        sourceWeightsWidget:setShown(true)
      
      end
      
    end

  end
  
  ----------------------------------------------------------------------------------------------------------------------
  -- Switches off updating while building the user interface
  ----------------------------------------------------------------------------------------------------------------------
  local refreshUI = function(object, attr)
    attributeEditor.suspendUpdates()
    buildUI()
    attributeEditor.resumeUpdates()
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- Core Display Info Entry point
  ----------------------------------------------------------------------------------------------------------------------
  -- Basic initialisation
  local attrWeights = { }
  local attrsWrap = { }
  local attrsDistr = { }
  
  -- Generate a list of paths to all attributes we'll be displaying
  for i, object in ipairs(selection) do
    local current = string.format("%s.SourceWeights", object)
    table.insert(attrWeights, current)
    attributeEditor.log("adding \"%s\" to attribute list", current)

    current = string.format("%s.SourceWeightDistribution", object)
    table.insert(attrsDistr, current)
    attributeEditor.log("adding \"%s\" to attribute list", current)
    
    current = string.format("%s.WrapWeights", object)
    table.insert(attrsWrap, current)
    attributeEditor.log("adding \"%s\" to attribute list", current)
  end

  attributeEditor.logEnterFunc("attributeEditor.sourceWeightDisplayInfoSection")

  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "sourceWeightDisplayInfoSection" }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginVSizer")
  rollPanel:beginVSizer{ flags = "expand" }
    -- Add the wrap weights checkbox
    attributeEditor.log("rollPanel:beginFlexGridSizer")
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 0 }
      rollPanel:setFlexGridColumnExpandable(2)
      attributeEditor.addAttributeLabel(rollPanel, getAttributeDisplayName(selection[1], "WrapWeights") , selection, "WrapWeights")
      attributeEditor.addAttributeWidget(rollPanel, attrsWrap, selection)
    rollPanel:endSizer()
    attributeEditor.log("rollPanel:endFlexGridSizer")

    -- Add the weight distribution combo
    attributeEditor.log("rollPanel:beginFlexGridSizer")
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 0 }
      rollPanel:setFlexGridColumnExpandable(2)
      attributeEditor.addAttributeLabel(rollPanel, getAttributeDisplayName(selection[1], "SourceWeightDistribution") , selection, "SourceWeightDistribution")
      sourceWeightDistributionCombo = attributeEditor.addIntAttributeCombo
      {
        panel = rollPanel,
        objects = selection,
        flags = "expand",
        proportion = 1,
        attribute = "SourceWeightDistribution",
        values = { [0] = "Custom", [1] = "Linear", [2] = "Integer" },
        --order = { "Custom", "Linear", "Integer" },
      }
    rollPanel:endSizer()
    attributeEditor.log("rollPanel:endFlexGridSizer")

    -- Add the weight widget
    attributeEditor.log("rollPanel:addAttributeWidget")
    sourceWeightsWidget = rollPanel:addAttributeWidget{
      attributes = attrWeights,
      flags = "expand",
      proportion = 1,
      displayCount = 0,
      showAddRemove = false,
      showAttributeLabels = true
    }

  rollPanel:endSizer()
  attributeEditor.log("rollPanel:endVSizer")


  local changeContext = attributeEditor.createChangeContext()
  changeContext:setObjects(selection)
  changeContext:addAttributeChangeEvent("SourceWeights")
  changeContext:addAttributeChangeEvent("SourceWeightDistribution")
  changeContext:addAttributeChangeEvent("WrapWeights")
  changeContext:setAttributeChangedHandler(refreshUI)

  -- Ensure any condition changes update editor
  attributeEditor.onFlowEdgeCreateDestroy = refreshUI
  
  -- Don't call refreshUI here because you can't suspend and resume updates at this point
  -- because the panel builder hasn't finished it's work
  buildUI()

  attributeEditor.logExitFunc("attributeEditor.sourceWeightDisplayInfoSection")
end


------------------------------------------------------------------------------------------------------------------------
-- Add's a display info section containing blend weights.
-- Use by Blend", Blend2MatchEvents.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.blend2WeightDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.blend2WeightDisplayInfoSection")

  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "sourceWeights" }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginVSizer")
  rollPanel:beginVSizer{ flags = "expand" }

    local attributes = { }
    for i, object in ipairs(selection) do
      for j, attribute in ipairs(displayInfo.usedAttributes) do
        local attributePath = string.format("%s.%s", object, attribute)
        table.insert(attributes, attributePath)
        attributeEditor.log("adding \"%s\" to attribute list", attributePath)
      end
    end

    attributeEditor.log("rollPanel:addAttributeWidget")
    rollPanel:addAttributeWidget{
      attributes = attributes,
      flags = "expand",
      proportion = 1,
      labels = { "Source 0", "Source 1", },
      displayCount = 2,
      showAddRemove = false,
      showAttributeLabels = false,
    }

  attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.blend2WeightDisplayInfoSection")
end

