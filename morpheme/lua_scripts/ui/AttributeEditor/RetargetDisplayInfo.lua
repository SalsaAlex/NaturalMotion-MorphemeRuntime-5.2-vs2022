------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"

-- Temporary code that displays the "PerAnimSetSection" for the animset widget.
local perAnimSetRetargetDisplayInfoSection = function(panel, selection, attributes, set)
  attributeEditor.logEnterFunc("perAnimSetRetargetDisplayInfoSection")

  -- first add the ui for the section
  attributeEditor.log("panel:beginHSizer")
  panel:beginHSizer{ flags = "expand", proportion = 1 }
    panel:addHSpacer(6)
    panel:setBorder(1)

    panel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 0 }
      panel:setFlexGridColumnExpandable(2)
      attributeEditor.addAttributeLabel(panel, "Source", selection, "InputAnimSet")
      attributeEditor.addAttributeWidget(panel, "InputAnimSet", selection, set)
    panel:endSizer()
    
    attributeEditor.log("panel:endSizer")
    
  panel:endSizer()
  attributeEditor.log("panel:endSizer")

  attributeEditor.logExitFunc("perAnimSetRetargetDisplayInfoSection")
end

------------------------------------------------------------------------------------------------------------------------
local kOff = "- Off -"
attributeEditor.retargetDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.retargetDisplayInfoSection")

  ----------------------------------------------------------------------------------------------------------------------
  local addAnimationSetLabel = function(panel, label, selection, attribute)
    local helpText = getAttributeHelpText(selection[1], attribute)

    local staticText = panel:addStaticText{
      text = label,
      onMouseEnter = function()
        attributeEditor.setHelpText(helpText)
      end,
      onMouseLeave = function()
        attributeEditor.clearHelpText()
      end
    }
  end

  ----------------------------------------------------------------------------------------------------------------------
  local addAnimationSetCombo = function(parent, panel, set)
  
    -- return a function that will set the value correctly
    local mkSetComboValueFunction = function(newValue)
      return function(selection)
        if newValue == nil then
          setCommonAttributeValue(selection, "InputAnimSet", "", set)
        else
          setCommonAttributeValue(selection, "InputAnimSet", newValue, set)
        end
      end   
    end
  
    -- sync the combo with values that it displays
    local syncValueWithUI = function(combo, selection)
      local sourceValue = getCommonAttributeValue(selection, "InputAnimSet", set)
      if sourceValue == nil then
        combo:setIsIndeterminate(true)
        return
      end
      if sourceValue == "" then
        sourceValue = kOff
      else
        local _
        _, sourceValue = splitStringAtLastOccurence(sourceValue, "|")
      end
      combo:setSelectedItem(sourceValue)
      combo:setIsIndeterminate(false)
    end
      
        
    -- make the two lists values, and order that define the combo items, with attached functions
    -- and the associated function
    local values = { }
    local order = { }

    -- Off value
    values[kOff] = mkSetComboValueFunction("")
    table.insert(order, kOff)

    -- add all sets in order
    local animationSets = listAnimSets()
    for _, theSet in animationSets do
      if theSet ~= set then
        values[theSet] = mkSetComboValueFunction("AnimationSets|" .. theSet)
      end
      table.insert(order, theSet)
    end
    
    -- create the combo
    local combo = attributeEditor.addCustomComboBox{
      panel = panel,
      objects = selection,
      attributes = { "InputAnimSet" },
      values = values,
      order = order,
      syncValueWithUI = syncValueWithUI
    }
    attributeEditor.bindAttributeHelpToWidget(combo, selection, "InputAnimSet")
  end

  local addAnimSetInfo
  addAnimSetInfo = function(parent, panel, indent)
    local children = listAnimSetChildren(parent)
    for _, set in children do
    
      panel:beginHSizer{ flags = "expand" }
        panel:addHSpacer(indent)
        addAnimationSetLabel(panel, set, selection, "InputAnimSet")
      panel:endSizer()
      
      panel:beginHSizer{ flags = "expand" }
        panel:addHSpacer(indent)
        addAnimationSetCombo(parent, panel, set)
      panel:endSizer()
      
      addAnimSetInfo(set, panel, indent + 10)
    end
  end
  
  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{
    label = displayInfo.title,
    flags = "mainSection",
    name = "InputAnimSetIndex"
  }
  local rollPanel = rollup:getPanel()

  rollPanel:beginVSizer{ flags = "expand" }

-- Uncomment this to be able to see exactly what happens when the attributes change
--[[
    local attrs = checkSetOrderAndBuildAttributeList(selection, {"InputAnimSet"})
    rollPanel:addAnimationSetWidget{
      attributes = attrs,
      displayFunc = perAnimSetRetargetDisplayInfoSection,
      flags = "expand",
      proportion = 1,
    }
]]--
    rollPanel:addHSpacer(12)
    rollPanel:setBorder(1)

    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 0 }
      rollPanel:setFlexGridColumnExpandable(2)
      local label = rollPanel:addStaticText{ text = "Animation Set" }
      label:setFont("bold")
      local label = rollPanel:addStaticText{ text = "Retarget from" }
      label:setFont("bold")

      addAnimSetInfo(nil, rollPanel, 0)
    rollPanel:endSizer()

  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.retargetDisplayInfoSection")
end