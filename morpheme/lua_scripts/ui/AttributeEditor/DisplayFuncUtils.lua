------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

displayFuncUtils = {}

------------------------------------------------------------------------------------------------------------------------
displayFuncUtils.renameAttributeDetails = function(attributeDetails, renameMapping)
  -- iterate backwards so that we can delete items from the table by index
  for i = table.getn(attributeDetails), 1, -1
  do
    local k = attributeDetails[i]
    local renameTo = renameMapping[k.title]
    if renameTo == ""
    then
      table.remove(attributeDetails, i)
    elseif renameTo
    then
      k.title = renameTo
    end
  end
 
end

------------------------------------------------------------------------------------------------------------------------
local expandLimbDetails = function(nodeManifestType, details)
  local topology = getTopology("BlendNode", nodeManifestType)
  local result = { }
  
  -- loop through all of the items in the table and expand any topology strings
  -- that appear in curly brackets
  for key, value in pairs(details) do
    local inserted = false
    for limb, count in pairs(topology) do
      local subs = string.format("{%s}", limb)
      if string.find(key, subs) then
        for i = 0, count - 1 do
          local iStr = string.format("%i", i)
          local key_i = string.gsub(key, subs, iStr)
          result[key_i] = string.gsub(value, subs, iStr)
        end
        inserted = true
        break
      end
    end
    
    if not inserted then
      result[key] = value
    end
  end
  
  return result
end

------------------------------------------------------------------------------------------------------------------------
displayFuncUtils.renameAttributeGroup = function(title, attributeDetails, attributeNames)
  local renameMapping = attributeNames[title]
  if renameMapping
  then
    displayFuncUtils.renameAttributeDetails(attributeDetails, renameMapping)
  end
end

------------------------------------------------------------------------------------------------------------------------
displayFuncUtils.renameAttributeGroupWithLimbDetails = function(nodeType, title, attributeDetails, attributeNames)
  local renameMapping = attributeNames[title]
  if renameMapping
  then
    displayFuncUtils.renameAttributeDetails(attributeDetails, expandLimbDetails(nodeType, renameMapping))
  end
end

------------------------------------------------------------------------------------------------------------------------
displayFuncUtils.renameSectionTitle= function(title, sectionNames)
  local newTitle = sectionNames[title]
  if newTitle
  then
    return newTitle
  end
  return title
end

------------------------------------------------------------------------------------------------------------------------
displayFuncUtils.attachUnits = function(attributeDetails, unitMapping)
  for i,k in attributeDetails do
    local units = unitMapping[k.title]
    if units then
      k.units = units
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
displayFuncUtils.attachReset = function(attributeDetails, resetMapping)
  for i,k in attributeDetails do
    local reset = resetMapping[k.title]
    if reset then
      k.reset = reset
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
displayFuncUtils.makeSectionWithCheckbox = function(attribute, subTitle)
  return function(title, headerPanel, selection)
   headerPanel:setBorder(0)
   headerPanel:beginHSizer()
    
      -- checkbox
      headerPanel:beginVSizer()
        headerPanel:addVSpacer(4)
        local enableStand = headerPanel:addCheckBox{ }
        attributeEditor.bindAttributeHelpToWidget(enableStand, selection, attribute)
        bindWidgetToAttribute(enableStand, selection, attribute)
      headerPanel:endSizer()

      -- spacer
      headerPanel:addHSpacer(4)

      -- label
      headerPanel:beginVSizer()
        headerPanel:addVSpacer(1)
        headerPanel:addStaticText{ text = title, font = "largebold" }
      headerPanel:endSizer()
     
      -- label2
      if subTitle then
        headerPanel:addHSpacer(2)
        headerPanel:beginVSizer()
          headerPanel:addVSpacer(3)
          local subTitleWidget = headerPanel:addStaticText{ text = subTitle, font = "bold" }
          subTitleWidget:enable(false)
        headerPanel:endSizer()
      end
     
    headerPanel:endSizer()
  end
end

------------------------------------------------------------------------------------------------------------------------
displayFuncUtils.makeSubSectionWithCheckbox = function(attribute, subTitle)
  return function(title, headerPanel, selection)
   headerPanel:setBorder(0)
   headerPanel:beginHSizer()
    
      -- checkbox
      headerPanel:beginVSizer()
        headerPanel:addVSpacer(4)
        local enableStand = headerPanel:addCheckBox{ }
        attributeEditor.bindAttributeHelpToWidget(enableStand, selection, attribute)
        bindWidgetToAttribute(enableStand, selection, attribute)
      headerPanel:endSizer()

      -- spacer
      headerPanel:addHSpacer(4)

      -- label
      headerPanel:beginVSizer()
        headerPanel:addVSpacer(3)
        headerPanel:addStaticText{ text = title, font = "bold" }
      headerPanel:endSizer()
     
      -- label2
      if subTitle then
        headerPanel:addHSpacer(2)
        headerPanel:beginVSizer()
          headerPanel:addVSpacer(3)
          local subTitleWidget = headerPanel:addStaticText{ text = subTitle, font = "bold" }
          subTitleWidget:enable(false)
        headerPanel:endSizer()
      end
     
    headerPanel:endSizer()
  end
end

------------------------------------------------------------------------------------------------------------------------
displayFuncUtils.makeSection = function(subTitle)
  return function(title, headerPanel, selection)
   headerPanel:setBorder(0)
   headerPanel:beginHSizer()
    
      -- label
      headerPanel:beginVSizer()
        headerPanel:addVSpacer(1)
        headerPanel:addStaticText{ text = title, font = "largebold" }
      headerPanel:endSizer()
     
      -- label2
      if subTitle then
        headerPanel:addHSpacer(2)
        headerPanel:beginVSizer()
          headerPanel:addVSpacer(3)
          local subTitleWidget = headerPanel:addStaticText{ text = subTitle, font = "bold" }
          subTitleWidget:enable(false)
        headerPanel:endSizer()
      end
     
    headerPanel:endSizer()
  end
end

------------------------------------------------------------------------------------------------------------------------
-- return the list of attributes that are defined in the attribute details
displayFuncUtils.listAttributes = function(attributeDetails, requestDetails)
  local result = { }
  
  -- list the "normal" attributes
  if attributeDetails then
    for _1, details in ipairs(attributeDetails) do
      if type(details.attributes) == "table" then
        for _2, attribute in ipairs(details.attributes) do
          table.insert(result, attribute)
        end
      else
       table.insert(result, details.attributes)
      end
    end
  end

  -- list the requests
  if requestDetails then
    local numRequests = table.getn(requestDetails)
    for j = 0, numRequests-1 do
      local requestAttr = string.format("EmittedRequest%i", j)
      local actionAttr = string.format("Action%i", j)
      local targetAttr = string.format("Target%i", j)
      table.insert(result, requestAttr)
      table.insert(result, actionAttr)
      table.insert(result, targetAttr)
    end
  end
  
  return result
end

------------------------------------------------------------------------------------------------------------------------
-- return the list of attributes that are defined in the attribute details
-- as being per anim set, and a list that are not. Any details that are marked as "noEditor" are ommitted.
displayFuncUtils.splitPerAnimSetDetails = function(attributeDetails)
  local perAnimSet = { }
  local notPerAnimSet = { }
  if attributeDetails then
    for _1, details in ipairs(attributeDetails) do
      if details.noEditor == nil or details.noEditor == false then
        if details.perAnimSet then
          table.insert(perAnimSet, details)
        else
          table.insert(notPerAnimSet, details)
        end
      end
    end
  end

  return perAnimSet, notPerAnimSet
end


------------------------------------------------------------------------------------------------------------------------
-- This adds a "simple" set of attributes from the attribute details to 'panel' for a given animation set
displayFuncUtils.simpleAttributeDisplayFunction = function(panel, selection, attributeDetails, set)
  local attributesWithReset = { }
  panel:beginFlexGridSizer{ cols = 4, flags = "expand", proportion = 1 }
    panel:setFlexGridColumnExpandable(2)

    for _i, attr in pairs(attributeDetails) do
      local label = utils.demacroizeLimbNames(utils.getDisplayString(attr.title))
      if attr.type == "vector3" then
        attributeEditor.addAttributeLabel(panel, label, selection, attr.attributes[1])
        attributeEditor.addVectorAttributeWidget(panel, attr.attributes, selection, set)
      else
        attributeEditor.addAttributeLabel(panel, label, selection, attr.attributes)
        attributeEditor.addAttributeWidget(panel, attr.attributes, selection, set)
      end
      
      -- add the units
      if attr.units then
        panel:addStaticText{ text = attr.units }
      else
        panel:addHSpacer(0)
      end
      
      -- add reset button
      if attr.reset then
        attributeEditor.addAttributeResetButton(panel, "Reset", attr.attributes, selection, set)
        if type(attr.attributes) == "table" then
          for _, attr in ipairs(attr.attributes) do
            table.insert(attributesWithReset, attr)
          end
        else
          table.insert(attributesWithReset, attr.attributes)
        end
      else
        panel:addHSpacer(0)
      end
    end

    -- if we added more than one "Reset" button then add a "Reset all" button
    if table.getn(attributesWithReset) > 1 then
      panel:addHSpacer(0)
      panel:addHSpacer(0)
      panel:addHSpacer(0)
      attributeEditor.addAttributeResetButton(panel, "Reset all", attributesWithReset, selection, set)
   end
  
  panel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
displayFuncUtils.notPerAnimSetSection = function(panel, selection, attributeDetails, attributeDisplayFunction)
  panel:beginHSizer{ flags = "expand", proportion = 0 }
    if attributeDisplayFunction then
      attributeDisplayFunction(panel, selection, attributeDetails)
    else
      displayFuncUtils.simpleAttributeDisplayFunction(panel, selection, attributeDetails)
    end
  panel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
displayFuncUtils.perAnimSetSection = function(panel, selection, attributeDetails, attributeDisplayFunction)
  local displayFunc = function(panel, selection, attributes, set)
    panel:setBorder(1)
    panel:beginHSizer{ flags = "expand", proportion = 0 }
      if attributeDisplayFunction then
        attributeDisplayFunction(panel, selection, attributeDetails, set)
      else
        displayFuncUtils.simpleAttributeDisplayFunction(panel, selection, attributeDetails, set)
      end
    panel:endSizer()
  end

  attributeEditor.addAnimationSetWidget(panel, displayFuncUtils.listAttributes(attributeDetails), selection, displayFunc)
end

------------------------------------------------------------------------------------------------------------------------
displayFuncUtils.substituteLimbIndex = function(details, i)
  local result = { }
  
  for _, detail in ipairs(details) do
    local newAttribute
    if type(detail.attributes) == "table" then
      newAttributes = { }
      for _, attr in ipairs(detail.attributes) do
        table.insert(newAttributes, string.format(attr, i))
      end
    else
      newAttributes = string.format(detail.attributes, i)
    end
    local newDetail = { title = detail.title, type = detail.type, attributes = newAttributes }
    table.insert(result, newDetail)
  end
  
  return result
end

------------------------------------------------------------------------------------------------------------------------
-- build a mapping from attributes to title
displayFuncUtils.buildTitleTableFromDetails= function(details)
  local result = { }
  
  for _, detail in ipairs(details) do
    local title = utils.getDisplayString(detail.title)
    if type(detail.attributes) == "table" then
      for _, attr in ipairs(detail.attributes) do
        result[attr] = title
      end
    else
      result[detail.attributes] = title
    end
  end
  
  return result
end

------------------------------------------------------------------------------------------------------------------------
-- Copy all the details with a given set of attributes
--   details - attrubute details table
--   attributes - list (table) of attributes that we should select from the details table in the order that they will appear
--   i - (optional) index

displayFuncUtils.copyDetailsWithAttributes = function(details, attributes, i)
  local attributeTable = { }
  
  -- build a mapping of attributes to details
  for _, detail in ipairs(details) do
    local title = utils.getDisplayString(detail.title)
    if type(detail.attributes) == "table" then
      for _, attr in ipairs(detail.attributes) do
        attributeTable[attr] = detail
      end
    else
      attributeTable[detail.attributes] = detail
    end
  end

  -- go through the list of attributes that we are interested in and get the details
  -- remove any attributes from attributeTable once we have added them
  local result = { }
  for _, attr in ipairs(attributes) do
    if i~= nil then
      attr = string.format(attr, i)
    end
    local detail = attributeTable[attr]
    if detail then
      table.insert(result, detail)
      
      if type(detail.attributes) == "table" then
        for _, attr in ipairs(detail.attributes) do
          attributeTable[attr] = null
        end
      else
        attributeTable[detail.attributes] = null
      end
    end
  end

  return result
end

