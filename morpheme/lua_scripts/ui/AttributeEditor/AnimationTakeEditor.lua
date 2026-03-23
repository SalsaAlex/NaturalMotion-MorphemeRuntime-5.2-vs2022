------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "luaAPI/MorphemeUnitAPI.lua"

------------------------------------------------------------------------------------------------------------------------
-- returns an attribute value
local stdGetter = function(resourceId, attributeName)
  if resourceId == 0 or anim.getResourceType(resourceId) ~= "take" then
    return nil
  end

  local path = anim.getResourcePath(resourceId)
  local attributeValue = anim.getAttribute(path .. "." .. attributeName)
  return attributeValue
end

------------------------------------------------------------------------------------------------------------------------
-- sets an attribute value
local stdSetter = function(resourceId, attributeName, value)
  if resourceId == 0 or anim.getResourceType(resourceId) ~= "take" then
    return
  end

  local path = anim.getResourcePath(resourceId)
  local attributeValue = anim.setAttribute(path .. "." .. attributeName, value)
end

------------------------------------------------------------------------------------------------------------------------
-- Just returns if the clip has changed from its default value
local clipChangedGetter = function(resourceId, attributeName)
  if resourceId == 0 or anim.getResourceType(resourceId) ~= "take" then
    return false
  end

  local path = anim.getResourcePath(resourceId)
  local attributeValue = anim.getAttribute(path .. "." .. attributeName)
  if attributeName == "ClipStartFraction" then
    return attributeValue ~= 0
  else
    return attributeValue ~= 1
  end
end

------------------------------------------------------------------------------------------------------------------------
local hasTake = function(selectedResources)
  for resourceId in selectedResources do
    if anim.getResourceType(resourceId) == "take" then
      return true
    end
  end
  return false
end

------------------------------------------------------------------------------------------------------------------------
local getChosenAttributeValue = function(selectedResources, attributeName, attributeGetter)
  local result = nil
  local mixed = false
  local totalResources = 0

  -- establish a getter if none is supplied
  if attributeGetter == nil then
    attributeGetter = stdGetter
  end

  -- first look through the selected resources
  for resourceId in selectedResources do
      totalResources = totalResources + 1
      local attributeValue = attributeGetter(resourceId, attributeName)
      if result == nil then
        result = attributeValue
      elseif result ~= attributeValue then
        mixed = true
      end
  end

  -- if the selected resources are empty then get it from the selected take
  if totalResources == 0 then
    local _, resourceId = anim.getSelectedTakeInAssetManager()
    if resourceId ~= 0 then
      result = attributeGetter(resourceId, attributeName)
      totalResources = 1
    end
  end

  return result, mixed, totalResources
end

------------------------------------------------------------------------------------------------------------------------
local setChosenAttributeValue = function(selectedResources, attributeName, value, attributeSetter)
  local totalResources = 0

  -- establish a getter if none is supplied
  if attributeSetter == nil then
    attributeSetter = stdSetter
  end

  -- first look through the selected resources
  for resourceId in selectedResources do
    if anim.getResourceType(resourceId) == "take" then
      totalResources = totalResources + 1
      attributeSetter(resourceId, attributeName, value)
    end
  end

  -- if the selected resources are empty then get it from the selected take
  if totalResources == 0 then
    local _, resourceId = anim.getSelectedTakeInAssetManager()
    if resourceId ~= 0 then
      attributeSetter(resourceId, attributeName, value)
      totalResources = 1
    end
  end

  return totalResources
end

------------------------------------------------------------------------------------------------------------------------
local convertToCheckboxValue = function(value, mixed)
  if mixed then
    return 2
  elseif value then
    return 1
  end

  return 0
end

local onRefreshPanel = { }

------------------------------------------------------------------------------------------------------------------------
-- Install the Attribute editor
------------------------------------------------------------------------------------------------------------------------
addAnimationTakeEditor = function(contextualPanel, theContext, assetManager, getSelection, setSelectionChanged)

  local panel
  local loopCheckBox
  local clipStartFraction
  local clipEndFraction
  local clipStartLabel
  local clipEndLabel
  local resetButton
  local timeDisplayUnits
  local mainLabel

  ------------------------------------------------------------------------------------------------------------------------
  -- sets a clip attribute value
  local clipSetter = function(resourceId, attributeName, value)
    if resourceId == 0 or anim.getResourceType(resourceId) ~= "take" then
      return nil
    end

    local duration = anim.getTakeDuration(resourceId)
    local framerate = anim.getTakeFramerate(resourceId)
    if timeDisplayUnits == "Seconds" then
      value =  value / duration
    else
      value =  value /(duration * framerate)
    end

    -- clamp the value
    if value < 0 then
      value = 0
    elseif value > 1 then
      value = 1
    end
    
    local path = anim.getResourcePath(resourceId)
    local attributeValue = anim.setAttribute(path .. "." .. attributeName, value)
  end

  ------------------------------------------------------------------------------------------------------------------------
  -- Returns a clip value
  local clipGetter = function(resourceId, attributeName)
    if resourceId == 0 or anim.getResourceType(resourceId) ~= "take" then
      return nil
    end

    local path = anim.getResourcePath(resourceId)
    local attributeValue = anim.getAttribute(path .. "." .. attributeName)
    local duration = anim.getTakeDuration(resourceId)
    local framerate = anim.getTakeFramerate(resourceId)
    if timeDisplayUnits == "Seconds" then
      return duration * attributeValue
    end

    return duration * attributeValue * framerate
  end

  ------------------------------------------------------------------------------------------------------------------------
  local doRefreshPanel = function()
    if panel == nil then
      return
    end

    local mainScrollPanel = panel:getChild("ScrollPanel")
    if mainScrollPanel == nil then
      return
    end

    local selectedResources = getSelection()
    if not hasTake(selectedResources) then
      mainScrollPanel:setShown(false)
      return
    end

    mainScrollPanel:setShown(true)

    local newTimeDisplayUnits = preferences.get("TimeDisplayUnits")
    if newTimeDisplayUnits ~= timeDisplayUnits then
      timeDisplayUnits = newTimeDisplayUnits
      clipStartLabel:setLabel(timeDisplayUnits)
      clipEndLabel:setLabel(timeDisplayUnits)
      mainScrollPanel:rebuild()
    end

    -- Loop
    local value, mixed, totalResources = getChosenAttributeValue(selectedResources, "Loop")
    loopCheckBox:setChecked(convertToCheckboxValue(value, mixed))

    -- If there are no resources then hide the panel
    if mainScrollPanel:setShown(totalResources > 0) then
      mainScrollPanel:getParent():rebuild()
    end

    if totalResources == 0 then
      return
    end

    -- Set the label
    if totalResources == 1 then
      mainLabel:setLabel("Animation Take")
    else
      mainLabel:setLabel(string.format("Animation Take (%s)", totalResources))
    end

    -- ClipStartFraction
    value, mixed, totalResources = getChosenAttributeValue(selectedResources, "ClipStartFraction", clipGetter)
    if totalResources == 1 then
      clipStartFraction:setValue(string.format("%.6f", value))
      clipStartFraction:enable(true)
      clipStartLabel:enable(true)
    else
      clipStartFraction:setValue(string.format("%.6f", 0))
      clipStartFraction:enable(false)
      clipStartLabel:enable(false)
    end

    -- ClipEndFraction
    value, mixed, totalResources = getChosenAttributeValue(selectedResources, "ClipEndFraction", clipGetter)
    if totalResources == 1 then
      clipEndFraction:setValue(string.format("%.6f", value))
      clipEndFraction:enable(true)
      clipEndLabel:enable(true)
    else
      clipEndFraction:setValue(string.format("%.6f", 0))
      clipEndFraction:enable(false)
      clipEndLabel:enable(false)
    end

    -- ClipStartFraction and ClipEndFraction
    local changed1, mixed = getChosenAttributeValue(selectedResources, "ClipStartFraction", clipChangedGetter)
    changed1 = changed1 or mixed
    local changed2, mixed = getChosenAttributeValue(selectedResources, "ClipEndFraction", clipChangedGetter)
    changed2 = changed2 or mixed
    resetButton:enable(changed1 or changed2)

    -- Ensure the whole panel is updated so it resizes to the content.
    panel:doLayout()
  end

  onRefreshPanel[theContext:getName()] = doRefreshPanel

  ------------------------------------------------------------------------------------------------------------------------
  local loopCheckboxChanged = function(self)
    local loopChecked = loopCheckBox:getChecked()
    local selectedResources = getSelection()
    setChosenAttributeValue(selectedResources, "Loop", loopChecked)
    local affectedAnimationTakes = anim.listAnimationTakeAttributes("", selectedResources)

    -- find which nodes have different animation
    local attributesToChange = { }
    for _, takes in pairs(affectedAnimationTakes) do
      local path = takes.path .. ".Loop"
      if getAttribute(path, loopChecked) ~= loopChecked then
        table.insert(attributesToChange, path)
      end
    end

    -- and set them
    if table.getn(attributesToChange) > 0 and
      ui.showMessageBox("Do you want to change the looping in all nodes that use this animation?", "ok;cancel") == "ok" then
      undoBlock(
        function()
          for _, takes in pairs(affectedAnimationTakes) do
            setAttribute(takes.path .. ".Loop", loopChecked)
          end
        end
      )
    end
  end

  panel = contextualPanel:addPanel{
    name = string.format("ChooserAttributeEditor_%s", theContext:getName()),
    forContext = theContext,
  }
  panel:beginVSizer{ flags = "expand", proportion = 1 }
    panel:addVSpacer(4)
    local mainScrollPanel = panel:addScrollPanel{
      name = "ScrollPanel",
      flags = "expand;vertical",
      proportion = 1,
    }
    mainScrollPanel:beginVSizer{ flags = "expand" }
    mainLabel = mainScrollPanel:addStaticText{ text = "Animation Takes", flags = "expand" }

      local rollup = mainScrollPanel:addRollup{ label = "Clip Range", flags = "mainSection;expand", name = "clipRange" }
      local rollPanel = rollup:getPanel()
      rollPanel:beginVSizer{ flags = "expand" }
        rollPanel:beginFlexGridSizer{ cols = 3, flags = "expand", proportion = 1 }
        rollPanel:setFlexGridColumnExpandable(2)

          -- Start
          rollPanel:addStaticText{ text = "Start" }
          clipStartFraction = rollPanel:addTextBox{
            name = "start",
            flags = "numeric;expand",
            size = { width = 40 },
            onEnter = function(self)
              local value = tonumber(self:getValue())
              setChosenAttributeValue(getSelection(), "ClipStartFraction", value, clipSetter)
            end,
          }
          clipStartLabel = rollPanel:addStaticText{ text = " " }

          -- End
          rollPanel:addStaticText{ text = "End" }
          clipEndFraction = rollPanel:addTextBox{
            name = "end",
            flags = "numeric;expand",
            size = { width = 40 },
            onEnter = function(self)
              local value = tonumber(self:getValue())
              setChosenAttributeValue(getSelection(), "ClipEndFraction", value, clipSetter)
            end,
          }
          clipEndLabel = rollPanel:addStaticText{ text = " " }
        rollPanel:endSizer()

        -- Reset
        resetButton = rollPanel:addButton{
          label = "Reset",
          flags = "right",
          size = { width = 74 },
          onClick = function(self)
            setChosenAttributeValue(getSelection(), "ClipStartFraction", 0)
            setChosenAttributeValue(getSelection(), "ClipEndFraction", 1)
          end,
        }

      rollPanel:endSizer()

      local rollup = mainScrollPanel:addRollup{ label = "Playback Options", flags = "mainSection;expand", name = "playbackOptions" }
      local rollPanel = rollup:getPanel()
      rollPanel:beginHSizer{ flags = "expand" }
        rollPanel:addStaticText{ text = "Loop" }
        loopCheckBox = rollPanel:addCheckBox{ name = "loopCheckBox" }
      rollPanel:endSizer()
    mainScrollPanel:endSizer()
  panel:endSizer()

  doRefreshPanel()
  panel:doLayout()
  loopCheckBox:setOnChanged(loopCheckboxChanged)
  setSelectionChanged(doRefreshPanel)
end

if not mcn.inCommandLineMode() then
  ----------------------------------------------------------------------------------------------------------------------
  -- Having a wrapper handler function ensures this script can be rerun
  -- without registering duplicate event handlers.
  ----------------------------------------------------------------------------------------------------------------------
  local refreshPanel = function()
    if not mcn.isOpen() then
      return
    end

    if type(onRefreshPanel) == "table" then
      for _, func in pairs(onRefreshPanel) do
        safefunc(func)
      end
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- This ensures no old callbacks are laying about in between a file close and open
  ----------------------------------------------------------------------------------------------------------------------
  local clearOnRefreshPanelTable = function()
    onRefreshPanel = { }
  end

  registerEventHandler("mcAnimationTakeSelectionChange", refreshPanel)
  registerEventHandler("mcPreferencesChanged", refreshPanel)
  registerEventHandler("mcAnimationTakeChange", refreshPanel)

  -- ensures we don't try to refresh any dead windows after a network has been closed
  registerEventHandler("mcFileCloseBegin", clearOnRefreshPanelTable)
end