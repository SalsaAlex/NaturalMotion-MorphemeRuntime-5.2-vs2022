------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "luaAPI/UIUtils.lua"
require "ui/PreferencesEditor/PreferencesAPI.lua"
require "ui/PreferencesEditor/ProjectSettingsPage.lua"

-- gets the animation location selected in a list control of animation locations
local getSelectedAnimationLocation = function(listControl)
  local selectedRow = listControl:getSelectedRow()
  if selectedRow then
    local defaults = project:getDefaults()
    local locations = defaults:listAnimationLocations()
    -- the order in the list control will be the same order as the list returned by listAnimationLocations
    return locations[selectedRow]
  end

  return nil
end

-- finds the row an animation location is on in a list control of animation locations,
-- assumes all locations present in the defaults are there and in the same order.
local findAnimationLocationRow = function(listControl, animationLocation)
  if not animationLocation then
    return nil
  end

  local defaults = project:getDefaults()
  local locations = defaults:listAnimationLocations()

  for i, location in ipairs(locations) do
    if location == animationLocation then
      return i
    end
  end

  return nil
end

-- gets the animation set selected in a list control of animation sets
local getSelectedAnimationSet = function(treeControl)
  local selectedItem = treeControl:getSelectedItem()
  if selectedItem then
    local defaults = project:getDefaults()
    local sets = defaults:listAnimationSets()
    local index = selectedItem:getUserDataInt()
    return sets[index]
  end

  return nil
end

-- finds the row an animation location is on in a list control of animation locations,
-- assumes all locations present in the defaults are there and in the same order.
local findAnimationSetItem = function(treeControl, animationSet)
  if not animationSet then
    return nil
  end

  local defaults = project:getDefaults()
  local sets = defaults:listAnimationSets()

  local name = animationSet:getName()
  local item = treeControl:findItem(name)
  return item
end

-- called when the animation sets ListControl selection is changed
local onAnimationLocationSelectionChanged = function(listControl)
  local panel = listControl:getParent()
  local animationLocationRollup = panel:getChild("AnimationLocationRollup")
  local animationLocationPanel = animationLocationRollup:getPanel()
  local animationSourceDirectoryControl = animationLocationPanel:getChild("AnimationSourceDirectoryControl")
  local animationMarkupDirectoryControl = animationLocationPanel:getChild("AnimationMarkupDirectoryControl")
  local animationRecursiveCheckBox = animationLocationPanel:getChild("AnimationRecursiveCheckBox")

  local enable = true

  local animationLocation = getSelectedAnimationLocation(listControl)
  if animationLocation then
    local sourceDirectory = animationLocation:getSourceDirectory()
    animationSourceDirectoryControl:setValue(sourceDirectory)
    sourceDirectory = utils.demacroizeString(sourceDirectory)
    animationSourceDirectoryControl:setDefaultDirectory(sourceDirectory)

    local markupDirectory = animationLocation:getMarkupDirectory()
    animationMarkupDirectoryControl:setValue(markupDirectory)
    markupDirectory = utils.demacroizeString(markupDirectory)
    animationMarkupDirectoryControl:setDefaultDirectory(markupDirectory)

    local recursive = animationLocation:getRecursive()
    animationRecursiveCheckBox:setChecked(recursive)
  else
    enable = false
    animationSourceDirectoryControl:setValue("")
    animationMarkupDirectoryControl:setValue("")
    animationRecursiveCheckBox:setChecked(false)
  end

  animationSourceDirectoryControl:enable(enable)
  animationMarkupDirectoryControl:enable(enable)
  animationRecursiveCheckBox:enable(enable)
end

-- called when the animation sets ListControl selection is changed
local onAnimationSetSelectionChanged = function(treeControl)
  local panel = treeControl:getParent()

  panel:freeze()

  local rigSettingsRollup = panel:getChild("RigSettingsRollup")
  local rigSettingsPanel = rigSettingsRollup:getPanel()
  local animationRigFileControl = rigSettingsPanel:getChild("AnimationRigFileControl")
  local physicsRigFileControl = rigSettingsPanel:getChild("PhysicsRigFileControl")

  local enable = true

  -- first get all the controls from the dialog
  local formatOptionsRollup = panel:getChild("AnimationFormatOptionsRollup")
  local formatOptionsPanel = formatOptionsRollup:getPanel()
  local formatCombo = formatOptionsPanel:getChild("AnimationFormatCombo")
  local formatLabel = formatOptionsPanel:getChild("AnimationFormatText")
  local formatSpecificPanel = formatOptionsPanel:getChild("AnimationFormatSpecificPanel")

  -- get the currently selected set and format
  local animationSet = getSelectedAnimationSet(treeControl)
  local format = animfmt.get(formatCombo:getSelectedItem())
  if animationSet then
    animationRigFileControl:setValue(animationSet:getAnimationRig())
    if not mcn.isPhysicsDisabled() then
      physicsRigFileControl:setValue(animationSet:getPhysicsRig())
    end

    -- get the format of the selected set and set the value of the combo
    local formatString = animationSet:getFormat()
    format = animfmt.get(formatString)
    formatCombo:setSelectedItem(formatString)
  else
    enable = false
    animationRigFileControl:setValue("")
    if not mcn.isPhysicsDisabled() then
      physicsRigFileControl:setValue("")
    end
  end

  -- these functions should do nothing if there is no set selected.
  local getSelectionCount = function()
    return 1
  end

  local getOptionsTable = function(index)
    local animationSet = getSelectedAnimationSet(treeControl)
    if animationSet then
      local optionsString = animationSet:getOptions()
      return animfmt.parseOptions(optionsString)
    end
    return { }
  end

  local shouldEnableControls = function()
    local animationSet = getSelectedAnimationSet(treeControl)
    return animationSet ~= nil
  end

  -- as this function passes itself to a function when called it has to be forward declared
  local setOptionsTable = nil
  setOptionsTable = function(index, options)
    local animationSet = getSelectedAnimationSet(treeControl)
    if animationSet then
      local optionsString = animfmt.compileOptions(options)
      animationSet:setOptions(optionsString)
      format.updateFormatOptionsPanel(formatSpecificPanel, getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
    end
  end

  -- set the format options section to display the controls for the currently selected format
  formatSpecificPanel:clear()
  formatSpecificPanel:beginVSizer{ flags = "expand" }
  if type(format.addFormatOptionsPanel) == "function" and type(format.updateFormatOptionsPanel) == "function" then
    -- add any format specific controls
    format.addFormatOptionsPanel(formatSpecificPanel, getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
    format.updateFormatOptionsPanel(formatSpecificPanel, getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
  end
  local enableControls = shouldEnableControls()
  formatCombo:enable(enableControls)
  formatLabel:enable(enableControls)
  
  formatSpecificPanel:endSizer()

  formatSpecificPanel:rebuild()

  animationRigFileControl:enable(enable)
  if not mcn.isPhysicsDisabled() then
    physicsRigFileControl:enable(enable)
  end

  panel:rebuild()
end

------------------------------------------------------------------------------------------------------------------------
-- updates the runtime targets preferences page on data changes
------------------------------------------------------------------------------------------------------------------------
local updatePreferencePage = function(panel)
  local animationLocationsListControl = panel:getChild("AnimationLocationsListControl")
  if not animationLocationsListControl then
    return
  end

  animationLocationsListControl:clearRows()

  local defaults = project:getDefaults()
  local animationLocations = defaults:listAnimationLocations()
  for i, location in ipairs(animationLocations) do
    local sourceDirectory = location:getSourceDirectory()
    animationLocationsListControl:addRow(sourceDirectory)
  end

  onAnimationLocationSelectionChanged(animationLocationsListControl)

  local animationSetsTreeControl = panel:getChild("AnimationSetsTreeControl")
  if not animationSetsTreeControl then
    return
  end

  local defaults = project:getDefaults()
  local animationSets = defaults:listAnimationSets()

  local root = animationSetsTreeControl:getRoot()
  root:clearChildren()

  for i, location in ipairs(animationSets) do
    local name = location:getName()
    local item = root:addChild(name)
    item:setUserDataInt(i)
  end

  onAnimationSetSelectionChanged(animationSetsTreeControl)
end

------------------------------------------------------------------------------------------------------------------------
local buildAnimationLocationSection = function(panel)
  panel:beginHSizer{ flags = "expand", }
    local addLocationButton = addImageButton(panel, "AddLocationButton", "additem.png")
    addLocationButton:setOnClick(
      function(self)
        local defaults = project:getDefaults()

        local appExecutableDirectory = app.getAppExecutableDir()

        local defaultSourceDirectory = preferences.get("DefaultAnimationSourceDirectory")
        if type(defaultSourceDirectory) ~= "string" or string.len(defaultSourceDirectory) == 0 then
          defaultSourceDirectory = string.format("%sCharacters\\MaleCharacter\\Animation\\XMD", appExecutableDirectory)
        end
        defaultSourceDirectory = utils.macroizeString(defaultSourceDirectory)

        local defaultMarkupDirectory = preferences.get("DefaultAnimationMarkupDirectory")
        if type(defaultMarkupDirectory) ~= "string" or string.len(defaultMarkupDirectory) == 0 then
          defaultMarkupDirectory = string.format("%sCharacters\\MaleCharacter\\MorphemeMarkup", appExecutableDirectory)
        end
        defaultMarkupDirectory = utils.macroizeString(defaultMarkupDirectory)

        local animationLocation = defaults:addAnimationLocation(defaultSourceDirectory)
        animationLocation:setMarkupDirectory(defaultMarkupDirectory)

        updatePreferencePage(panel)

        local listControl = panel:getChild("AnimationLocationsListControl")
        local row = findAnimationLocationRow(listControl, animationLocation)
        if row then
          listControl:selectRow(row)
        end
      end
    )
    panel:addHSpacer(2)

    local removeLocationButton = addImageButton(panel, "RemoveLocationButton", "removeitem.png")
    removeLocationButton:setOnClick(
      function(self)
        local listControl = panel:getChild("AnimationLocationsListControl")
        local location = getSelectedAnimationLocation(listControl)
        if location then
          local defaults = project:getDefaults()
          defaults:removeAnimationLocation(location)
        end

        updatePreferencePage(panel)
      end
    )
  panel:endSizer()

  local animationLocationsListControl = panel:addListControl{
    name = "AnimationLocationsListControl",
    flags = "expand;rename",
    size = {
      height = 150,
    },
    columnNames = {
      "Animation location",
    },
    onSelectionChanged = onAnimationLocationSelectionChanged,
  }

  local animationLocationRollup = panel:addRollup{
    name = "AnimationLocationRollup",
    label = "Animation location",
    flags = "expand;mainSection",
  }

  local animationLocationPanel = animationLocationRollup:getPanel()
  animationLocationPanel:beginVSizer()
    animationLocationPanel:beginFlexGridSizer{ cols = 2, flags = "expand" }
    animationLocationPanel:setFlexGridColumnExpandable(2)

      animationLocationPanel:addStaticText{ text = "Source directory" }
      animationLocationPanel:addDirectoryControl{
        name = "AnimationSourceDirectoryControl",
        flags = "expand",
        proportion = 1,
        onMacroize = utils.macroizeString,
        onDemacroize = utils.demacroizeString,
        onChanged = function(self)
          -- get the currently selected location
          local animationLocation = getSelectedAnimationLocation(animationLocationsListControl)
          if animationLocation then
            local value = self:getValue()
            animationLocation:setSourceDirectory(value)
            local actualValue = animationLocation:getSourceDirectory()
            self:setValue(actualValue)
            updatePreferencePage(panel)
          else
            self:clear()
          end
        end,
      }

      animationLocationPanel:addStaticText{ text = "Markup directory" }
      animationLocationPanel:addDirectoryControl{
        name = "AnimationMarkupDirectoryControl",
        flags = "expand",
        proportion = 1,
        onChanged = function(self)
          -- get the currently selected location
          local animationLocation = getSelectedAnimationLocation(animationLocationsListControl)
          if animationLocation then
            local value = self:getValue()
            animationLocation:setMarkupDirectory(value)
            local actualValue = animationLocation:getMarkupDirectory()
            self:setValue(actualValue)
          else
            self:clear()
          end
        end,
      }

      animationLocationPanel:addVSpacer(0)
      animationLocationPanel:addCheckBox{
        name = "AnimationRecursiveCheckBox",
        label = "Recursive",
        onChanged = function(self)
          -- get the currently selected location
          local animationLocation = getSelectedAnimationLocation(animationLocationsListControl)
          if animationLocation then
            local value = self:getChecked()
            animationLocation:setRecursive(value)
          else
            self:clear()
          end
        end,
      }
    animationLocationPanel:endSizer()
  animationLocationPanel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
local buildAnimationSetSection = function(panel)
  panel:beginHSizer{ flags = "expand", }
    local addSetButton = addImageButton(panel, "AddSetButton", "additem.png")
    addSetButton:setOnClick(
      function(self)
        local defaults = project:getDefaults()
        local animSetName = preferences.get("DefaultAnimationSetName") or "AnimationSet"
        local animationSet = defaults:addAnimationSet(animSetName)

        local defaultAnimationFormat = preferences.get("DefaultAnimationFormat")
        animationSet:setFormat(defaultAnimationFormat)
        local defaultAnimationFormatOptions = preferences.get("DefaultAnimationFormatOptions")
        animationSet:setOptions(defaultAnimationFormatOptions)

        updatePreferencePage(panel)

        local treeControl = panel:getChild("AnimationSetsTreeControl")
        local item = findAnimationSetItem(treeControl, set)
        if item then
          treeControl:selectItem(item)
          onAnimationSetSelectionChanged(treeControl)
        end
      end
    )
    panel:addHSpacer(2)

    local removeSetButton = addImageButton(panel, "RemoveSetButton", "removeitem.png")
    removeSetButton:setOnClick(
      function(self)
        local treeControl = panel:getChild("AnimationSetsTreeControl")
        local set = getSelectedAnimationSet(treeControl)
        if set then
          local defaults = project:getDefaults()
          defaults:removeAnimationSet(set)
        end

        updatePreferencePage(panel)
      end
    )
  panel:endSizer()

  local animationSetsTreeControl = panel:addTreeControl{
    name = "AnimationSetsTreeControl",
    flags = "expand;rename;hideRoot",
    size = {
      height = 150,
    },
    columnNames = {
      "Animation Set",
    },
    onSelectionChanged = onAnimationSetSelectionChanged,
    onItemRenamed = function(self, item)
      local defaults = project:getDefaults()
      local sets = defaults:listAnimationSets()
      local index = item:getUserDataInt()
      local set = sets[index]
      if set then
        local value = item:getValue()
        set:setName(value)
      end
    end,
    onContextualMenu = function(menu, item)
      menu:addItem{
        label = "Rename animation set",
        onClick = function(self)
          item:getTreeControl():editItem(item)
        end,
      }
    end,
  }

  local rigSettingsRollup = panel:addRollup{
    name = "RigSettingsRollup",
    label = "Rigs",
    flags = "expand;mainSection",
  }

  local rigSettingsPanel = rigSettingsRollup:getPanel()
  rigSettingsPanel:beginVSizer()
    rigSettingsPanel:beginFlexGridSizer{ cols = 2, flags = "expand" }
    rigSettingsPanel:setFlexGridColumnExpandable(2)

      rigSettingsPanel:addStaticText{ text = "Animation rig" }
      local animationRigFileControl = rigSettingsPanel:addFilenameControl{
        name = "AnimationRigFileControl",
        flags = "expand",
        wildcard = "morpheme:connect animation rig files|mcarig",
        proportion = 1,
        onMacroize = utils.macroizeString,
        onDemacroize = utils.demacroizeString,
        onChanged = function(self)
          -- get the currently selected set
          local animationSet = getSelectedAnimationSet(animationSetsTreeControl)
          if animationSet then
            animationSet:setAnimationRig(self:getValue())
          else
            self:clear()
          end
        end,
      }

      if not mcn.isPhysicsDisabled() then
        rigSettingsPanel:addStaticText{ text = "Physics rig" }
        local physicsRigFileControl = rigSettingsPanel:addFilenameControl{
          name = "PhysicsRigFileControl",
          flags = "expand",
          wildcard = "morpheme:connect physics rig files|mcprig",
          proportion = 1,
          onMacroize = utils.macroizeString,
          onDemacroize = utils.demacroizeString,
          onChanged = function(self)
            -- get the currently selected set
            local animationSet = getSelectedAnimationSet(animationSetsTreeControl)
            if animationSet then
              animationSet:setPhysicsRig(self:getValue())
            else
              self:clear()
            end
          end,
        }
      end
    rigSettingsPanel:endSizer()
  rigSettingsPanel:endSizer()

  -- add animation format options controls
  local formatOptionsRollup = panel:addRollup{
    name = "AnimationFormatOptionsRollup",
    label = "Animation compression",
    flags = "expand;mainSection",
  }

  local formatOptionsPanel = formatOptionsRollup:getPanel()
  formatOptionsPanel:beginVSizer{ flags = "expand", proportion = 1 }
    formatOptionsPanel:beginHSizer{ flags = "expand", proportion = 1 }
      formatOptionsPanel:addStaticText{ name = "AnimationFormatText", text = "Format" }

      local items = { }
      for i, value in ipairs(animfmt.ls()) do
        table.insert(items, value.format)
      end

      local formatCombo = formatOptionsPanel:addComboBox{
        name = "AnimationFormatCombo",
        flags = "expand",
        proportion = 1,
        items = items,
        onChanged = function(self)
          local animationSet = getSelectedAnimationSet(animationSetsTreeControl)
          if animationSet ~= nil then
            local formatString = self:getSelectedItem()
            animationSet:setFormat(formatString)

            local optionsString = animationSet:getOptions()
            local optionsTable = animfmt.parseOptions(optionsString)
            optionsTable = animfmt.removeInvalidOptions(formatString, optionsTable)
            optionsString = animfmt.compileOptions(optionsTable)
            animationSet:setOptions(optionsString)

            onAnimationSetSelectionChanged(animationSetsTreeControl)
          end
        end,
      }
      local defaultAnimationFormat = preferences.get("DefaultAnimationFormat")
      formatCombo:setSelectedItem(defaultAnimationFormat)
    formatOptionsPanel:endSizer()

    -- used later
    formatOptionsPanel:addPanel{ name = "AnimationFormatSpecificPanel", flags = "expand" }
  formatOptionsPanel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- builds the settings preferences page
------------------------------------------------------------------------------------------------------------------------
local buildPreferencePage = function(panel)
  panel:beginVSizer{ flags = "expand", proportion = 1, }
    panel:addStaticText{
      name = "Heading",
      text = "New network defaults",
      font = "bold",
      flags = "parentBackground;truncate;expand;decoration",
    }

    buildAnimationLocationSection(panel)
    buildAnimationSetSection(panel)

  panel:endSizer()
end

removePreferencesPage("NewNetworkDefaults")
addPreferencesPage(
  "NewNetworkDefaults",
  {
    title = "New network defaults",
    parent = "ProjectSettings",
    create = buildPreferencePage,
    update = updatePreferencePage,
  }
)

