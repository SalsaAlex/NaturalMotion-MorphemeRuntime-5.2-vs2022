------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "luaAPI/MorphemeUnitAPI.lua"
require "luaAPI/UIUtils.lua"
require "ui/PreferencesEditor/PreferencesAPI.lua"

------------------------------------------------------------------------------------------------------------------------
-- updates the preset defaults
------------------------------------------------------------------------------------------------------------------------
local updatePresetDefaultsSection = function(panel, physicsEngine)
  local presetDefaultsPanel = panel:getChild("PresetDefaultsPanel")
  assert(presetDefaultsPanel, debug.traceback())

    preferences.updateNumberWidget(presetDefaultsPanel, "DefaultDensity")

    if physicsEngine == "PhysX3" then
      preferences.updateNumberWidget(presetDefaultsPanel, "DefaultSleepThreshold")
    end

    if physicsEngine == "PhysX2" or physicsEngine == "PhysX3" then
      preferences.updateNumberWidget(presetDefaultsPanel, "DefaultJointStrength")
      preferences.updateNumberWidget(presetDefaultsPanel, "DefaultJointDamping")
      preferences.updateNumberWidget(presetDefaultsPanel, "DefaultSkinWidth")
    end

  presetDefaultsPanel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- adds a preset defaults widget to a panel
------------------------------------------------------------------------------------------------------------------------
local addPresetDefaultsWidget = function(panel, name, attr)
  -- add the name text
  --
  panel:addStaticText{ text = name, }

  -- add the number widget
  --
  local widget = preferences.addNumberWidget(panel, attr)

  -- override the on changed function to update the presets combo box
  --
  local originalOnChange = widget:getOnChanged()
  widget:setOnChanged(
    function (self)
      setToCustomUnitPreset()
      if originalOnChange ~= nil then
        originalOnChange(self)
      end
    end
  )
end

------------------------------------------------------------------------------------------------------------------------
-- rebuilds the preset defaults hiding some engine specific
------------------------------------------------------------------------------------------------------------------------
local rebuildPresetDefaultsSection = function(panel, physicsEngine)
  local presetDefaultsPanel = panel:getChild("PresetDefaultsPanel")
  assert(presetDefaultsPanel, debug.traceback())
  presetDefaultsPanel:clear()

  presetDefaultsPanel:beginFlexGridSizer{ flags = "expand", cols = 2, }
  presetDefaultsPanel:setFlexGridColumnExpandable(2)

    addPresetDefaultsWidget(presetDefaultsPanel, "Density", "DefaultDensity")

    if physicsEngine == "PhysX3" then
      addPresetDefaultsWidget(presetDefaultsPanel, "Sleep energy threshold", "DefaultSleepThreshold")
    end

    if physicsEngine == "PhysX2" or physicsEngine == "PhysX3" then
      addPresetDefaultsWidget(presetDefaultsPanel, "SLERP drive spring", "DefaultJointStrength")
      addPresetDefaultsWidget(presetDefaultsPanel, "SLERP drive damping", "DefaultJointDamping")
      addPresetDefaultsWidget(presetDefaultsPanel, "Skin width", "DefaultSkinWidth")
    end

  presetDefaultsPanel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- adds the preset defaults section
------------------------------------------------------------------------------------------------------------------------
local addPresetDefaultsSection = function(panel)
  local presetDefaultsPanel = panel:addPanel{
    name = "PresetDefaultsPanel",
    flags = "expand",
    proportion = 1,
  }

  local physicsEngine = preferences.get("PhysicsEngine")
  rebuildPresetDefaultsSection(panel, physicsEngine)
end

------------------------------------------------------------------------------------------------------------------------
-- adds a widget for the PhysicsEngine preference
------------------------------------------------------------------------------------------------------------------------
local addPhysicsEngineWidget = function(panel)
  assert(panel)

  local physicsEngineCount = getPhysicsEngineCount()

  local physicsEngineNames = {}
  for i = 1, getPhysicsEngineCount() do
    local physicsEngine = getPhysicsEngine(i)
    local physicsEngineName = physicsEngine:getPhysicsEngineName()
    if physicsEngineName ~= "Euphoria" then
      table.insert(physicsEngineNames, physicsEngineName)
    end
  end

  local preferenceComboBox = panel:addComboBox{
    name = "PhysicsEngineComboBox",
    items = physicsEngineNames,
    flags = "expand",
    proportion = 1,
    onChanged = function(self)
      local panel = self:getParent()

      local _, info = preferences.get("PhysicsEngine")
      local selected = self:getSelectedItem()
      info.value = selected
      preferences.set(info)

      panel:freeze()
      rebuildPresetDefaultsSection(panel, selected)
      updatePresetDefaultsSection(panel, selected)
      panel:rebuild()
    end,
  }

  return preferenceComboBox
end

------------------------------------------------------------------------------------------------------------------------
-- updates a widget for the PhysicsEngine preference
------------------------------------------------------------------------------------------------------------------------
local updatePhysicsEngineWidget = function(panel, preference)
  assert(panel)

  local preferenceComboBox = panel:getChild("PhysicsEngineComboBox")
  local value = preferences.get("PhysicsEngine")
  preferenceComboBox:setSelectedItem(value)
end

------------------------------------------------------------------------------------------------------------------------
-- updates the runtime targets preferences page on data changes
------------------------------------------------------------------------------------------------------------------------
local updatePreferencePage = function(panel)
  local eventDirControl = panel:getChild("EventDetectionOutputDirectory")
  local eventDir = preferences.get("EventDetectionOutputDirectory")
  eventDirControl:setValue(eventDir)
  eventDirControl:setDefaultDirectory(utils.demacroizeString(eventDir))

  local eventTemplatesControl = panel:getChild("EventDetectionTemplatesFile")
  local eventTemplates = preferences.get("EventDetectionTemplatesFile")
  eventTemplatesControl:setValue(eventTemplates)

  if not mcn.isPhysicsDisabled() then
    updatePhysicsEngineWidget(panel)
  end
  preferences.updateEnumWidget(panel, "WorldUpAxis")

  local runtimeAssetUnitComboBox = panel:getChild("RuntimeAssetUnitComboBox")
  local runtimeAssetScaleFactor = preferences.get("RuntimeAssetScaleFactor")
  local runtimeAssetUnit = units.findByScaleFactor(runtimeAssetScaleFactor)

  runtimeAssetUnitComboBox:setSelectedItem(runtimeAssetUnit.longname or runtimeAssetUnit.name)

  if not mcn.isPhysicsDisabled() then
    updatePresetDefaultsSection(panel, preferences.get("PhysicsEngine"))
  end
end

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
local generateOnGlobalComboChanged = function(preferenceName)
  local func = function(self)
    local _, info = preferences.get(preferenceName)
    local selection = self:getSelectedItem()
    local unit = units.findByName(selection)
    info.value = unit.scaleFactor
    preferences.set(info)
  end

  return func
end

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
local unitCombo = nil
local addUnitsPresetCombo = function (panel)
  local nmxApp = nmx.Application.new()
  local roamingUserSettings = nmxApp:getRoamingUserSettings()

  local presets = roamingUserSettings:getSetting("|MorphemeAttributeDefaultsPresets")

  local items = { }
  local child = presets:getFirstChild()
  while child ~= nil do
    table.insert(items, child:getName())
    child = child:getNextSibling()
  end
  table.insert(items, "Custom")

  unitCombo = panel:addComboBox{
    flags = "expand", proportion = 1,
    items = items,
    onChanged = function (self)
      local nmxApp = nmx.Application.new()
      local roamingUserSettings = nmxApp:getRoamingUserSettings()

      local presets = roamingUserSettings:getSetting("|MorphemeAttributeDefaultsPresets")

      local child = presets:getFirstChild()
      while child ~= nil do
        if child:getName() == self:getSelectedItem() then
          local copyPreset = function (attrName, prefName)
            local attr = child:findAttribute(attrName)
            if not attr:isValid() then
              app.error("invalid MorphemeAttributeDefaultsNode attribute `" .. attrName .. "`")
              return
            end

            local _, info = preferences.get(prefName)
            info.value = attr:asDouble()
            preferences.set(info)
          end

          copyPreset("UpAxis", "WorldUpAxis")
          copyPreset("DistanceScaleFactor", "RuntimeAssetScaleFactor")
          copyPreset("Density", "DefaultDensity")
          copyPreset("SleepThreshold", "DefaultSleepThreshold")
          copyPreset("JointStrength", "DefaultJointStrength")
          copyPreset("JointDamping", "DefaultJointDamping")
          copyPreset("SkinWidth", "DefaultSkinWidth")

          -- Set the preset hint in the project settings.
          local _, hintInfo = preferences.get("RuntimeSettingsPreset")
          hintInfo.value = self:getSelectedItem()
          preferences.set(hintInfo)

          updatePreferencePage(panel)
          break
        end

        child = child:getNextSibling()
      end
    end,
  }

  return unitCombo
end

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
local setToCustomUnitPreset = function ()
  if unitCombo ~= nil then
    unitCombo:setSelectedItem("Custom")
  end
end

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
local findCorrectUnitPreset = function ()
  if unitCombo ~= nil then
    local nmxApp = nmx.Application.new()
    local roamingUserSettings = nmxApp:getRoamingUserSettings()

    local presets = roamingUserSettings:getSetting("|MorphemeAttributeDefaultsPresets")

    local matches = {}
    local child = presets:getFirstChild()
    while child ~= nil do
      local compareUpAxis = function ()
        local value, valTable = preferences.get("WorldUpAxis")
        local attr = child:findAttribute("UpAxis")
        if not attr:isValid() then
          app.error("invalid MorphemeAttributeDefaultsNode attribute `UpAxis`")
          return false
        end

        return value == valTable.choices[attr:asInt() + 1]
      end

      local comparePreference = function (prefName, attrName)
        local currVal = preferences.get(prefName)
        local attr = child:findAttribute(attrName)
        if not attr:isValid() then
          app.error("invalid MorphemeAttributeDefaultsNode attribute `" .. attrName .. "`")
          return false
        end

        return currVal == attr:asDouble()
      end

      if comparePreference("RuntimeAssetScaleFactor", "DistanceScaleFactor") and
         comparePreference("DefaultDensity", "Density") and
         comparePreference("DefaultSleepThreshold", "SleepThreshold") and
         comparePreference("DefaultJointStrength", "JointStrength") and
         comparePreference("DefaultJointDamping", "JointDamping") and
         comparePreference("DefaultSkinWidth", "SkinWidth") and
         compareUpAxis()
      then
        table.insert(matches, child:getName())
      end

      child = child:getNextSibling()
    end

    local numMatches = table.getn(matches)
    if numMatches == 1 then
      unitCombo:setSelectedItem(matches[1])
      return
    end

    local presetHint = preferences.get("RuntimeSettingsPreset")
    for i=1, numMatches do
      if presetHint == matches[i] then
        unitCombo:setSelectedItem(matches[i])
        return
      end
    end

    setToCustomUnitPreset()
  end
end

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
local addAttributePresetSection = function (panel)
  panel:beginVSizer{ flags = "expand" }
    panel:beginHSizer{
      flags = "expand"
    }
      panel:addHSpacer(5)
      panel:beginFlexGridSizer{flags = "expand", cols = 2}
      panel:setFlexGridColumnExpandable(2)
        local widget

        panel:addStaticText{ text = "Up axis", }
        widget = preferences.addEnumWidget(panel, "WorldUpAxis")
        local upAxisOnChange = widget:getOnChanged()
        widget:setOnChanged(
          function (self)
            setToCustomUnitPreset()
            upAxisOnChange(self)
          end
        )

        panel:addStaticText{ text = "Distance unit", flags = "expand", }
        widget = addUnitComboBox(panel, "RuntimeAssetUnitComboBox")
        widget:setOnChanged(
          function (self)
            setToCustomUnitPreset()
            generateOnGlobalComboChanged("RuntimeAssetScaleFactor")(self)
          end
        )
      panel:endSizer()
      panel:addHSpacer(5)
    panel:endSizer()
    panel:addVSpacer(7)

    if not mcn.isPhysicsDisabled() then
      panel:beginVSizer{
        flags = "group;expand",
        label = "Creation defaults"
      }
        panel:addVSpacer(5)

        addPresetDefaultsSection(panel)
      panel:endSizer()
    end
  panel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- builds the settings preferences page
------------------------------------------------------------------------------------------------------------------------
local buildPreferencePage = function(panel)
  panel:beginVSizer{ flags = "expand", proportion = 1, }
    panel:addStaticText{
      name = "Heading",
      text = "Project settings",
      font = "bold",
      flags = "parentBackground;truncate;expand;decoration",
    }

    panel:addVSpacer(5)

    panel:beginHSizer{ flags = "expand", }
      panel:addStaticText{ text = "Root directory", }

      local projectFile = project.getFilename()
      local rootDirectory, filename = splitFilePath(projectFile)
      local rootDirectoryTextBox = panel:addTextBox{
        name = "RootDirectoryTextBox",
        proportion = 1,
        flags = "expand",
        value = rootDirectory,
      }
      rootDirectoryTextBox:setReadOnly(true)
    panel:endSizer()

    panel:addVSpacer(5)

    panel:beginVSizer{
      flags = "group;expand",
      label = "Runtime world settings",
    }
      panel:beginFlexGridSizer{flags = "expand", cols = 2}
      panel:setFlexGridColumnExpandable(2)

        if not mcn.isPhysicsDisabled() then
          panel:addStaticText{ text = "Physics Engine", proportion = 0 }
          addPhysicsEngineWidget(panel)
        end

        panel:addStaticText{ text = "Preset", proportion = 0 }
        addUnitsPresetCombo(panel)
        findCorrectUnitPreset()
      panel:endSizer()

      panel:addVSpacer(5)

      addAttributePresetSection(panel)
    panel:endSizer()

    panel:addVSpacer(5)

    panel:beginVSizer{
      flags = "group;expand",
      label = "Event detection",
    }
      panel:addVSpacer(2)
      -- event detection directory
      panel:beginFlexGridSizer{
        flags = "expand",
        cols = 2,
      }
      panel:setFlexGridColumnExpandable(2)
        panel:addStaticText{ text = "Output directory" }

        local eventDir = preferences.get("EventDetectionOutputDirectory")
        local eventDirControl = panel:addDirectoryControl{
          name = "EventDetectionOutputDirectory",
          flags = "expand",
          proportion = 1,
          onMacroize = utils.macroizeString,
          onDemacroize = utils.demacroizeString,
          onChanged = function(self)
            local _, info = preferences.get("EventDetectionOutputDirectory")
            info.value = self:getValue()
            preferences.set(info)
          end,
        }
        eventDirControl:setValue(eventDir)
        eventDirControl:setDefaultDirectory(utils.demacroizeString(eventDir))

        -- event detection templates file
        panel:addStaticText{ text = "Templates file" }

        local eventTemplates = preferences.get("EventDetectionTemplatesFile")
        local eventTemplatesControl = panel:addFilenameControl{
          name = "EventDetectionTemplatesFile",
          flags = "expand",
          proportion = 1,
          wildcard = "Event detection XML|xml",
          onMacroize = utils.macroizeString,
          onDemacroize = utils.demacroizeString,
          onChanged = function(self)
            local _, info = preferences.get("EventDetectionTemplatesFile")
            info.value = self:getValue()
            preferences.set(info)
          end,
        }
        eventTemplatesControl:setValue(eventTemplates)
      panel:endSizer()
    panel:endSizer()

  panel:endSizer()
end

removePreferencesPage("ProjectSettings")
addPreferencesPage(
  "ProjectSettings",
  {
    title = "Project settings",
    create = buildPreferencePage,
    update = updatePreferencePage,
  }
)
