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

------------------------------------------------------------------------------------------------------------------------
-- gets the target selected in a list control of runtime target names
------------------------------------------------------------------------------------------------------------------------
local getSelectedTarget = function(listControl)
  local selectedRow = listControl:getSelectedRow()
  if selectedRow then
    local selectedTargetName = listControl:getItemValue(selectedRow)
    return target.find(selectedTargetName)
  end

  return nil
end

------------------------------------------------------------------------------------------------------------------------
-- finds the row a runtime target is on in a list control of runtime target names
------------------------------------------------------------------------------------------------------------------------
local findTargetRow = function(listControl, target)
  if not target then
    return nil
  end

  local name = target:getName()
  for i = 1, listControl:getNumRows() do
    local value = listControl:getItemValue(i)
    if value == name then
      return i
    end
  end

  return nil
end

------------------------------------------------------------------------------------------------------------------------
local updatePortToTextBox = function(panel, port, portToTextBox)
  portToTextBox:setLabel(" to " .. tostring(port:getValue() + 2))
  portToTextBox:bestSizeChanged()
end

------------------------------------------------------------------------------------------------------------------------
local refreshTargetRunSettings = function(panel, hasValidPath)
  local runtimeTargetArgumentsTextBox = panel:getChild("RuntimeTargetArgumentsTextBox")
  local showTargetWindowTextBox = panel:getChild("ShowTargetWindowCheckBox")
  
  runtimeTargetArgumentsTextBox:enable(hasValidPath)
  showTargetWindowTextBox:enable(hasValidPath)
end

------------------------------------------------------------------------------------------------------------------------
local refreshAssetCompilerConfigs = function(panel, selectedTarget)
  -- Retrieve the asset compiler directory. Configs are found relative to this.
  local assetCompilerPath = selectedTarget:getAssetCompilerPath()

  local configs = {}
  if assetCompilerPath ~= nil then
    local assetCompilerPath = utils.demacroizeString(assetCompilerPath)
    local directory, file = splitFilePath(assetCompilerPath)
   
    assetCompilerConfigurationDirectory = preferences.get("AssetCompilerConfigurationDirectory")
    local configFileDirectory = directory .. assetCompilerConfigurationDirectory
    local configFiles = app.enumerateFiles(configFileDirectory, "*.cfg")
    
    for i, file in ipairs(configFiles) do
      local configDir, configFile = splitFilePath(file)
      configFile = stripFilenameExtension(configFile)
      table.insert(configs, i, configFile)
    end
  end
  
  local assetCompilerConfigComboBox = panel:getChild("AssetCompilerConfigComboBox")
  assetCompilerConfigComboBox:setItems(configs)
  assetCompilerConfigComboBox:setValue(selectedTarget:getAssetCompilerConfig())
end
  
------------------------------------------------------------------------------------------------------------------------
-- called when the listControl selection is changed
------------------------------------------------------------------------------------------------------------------------
local onSelectionChanged = function(listControl)
  local panel = listControl:getParent()
  
  panel:suspendLayout()

  local nameTextBox = panel:getChild("NameTextBox")
  local descriptionTextBox = panel:getChild("DescriptionTextBox")
  local ipAddressTextBox = panel:getChild("IPAddressTextBox")
  local portTextBox = panel:getChild("PortTextBox")
  local portUpToText = panel:getChild("PortUpToText")
  local timeOutTextBox = panel:getChild("TimeOutTextBox")
  local assetCompilerPathControl = panel:getChild("AssetCompilerPathControl")
  local assetCompilerArgumentsTextBox = panel:getChild("AssetCompilerArgumentsTextBox")
  local assetCompilerConfigComboBox = panel:getChild("AssetCompilerConfigComboBox")
  local runtimeTargetPathControl = panel:getChild("RuntimeTargetPathControl")
  local runtimeTargetArgumentsTextBox = panel:getChild("RuntimeTargetArgumentsTextBox")
  local showTargetWindowTextBox = panel:getChild("ShowTargetWindowCheckBox")

  local selectedTarget = getSelectedTarget(listControl)
  if selectedTarget then
    nameTextBox:setValue(selectedTarget:getName())
    descriptionTextBox:setValue(selectedTarget:getDescription())
    ipAddressTextBox:setValue(selectedTarget:getIPAddress())
    
    portTextBox:setValue(tostring(selectedTarget:getPort()))
    timeOutTextBox:setValue(tostring(selectedTarget:getTimeOut()))

    updatePortToTextBox(panel, portTextBox, portUpToText)

    local executableDirectory = app.getAppExecutableDir()

    local assetCompilerPath = selectedTarget:getAssetCompilerPath()
    assetCompilerPathControl:setValue(assetCompilerPath)
    local directory, file = splitFilePath(assetCompilerPath)
    if type(directory) ~= "string" or string.len(directory) == 0 then
      directory = exectuableDirectory
    end
    assetCompilerPathControl:setDefaultDirectory(directory)
    
    assetCompilerArgumentsTextBox:setValue(selectedTarget:getAssetCompilerArguments())

    refreshAssetCompilerConfigs(panel, selectedTarget)

    local runtimeTargetPath = selectedTarget:getRuntimeTargetPath()
    runtimeTargetPathControl:setValue(runtimeTargetPath)
    local directory, file = splitFilePath(runtimeTargetPath)
    if type(directory) ~= "string" or string.len(directory) == 0 then
      directory = exectuableDirectory
    end
    runtimeTargetPathControl:setDefaultDirectory(directory)
    
    refreshTargetRunSettings(panel, string.len(runtimeTargetPath) ~= 0)

    runtimeTargetArgumentsTextBox:setValue(selectedTarget:getRuntimeTargetArguments())
    showTargetWindowTextBox:setChecked(selectedTarget:getShowTargetWindow())
  else
    nameTextBox:setValue("")
    descriptionTextBox:setValue("")
    ipAddressTextBox:setValue("")
    portTextBox:setValue("")
    portUpToText:setLabel(" to     ")
    timeOutTextBox:setValue("")
    assetCompilerPathControl:setValue("")
    assetCompilerArgumentsTextBox:setValue("")
    runtimeTargetPathControl:setValue("")
    runtimeTargetArgumentsTextBox:setValue("")
    showTargetWindowTextBox:setChecked(false)
    assetCompilerConfigComboBox:setItems({})

    refreshTargetRunSettings(panel, false)
  end
  
  local childrenToExclude = {
    ["Heading"] = true,
    ["AddItemButton"] = true,
    ["RemoveItemButton"] = true,
    ["RuntimeTargetsListControl"] = true,
    ["RuntimeTargetArgumentsTextBox"] = true,
    ["ShowTargetWindowCheckBox"] = true,
  }

  local enable = selectedTarget ~= nil
  for _, child in ipairs(panel:getChildren()) do
    if not childrenToExclude[child:getName()] then
      child:enable(enable)
    end
  end
  
  panel:resumeLayout()
  panel:doLayout()
end

------------------------------------------------------------------------------------------------------------------------
-- updates the runtime targets preferences page on data changes
------------------------------------------------------------------------------------------------------------------------
local updatePreferencePage = function(panel)
  local targetListControl = panel:getChild("RuntimeTargetsListControl")
  if not targetListControl then
    return
  end

  local selectedRow = targetListControl:getSelectedRow()

  local selectedTarget = nil
  if selectedRow then
    selectedTarget = targetListControl:getItemValue(selectedRow)
  end

  targetListControl:clearRows()
  for i, t in ipairs(target.ls()) do
    local name = t:getName()
    targetListControl:addRow(name)

    if selectedTarget and name == selectedTarget then
      selectedRow = i
    end
  end

  if selectedRow then
    targetListControl:selectRow(selectedRow)
  end
end

------------------------------------------------------------------------------------------------------------------------
-- builds the runtime targets preferences page
------------------------------------------------------------------------------------------------------------------------
local buildPreferencePage = function(panel)
  panel:beginVSizer{ flags = "expand", proportion = 1, }
    panel:addStaticText{
      name = "Heading",
      text = "Runtime targets",
      font = "bold",
      flags = "parentBackground;truncate;expand;decoration",
    }

    panel:beginHSizer{ flags = "expand", }
      local addItemButton = addImageButton(panel, "AddItemButton", "additem.png")
      addItemButton:setOnClick(
        function(self)
        
          local targets = target.ls()
          local targetNames = { }
          for _,v in pairs(targets) do
            targetNames[v:getName()] = true
          end
          
          local baseName = "New Runtime Target"
          local uniqueName = baseName
          local index = 1
          while targetNames[uniqueName] == true do
            uniqueName = baseName .. tostring(index)
            index = index + 1
          end
          
          local t = target.add(uniqueName)

          updatePreferencePage(panel)

          local listControl = panel:getChild("RuntimeTargetsListControl")
          local row = findTargetRow(listControl, t)
          if row then
            listControl:selectRow(row)
          end
        end
      )
      panel:addHSpacer(2)

      local removeItemButton = addImageButton(panel, "RemoveItemButton", "removeitem.png")
      removeItemButton:setOnClick(
        function(self)
          local listControl = panel:getChild("RuntimeTargetsListControl")
          local t = getSelectedTarget(listControl)
          if t then
            target.remove(t)
          end

          updatePreferencePage(panel)
        end
      )
      panel:addHSpacer(2)

      local moveUpButton = addImageButton(panel, "MoveUpButton", "moveup.png")
      moveUpButton:setOnClick(
        function(self)
          local listControl = panel:getChild("RuntimeTargetsListControl")
          local selectedTarget = getSelectedTarget(listControl)
          if selectedTarget then
            local settingsNodePath = selectedTarget:getRuntimeTargetSettingsPath()
            local settingsNode = nmx.Application.new():getLocalUserSettings():getNodeFromPath(settingsNodePath)
            local previousSibling = settingsNode:getPreviousSibling()
            if previousSibling then
              settingsNode:reorder(previousSibling)
              updatePreferencePage(self:getParent())
            end
          end
        end)
      panel:addHSpacer(2)

      local moveDownButton = addImageButton(panel, "MoveDownButton", "movedown.png")
      moveDownButton:setOnClick(
        function(self)
          local listControl = panel:getChild("RuntimeTargetsListControl")
          local selectedTarget = getSelectedTarget(listControl)
          if selectedTarget then
            local settingsNodePath = selectedTarget:getRuntimeTargetSettingsPath()
            local settingsNode = nmx.Application.new():getLocalUserSettings():getNodeFromPath(settingsNodePath)
            local nextSibling = settingsNode:getNextSibling()
            if nextSibling then
              nextSibling:reorder(settingsNode)
              updatePreferencePage(self:getParent())
            end
          end
        end)
      panel:addHSpacer(2)
    panel:endSizer()

    local targetListControl = panel:addListControl{
      name = "RuntimeTargetsListControl",
      flags = "expand",
      size = {
        height = 200,
      },
      onSelectionChanged = onSelectionChanged,
    }

    panel:beginFlexGridSizer{ cols = 2, flags = "expand" }
      panel:setFlexGridColumnExpandable(2)

      panel:addStaticText{ text = "Name" }
      panel:addTextBox{
        name = "NameTextBox",
        flags = "expand",
        onEnter = function(self)
          local selectedTarget = getSelectedTarget(targetListControl)
          if selectedTarget then
            selectedTarget:setName(self:getValue())
            -- get the name after the set in case it was changed to be unique
            local name = selectedTarget:getName()
            updatePreferencePage(panel)

            local row = findTargetRow(targetListControl, selectedTarget)
            if row ~= nil then
              targetListControl:selectRow(row)
            end
          end
        end,
      }

      panel:addStaticText{ text = "Description" }
      panel:addTextBox{
        name = "DescriptionTextBox",
        flags = "expand",
        onEnter = function(self)
          local selectedTarget = getSelectedTarget(targetListControl)
          if selectedTarget then
            selectedTarget:setDescription(self:getValue())
          end
        end,
      }

      panel:addStaticText{ text = "IP address" }
      panel:beginHSizer{ flags = "expand", proportion = 1 }
        local ipAddressTextBox = panel:addTextBox{
          name = "IPAddressTextBox",
          flags = "expand",
          onEnter = function(self)
            local selectedTarget = getSelectedTarget(targetListControl)
            if selectedTarget then
              selectedTarget:setIPAddress(self:getValue())
            end
          end,
          size = {
            width = 100,
          }
        }

        panel:addButton{
          label = "...",
          onClick = function(self)
            local selectedTarget = getSelectedTarget(targetListControl)
            if selectedTarget then
              local ipAddress = ipAddressTextBox:getValue()
              local newIPAddress = ui.showConsoleSelectorDialog(ipAddress)

              if newIPAddress then
                selectedTarget:setIPAddress(newIPAddress)
                ipAddressTextBox:setValue(newIPAddress)
              end
            end
          end
        }
        panel:addHSpacer(16)

        local portToTextBox = nil
        panel:addStaticText{ text = "Port" }
        panel:addTextBox{
          name = "PortTextBox",
          flags = "numeric",
          onEnter = function(self)
            local selectedTarget = getSelectedTarget(targetListControl)
            if selectedTarget then
              selectedTarget:setPort(tonumber(self:getValue()))
            end
          end,
          size = {
            width = 40,
          },
          onChanged = function(self)
            updatePortToTextBox(panel, self, portToTextBox)
          end
        }
        portToTextBox = panel:addStaticText{ name = "PortUpToText", size = { width = 50 } }

        panel:addHSpacer(16)
        
        panel:addStaticText{ text = "Time out" }
        panel:addTextBox{
          name = "TimeOutTextBox",
          flags = "numeric",
          onEnter = function(self)
            local selectedTarget = getSelectedTarget(targetListControl)
            if selectedTarget then
              selectedTarget:setTimeOut(tonumber(self:getValue()))
            end
          end,
          size = {
            width = 40,
          },
        }
      panel:endSizer()

      panel:addStaticText{ text = "Asset compiler path" }
      panel:addFilenameControl{
        name      = "AssetCompilerPathControl",
        caption   = "Pick Asset Compiler Executable",
        wildcard  = "Executable files|exe",
        directory = app.getAppExecutableDir(),
        onMacroize = utils.macroizeString,
        onDemacroize = utils.demacroizeString,
        onChanged = function(self)
          local selectedTarget = getSelectedTarget(targetListControl)
          if selectedTarget then
            selectedTarget:setAssetCompilerPath(self:getValue())
            refreshAssetCompilerConfigs(panel, selectedTarget)
          end
        end,
        flags     = "expand",
      }
      
      panel:addStaticText{ text = "Asset compiler arguments" }
      panel:addTextBox{
        name = "AssetCompilerArgumentsTextBox",
        flags = "expand",
        onEnter = function(self)
          local selectedTarget = getSelectedTarget(targetListControl)
          if selectedTarget then
            selectedTarget:setAssetCompilerArguments(self:getValue())
          end
        end,
      }
      
      panel:addStaticText { text = "Asset compiler configuration" }
      panel:addComboBox{
        name = "AssetCompilerConfigComboBox",
        flags = "expand",
        onChanged = function(self)
          local selectedTarget = getSelectedTarget(targetListControl)
          selectedTarget:setAssetCompilerConfig(self:getValue())
        end
      }

      panel:addStaticText{ text = "Runtime target path" }
      panel:addFilenameControl{
        name      = "RuntimeTargetPathControl",
        caption   = "Pick Runtime Target Executable",
        wildcard  = "Executable files|exe",
        directory = app.getAppExecutableDir(),
        onMacroize = utils.macroizeString,
        onDemacroize = utils.demacroizeString,
        onChanged = function(self)
          local selectedTarget = getSelectedTarget(targetListControl)
          if selectedTarget then
            local path = self:getValue()
            selectedTarget:setRuntimeTargetPath(path)
            refreshTargetRunSettings(panel, string.len(path) ~= 0)
          else
            refreshTargetRunSettings(panel, false)
          end
        end,
        flags     = "expand",
      }

      panel:addStaticText{ text = "Runtime target arguments" }
      panel:addTextBox{
        name = "RuntimeTargetArgumentsTextBox",
        flags = "expand",
        onEnter = function(self)
          local selectedTarget = getSelectedTarget(targetListControl)
          if selectedTarget then
            selectedTarget:setRuntimeTargetArguments(self:getValue())
          end
        end,
      }

      panel:addStaticText{ text = "Show target window" }
      panel:addCheckBox{
        name = "ShowTargetWindowCheckBox",
        onChanged = function(self)
          local selectedTarget = getSelectedTarget(targetListControl)
          if selectedTarget then
            selectedTarget:setShowTargetWindow(self:getChecked())
          end
        end,
      }
    panel:endSizer()
  panel:endSizer()

  onSelectionChanged(targetListControl)
end

removePreferencesPage("RuntimeTargets")
addPreferencesPage(
  "RuntimeTargets",
  {
    title = "Runtime targets",
    create = buildPreferencePage,
    update = updatePreferencePage,
  }
)