------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/PreferencesEditor/PreferencesAPI.lua"
require "ui/NMXAttributeWidgets.lua"

-- forward declaration so it can be called recursively
local addPreferencesTreeControlItem = nil
------------------------------------------------------------------------------------------------------------------------
addPreferencesTreeControlItem = function(treeControl, name, page)
  assert(type(name) == "string")
  assert(string.len(name) > 0)

  assert(type(page) == "table")

  local parentItem = nil

  if page.parent then
    local preferencePages = getPreferencePages()
    local parentPage = preferencePages[page.parent]

    parentItem = treeControl:findItem(parentPage.title)
    if not parentItem then
      if not parentPage then
        return nil
      end

      parentItem = addPreferencesTreeControlItem(treeControl, page.parent, parentPage)
    end
  else
    parentItem = treeControl:getRoot()
  end

  if not parentItem then
    return nil
  end

  local item = parentItem:addChild(page.title)

  if page.collapse ~= nil and page.collapse == true then
    item:collapse()
  end

  item:setUserDataString(name)
  return item
end

------------------------------------------------------------------------------------------------------------------------
local buildPreferencesTreeControl = function(treeControl)
  local root = treeControl:getRoot()
  root:clearChildren()

  for name, page in pairs(getPreferencePages()) do
    local item = treeControl:findItem(page.title)

    if not item then
      item = addPreferencesTreeControlItem(treeControl, name, page)
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
local addPreferencesPages = function(switchablePanel)
  for name, page in pairs(getPreferencePages()) do
    if type(page.create) == "function" then
      local panel = switchablePanel:addPanel{
        name = name,
      }

      panel:beginVSizer{ proportion = 1, flags = "expand" }

        local scrollPanel = panel:addScrollPanel{
          name = "ScrollPanel",
          proportion = 1,
          flags = "both;expand",
        }

        page.create(scrollPanel, page)
        safefunc(page.update, scrollPanel, page)

      panel:endSizer()
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
-- adds a widget for manipulating a specific preference of type float
------------------------------------------------------------------------------------------------------------------------
preferences.addNumberWidget = function(panel, preference, minValue, maxValue)
  assert(panel)
  assert(type(preference) == "string")
  assert(string.len(preference) > 0)

  local _, info = preferences.get(preference)
  if not (info.type == "float" or info.type == "int") then
    -- preference is of the wrong type or doesn't exist
    return nil
  end

  local textBox = panel:addTextBox{
    name = string.format("%sTextBox", preference),
    flags = "numeric",
    onEnter = function(self)
      local _, info = preferences.get(preference)
      info.value = tonumber(self:getValue())

      if minValue ~= nil then
        if info.value < minValue then
          info.value = minValue
        end
      end
      if maxValue ~= nil then
        if info.value > maxValue then
          info.value = maxValue
        end
      end

      preferences.set(info)
      local clampedValue = preferences.get(preference)
      self:setValue(string.format("%g", clampedValue))
    end,
  }

  return textBox
end

------------------------------------------------------------------------------------------------------------------------
-- updates a widget created by calling preferences.addNumberWidget by getting the current value from the preferences
------------------------------------------------------------------------------------------------------------------------
preferences.updateNumberWidget = function(panel, preference)
  assert(panel)
  assert(type(preference) == "string")
  assert(string.len(preference) > 0)

  local textBoxName = string.format("%sTextBox", preference)
  local preferenceTextBox = panel:getChild(textBoxName)
  if preferenceTextBox then
    local value = preferences.get(preference)
    if type(value) == "number" then
      preferenceTextBox:setValue(string.format("%g", value))
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
-- adds a widget for manipulating a specific preference of type enum
------------------------------------------------------------------------------------------------------------------------
preferences.addEnumWidget = function(panel, preference)
  assert(panel)
  assert(type(preference) == "string")
  assert(string.len(preference) > 0)

  local _, info = preferences.get(preference)
  local comboBoxName = string.format("%sComboBox", preference)
  local preferenceComboBox = panel:addComboBox{
    name = comboBoxName,
    items = info.choices,
    flags = "expand",
    proportion = 1,
    onChanged = function(self)
      local _, info = preferences.get(preference)
      local selected = self:getSelectedItem()
      info.value = selected
      preferences.set(info)
    end,
  }

  return preferenceComboBox
end

------------------------------------------------------------------------------------------------------------------------
-- updates a widget created by calling preferences.addEnumWidget by getting the current value from the preferences
------------------------------------------------------------------------------------------------------------------------
preferences.updateEnumWidget = function(panel, preference)
  assert(panel)
  assert(type(preference) == "string")
  assert(string.len(preference) > 0)

  local comboBoxName = string.format("%sComboBox", preference)
  local preferenceComboBox = panel:getChild(comboBoxName)
  local value = preferences.get(preference)
  preferenceComboBox:setSelectedItem(value)
end

------------------------------------------------------------------------------------------------------------------------
-- shows the preferences dialog
------------------------------------------------------------------------------------------------------------------------
showPreferencesDialog = function(preferencePageToShow)
  local dlg = ui.getWindow("EditPreferencesDialog")

  -- hide the dialog and prepare it for showing next time.
  local hideDlg = function()
    dlg:hide()
    
    -- clear the panel n ow so next time all the components have been removed
    -- as clear in modal dialogs doesn't delete the components straight away.
    -- make sure we rebuild next time it with the same dimensions
    local size = dlg:getSize()
    dlg:clear()
    dlg:setSize(size)
    
  end

  if not dlg then
    local screenSize = ui.getDisplaySize()

    -- ensure the dialog isn't too big for the screen
    local size = {
      width = math.min(700, screenSize.width),
      height = math.min(700, screenSize.height),
    }

    dlg = ui.createModalDialog{
      name = "EditPreferencesDialog",
      caption = "Edit Preferences",
      size = size,
      onClose = function(self)
        hideDlg()
        preferences.cancelChange()
      end,
    }
  end

  dlg:setBorder(0)

  -- this panel is to force the tree control to resize
  local mainPanel = dlg:addPanel{ flags = "expand", proportion = 1 }
  mainPanel:freeze()

  mainPanel:beginVSizer()
    mainPanel:beginHSizer{ flags = "expand", proportion = 1 }
      local treeControl = mainPanel:addTreeControl{
        name = "PreferencesTreeControl",
        flags = "expand;hideRoot;sizeToContent",
      }
      buildPreferencesTreeControl(treeControl)

      local switchablePanel = mainPanel:addSwitchablePanel{
        name = "PreferencesSwitchablePanel",
        flags = "expand",
        proportion = 1,
      }
      addPreferencesPages(switchablePanel)

      treeControl:setOnSelectionChanged(
        function(self)
          local selection = self:getSelectedItem()
          if selection then
            local page = selection:getUserDataString()

            local panel = switchablePanel:findPanel(page)
            if panel then
              switchablePanel:setCurrentPanel(panel)
            end
          end
        end
      )
    mainPanel:endSizer()

    mainPanel:beginHSizer{ flags = "right" }
      mainPanel:addButton{
        label = "Ok",
        size = { width = 74 },
        onClick = function(self)
          local viewport = ui.getWindow("MainFrame|LayoutManager|AssetManager|AssetManagerViewport")
          viewport:rebuild3D();
          hideDlg()
          preferences.endChange()
        end,
      }

      mainPanel:addButton{
        label = "Apply",
        size = { width = 74 },
        onClick = function(self)
          local viewport = ui.getWindow("MainFrame|LayoutManager|AssetManager|AssetManagerViewport")
          viewport:rebuild3D();
          preferences.endChange()
          preferences.beginChange()
        end
      }

      mainPanel:addButton{
        label = "Cancel",
        size = { width = 74 },
        onClick = function(self)
          hideDlg()
          preferences.cancelChange()
        end,
      }
    mainPanel:endSizer()
  mainPanel:endSizer()

  mainPanel:rebuild()

  if type(preferencePageToShow) == "string" and string.len(preferencePageToShow) then
    local item = treeControl:findItem(preferencePageToShow)
    if item then
      treeControl:selectItem(item)

      local page = item:getUserDataString()
      local panel = switchablePanel:findPanel(page)
      switchablePanel:setCurrentPanel(panel)
    end
  end

  preferences.beginChange()

  dlg:show()
end

local attributeSpacing = 5

------------------------------------------------------------------------------------------------------------------------
-- builds the command command settings page
------------------------------------------------------------------------------------------------------------------------
buildNodeSettingsPage = function(panel, page)

  local roamingSettings = nmx.Application.new():getRoamingUserSettings()

  local settingsNode = roamingSettings:getNodeFromPath("|" .. page.nodeName)

  if settingsNode == nil then
    return
  end

  -- Add a title to the panel
  panel:beginVSizer{ flags = "expand", proportion = 1, }
    panel:addStaticText{
      name = "Heading",
      text = page.title,
      font = "bold",
      flags = "parentBackground;truncate;expand;decoration",
    }

    panel:beginFlexGridSizer{ flags = "expand", proportion = 1, cols = 2, }
    panel:setFlexGridColumnExpandable(2)

      if page.displayAllAttributes then
        local hiddenAttributes = {
          ["Tags"] = true,
          ["ExplicitWatch"] = true,
          ["Deletable"] = true,
          ["Reparentable"] = true,
        }

        local attributeCount = settingsNode:getNumElements()
        for attributeIndex = 0, attributeCount do
          local attribute = settingsNode:getAttribute(attributeIndex)

          local attributeName = attribute:getName()
          local attributeTypeId = attribute:getTypeId()
          if not hiddenAttributes[attributeName] and nmx.hasAttributeWidgetForType(attributeTypeId) then
            panel:addStaticText{ text = attribute:getDisplayName(), }
            nmx.addAttributeWidget(panel, attribute)
          end
        end
      else
        -- Add all the attributes for this node
        for k, v in pairs (page.attributes) do
          local attribute = settingsNode:findAttribute(v.attributeName)

          local attributeTypeId = attribute:getTypeId()
          if attribute and attribute:isValid() and nmx.hasAttributeWidgetForType(attributeTypeId) then
            panel:addStaticText{ text = v.displayName, }
            nmx.addAttributeWidget(panel, attribute)
          end
        end -- for loop
      end
    
    panel:endSizer()

  panel:endSizer()

end

------------------------------------------------------------------------------------------------------------------------
-- builds the command command settings page
------------------------------------------------------------------------------------------------------------------------
updateNodeSettingsPage = function(panel, page)
  -- Find the relevant settings node for this panel
  local nmxApp = nmx.Application.new()
  local roamingSettings = nmxApp:getRoamingUserSettings()

  local settingsNode = roamingSettings:getNodeFromPath("|" .. page.nodeName)
  if settingsNode == nil then
    app.error("Failed to find settings node " .. page.nodeName)
    return
  end

  -- Update the attributes
  if page.displayAllAttributes then
    local attributeCount = settingsNode:getNumElements()
    for attributeIndex = 0, attributeCount do
      nmx.updateAttributeWidget(panel, settingsNode:getAttribute(attributeIndex))
    end
  else
    for k, v in pairs (page.attributes) do
      nmx.updateAttributeWidget(panel, settingsNode:findAttribute(v.attributeName))
    end
  end
end