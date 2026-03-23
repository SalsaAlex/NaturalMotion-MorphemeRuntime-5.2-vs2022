------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
local kProductFeatures = {}

------------------------------------------------------------------------------------------------------------------------
-- get the typename for a node from its id
------------------------------------------------------------------------------------------------------------------------
local getNiceTypeName = function(id)
  return nmx.Application.new():lookupTypename(id)
end

------------------------------------------------------------------------------------------------------------------------
-- Preset Functions
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- query whether a set of the given index exists in the registry
------------------------------------------------------------------------------------------------------------------------
local getPresetContainer = function()
  local localUserSettings = nmx.Application.new():getRoamingUserSettings()
  local node = localUserSettings:getSetting("|Presets")

  if node ~= nil then
    return node
  end

  app.warning("Can't find the preset node")
  return nil
end

------------------------------------------------------------------------------------------------------------------------
-- get a set from the registry by index returning nil if it doesn't exist
------------------------------------------------------------------------------------------------------------------------
local findPresetNode = function(name)
  local presets = getPresetContainer()
  local child = presets:getFirstChild()
  while child ~= nil do
    if child:getName() == name then
      return child
    end
    child = child:getNextSibling()
  end
  return nil
end

------------------------------------------------------------------------------------------------------------------------
-- get a set from the registry by index returning nil if it doesn't exist
------------------------------------------------------------------------------------------------------------------------
local getPresetNode = function(index)
  local presets = getPresetContainer()
  local child = presets:getFirstChild()
  local childIndex = 1
  while child ~= nil do
    if childIndex == index then
      return child
    end
    child = child:getNextSibling()
    childIndex = childIndex + 1
  end
  return nil
end

local cbRef = -1

------------------------------------------------------------------------------------------------------------------------
-- begin a change block for the preset system
------------------------------------------------------------------------------------------------------------------------
local beginPresetChange = function()
  local localUserSettings = nmx.Application.new():getRoamingUserSettings()
  if cbRef ~= -1 then
    nmx.Application.new():logError("Only one presets change block should ever be in active.");
  end
  local status
  status, cbRef = localUserSettings:beginChangeBlock(getCurrentFileAndLine())
end

------------------------------------------------------------------------------------------------------------------------
-- end a change block for the preset system
------------------------------------------------------------------------------------------------------------------------
local endPresetChange = function()
  local localUserSettings = nmx.Application.new():getRoamingUserSettings()
  localUserSettings:endChangeBlock(cbRef, changeBlockInfo("End Preset Change"))
  cbRef = -1
end

------------------------------------------------------------------------------------------------------------------------
-- get a set from the registry by index returning nil if it doesn't exist
------------------------------------------------------------------------------------------------------------------------
local getPreset = function(index)
  local node = getPresetNode(index)

  if node == nil then
    return nil
  end

  return { label = node:getName(),
           camera = node:findAttribute("Camera"):asString(),
           selectionFilter = node:findAttribute("SelectionFilter"):asIntArray(),
           displayFilter = node:findAttribute("DisplayFilter"):asIntArray(),
           displayMode = node:findAttribute("DisplayMode"):asString(),
           colourMode = node:findAttribute("ColourMode"):asString() }
end

------------------------------------------------------------------------------------------------------------------------
-- add a set to the registry at the next available index
------------------------------------------------------------------------------------------------------------------------
local addPreset = function(label, settings, overwrite)
  -- label must be a string of greater than zero length
  if type(label) ~= "string" or string.len(label) == 0 then
    return false
  end

  -- settings must be a string containing valid lua code
  if type(settings) ~= "table" then
    return false
  end

  beginPresetChange()
    local presets = getPresetContainer()
    local preset

    if overwrite then
      preset = findPresetNode(label)
    end

    if preset == nil then
      preset = nmx.Application.new():getRoamingUserSettings():createNode(nmx.Application.new():lookupTypeId("PresetNode"), label, presets)
    end
    
    preset:findAttribute("Camera"):setString(settings.camera)
    preset:findAttribute("SelectionFilter"):setIntArray(settings.selectionFilter)
    preset:findAttribute("DisplayFilter"):setIntArray(settings.displayFilter)
    preset:findAttribute("DisplayMode"):setString(settings.displayMode)
    preset:findAttribute("ColourMode"):setString(settings.colourMode)
  endPresetChange()
  
  return true
end

------------------------------------------------------------------------------------------------------------------------
-- update a set in the registry at the index specified with new settings
------------------------------------------------------------------------------------------------------------------------
local editPreset = function(index, settings)
  -- settings must be a string containing valid lua code
  if type(settings) ~= "string" or string.len(settings) == 0 or loadstring(settings) == nil then
    return false
  end

  local preset = getPresetNode(index)

  if preset == nil then
    return
  end

  beginPresetChange()
    preset:findAttribute("Camera"):setString(settings.camera)
    preset:findAttribute("SelectionFilter"):setIntArray(settings.selectionFilter)
    preset:findAttribute("DisplayFilter"):setIntArray(settings.displayFilter)
    preset:findAttribute("DisplayMode"):setString(settings.displayMode)
    preset:findAttribute("ColourMode"):setString(settings.colourMode)
  endPresetChange()
end

------------------------------------------------------------------------------------------------------------------------
-- remove a preset from the registry by index
------------------------------------------------------------------------------------------------------------------------
local removePreset = function(index)
  local node = getPresetNode(index)

  if node == nil then
    return false
  end

  beginPresetChange()
    node:getDatabase():deleteNode(node, false)
  endPresetChange()
  return true
end

------------------------------------------------------------------------------------------------------------------------
-- showNewPresetDialog
------------------------------------------------------------------------------------------------------------------------
local showNewPresetDialog = function(settings)
  local dlg = ui.getWindow("CreatePreset")

  local labelTextBox
  local cameraTextBox, displayModeTextBox, colourModeTextBox
  local hiddenTypesListControl, frozenTypesListControl
  local addButton
  local numRows = 2

  if not dlg then
    dlg = ui.createModalDialog{
      parent = ui.getWindow("ManagePresets"),
      name = "CreatePreset",
      caption = "Add Preset",
      centre = true,
    }

    dlg:beginVSizer()
      dlg:beginHSizer{ flags = "expand" }
        dlg:addStaticText{ text = "Name" }
        labelTextBox = dlg:addTextBox{
          name = "LabelTextBox",
          proportion = 1,
        }
      dlg:endSizer()

      dlg:addStaticText{ text = "Settings", font = "bold" }

      dlg:beginFlexGridSizer{
        cols = 2,
        flags = "expand",
      }
        dlg:setFlexGridColumnExpandable(2)

        dlg:addStaticText{ text = "Active Camera" }

        cameraTextBox = dlg:addTextBox{
          name = "CameraTextBox",
          flags = "expand",
        }
        cameraTextBox:setReadOnly(true)

        dlg:addStaticText{ text = "Display Mode" }

        displayModeTextBox = dlg:addTextBox{
          name = "DisplayModeTextBox",
          flags = "expand",
        }
        displayModeTextBox:setReadOnly(true)

        dlg:addStaticText{ text = "Colour Mode" }

        colourModeTextBox = dlg:addTextBox{
          name = "ColourModeTextBox",
          flags = "expand",
        }
        colourModeTextBox:setReadOnly(true)

        dlg:addStaticText{ text = "Hidden Types" }

        hiddenTypesListControl = dlg:addListControl{
          name = "HiddenTypesListControl",
          numColumns = 1,
          numRows = numRows,
          flags = "expand"
        }

        dlg:addStaticText{ text = "Frozen Types" }

        frozenTypesListControl = dlg:addListControl{
          name = "FrozenTypesListControl",
          numColumns = 1,
          numRows = numRows,
          flags = "expand"
        }

      dlg:endSizer()

      dlg:beginHSizer{ flags = "right" }
        addButton = dlg:addButton{
          name = "AddButton",
          label = "Add",
          size = { width = 74 },
        }

        dlg:addButton{
          label = "Cancel",
          size = { width = 74 },
          onClick = function(this)
            dlg:hide()
          end,
        }
      dlg:endSizer()
    dlg:endSizer()

    dlg:setSize{ width = 250, height = -1 }
  else
    labelTextBox = dlg:getChild("LabelTextBox")
    labelTextBox:setValue("")

    cameraTextBox = dlg:getChild("CameraTextBox")
    displayModeTextBox = dlg:getChild("DisplayModeTextBox")
    colourModeTextBox = dlg:getChild("ColourModeTextBox")

    hiddenTypesListControl = dlg:getChild("HiddenTypesListControl")
    hiddenTypesListControl:clearRows()
    frozenTypesListControl = dlg:getChild("FrozenTypesListControl")
    frozenTypesListControl:clearRows()

    addButton = dlg:getChild("AddButton")

    -- they have all been cleared
    numRows = 0
  end

  cameraTextBox:setValue(settings.camera)
  displayModeTextBox:setValue(settings.displayMode)
  colourModeTextBox:setValue(settings.colourMode)

  local sf = nmx.SelectionFilter.new()
  local hiddenNodes = nmx.IntArray.new()
  sf:deserialise(settings.displayFilter)
  sf:getDisabledNodes(hiddenNodes)
  for i = 1, hiddenNodes:size() do
    if i <= numRows then
      hiddenTypesListControl:setRow(i, getNiceTypeName(hiddenNodes:at(i)))
    else
      hiddenTypesListControl:addRow(getNiceTypeName(hiddenNodes:at(i)))
   end
  end

  sf:deserialise(settings.selectionFilter)
  sf:getDisabledNodes(hiddenNodes)
  for i = 1, hiddenNodes:size() do
    if i <= numRows then
      frozenTypesListControl:setRow(i, getNiceTypeName(hiddenNodes:at(i)))
    else
      frozenTypesListControl:addRow(getNiceTypeName(hiddenNodes:at(i)))
    end
  end

  addButton:setOnClick(
    function(this)
      addPreset(labelTextBox:getValue(), settings)
      dlg:hide()
    end
  )

  dlg:show()
end

------------------------------------------------------------------------------------------------------------------------
-- addNewPresetDialog
------------------------------------------------------------------------------------------------------------------------
local addNewPresetDialog = function(settings)
  local dlg = ui.getWindow("AddPreset")
  local labelTextBox

  local presetNames = {}
  local index = 1
  local nameSet = {}
  while true do
    local set = getPreset(index)
    if set == nil then
      break
    end

    table.insert(presetNames, set.label)
    nameSet[set.label] = true;
    index = index + 1
  end
  
  -- Get a unique name for the new preset
  local untitledName = "Untitled"
  local i = 1
  while nameSet[untitledName] do
    untitledName = "Untitled" .. i
    i = i + 1
  end
  
  local saveButton, labelTextBox
  if not dlg then
    dlg = ui.createModalDialog{
      name = "SavePreset",
      caption = "Save Preset",
      centre = true,
    }

    dlg:beginVSizer()
      dlg:beginHSizer{ flags = "expand" }
        dlg:addStaticText{ text = "Name" }
        labelTextBox = dlg:addComboBox{ name = "LabelTextBox", flags = "expand", proportion = 1 }
        labelTextBox:setEditable(true)
      dlg:endSizer()

      dlg:beginHSizer{ flags = "right" }
        saveButton = dlg:addButton{ name = "SaveButton", label = "Save", size = { width = 74 } }
        dlg:addButton{label = "Cancel", size = { width = 74 },
          onClick = function(this)
            dlg:hide()
          end,
        }
      dlg:endSizer()
    dlg:endSizer()

    dlg:setSize{ width = 280, height = -1 }
  else
    labelTextBox = dlg:getChild("LabelTextBox")
    saveButton = dlg:getChild("SaveButton")
  end

  labelTextBox:setItems(presetNames)
  labelTextBox:setValue(untitledName)
  
  saveButton:setOnClick(
    function(this)
      addPreset(labelTextBox:getValue(), settings, true)
      dlg:hide()
    end
  )

  dlg:show()
end

------------------------------------------------------------------------------------------------------------------------
-- rebuildPresetsList
------------------------------------------------------------------------------------------------------------------------
local rebuildPresetsList = function(listControl)
  listControl:clearRows()

  -- find the next free set index
  local index = 1
  while true do
    local set = getPreset(index)

    if set == nil then
      break
    end

    listControl:addRow{ set.label }
    index = index + 1
  end
end

------------------------------------------------------------------------------------------------------------------------
-- showEditPresetsDialog
------------------------------------------------------------------------------------------------------------------------
showEditPresetsDialog = function(viewport)
  local dlg = ui.getWindow("ManagePresets")
  local listControl = nil
  if dlg == nil then
    dlg = ui.createModelessDialog{
      name = "ManagePresets",
      caption = string.format("Edit Presets"),
      resize = true,
      centre = true,
      size = { width = 300, height = 250 }
    }

    dlg:beginVSizer()
      listControl = dlg:addListControl{
        name = "PresetsList",
        flags = "expand",
        proportion = 1,
        columnNames = {
          "Preset"
        }
      }
      
      listControl:showColumnHeader(false)
      dlg:beginHSizer{ flags = "right" }
      
        --[[
        dlg:addButton{
          label = "Add",
          size = { width = 74 },
          onClick = function(this)
            local settings = { }
            settings.camera = viewport:getCamera()
            settings.displayMode = viewport:getDisplayMode()
            settings.colourMode = viewport:getColourMode()

            settings.displayFilter = nmx.IntArray.new()
            settings.selectionFilter = nmx.IntArray.new()
            viewport:getShownObjectFilter():serialise(settings.displayFilter)
            viewport:getSelectableObjectFilter():serialise(settings.selectionFilter)

            showNewPresetDialog(settings)
            rebuildPresetsList(listControl)
          end,
        }
        ]]--
        
        dlg:addButton{
          label = "Delete",
          size = { width = 74 },
          onClick = function(this)
            local selected = listControl:getSelectedRow()
            removePreset(selected)
            rebuildPresetsList(listControl)
          end,
        }
        dlg:addButton{
          label = "Done",
          size = { width = 74 },
          onClick = function(this)
            dlg:hide()
          end,
        }
      dlg:endSizer()
    dlg:endSizer()
  else
    listControl = dlg:getChild("PresetsList")
  end

  rebuildPresetsList(listControl)

  dlg:show()
end

------------------------------------------------------------------------------------------------------------------------
-- Viewport Preset menu
------------------------------------------------------------------------------------------------------------------------
addPresetMenu = function(menubar, viewport, scene, unselectableNodes)
  local buildPresetMenu = function(menu)
  
    menu:addItem{
      label = "Save Settings as Preset...",
      onClick = function()
        local settings = { }
        settings.camera = viewport:getCamera()
        settings.displayMode = viewport:getDisplayMode()
        settings.colourMode = viewport:getColourMode()

        settings.displayFilter = nmx.IntArray.new()
        viewport:getShownObjectFilter():serialise(settings.displayFilter)
       
        settings.selectionFilter = nmx.IntArray.new()
        viewport:getSelectableObjectFilter():serialise(settings.selectionFilter)

        addNewPresetDialog(settings)
      end
    }

    menu:addItem{
      label = "Edit Presets...",
      onClick = function()
        showEditPresetsDialog(viewport)
      end
    }
    
    -- find the next free set index
    local index = 1
    while true do
      local set = getPreset(index)
      if set == nil then
        break
      end

      if index == 1 then  
        menu:addSeparator() 
      end

      menu:addItem{
        label = set.label,
        onClick = function(self)
          viewport:setCamera(set.camera)
          viewport:setDisplayMode(set.displayMode)
          viewport:setColourMode(set.colourMode)

          local filter = nmx.SelectionFilter.new()
          filter:deserialise(set.displayFilter)

          viewport:setShownObjectFilter(filter)
          viewport:getSelectableObjectFilter():deserialise(set.selectionFilter)
          if unselectableNodes then
            for _, node in unselectableNodes do
              viewport:setShouldSelectType(nmx.Application.new():lookupTypeId(node), false)
            end
          end
        end,
      }

      index = index + 1
    end
  end

  local presetMenu = menubar:addSubMenu{
    name = "PresetMenu",
    label = "Presets",
    onPoppedUp = function(self)
      self:clear()
      buildPresetMenu(self)
    end,
  }

  buildPresetMenu(presetMenu)
end

------------------------------------------------------------------------------------------------------------------------
-- Viewport Functions
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- helper function to set an nmx2 enum attribute via a string value
------------------------------------------------------------------------------------------------------------------------
local setEnumAttributeByString = function(attribute, value)
  assert(attribute ~= nil)
  assert(type(attribute) == "userdata")
  assert(attribute:isValid())
  assert(type(value) == "string")
  assert(string.len(value) > 0)

  local choices = attribute:getChoices()
  local count = choices:size()
  for i = 1, count do
    local choice = choices:at(i)

    if choice == value then
      local database = attribute:getDatabase()

      local status, cbRef = database:beginChangeBlock(getCurrentFileAndLine())

      local result = attribute:setInt(i - 1)

      local description = string.format(
        "setEnumAttributeByString(%q, %q)",
        attribute:getName(),
        value)
        
      database:endChangeBlock(cbRef, changeBlockInfo(description))
      return result
    end
  end

  return false
end

------------------------------------------------------------------------------------------------------------------------
local getActiveNavigator = function()
  local currentContext = ui.getCurrentContextName()
  if currentContext == "PreviewViewport" then
    return ui.getWindow("MainFrame|LayoutManager|Navigator|ViewportSceneExplorer")
  elseif currentContext == "AssetManagerViewport" then
    return ui.getWindow("MainFrame|LayoutManager|Navigator|AssetSceneExplorer")
  end
end

------------------------------------------------------------------------------------------------------------------------
local upArrowAccelerator = function()
  getActiveNavigator():navigateSelectionToParents()
end

------------------------------------------------------------------------------------------------------------------------
local downArrowAccelerator = function()
  getActiveNavigator():navigateSelectionToChildren()
end

------------------------------------------------------------------------------------------------------------------------
local leftArrowAccelerator = function()
  getActiveNavigator():navigateSelectionToPreviousSiblings()
end

------------------------------------------------------------------------------------------------------------------------
local rightArrowAccelerator = function()
  getActiveNavigator():navigateSelectionToNextSiblings()
end

--[[
------------------------------------------------------------------------------------------------------------------------
-- Add viewport accelerators
------------------------------------------------------------------------------------------------------------------------
local addArrowAccelerators = function(viewport)
  --viewport:addAccelerator("Shift+Up", upArrowAccelerator)
  viewport:addAccelerator("Shift+Down", downArrowAccelerator)
  viewport:addAccelerator("Shift+Left", leftArrowAccelerator)
  viewport:addAccelerator("Shift+Right", rightArrowAccelerator)
end
--]]

------------------------------------------------------------------------------------------------------------------------
-- Viewport Edit menu
------------------------------------------------------------------------------------------------------------------------
addEditMenu = function(menubar, scene, viewport, cutCopyPaste, readOnlyFn, pasteRoot, extraToolsFn)
  local editMenu = menubar:addSubMenu{
    name = "EditMenu",
    label = "Edit",
    onPoppedUp = function(self)
      if readOnlyFn ~= nil then
        self:enable(not readOnlyFn())
      else
        self:enable(true)
      end
    end
  }

  editMenu:addItem{
    label = "Undo",
    accelerator = "Ctrl+Z",
    onClick = function()
      scene:undo()
    end,
    onPoppedUp = function(menuItem)
      if readOnlyFn ~= nil then
        menuItem:enable(not readOnlyFn() and scene:canUndo())
      else
        menuItem:enable(true)
      end
    end,
  }

  editMenu:addItem{
    label = "Redo",
    accelerator = "Ctrl+Y",
    onClick = function()
      scene:redo()
    end,
    onPoppedUp = function(menuItem)
      if readOnlyFn ~= nil then
        menuItem:enable(not readOnlyFn() and scene:canRedo())
      else
        menuItem:enable(true)
      end
    end,
  }

  if cutCopyPaste then
    editMenu:addSeparator()

    editMenu:addItem{
      label = "Copy",
      accelerator = "Ctrl+C",
      onClick = function() nmx.Application.new():runCommand("Connect Core", "Copy", scene) end,
    }

    editMenu:addItem{
      label = "Paste",
      accelerator = "Ctrl+V",
      onClick = function()
        local status, cbRef = scene:beginChangeBlock(getCurrentFileAndLine())
        if type(pasteRoot) == "string" then
          local sl = nmx.SelectionList.new()
          local node = scene:getNodeFromPath(pasteRoot)
          sl:add(node)
          scene:setSelectionList(sl)
        end
        nmx.Application.new():runCommand("Connect Core", "Paste", scene)
        scene:endChangeBlock(cbRef, changeBlockInfo("Paste"))
      end,
    }

  end

  editMenu:addSeparator()

  editMenu:addCheckedItem{
    label = "Local Tool Axis",
    accelerator = "X",
    onClick = function(self)
      viewport:setFocus()
      viewport:applyToolCommand(nmx.ToolCommands.kToggleLocalToolAxis)
    end,
    onPoppedUp = function(menuItem)
      menuItem:setChecked(viewport:isCurrentToolLocalAlign())
    end
  }

  editMenu:addCheckedItem{
    label = "Show Manipulators",
    accelerator = "R",
    onClick = function()
      viewport:setCurrentTool("ShowManipsTool")
    end,
    onPoppedUp = function(menuItem)
      menuItem:setChecked(viewport:getCurrentTool() == "ShowManipsTool")
    end
  }

  editMenu:addSeparator()

  editMenu:addCheckedItem{
    label = "Select Tool",
    accelerator = "Q",
    onClick = function()
      viewport:setFocus()
      viewport:setCurrentTool("SelectTool")
    end,
    onPoppedUp = function(menuItem)
      menuItem:setChecked(viewport:getCurrentTool() == "SelectTool")
    end
  }

  editMenu:addCheckedItem{
    label = "Move Tool",
    accelerator = "W",
    onClick = function()
      viewport:setFocus()
      viewport:setCurrentTool("TranslateTool")
    end,
    onPoppedUp = function(menuItem)
      menuItem:setChecked(viewport:getCurrentTool() == "TranslateTool")
    end
  }

  editMenu:addCheckedItem{
    label = "Rotate Tool",
    accelerator = "E",
    onClick = function()
      viewport:setFocus()
      viewport:setCurrentTool("RotateTool")
    end,
    onPoppedUp = function(menuItem)
      menuItem:setChecked(viewport:getCurrentTool() == "RotateTool")
    end
  }

  if extraToolsFn then
    extraToolsFn(editMenu)
  end

  editMenu:addSeparator()

  editMenu:addCheckedItem{
    label = "Snap to Points",
    accelerator = "V", 
    onClick = function()
      viewport:setSnapToPoints(not viewport:getSnapToPoints())
    end,
    onPoppedUp = function(menuItem)
      menuItem:setChecked(viewport:getSnapToPoints())
    end
  }
  
  editMenu:addCheckedItem{
    label = "Snap Positions to Surfaces",
    accelerator = "", 
    onClick = function()
      viewport:setSnapToSurfaces(not viewport:getSnapToSurfaces())
    end,
    onPoppedUp = function(menuItem)
      menuItem:setChecked(viewport:getSnapToSurfaces())
    end
  }
  
  editMenu:addCheckedItem{
    label = "Snap Positions and Rotations to Surfaces",
    accelerator = "", 
    onClick = function()
      viewport:setSnapToSurfacesWithOrientation(not viewport:getSnapToSurfacesWithOrientation())
    end,
    onPoppedUp = function(menuItem)
      menuItem:setChecked(viewport:getSnapToSurfacesWithOrientation())
    end
  }
  
  editMenu:addSeparator()

  editMenu:addItem{
    label = "Clear Selection",
    onClick = function()
      scene:clearSelection()
    end,
  }

  editMenu:addItem{
    label = "Select Parent",
    accelerator = "Shift+Up",
    onClick = upArrowAccelerator,
  }

  editMenu:addItem{
    label = "Select Child",
    accelerator = "Shift+Down",
    onClick = downArrowAccelerator,
  }

  editMenu:addItem{
    label = "Select Previous",
    accelerator = "Shift+Left",
    onClick = leftArrowAccelerator,
  }

  editMenu:addItem{
    label = "Select Next",
    accelerator = "Shift+Right",
    onClick = rightArrowAccelerator,
  }
  
  viewport:addAccelerator("Shift+Up", upArrowAccelerator)
  viewport:addAccelerator("Shift+Down", downArrowAccelerator)
  viewport:addAccelerator("Shift+Left", leftArrowAccelerator)
  viewport:addAccelerator("Shift+Right", rightArrowAccelerator)

  editMenu:addSeparator()

  editMenu:addItem{
    label = "Larger Tools",
    accelerator = "+",
    onClick = function()
      viewport:applyToolCommand(nmx.ToolCommands.kEnlargeTools)
    end,
  }

  editMenu:addItem{
    label = "Smaller Tools",
    accelerator = "-",
    onClick = function()
      viewport:applyToolCommand(nmx.ToolCommands.kShrinkTools)
    end,
  }

  return editMenu
end

------------------------------------------------------------------------------------------------------------------------
-- Viewport View menu
------------------------------------------------------------------------------------------------------------------------
local addViewMenu = function(viewportPanel, additionalFrameMenuItems)
  local scene = viewportPanel:getScene()
  local menubar = viewportPanel:getMenuBar()
  local viewport = viewportPanel:getViewport()

  local viewmenu = menubar:addSubMenu{ name = "ViewMenu", label = "View" }

  local addCameraMenuItem = function(itemName, camera)
    viewmenu:addCheckedItem{
      label = itemName,
      onClick = function()
        viewport:setCamera(camera)
      end,
      onPoppedUp = function(menuItem)
        menuItem:setChecked(viewport:getCamera() == camera)
      end
    }
  end

  -- Build the menu
  addCameraMenuItem("Perspective", "|LogicalRoot|Cameras|PerspectiveCameraTransform|PerspectiveCamera")
  addCameraMenuItem("Front", "|LogicalRoot|Cameras|FrontCameraTransform|FrontCamera")
  addCameraMenuItem("Side", "|LogicalRoot|Cameras|SideCameraTransform|SideCamera")
  addCameraMenuItem("Top", "|LogicalRoot|Cameras|TopCameraTransform|TopCamera")

  viewmenu:addSeparator()
 
  viewmenu:addItem{
    label = "Frame",
    accelerator = "F",
    onClick = function()
      local scene = nmx.Application.new():getSceneByName(viewport:getSceneName())
      local sl = nmx.SelectionList.new()
      scene:getSelectionList(sl)
      if sl:empty() then
        viewport:frameCharacter(1.6)
      else
        viewport:frame()
      end
      
      -- Force a compute incase we are inside a change block
      scene:update()
    end
  }

  if additionalFrameMenuItems ~= nil then
    additionalFrameMenuItems(viewmenu, viewportPanel)
  end
  
  viewmenu:addSeparator()

  viewmenu:addItem{
    label = "Reset Camera",
    onClick = function()
      viewport:resetCurrentCamera()
    end
  }

  viewmenu:addSeparator()

  local followmenu = viewmenu:addSubMenu{ name = "FollowJoint", label = "Follow Joint" }
  
  local getSelectedAnimSet = function (sceneName)
    if sceneName == "Network" then
      return getSelectedAnimSet()
    end
    return getSelectedAssetManagerAnimSet()
  end

  followmenu:addCheckedItem{
    label = "None",
    onClick = function()
      local scene = viewport:getSceneName()
      local animSet = getSelectedAnimSet(scene)
      anim.setAnimSetFollowJoint(animSet, scene, "")
    end,
    onPoppedUp = function(menuItem)
      local scene = viewport:getSceneName()
      local animSet = getSelectedAnimSet(scene)
      menuItem:setChecked(anim.getAnimSetFollowJoint(animSet, scene) == "")
    end
  }

  followmenu:addCheckedItem{
    label = "Trajectory",
    onClick = function()
      local scene = viewport:getSceneName()
      local animSet = getSelectedAnimSet(scene)
      local markup = anim.getRigMarkupData(animSet)
      local trajectory = anim.getAnimChannelName(markup.trajectoryIndex, animSet)
      anim.setAnimSetFollowJoint(animSet, scene, trajectory)
    end,
    onPoppedUp = function(menuItem)
      local scene = viewport:getSceneName()
      local animSet = getSelectedAnimSet(scene)
      local markup = anim.getRigMarkupData(animSet)
      local trajectory = anim.getAnimChannelName(markup.trajectoryIndex, animSet)
      menuItem:setChecked(anim.getAnimSetFollowJoint(animSet, scene) == trajectory)
    end
  }

  followmenu:addCheckedItem{
    label = "Character Root",
    onClick = function()
      local scene = viewport:getSceneName()
      local animSet = getSelectedAnimSet(scene)
      local markup = anim.getRigMarkupData(animSet)
      local hip = anim.getAnimChannelName(markup.hipIndex, animSet)
      anim.setAnimSetFollowJoint(animSet, scene, hip)
    end,
    onPoppedUp = function(menuItem)
      local scene = viewport:getSceneName()
      local animSet = getSelectedAnimSet(scene)
      local markup = anim.getRigMarkupData(animSet)
      local hip = anim.getAnimChannelName(markup.hipIndex, animSet)
      menuItem:setChecked(anim.getAnimSetFollowJoint(animSet, scene) == hip)
    end
  }

  if mcn ~= nil
  then
    addFollowJointItem(followmenu, scene, viewport, "Select...")
  end

  followmenu:addSeparator()

  followmenu:addCheckedItem{
    label = "Orientation",
    onClick = function()
      local scene = viewport:getSceneName()
      local animSet = getSelectedAnimSet(scene)
      anim.setAnimSetFollowOrient(animSet, scene, not anim.getAnimSetFollowOrient(animSet, scene))
    end,
    onPoppedUp = function(menuItem)
      local scene = viewport:getSceneName()
      local animSet = getSelectedAnimSet(scene)
      menuItem:setChecked(anim.getAnimSetFollowOrient(animSet, scene))
    end
  }

  viewmenu:addSeparator()
  local cullmenu = viewmenu:addSubMenu{ name = "FaceCulling", label = "Cull Face" }

  cullmenu:addCheckedItem{
    label = "Cull None",
    onClick = function()
      viewport:setFaceCulling("None")
    end,
    onPoppedUp = function(menuItem)
      menuItem:setChecked(viewport:getFaceCulling() == "None")
    end
  }
  cullmenu:addCheckedItem{
    label = "Cull Back",
    onClick = function()
      viewport:setFaceCulling("Back")
    end,
    onPoppedUp = function(menuItem)
      menuItem:setChecked(viewport:getFaceCulling() == "Back")
    end
  }
  cullmenu:addCheckedItem{
    label = "Cull Front",
    onClick = function()
      viewport:setFaceCulling("Front")
    end,
    onPoppedUp = function(menuItem)
      menuItem:setChecked(viewport:getFaceCulling() == "Front")
    end
  }

  return viewmenu
end

------------------------------------------------------------------------------------------------------------------------
-- Viewport Display menu
------------------------------------------------------------------------------------------------------------------------
local addDisplayMenu = function(viewportPanel, colourModes)
  local scene = viewportPanel:getScene()
  local menubar = viewportPanel:getMenuBar()
  local viewport = viewportPanel:getViewport()

  local displaymenu = menubar:addSubMenu{ name = "DisplayMenu", label = "Display" }

  local addDisplayModeMenuItem = function(name, displayMode)
    displaymenu:addCheckedItem{
      label = name,
      onClick = function()
        viewport:setDisplayMode(displayMode)
      end,
      onPoppedUp = function(menuItem)
        menuItem:setChecked(viewport:getDisplayMode() == displayMode)
      end
    }
  end

  local addObjectColourMenuItem = function(name, colourMode)
    displaymenu:addCheckedItem{
      label = name,
      onClick = function()
       viewport:setColourMode(colourMode)
      end,
      onPoppedUp = function(menuItem)
        menuItem:setChecked(viewport:getColourMode() == colourMode)
      end
    }
  end

  -- Build the menu
  addDisplayModeMenuItem("Normal", "Shaded")
  addDisplayModeMenuItem("X-Ray", "XRay")
  addDisplayModeMenuItem("Wireframe", "Wireframe")

  displaymenu:addSeparator()
  
  if colourModes then
    for i, v in colourModes do
      addObjectColourMenuItem(v.MenuName, v.ColourMode)
    end
  end
  
  return dispaymenu
end

------------------------------------------------------------------------------------------------------------------------
function deepcopy(object)
  local lookup_table = {}
  local function _copy(object)
    if type(object) ~= "table" then
      return object
    elseif lookup_table[object] then
      return lookup_table[object]
    end
    local new_table = {}
    lookup_table[object] = new_table
    for index, value in pairs(object) do
      new_table[_copy(index)] = _copy(value)
    end
    return new_table
  end
  return _copy(object)
end

------------------------------------------------------------------------------------------------------------------------
local copyViewMenuItems = function(mainList)

  local result = {}
  for _, section in mainList do
    if (section.availability == nil) or kProductFeatures[section.availability] then
    
      local newSection = { name = section.name, items = {} }
      for _, item in section.items do
        if (item.availability == nil) or kProductFeatures[item.availability] then
          table.insert(newSection.items, deepcopy(item))
        end
      end
      
      table.insert(result, newSection)
    end
  end
  
  return result
end

------------------------------------------------------------------------------------------------------------------------
local copySelectionItems = function(mainList, nonSelectableNodes)

  local nodeSet = { }
  for _, node in nonSelectableNodes do
    nodeSet[node] = true
  end

  local result = {}
  for _, section in mainList do
    if (section.availability == nil) or kProductFeatures[section.availability] then
      local newSection = { name = section.name, items = {} }
      for _, item in section.items do
        if (item.availability == nil) or kProductFeatures[item.availability] then
          if item.nodes ~= nil then
            local newItem = { name = item.name, nodes = {} }
            for _, node in item.nodes do
              if nodeSet[node] then
                -- do nothing
              else
                table.insert(newItem.nodes, node)
              end
            end
            if table.getn(newItem.nodes) > 0 then
              table.insert(newSection.items, newItem)
            end
          end
        end
      end
      
      if table.getn(newSection.items) > 0 then
        table.insert(result, newSection)
      end
    end
  end
  
  return result
end

------------------------------------------------------------------------------------------------------------------------
local getVisible = function(viewport, item)
  if item.get then
    return item.get(viewport)
  elseif item.nodes then
    local app = nmx.Application.new()
    local classTypeId = app:lookupTypeId(item.nodes[1])
    return viewport:getShouldRenderType(classTypeId)
  end

  return false
end

------------------------------------------------------------------------------------------------------------------------
local setVisible = function(viewport, item, visible)
  if item.set then
    item.set(viewport, visible)
  elseif item.nodes then
    local app = nmx.Application.new()
    for _, node in item.nodes do
      local classTypeId = app:lookupTypeId(node)
      viewport:setShouldRenderType(classTypeId, visible)
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
local getShouldSelect = function(viewport, item)
  if not item.get and item.nodes then
    local app = nmx.Application.new()
    local classTypeId = app:lookupTypeId(item.nodes[1])
    return viewport:getShouldSelectType(classTypeId)
  end

  return false
end

------------------------------------------------------------------------------------------------------------------------
local setShouldSelect = function(viewport, item, visible)
  if not item.set and item.nodes then
    local app = nmx.Application.new()
    for _, node in item.nodes do
      local classTypeId = app:lookupTypeId(node)
      viewport:setShouldSelectType(classTypeId, visible)
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
local showNodes = function(viewport, items, visible)
  if items then
    for _, item in pairs(items) do
      setVisible(viewport, item, visible)
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
local addShowMenuItem = function(menu, viewport, info)
  local newItem
  if info.subMenu then
    menu:addSubMenu{
      label = info.name,
      
      onClick = function()
        setVisible(viewport, info, not getVisible(viewport, info))
      end,
      
      onPoppedUp = function(menuItem)
        menuItem:setChecked(getVisible(viewport, info))
      end,
    }
  else
    newItem = menu:addCheckedItem{
      label = info.name,
      
      onClick = function()
        setVisible(viewport, info, not getVisible(viewport, info))
      end,
      
      onPoppedUp = function(menuItem)
        menuItem:setChecked(getVisible(viewport, info))
      end
    }
  end
  
  return newItem
end

------------------------------------------------------------------------------------------------------------------------
local addShouldSelectMenuItem = function(menu, viewport, info)
  local newItem = menu:addCheckedItem{
    label = info.name,
    
    onClick = function()
      setShouldSelect(viewport, info, not getShouldSelect(viewport, info))
    end,
    
    onPoppedUp = function(menuItem)
      menuItem:setChecked(getShouldSelect(viewport, info))
    end
  }
  
  return newItem
end

------------------------------------------------------------------------------------------------------------------------
local areAllChecked = function(items)
  local value = nil
  for _, item in items do
    if item.menuItem then
      if value == nil then
        value = item.menuItem:isChecked()
      elseif value ~= item.menuItem:isChecked() then
        return false
      end
    end
  end
  
  return value
end

------------------------------------------------------------------------------------------------------------------------
local addShowAllItem = function(menu, viewport, name, allNodes, visible)
  local item = menu:addItem{label = name}
  
  -- onClick
  item:setOnClick(function(menuItem)
    for _, section in allNodes do
      for _, item in section.items do
        setVisible(viewport, item, visible)
      end
    end
  end)
end  

------------------------------------------------------------------------------------------------------------------------
local addShowMenuSection = function(menu, viewport, name, items)
  local titleItem = menu:addItem{ label = name, style = "subtitle" }
  
  -- onPoppedUp
  titleItem:setOnPoppedUp(function(menuItem)
    menuItem:setChecked(false)
  end)

  -- onItemMouseEnter
  titleItem:setOnItemMouseEnter(function(menuItem)
    local allChecked = areAllChecked(items)
    for _, item in items do
      if item.menuItem then
        item.wasChecked = item.menuItem:isChecked()
        item.menuItem:setMarkColumnSelected(true)
      end
    end
  end)
    
  -- onItemMouseLeave
  titleItem:setOnItemMouseLeave(function(menuItem)
    for _, item in items do
      if item.menuItem then
        item.menuItem:setMarkColumnSelected(false)
        item.menuItem:setChecked(item.wasChecked)
      end
    end
  end)
  
  -- onClick
  titleItem:setOnClick(function(menuItem)
    for _, item in items do
      if item.menuItem then
        item.menuItem:setChecked(item.wasChecked)
      end
    end
    local app = nmx.Application.new()
    local allChecked = areAllChecked(items)
    for _, item in items do
      if item.menuItem then
        setVisible(viewport, item, not allChecked)
      end
    end
  end)
    
  -- add Items in the section
  for _, item in items do
    item.menuItem = addShowMenuItem(menu, viewport, item)
  end
end

------------------------------------------------------------------------------------------------------------------------
local addSelectMenuSection = function(menu, viewport, name, items)
  local titleItem = menu:addItem{ label = name, style = "subtitle" }

  -- onPoppedUp
  titleItem:setOnPoppedUp(function(menuItem)
    menuItem:setChecked(false)
  end)

  -- onItemMouseEnter
  titleItem:setOnItemMouseEnter(function(menuItem)
    local allChecked = areAllChecked(items)
    for _, item in items do
      if item.menuItem then
        item.wasChecked = item.menuItem:isChecked()
        item.menuItem:setMarkColumnSelected(true)
      end
    end
  end)
  
  -- onItemMouseLeave
  titleItem:setOnItemMouseLeave(function(menuItem)
    for _, item in items do
      if item.menuItem then
        item.menuItem:setMarkColumnSelected(false)
        item.menuItem:setChecked(item.wasChecked)
      end
    end
  end)

  -- onClick
  titleItem:setOnClick(function(menuItem)
    for _, item in items do
      if item.menuItem then
        item.menuItem:setChecked(item.wasChecked)
      end
    end
    local app = nmx.Application.new()
    local allChecked = areAllChecked(items)
    for _, item in items do
      if item.menuItem then
        setShouldSelect(viewport, item, not allChecked)
      end
    end
  end)
    
  for _, item in items do
    item.menuItem = addShouldSelectMenuItem(menu, viewport, item)
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Viewport Show menu
------------------------------------------------------------------------------------------------------------------------
local addShowMenu = function(viewportPanel, menuItems)

  local menuItems = copyViewMenuItems(menuItems)
  local scene = viewportPanel:getScene()
  local menubar = viewportPanel:getMenuBar()
  local viewport = viewportPanel:getViewport()
  
  -- Create the Show menu
  local menu = menubar:addSubMenu{ name = "ShowMenu", label = "Show" }
  
  -- Show All
  addShowAllItem(menu, viewport, "Show All", menuItems, true)

  -- Hide All
  addShowAllItem(menu, viewport, "Hide All", menuItems, false)
  
  addShowMenuItem(menu, viewport, { name = "Grid",
        get = function(viewport) return viewport:getShouldRenderGrid() end,
        set = function(viewport, val) viewport:setShouldRenderGrid(val) end
      })

  -- Add the sections
  for _, section in menuItems do
    addShowMenuSection(menu, viewport, section.name, section.items)
  end

  return menu
end

------------------------------------------------------------------------------------------------------------------------
-- Viewport Selection menu
------------------------------------------------------------------------------------------------------------------------
local addSelectionMenu = function(viewportPanel, menuItems, unselectableNodes)
  local menubar = viewportPanel:getMenuBar()
  local viewport = viewportPanel:getViewport()
  
  if unselectableNodes == nil
  then
    unselectableNodes = {}
  end

  local menuItems = copySelectionItems(menuItems, unselectableNodes)
  local menu = menubar:addSubMenu{ name = "SelectionMenu", label = "Selection" }

  menu:addCheckedItem{
    label = "Lock Selection",
    -- accelerator = "Space", maybe space is a bit to big for this shortcut... but it should probably have an accelerator...
    onClick = function()
      viewport:setLockSelection(not viewport:getLockSelection())
    end,
    onPoppedUp = function(menuItem)
      menuItem:enable(not viewportPanel:isSceneReadOnly())
      menuItem:setChecked(viewport:getLockSelection())
    end
  }
  menu:addCheckedItem{
    label = "Click Through Selection",
    onClick = function()
      viewport:setClickThroughSelection(not viewport:getClickThroughSelection())
    end,
    onPoppedUp = function(menuItem)
      menuItem:setChecked(viewport:getClickThroughSelection())
    end
  }
  
  for _, section in menuItems do
    addSelectMenuSection(menu, viewport, section.name, section.items)
  end

  return menu
end

setToDefault = function(viewport, defaultPreset, unselectableNodes)
  if viewport ~= nil then
    viewport:setCamera("|LogicalRoot|Cameras|PerspectiveCameraTransform|PerspectiveCamera")
    viewport:setDisplayMode("XRay")
    viewport:setColourMode("Object")
    viewport:setShouldRenderAllTypes(true)

    -- try to restore the default preset
    local index = 1
    local set = getPreset(index)
    while set ~= nil do
      if set.label == defaultPreset then
        viewport:setCamera(set.camera)
        viewport:setDisplayMode(set.displayMode)
        viewport:setColourMode(set.colourMode)

        local filter = nmx.SelectionFilter.new()
        filter:deserialise(set.displayFilter)

        viewport:setShownObjectFilter(filter)
        viewport:getSelectableObjectFilter():deserialise(set.selectionFilter)
        break
      end

      -- Get the next preset
      index = index + 1
      set = getPreset(index)
    end
  end
 
  if unselectableNodes then
    local app = nmx.Application.new()
    for _, node in unselectableNodes do
      local classTypeId = app:lookupTypeId(node)
      viewport:setShouldSelectType(classTypeId, false)
    end
  end
end


addViewportToolbar = function (toolBar, scene, viewport, extraToolsFn)

  local selectButton = toolBar:addButton{
    name = "SelectTool",
    label = "Select Tool (q)",
    helpText = "Select Tool (q)",
    image = app.loadImage("toolIcons\\SelectTool.png"),
    flags = "expand;parentBackground",
    onClick = function()
      viewport:setFocus()
      viewport:setCurrentTool("SelectTool")
    end,
  }
  selectButton:setChecked(true)

  local translateButton = toolBar:addButton{
    name = "TranslateTool",
    label = "Translate Tool (w)",
    helpText = "Translate Tool (w)",
    image = app.loadImage("toolIcons\\TranslateTool.png"),
    flags = "expand;parentBackground",
    onClick = function()
      viewport:setFocus()
      viewport:setCurrentTool("TranslateTool")
    end,
  }

  local rotateButton = toolBar:addButton{
    name = "RotateTool",
    label = "Rotate Tool (e)",
    helpText = "Rotate Tool (e)",
    image = app.loadImage("toolIcons\\RotateTool.png"),
    flags = "expand;parentBackground",
    onClick = function()
      viewport:setFocus()
      viewport:setCurrentTool("RotateTool")
    end,
  }

  local manipsButton = toolBar:addButton{
    name = "ShowManipsTool",
    label = "Show Manipulators (r)",
    helpText = "Show Manipulators (r)",
    image = app.loadImage("toolIcons\\ShowManipsTool.png"),
    flags = "expand;parentBackground",
    onClick = function()
      viewport:setFocus()
      viewport:setCurrentTool("ShowManipsTool")
    end,
  }

  viewport:setOnToolChanged(
    function (tool)
      selectButton:setChecked(tool == "SelectTool")
      translateButton:setChecked(tool == "TranslateTool")
      rotateButton:setChecked(tool == "RotateTool")
      manipsButton:setChecked(tool == "ShowManipsTool")
    end
  )

  if type(extraToolsFn) == "function" then
    extraToolsFn(toolBar, scene, viewport)
  end

  toolBar:addSeparator()

  local lockSelectionButton = toolBar:addButton{
    name = "LockSelection",
    label = "Enable selection lock",
    helpText = "Toggle lock current selection",
    image = app.loadImage("toolIcons\\LockSelection.png"),
    flags = "expand;parentBackground",
    onClick = function()
      viewport:setLockSelection(not viewport:getLockSelection())
    end,
  }
  lockSelectionButton:setChecked(viewport:getLockSelection())

  viewport:setOnLockSelectionChanged(function (enable)
      lockSelectionButton:setChecked(enable)
    end)

  local clickThroughButton = toolBar:addButton{
    name = "ClickThroughSelection",
    label = "Enable click through selection",
    helpText = "Toggle click through selection",
    image = app.loadImage("toolIcons\\ClickThroughSelection.png"),
    flags = "expand;parentBackground",
    onClick = function()
      viewport:setClickThroughSelection(not viewport:getClickThroughSelection())
    end,
  }
  clickThroughButton:setChecked(viewport:getClickThroughSelection())

  viewport:setOnClickThroughSelectionChanged(
    function (enable)
      clickThroughButton:setChecked(enable)
    end
  )

  toolBar:addSeparator()

  local wireframeButton = toolBar:addButton{
    name = "Wireframe",
    label = "Display wireframe",
    helpText = "Display wireframe",
    image = app.loadImage("toolIcons\\Wireframe.png"),
    flags = "expand;parentBackground",
    onClick = function()
      viewport:setDisplayMode("Wireframe")
    end,
  }
  wireframeButton:setChecked(viewport:getDisplayMode() == "Wireframe")

  local shadedButton = toolBar:addButton{
    name = "Shaded",
    label = "Display normal",
    helpText = "Display normal",
    image = app.loadImage("toolIcons\\Shaded.png"),
    flags = "expand;parentBackground",
    onClick = function()
      viewport:setDisplayMode("Shaded")
    end,
  }
  shadedButton:setChecked(viewport:getDisplayMode() == "Shaded")

  local xRayButton = toolBar:addButton{
    name = "XRay",
    label = "Display x-ray",
    helpText = "Display x-ray",
    image = app.loadImage("toolIcons\\XRay.png"),
    flags = "expand;parentBackground",
    onClick = function()
      viewport:setDisplayMode("XRay")
    end,
  }
  xRayButton:setChecked(viewport:getDisplayMode() == "XRay")

  viewport:setOnDisplayModeChanged(
    function (newMode)
      wireframeButton:setChecked(newMode == "Wireframe")
      shadedButton:setChecked(newMode == "Shaded")
      xRayButton:setChecked(newMode == "XRay")
    end
  )

  toolBar:addSeparator()
end

------------------------------------------------------------------------------------------------------------------------
-- Add the view, display, show and preset menus
------------------------------------------------------------------------------------------------------------------------
addCommonViewportMenus = function(
  name,
  menubar,
  viewport,
  scene,
  readOnlyFn,
  menuSections,
  unselectableNodes,
  colourModes,
  addAdditionalFrameMenuItemsFunc)

  if mcn ~= nil then
    kProductFeatures["Euphoria"] = not mcn:isEuphoriaDisabled()
    kProductFeatures["Physics"] = not mcn:isPhysicsDisabled()
    kProductFeatures["Kinect"] = not mcn:isKinectDisabled()
  end

  if menuSections == nil then
    menuSections = {}
  end

  local showMenuObjectTypes = { }
  local freezeMenuObjectTypes = { }

  local viewportPanel = {
    getName = function(self)
      return name
    end,
    getMenuBar = function(self)
      return menubar
    end,
    getViewport = function(self)
      return viewport
    end,
    getScene = function(self)
      return scene
    end,
    getShowMenuItems = function(self)
      return showMenuObjectTypes
    end,
    setShowMenuItems = function(self, objectTypes)
      showMenuObjectTypes = objectTypes
    end,
    getFreezeMenuItems = function(self)
      return freezeMenuObjectTypes
    end,
    setFreezeMenuItems = function(self, objectTypes)
      freezeMenuObjectTypes = objectTypes
    end,
    isSceneReadOnly = function(self)
      if readOnlyFn ~= nil then
        return readOnlyFn()
      end
      return false
    end
  }

  local viewMenu = addViewMenu(viewportPanel, addAdditionalFrameMenuItemsFunc)
  local dispMenu = addDisplayMenu(viewportPanel, colourModes)
  local showMenu = addShowMenu(viewportPanel, menuSections)
  local selectionMenu = addSelectionMenu(viewportPanel, menuSections, unselectableNodes)

  --addArrowAccelerators(viewportPanel:getViewport())

  return viewportPanel, viewMenu, dispMenu, showMenu, selectionMenu
end
