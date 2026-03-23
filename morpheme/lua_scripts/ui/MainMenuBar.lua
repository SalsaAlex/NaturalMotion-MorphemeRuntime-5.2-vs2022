------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/FileNewDialog.lua"
require "ui/NetworkValidationDialog.lua"
require "ui/PreferencesEditor/PreferencesEditor.lua"
require "ui/ProjectMenu.lua"
require "ui/RuntimeTargetSelector.lua"
require "ui/RuntimeDebuggingConfigSelector.lua"

------------------------------------------------------------------------------------------------------------------------
-- Select all nodes in the current graph
------------------------------------------------------------------------------------------------------------------------
local onEditSelectAllClick = function()
  local currentContext = ui.getCurrentContextName()

  if currentContext == "Network" then
    local currentGraph = getCurrentGraph()
    local allNodes = listChildren(currentGraph)
    select(allNodes, true)
  else
    local app = nmx.Application.new()
    local scene
    local viewport
    
    if currentContext == "PreviewViewport" then
      viewport = ui.getWindow("MainFrame|LayoutManager|PreviewViewport|Viewport|Viewport")
    elseif currentContext == "AssetManagerViewport" then
      viewport = ui.getWindow("MainFrame|LayoutManager|AssetManager|AssetManagerViewport")
    end
    
    viewport:selectVisible()
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Show colour editor dialog
------------------------------------------------------------------------------------------------------------------------
local onEditColoursClick = function()
  editColoursDialog = ui.getWindow("ColourEditor")
  if not editColoursDialog then
    editColoursDialog = ui.createModelessDialog{ caption = "Colour Editor", resize = true, centre = true }
    editColoursDialog:addStockWindow{ type = "ColourEditor", proportion = 1, flags = "expand" }
  end
  editColoursDialog:show()
end

------------------------------------------------------------------------------------------------------------------------
-- Show export dialog
------------------------------------------------------------------------------------------------------------------------
local onFileExportClick = function()
  local dlg = ui.createFileDialog{
    style = "save",
    caption = "Export",
    wildcard = "Network XML description|xml" }

  if dlg:show() then
    local result, ids, errors, warnings = mcn.export(dlg:getFullPath())

    local show = false
    if table.getn(errors) > 0 then
      show = true
    elseif preferences.get("DisplayNetworkValidationWarnings") then
      if table.getn(warnings) > 0 then
        show = true
      end
    end

    if not result or show then
      safefunc(showNetworkValidationReport, ids, warnings, errors)
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Show export and process dialog
------------------------------------------------------------------------------------------------------------------------
local onFileExportAndProcessClick = function()
  local dlg = ui.createFileDialog{
    style = "save",
    caption = "Export",
    wildcard = "Network XML description|xml" }

  if dlg:show() then
    local exportDir = stripFilenameExtension(dlg:getFullPath()) .. "_runtimeBinary"
    app.createDirectory(exportDir)
    local result, ids, errors, warnings = mcn.export(dlg:getFullPath(), nil, nil, nil, exportDir)

    local show = false
    if table.getn(errors) > 0 then
      show = true
    elseif preferences.get("DisplayNetworkValidationWarnings") then
      if table.getn(warnings) > 0 then
        show = true
      end
    end

    if not result or show then
      safefunc(showNetworkValidationReport, ids, warnings, errors)
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Clear anim cache
------------------------------------------------------------------------------------------------------------------------
local onFileClearAnimCacheClick = function()
  local result = ui.showMessageBox("Are you sure you wish to clear the animation cache?", "yesno")
  if result == "yes" then
    anim.clearAnimCache()
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Show requests dialog
------------------------------------------------------------------------------------------------------------------------
local onEditBodyGroupsClick = function()
  showBodyGroups()
end

------------------------------------------------------------------------------------------------------------------------
-- Export menu
------------------------------------------------------------------------------------------------------------------------
local addExportSubMenu = function(fileMenu)
  local exportMenu = fileMenu:addSubMenu{
    name = "Export",
    label = "Export" }

  exportMenu:addItem{
    name = "Export",
    label = "Export",
    onClick = onFileExportClick,
  }

  exportMenu:addItem{
    name = "ExportProcess",
    label = "Export and Process",
    onClick = onFileExportAndProcessClick,
  }

  exportMenu:addSeparator()

  exportMenu:addItem{
    name = "ClearCache",
    label = "Clear cache",
    onClick = onFileClearAnimCacheClick,
  }
end

------------------------------------------------------------------------------------------------------------------------
-- File menu
------------------------------------------------------------------------------------------------------------------------
local addFileMenu = function(mainMenuBar)
  local fileMenu = mainMenuBar:addSubMenu{ name = "FileMenu", label = "&File" }

  -- Basic file menu operations
  fileMenu:addItem{
    name = "New",
    label = "&New",
    accelerator = "CTRL+N",
    onClick = function()
      -- if the user saved or chose to not save then show the file new dialog
      if ui.showUnsavedTasksDialog() ~= "cancel" then
        showFileNewDialog()
      end
    end }

  fileMenu:addItem{
    name = "Open",
    label = "&Open",
    accelerator = "CTRL+O",
    onClick = "mcn.open()" }

  -- Save file operations
  fileMenu:addSeparator()
  fileMenu:addItem{
    name = "Save",
    label = "&Save",
    accelerator = "CTRL+S",
    onClick = "mcn.save()" }

  fileMenu:addItem{
    name = "SaveAs",
    label = "Save &As",
    onClick = "mcn.saveAs()" }

  fileMenu:addSeparator()

  addProjectMenu(fileMenu)

  fileMenu:addSeparator()
  fileMenu:addItem{
    name = "Properties",
    label = "&Properties",
    onClick = showNetworkProperties }

  -- Export and import
  fileMenu:addSeparator()
  addExportSubMenu(fileMenu)

  -- Recent file list
  fileMenu:addSeparator()
  local recentFilesMenu
  recentFilesMenu = fileMenu:addMRUFilesMenu{
    label = "Open &Recent",
    onMRUFileClick = function(self)
      local label = self:getLabel()
      local isOk,isCancelled = mcn.open(label);
      if not isOk and isCancelled ~= "Cancelled" then
        local settingsNode = recentFilesMenu:getSettingsNode()
        if settingsNode then
          settingsNode:removeFileFromList(label)
        end
      end
    end
  }

  local settings = nmx.Application.new():getLocalUserSettings()
  local settingsNode = settings:getSetting("|GlobalSettings|RecentNetworkList")
  recentFilesMenu:setSettingsNode(settingsNode)

  -- Session file management
  fileMenu:addSeparator()
  fileMenu:addItem{
    name = "LoadSession",
    label = "Load Session",
    onClick = "mcn.loadSession()",
    onPoppedUp = function(menuItem)
      menuItem:enable(not mcn.isNetworkRunning())
    end,
  }

  fileMenu:addSeparator()
  fileMenu:addItem{
    name = "SaveSession",
    label = "Save Session",
    onClick = "mcn.saveSession()",
    onPoppedUp = function(menuItem)
      menuItem:enable(mcn.isPaused())
    end,
    }

  -- Exit application
  fileMenu:addSeparator()
  fileMenu:addItem{
    name = "Exit",
    label = "E&xit",
    onClick = app.exit }
end

------------------------------------------------------------------------------------------------------------------------
-- doDelete
------------------------------------------------------------------------------------------------------------------------
local doDelete = function()
  local currentContext = ui.getCurrentContextName()

  -- Undo the asset manager
  --
  if currentContext == "PreviewViewport" then
    nmx.Application.new():runCommand("Core", "Delete", nmx.Application.new():getSceneByName("Network"))

  -- Undo the viewport
  elseif currentContext == "AssetManagerViewport" then
    nmx.Application.new():runCommand("Core", "Delete", nmx.Application.new():getSceneByName("AssetManager"))

  -- Undo the network
  elseif(currentContext == "Network") then
    undoBlock(function()
      for i, v in ipairs(ls("Selection")) do
        delete(v)
      end
    end)
  end
end

------------------------------------------------------------------------------------------------------------------------
-- doUndo
------------------------------------------------------------------------------------------------------------------------
local doUndo = function()
  local currentContext = ui.getCurrentContextName()

  -- Undo the asset manager
  --
  if currentContext == "PreviewViewport" then
    nmx.Application.new():getSceneByName("Network"):undo()

  -- Undo the viewport
  elseif currentContext == "AssetManagerViewport" then
    nmx.Application.new():getSceneByName("AssetManager"):undo()

  elseif currentContext == "ComponentsPanel" then
    componentEditorUndo()
    
  -- Undo the network
  --
  else
    mcn.undo()
  end
end

------------------------------------------------------------------------------------------------------------------------
-- doRedo
------------------------------------------------------------------------------------------------------------------------
local doRedo = function()
  local currentContext = ui.getCurrentContextName()

  -- Redo the asset manager
  --
  if (currentContext == "PreviewViewport")
  then
    nmx.Application.new():getSceneByName("Network"):redo()

  -- Undo the viewport
  elseif currentContext == "AssetManagerViewport" then
    nmx.Application.new():getSceneByName("AssetManager"):redo()

  elseif currentContext == "ComponentsPanel" then
    componentEditorRedo()
    
  -- Redo the network
  --
  else
    mcn.redo()
  end
end

------------------------------------------------------------------------------------------------------------------------
-- doCanUndo
------------------------------------------------------------------------------------------------------------------------
local doCanUndo = function()
  local currentContext = ui.getCurrentContextName()

  -- Undo the asset manager
  --
  if currentContext == "PreviewViewport" or currentContext == "AssetManagerViewport" then
    return true

  elseif currentContext == "ComponentsPanel" then
    return canComponentEditorUndo()
    
  -- Undo the network
  --
  else
    return mcn.canUndo()
  end
  
  return false
end

------------------------------------------------------------------------------------------------------------------------
-- doCanRedo
------------------------------------------------------------------------------------------------------------------------
local doCanRedo = function()
  local currentContext = ui.getCurrentContextName()

  -- Redo the asset manager
  --
  if currentContext == "PreviewViewport" or currentContext == "AssetManagerViewport" then
    return true

  elseif currentContext == "ComponentsPanel" then
    return canComponentEditorRedo()
    
  -- Redo the network
  --
  else
    return mcn.canRedo()
  end
  
  return false
end

------------------------------------------------------------------------------------------------------------------------
-- doCanCopy
------------------------------------------------------------------------------------------------------------------------
local doCanCut = function()
  local currentContext = ui.getCurrentContextName()

  -- CanCopy the asset manager
  --
  if currentContext == "PreviewViewport" or currentContext == "AssetManagerViewport" then
    return true

  -- CanCut the network
  --
  else
    return mcn.canCut()
  end
  
  return false
end

------------------------------------------------------------------------------------------------------------------------
-- doCanCopy
------------------------------------------------------------------------------------------------------------------------
local doCanCopy = function()
  local currentContext = ui.getCurrentContextName()

  -- CanCopy the asset manager
  --
  if currentContext == "PreviewViewport" or currentContext == "AssetManagerViewport" then
    return true

  -- CanCopy the network
  --
  else
    return mcn.canCopy()
  end
  
  return false
end

------------------------------------------------------------------------------------------------------------------------
-- doCanPaste
------------------------------------------------------------------------------------------------------------------------
local doCanPaste= function()
  local currentContext = ui.getCurrentContextName()

  -- CanPaste the asset manager
  --
  if currentContext == "PreviewViewport" or currentContext == "AssetManagerViewport" then
    return true

  -- CanPaste the network
  --
  else
    return mcn.canPaste()
  end
  
  return false
end

------------------------------------------------------------------------------------------------------------------------
-- doCut
------------------------------------------------------------------------------------------------------------------------
local doCut = function()
  local currentContext = ui.getCurrentContextName()

  -- Cut the asset manager
  --
  if (currentContext == "PreviewViewport")
  then
     -- The application
    --
    local application = nmx.Application.new()
    
    -- The scene
    --
    local scene = application:getSceneByName("Network")
    
    -- Run the command
    --
    application:runCommand("Connect Core", "Cut", scene)

  -- Cut the viewport
  --
  elseif currentContext == "AssetManagerViewport"
  then
    -- The application
    --
    local application = nmx.Application.new()
    
    -- The scene
    --
    local scene = application:getSceneByName("AssetManager")
    
    -- Run the command
    --
    application:runCommand("Connect Core", "Cut", scene)

  -- Cut the network
  --
  else
    return mcn.cut()
  end
end

------------------------------------------------------------------------------------------------------------------------
-- doCopy
------------------------------------------------------------------------------------------------------------------------
local doCopy = function()
  local currentContext = ui.getCurrentContextName()

  -- Copy the asset manager
  --
  if (currentContext == "PreviewViewport")
  then
     -- The application
    --
    local application = nmx.Application.new()
    
    -- The scene
    --
    local scene = application:getSceneByName("Network")
    
    -- Run the command
    --
    application:runCommand("Connect Core", "Copy", scene)

  -- Copy the viewport
  --
  elseif currentContext == "AssetManagerViewport"
  then
    -- The application
    --
    local application = nmx.Application.new()
    
    -- The scene
    --
    local scene = application:getSceneByName("AssetManager")
    
    -- Run the command
    --
    application:runCommand("Connect Core", "Copy", scene)

  -- Copy the network
  --
  else
    return mcn.copy()
  end
end

------------------------------------------------------------------------------------------------------------------------
-- doPaste
------------------------------------------------------------------------------------------------------------------------
local doPaste = function()
  local currentContext = ui.getCurrentContextName()

  -- Copy the asset manager
  --
  if (currentContext == "PreviewViewport")
  then
     -- The application
    --
    local application = nmx.Application.new()
    
    -- The scene
    --
    local scene = application:getSceneByName("Network")
    
    -- Run the command
    --
    application:runCommand("Connect Core", "Paste", scene)

  -- Copy the viewport
  --
  elseif currentContext == "AssetManagerViewport"
  then
    -- The application
    --
    local application = nmx.Application.new()
    
    -- The scene
    --
    local scene = application:getSceneByName("AssetManager")
    
    -- Run the command
    --
    application:runCommand("Connect Core", "Paste", scene)

  -- Copy the network
  --
  else
    return mcn.paste()
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Edit menu
------------------------------------------------------------------------------------------------------------------------
local addEditMenu = function(mainMenuBar)
  local editMenu = mainMenuBar:addSubMenu{ name = "EditMenu", label = "&Edit" }

  -- Undo / redo
  editMenu:addItem{
    name = "UndoMenuItem",
    label = "Undo",
    accelerator = "CTRL+Z",
    onClick = doUndo,
    onPoppedUp = function(menuItem) menuItem:enable(doCanUndo()) end
  }
  editMenu:addItem{
    name = "RedoMenuItem",
    label = "Redo",
    accelerator = "CTRL+Y",
    onClick = doRedo,
    onPoppedUp = function(menuItem) menuItem:enable(doCanRedo()) end
  }

  -- Copy / paste
  editMenu:addSeparator()
  editMenu:addItem{
    name = "CutMenuItem",
    label = "Cut",
    accelerator = "CTRL+X",
    onClick = function() doCut() end,
    onPoppedUp = function(menuItem) menuItem:enable(doCanCut())
    end
  }
  editMenu:addItem{
    name = "CopyMenuItem",
    label = "Copy",
    accelerator = "CTRL+C",
    onClick = doCopy,
    onPoppedUp = function(menuItem) menuItem:enable(doCanCopy())
    end
  }
  editMenu:addItem{
    name = "PasteMenuItem",
    label = "Paste",
    accelerator = "CTRL+V",
    onClick = doPaste,
    onPoppedUp = function(menuItem) menuItem:enable(doCanPaste())
    end
  }

  -- Delete
  editMenu:addSeparator()
  editMenu:addItem{
    name = "DeleteMenuItem",
    label = "Delete",
    accelerator = "Del",
    onClick = doDelete
  }

  -- Select all
  editMenu:addSeparator()
  editMenu:addItem{
    name = "SelectAll",
    label = "Select All",
    accelerator = "CTRL+A",
    onClick = onEditSelectAllClick,
  }

  -- Control params, requests and set rig
  editMenu:addSeparator()
  editMenu:addItem{
    name = "BodyGroups",
    label = "&Body Groups...",
    onClick = onEditBodyGroupsClick,
  }

  -- Colours
  editMenu:addSeparator()
  editMenu:addItem{
    name = "Colours",
    label = "&Colours...",
    onClick = onEditColoursClick,
  }

  -- Preferences
  editMenu:addSeparator()
  editMenu:addItem{
    name = "Preferences",
    label = "&Preferences...",
    onPoppedUp = function(self)
      self:enable(not mcn.isNetworkRunning())
    end,
    onClick = function()
      showPreferencesDialog("Settings")
    end,
  }
end

------------------------------------------------------------------------------------------------------------------------
-- Main menu bar
------------------------------------------------------------------------------------------------------------------------
addMainMenuBar = function(mainFrame)

  mainFrame:setBorder(0)
  mainFrame:beginHSizer{ flags = "expand", }
    local mainMenuBar = mainFrame:addMenuBar{
      name = "MainMenu",
      proportion = 1,
      flags = "expand"
    }

    -- add file and edit menus
    addFileMenu(mainMenuBar)
    addEditMenu(mainMenuBar)

    -- Application specific menus that are fundimentally bound to app.
    mainMenuBar:addStockMenu{ type = "WindowMenu", label = "&Window" }
    mainMenuBar:addStockMenu{ type = "LayoutMenu", label = "&Layout" }
    mainMenuBar:addStockMenu{ type = "HelpMenu", label = "&Help" }

    -- drop-downs that control runtime selection / debugging level
    addRuntimeTargetsSelector(mainFrame)
    mainFrame:addHSpacer(4)
    addRuntimeDebuggingConfigSelector(mainFrame)

  mainFrame:endSizer()
end

