------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AssetManager/ContextualMenu.lua"
require "ui/AssetManager/MarkupPropertiesDialog.lua"
require "ui/AssetManager/RigsMenu.lua"
require "ui/AddAnimAttributeDialog.lua"
require "ui/ScaleOptions.lua"
require "ui/FollowJoint.lua"
require "manifest/extras/CharacterController.lua"

require "ui/Components/Template.lua"
require "ui/Components/Tags.lua"
require "ui/Components/Limbs.lua"
require "ui/Components/Poses.lua"

local assetManager
local removeLocationButton
local addLocationButton
local fileRollupHeadingLable
local filterButton
local configureButton

local pluralize = function(singular, plural, isPlural)
  if isPlural then
    return plural
  end
  return singular
end

------------------------------------------------------------------------------------------------------------------------
local setAssetManagerFilterLabel = function()
  local filter = assetManager:getFilterByAnimationSet()
  if filter then
    assetManager:setTreeFilterLabel(string.format("Search (%s)", getSelectedAssetManagerAnimSet(true)))
  else
    assetManager:setTreeFilterLabel("Search")
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Sets the callbacks for the toolbar and buttons at the top of the animation sets dialog
------------------------------------------------------------------------------------------------------------------------
local enableFileRollupHeadingControls = function(panel)
  if filterButton and configureButton and removeLocationButton then
    local filter = assetManager:getFilterByAnimationSet()
    filterButton:setChecked(filter)
    configureButton:setChecked(not filter)
    setAssetManagerFilterLabel()
    local fileTree = assetManager:getAnimationFileTreeControl()
    local selectedResource = fileTree:getSelectedItem()
    removeLocationButton:enable(anim.isAnimationLocation(selectedResource))
    return false
  end
end

------------------------------------------------------------------------------------------------------------------------
-- shows a message that lets the user disable retargeting if they want to configure the filter
------------------------------------------------------------------------------------------------------------------------
local showAddRemoveLocationNotAllowedDuringRetargetMessage = function()
  local message = 
[[
Locations in the asset explorer can't be added or removed while retargeting preview is enabled.
  
  
To disable retargeting preview select 'Disable Preview' in the retargeting combo box
  
]]
  local result, ignore = ui.showMessageBox(message, "ok")
end

------------------------------------------------------------------------------------------------------------------------
-- shows a message that lets the user disable retargeting if they want to configure the filter
------------------------------------------------------------------------------------------------------------------------
local showConfigureNotAllowedDuringRetargetMessage = function()
  local message = 
[[
The filter for the asset explorer can't be modified while retargeting preview is enabled.


To disable retargeting preview select 'Disable Preview' in the retargeting combo box

]]
  local result, ignore = ui.showMessageBox(message, "ok")
end

------------------------------------------------------------------------------------------------------------------------
-- Sets the callbacks for the toolbar and buttons at the top of the animation sets dialog
------------------------------------------------------------------------------------------------------------------------
local createFilesRollupHeadingControls = function(panel)
  panel:setLabel("")
  panel:setBorder(0)

  local fileTree = assetManager:getAnimationFileTreeControl()
  local removeitem = app.loadImage("removeitem.png")
  local additem = app.loadImage("additem.png")

  panel:beginHSizer{ flags = "expand", proportion = 0 }
    fileRollupHeadingLable = panel:addStaticText{ text = "Files", font = "bold", flags = "parentBackground;truncate;expand;decoration", proportion = 1 }

    filterButton = panel:addButton{
      name = "FilterButton",
      helpText = "Only show animations in the current animation set",
      label = "Filter",
      flags = "expand;parentBackground;segmentLeft",
      size = { width = -1, height = 12 },
      onClick = function(self)
        assetManager:setFilterByAnimationSet(true)
        panel:expand(true)
      end
    }

    configureButton = panel:addButton{
      name = "ConfigureButton",
      helpText = "Choose the files in the current animation set",
      label = "Configure",
      flags = "expand;parentBackground;segmentRight",
      size = { width = -1, height = 12 },
      onClick = function(self)
       local selectedSet = getSelectedAssetManagerAnimSet()
       local activeSource = anim.getActiveRetargetingSource(selectedSet)
        if activeSource ~= "" then
         showConfigureNotAllowedDuringRetargetMessage()
        end
        
        activeSource = anim.getActiveRetargetingSource(selectedSet)
        if activeSource == "" then
          assetManager:setFilterByAnimationSet(false)
          panel:expand(true)
        end
      end
    }

    panel:addHSpacer(2)

    removeLocationButton = panel:addButton{
      name = "RemoveButton", label = "",
      image = removeitem, flags = "expand;parentBackground",
      helpText = "Delete Animation Set",
      size = { width = removeitem:getWidth(), height = removeitem:getHeight() },
      onClick = function(self)
        local selectedSet = getSelectedAssetManagerAnimSet()
        local activeSource = anim.getActiveRetargetingSource(selectedSet)
        if activeSource ~= "" then
          showAddRemoveLocationNotAllowedDuringRetargetMessage()
        end
        
        activeSource = anim.getActiveRetargetingSource(selectedSet)
        if activeSource == "" then
          local selectedResource = fileTree:getSelectedItem()
          assetManager:removeLocation(selectedResource)
        end
        
      end
    }

    addLocationButton = panel:addButton{
      name = "AddButton", label = "",
      image = additem, flags = "expand;parentBackground",
      helpText = "Add Animation Set",
      size = { width = additem:getWidth(), height = additem:getHeight() },
      onClick = function(self)
        local selectedSet = getSelectedAssetManagerAnimSet()
        local activeSource = anim.getActiveRetargetingSource(selectedSet)
        if activeSource ~= "" then
          showAddRemoveLocationNotAllowedDuringRetargetMessage()
        end
        
        activeSource = anim.getActiveRetargetingSource(selectedSet)
        if activeSource == "" then
          assetManager:addNewLocation()
          panel:expand(true)
        end
      end
    }

    enableFileRollupHeadingControls()
    fileTree:setOnSelectionChanged(enableFileRollupHeadingControls)
    assetManager:setOnFilterByAnimationSetChanged(enableFileRollupHeadingControls)
  panel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
local missingColour = { r = 255, g = 0, b = 0 }
local errorColour = { r = 100, g = 100, b = 100 }

local assetTextColour = function(resourceId)
  if anim.getResourceHasError(resourceId) then
    return errorColour
  elseif anim.getResourceIsMissing(resourceId) then
    return missingColour
  end
end

------------------------------------------------------------------------------------------------------------------------
local getDisplayFileName = function(resourceId)
  if anim.hasMarkupChanged(resourceId) then
    return anim.getFileName(resourceId) .. " *"
  else
    return anim.getFileName(resourceId)
  end
end

------------------------------------------------------------------------------------------------------------------------
local getNameForTree = function(resourceId)
  if anim.isAnimationLocation(resourceId) then
    return utils.macroizeString(anim.getResourceName(resourceId))
  elseif anim.hasMarkupChanged(resourceId) then
    return anim.getResourceName(resourceId) .. " *"
  else
    return anim.getResourceName(resourceId)
  end
end

------------------------------------------------------------------------------------------------------------------------
local getFileName = function(resourceId)
  return anim.getFileName(resourceId)
end

------------------------------------------------------------------------------------------------------------------------
local getTakeName = function(resourceId)
  return anim.getTakeName(resourceId)
end

------------------------------------------------------------------------------------------------------------------------
local getTakeFramerate = function(resourceId)
  local frameRate = anim.getTakeFramerate(resourceId)
  if frameRate > 0 then
    return string.format("%.4g", frameRate)
  end
  return "-"
end

------------------------------------------------------------------------------------------------------------------------
local getTakeDuration = function(resourceId)
  local duration = anim.getTakeDuration(resourceId)
  if duration > 0 then
    return string.format("%.4g", duration)
  end
  return "-"
end

------------------------------------------------------------------------------------------------------------------------
local getFilePath = function(resourceId)
  return utils.macroizeString(anim.getFilePath(resourceId))
end

------------------------------------------------------------------------------------------------------------------------
local folderImage = app.loadImage("folder.png")
local blueFolderImage = app.loadImage("bluefolder.png")
local retargetFolderImage = app.loadImage("retargetfolder.png")

local getImage = function(resourceId)
  local resourceType = anim.getResourceType(resourceId)
  if resourceType == "directory" then
    local selectedSet = getSelectedAssetManagerAnimSet()
    local activeSource = anim.getActiveRetargetingSource(selectedSet)
    if anim.isAnimationLocation(resourceId) then
      if activeSource ~= "" then
        return retargetFolderImage
      else
        return folderImage
      end
    else 
      if activeSource ~= "" then
        return retargetFolderImage
      else
        return blueFolderImage
      end
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
local addSearchOption = function(menu, columnName, searchColumn)
  local onClick = function()
    assetManager:setChooserFilterColumn(columnName)
    assetManager:setChooserFilterLabel("Search " .. columnName);
  end
  local checked = searchColumn == menu:countItems() + 1;
  menu:addCheckedItem{ label = columnName, checked = checked, onClick = onClick }
end

------------------------------------------------------------------------------------------------------------------------
local onChooserFilterMenuPoppedUp = function(menu)
  local searchColumn = assetManager:getChooserSearchColumn()
  menu:clear()
  addSearchOption(menu, "File", searchColumn)
  addSearchOption(menu, "Take", searchColumn)
end

------------------------------------------------------------------------------------------------------------------------
local setTreeShowsFoldersOnly = function(show)
  return function()
    assetManager:setTreeShowsFoldersOnly(show);
    assetManager:setShowChooser(not show);
  end
end

------------------------------------------------------------------------------------------------------------------------
local setChooserRight = function(onRight)
  return function()
    assetManager:setChooserRight(onRight);
  end
end

------------------------------------------------------------------------------------------------------------------------
local onTreeFilterMenuPoppedUp = function(menu)
  menu:clear()
  local showMatchesOnly = assetManager:getTreeShowMatchingItemsOnly()
  menu:addCheckedItem{ label = "Show matches only", checked = showMatchesOnly, onClick = function() assetManager:setTreeShowMatchingItemsOnly(true) end }
  menu:addCheckedItem{ label = "Show matches with contents", checked = not showMatchesOnly, onClick = function() assetManager:setTreeShowMatchingItemsOnly(false) end }
end

------------------------------------------------------------------------------------------------------------------------
local onTreeOptionsMenuPoppedUp = function(menu)
  menu:clear()

  local foldersOnly = assetManager:getTreeShowsFoldersOnly()
  menu:addItem{ label = "Show", enable = false }
  menu:addCheckedItem{ label = "  Folders Only", checked = foldersOnly, onClick = setTreeShowsFoldersOnly(true) }
  menu:addCheckedItem{ label = "  Files and Folders", checked = not foldersOnly, onClick = setTreeShowsFoldersOnly(false) }

  local chooserRight = assetManager:getChooserRight()
  menu:addSeparator()
  menu:addItem{ label = "Chooser Position", enable = false }
  menu:addCheckedItem{ label = "  At Bottom", checked = not chooserRight, onClick = setChooserRight(false) }
  menu:addCheckedItem{ label = "  On Right", checked = chooserRight, onClick = setChooserRight(true) }
end

------------------------------------------------------------------------------------------------------------------------
local kColumns =
{
  { name = "File",     sorts = true, text = getDisplayFileName,colour = assetTextColour, value = getFileName, minWidth = 40, width = 220, isVisible = true },
  { name = "Take",     sorts = true, text = getTakeName,       colour = assetTextColour, value = getTakeName, minWidth = 40, width = 100, isVisible = true },
  { name = "FPS",      sorts = true, text = getTakeFramerate,  colour = assetTextColour, value = getTakeFramerate, type = "number", minWidth = 40, width = 60, isVisible = false },
  { name = "Duration", sorts = true, text = getTakeDuration,   colour = assetTextColour, value = getTakeDuration, type = "number", minWidth = 40, width = 60, isVisible = false },
  { name = "Path",     sorts = true, text = getFilePath,       colour = assetTextColour, value = getFilePath, minWidth = 40, width = 100, isVisible = false }
}

------------------------------------------------------------------------------------------------------------------------
local toggleVisiblity = function(name)
  return function()
    local isVisible = assetManager:getChooserColumnVisible(name)
    assetManager:setChooserColumnVisible(name, not isVisible)
  end
end

------------------------------------------------------------------------------------------------------------------------
local onChooserOptionsMenuPoppedUp = function(menu)
  menu:clear()

  menu:addItem{ label = "Columns", enable = false }
  for _, column in ipairs(kColumns) do
    local ischecked = assetManager:getChooserColumnVisible(column.name)
    menu:addCheckedItem{ label = "  " .. column.name, checked = ischecked, onClick = toggleVisiblity(column.name) }
  end
end

------------------------------------------------------------------------------------------------------------------------
local addMirrorOnSelectionOption = function(menu)
  menu:addSeparator()
  menu:addCheckedItem{
    label = "Mirror on Selection",
    onClick = function()
      local roamingSettings = nmx.Application.new():getGlobalRoamingSettingsNode()
      local attr = roamingSettings:findAttribute("MirrorOnSelection")

      local settingdb = roamingSettings:getDatabase()
      local status, cbRef = settingdb:beginChangeBlock(getCurrentFileAndLine())

      attr:setBool(not attr:asBool())

      settingdb:endChangeBlock(cbRef, changeBlockInfo("Set Mirror on Selection"))
    end,
    onPoppedUp = function(self)
      local roamingSettings = nmx.Application.new():getGlobalRoamingSettingsNode()
      local attr = roamingSettings:findAttribute("MirrorOnSelection")
      self:setChecked(attr:asBool())
    end
    }

end

------------------------------------------------------------------------------------------------------------------------
local addSkinMenu = function(menubar)

  ----------------------------------------------------------------------------------------------------------------------
  local mkSelectSkin = function(theSkin, selectedSet)
    return function()
      anim.setCurrentAssetManagerSkin(theSkin, selectedSet)
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  local addSkinsToMenu = function(menu, selectedSet, prefix)
    local skins = anim.getRigSkins(selectedSet)
    local skinNames = { }
    for _, skin in skins do
      local fullPath = utils.demacroizeString(skin.Path)
      if app.fileExists(fullPath) then
        table.insert(skinNames, skin.Name)
      end
    end
    
    if table.getn(skinNames) == 0 then
      menu:addItem{label = "No valid skins", enable = false}
    else
      local currentSkin = anim.getCurrentAssetManagerSkin(selectedSet)
      local currentSkinExists = currentSkin and app.fileExists(utils.demacroizeString(currentSkin.Path))
      for _, skinName in skinNames do
        local checked = currentSkinExists and skinName == currentSkin.Name
        menu:addCheckedItem{
          label = prefix .. skinName,
          checked = checked,
          onClick = mkSelectSkin(skinName, selectedSet),
        }
      end
    end
  end
  
  ----------------------------------------------------------------------------------------------------------------------
  local buildMenu = function(menu)    
    local selectedSet = getSelectedAssetManagerAnimSet()
    local retargetingSource = anim.getActiveRetargetingSource(selectedSet)
    if retargetingSource == "" then
      addSkinsToMenu(menu, selectedSet, "")
    else
      menu:addItem{ label = selectedSet, style = "subtitle" }
      addSkinsToMenu(menu, selectedSet, "")
      menu:addItem{ label = retargetingSource .. " (retargeted from)", style = "subtitle" }
      addSkinsToMenu(menu, retargetingSource, "")
    end
  end

  local menu = menubar:addSubMenu{
    name = "SkinMenu",
    label = "Skin",
    onPoppedUp = function(self)
      self:clear()
      buildMenu(self)
    end,
  }

  buildMenu(menu)
end

------------------------------------------------------------------------------------------------------------------------
local kMenuSections =
{ 
  -- Animation
  { name = "Animation",
    items = { 
      { name = "Joints", nodes = {"JointNode", "MorphemeMainSkeletonNode"} },
      { name = "Limits", nodes = {"JointLimitNode"} },
      { name = "Character Controller", nodes = {"CharacterControllerNode"} },
      { name = "Blend Frame Transform", nodes = {"BlendFrameTransformLocatorNode"} },
    },
  },

  -- Physics
  { name = "Physics", availability = "Physics",
    items = {  
      { name = "Joints", nodes = {"PhysicsJointNode"} },
      { name = "Limits", nodes = {"PhysicsJointLimitNode"} },
      { name = "Soft Limits", nodes = {"PhysicsSoftTwistSwingNode"}, availability = "Euphoria" },
      { name = "Volumes", nodes = {"EnvironmentPhysicsConstraintNode", "PhysicsVolumeNode", "PhysicsBodyNode"} },
    },
  },
  
  -- Euphoria
  { name = "Euphoria", availability = "Euphoria",
    items = {  
      { name = "Reach Limits", nodes = {"ReachLimitNode"} },
      { name = "Limb Locators", nodes = {"LocatorNode"} },
      { name = "Self Avoidance", nodes = {"SelfAvoidanceNode"} },
      { name = "Interaction Proxy", nodes = {"InteractionProxyNode"} },
    }
  },
  
  -- Other
  { name = "Other", 
    items = {  
      { name = "Meshes", nodes = {"MeshNode"} },
      { name = "Character Start Point", nodes = {"CharacterStartPointNode"} },
      { name = "Offset Frames", nodes = {"OffsetFrameNode"} },
    },
  },
}

------------------------------------------------------------------------------------------------------------------------
kUnselectableNodes = {
  "PhysicsBodyNode",
  "DebugOutputNode"
}

------------------------------------------------------------------------------------------------------------------------
local clearAssetManagerUICallbacks = function()
  removeLocationButton = nil
  addLocationButton = nil
  fileRollupHeadingLable = nil
  filterButton = nil
  configureButton = nil
end

------------------------------------------------------------------------------------------------------------------------
-- addAssetManager
------------------------------------------------------------------------------------------------------------------------
addAssetManager = function(layoutManager)
  assetManager = layoutManager:addStockWindow{
    type = "AssetManager",
    name = "AssetManager"
  }

  -- set the chooser and file tree contextual menu
  assetManager:setChooserContextMenuFunction(onAssetChooserContextualMenu)
  assetManager:setTreeContextMenuFunction(onAssetTreeContextualMenu)

  -- Add all the columns to the choooser
  for _, column in ipairs(kColumns) do
   assetManager:addChooserColumn(column)
  end

  assetManager:setChooserFilterLabel("Search " .. kColumns[1].name);
  assetManager:getChooserOptionsMenu():setOnPoppedUp(onChooserOptionsMenuPoppedUp)
  assetManager:sortChooserByColumn(1)
  assetManager:setTreeFunctions({ text = getNameForTree, colour = assetTextColour, value = getNameForTree, image = getImage })
  local panel = assetManager:getFileTreeRollupHeading()
  createFilesRollupHeadingControls(panel)

  assetManager:getChooserFilterMenu():setOnPoppedUp(onChooserFilterMenuPoppedUp)
  assetManager:getTreeFilterMenu():setOnPoppedUp(onTreeFilterMenuPoppedUp)
  assetManager:getTreeOptionsMenu():setOnPoppedUp(onTreeOptionsMenuPoppedUp)
  setAssetManagerFilterLabel()
  registerEventHandler("mcAnimationSetSelectionChange", setAssetManagerFilterLabel)
  registerEventHandler("mcAnimationSetCurrentRetargetingSource", setAssetManagerFilterLabel)
  registerEventHandler("mcFileCloseBegin", clearAssetManagerUICallbacks, "clearAssetManagerUICallbacks")

  local mainMenubar = assetManager:getMainMenuBar()
  local toolbar = assetManager:getViewportToolBar()
  local displayMenubar = assetManager:getDisplayMenuBar()
  local viewport = assetManager:getChild("AssetManagerViewport")
  local scene = nmx.Application.new():getSceneByName("AssetManager")

  -- Rigs Menu
  addAssetManagerRigsMenu(mainMenubar)

  -- Edit Menu
  local editMenu = addEditMenu(mainMenubar, scene, viewport, true, nil, nil)

  -- option for mirroring selection
  addMirrorOnSelectionOption(editMenu)
  
  -- Create Menu
  addAssetManagerCreateMenu(mainMenubar)
  
  -- Colour modes in the display menu
  local colourModes = {}
  table.insert(colourModes, { MenuName = "Object Colour", ColourMode = "Object" })
  table.insert(colourModes, { MenuName = "Collision Colour", ColourMode = "Collision" })
  table.insert(colourModes, { MenuName = "Body Colour", ColourMode = "Body" })

  if not mcn.isEuphoriaDisabled() then
    table.insert(colourModes, { MenuName = "Limb Colour", ColourMode = "Limb" })
  end

  -- add the view, display, show and quick set menus
  local commonViewportPanel, viewMenu, dispMenu, showMenu, selectionMenu = addCommonViewportMenus(
    "AssetManager", 
    displayMenubar, 
    viewport, 
    scene,
    nil,
    kMenuSections,
    kUnselectableNodes,
    colourModes)

  addSkinMenu(displayMenubar)
  addPresetMenu(displayMenubar, viewport, scene, kUnselectableNodes)

  addViewportToolbar(toolbar, scene, viewport)

  -- set the default options
  setToDefault(viewport, "Default", kUnselectableNodes)

  -- set the default display mode to normal
  viewport:setDisplayMode("Shaded")

  -- by default hide the following objects
  viewport:setShouldRenderTypeByName("CharacterControllerNode", false)

  local eventDetector = assetManager:getEventDetectionManager()
  local timeline = assetManager:getTimeline()

  local onDeleteKey = function()
    if not mcn.isNetworkRunning() then
      local animSelection = anim.ls("selection")

      local event = nil
      local track = nil

      for i, object in animSelection do
        local type = anim.getType(object)

        if type == "event" then
          event = object
        elseif type == "track" then
          track = object
        end
      end

      if event then
        anim.delete(event)
      elseif track then
        anim.delete(track)
      end
    end
  end

  timeline:addAccelerator("del", onDeleteKey)
  eventDetector:addAccelerator("del", onDeleteKey)

  -- set the context menu function for the timeline to add an add attribute function
  timeline:setOnContextMenuFunction(
    function(menu, clickTime, track, event)
      local assetManager = ui.getWindow("MainFrame|LayoutManager|AssetManager")

      local timeline = assetManager:getTimeline()

      local readOnly = false
      if assetManager then
        readOnly = timeline:isReadOnly()
      end

      if not readOnly then
        local trackOrEvent = nil
        local type = nil

        if event then
          trackOrEvent = event
          type = "event"
        elseif track then
          trackOrEvent = track
          type = "track"
        end

        if trackOrEvent then
          menu:addItem{
            label = "Add attribute",
            onClick = function()
              showAddAnimAttributeDialog(trackOrEvent)
            end
          }

          -- only provide the option to add an event if there
          -- is only a track selected
          if not event then
            menu:addItem{
              label = "Add event",
              onClick = function()
                anim.undoBlock(track, function()
                  local event = anim.create(track, "Event[0]")
                  anim.setAttribute(string.format("%s|%s.TimePosition", track, event), clickTime)
                  local eventType = anim.getAttribute(string.format("%s.%s", track, "EventType"))
                  if eventType ~= "Tick" then
                    anim.setAttribute(string.format("%s|%s.TimeDuration", track, event), 0.5)
                  end
                end)
             end
            }
          end

          menu:addItem{
            label = string.format("Edit %s", type),
            onClick = function()
              showAnimPropertiesEditor()
            end
          }

          menu:addItem{
            label = string.format("Delete %s", type),
            onClick = function()
              anim.delete(trackOrEvent)
            end
          }
        end

        menu:addSeparator()

        -- Cut
        if timeline:canCut() then
          menu:addItem{
            label = "Cut track",
            onClick = function() timeline:cut() end
         }
        end

        -- Copy
        if timeline:canCopy() then
          menu:addItem{
            label = "Copy track",
            onClick = function() timeline:copy() end
          }
        end

        -- Copy All
        local canCopyAll, copyCount = timeline:canCopyAll()
        if (not timeline:canCopy()) or copyCount > 1 then
          menu:addItem{
            label = pluralize("Copy track", "Copy all tracks", copyCount > 0),
            onClick = function() timeline:copyAll() end
         }
        end

        -- Paste and PasteOver
        local canPaste, pasteCount = timeline:canPaste()
        if canPaste then
          menu:addItem{
            label = pluralize("Paste track", "Paste tracks", pasteCount > 0),
            onClick = function() timeline:paste() end
          }
          menu:addItem{
              label = pluralize("Paste over track", "Paste over tracks", pasteCount > 0),
              onClick = function() timeline:pasteOver() end
          }
        end
      end
    end
  )

  -- set the context menu function for the event detection manager to add an add attribute function
  eventDetector:setOnContextMenuFunction(
    function(menu, track, event)
      local assetManager = ui.getWindow("MainFrame|LayoutManager|AssetManager")

      local eventDetector = assetManager:getEventDetectionManager()

      local readOnly = false
      if assetManager then
        readOnly = eventDetector:isReadOnly()
      end

      if not readOnly then
        -- add custom menu items here
      end
    end
  )

  assetManager:setOnMarkupEditorTabSelected(
    function(item)
      if item:getName() ~= "AnimationMarkupEditor" then
        hideAnimPropertiesEditor()
      end
    end
  )

  return assetManager
end

------------------------------------------------------------------------------------------------------------------------
