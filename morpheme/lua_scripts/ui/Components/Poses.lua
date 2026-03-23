------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "luaAPI/MorphemeUnitAPI.lua"

local requiredPoses = {
  "DefaultPose",
  "GuidePose",
}

-- list of icons used in the tree
local treeIcons = { app.loadImage("folder.png")}

local kRequiredPosesHelpText = {
  ["DefaultPose"] = "The pose that used by behaviours requiring a pose as an input when no pose has been specified.",
  ["GuidePose"] = "The pose used to guide the style of the IK solution.",
}

local kAddPoseButtonHelpText = "Save the positions and orientation of the bones in the current rig (in the Asset manager viewport) as a pose in a file."
local kPoseDirectoryControl = "The folder in which the poses for this character can be found."
local treeItemsToPoses= { } -- mapping of tree items to pose information
local kOtherOption = "Other..."
local kNothingSelectedOption = ""

------------------------------------------------------------------------------------------------------------------------
local lastValidatedLocation

local validateFunction = function(selectedSet, isCurrentComponent)
  lastValidatedLocation = anim.getRigPosesLocation(selectedSet)
  if not anim.isRigValid(selectedSet) then
    return "Inactive"
  end

  for _, poseName in ipairs(requiredPoses) do
    local pose = anim.getRigPose(selectedSet, poseName)
    
    local takePath = string.format("%s|%s", pose.filename, pose.takename)
    if not (fileExists(takePath) or app.fileExists(utils.demacroizeString(pose.filename))) then
      return "Error"
    end
  end

  return "Ok"
end

------------------------------------------------------------------------------------------------------------------------
local isResourceInLocation = function(resources, location)
  if not anim.isResource(location) then
    return false
  end
  
  local locationId = anim.getResourceId(location)
  if locationId ~= 0 then
    for id in pairs(resources) do
      if anim.isResourceDescendant(id, locationId) then
        return true
      end
    end
  end

  return false
end

------------------------------------------------------------------------------------------------------------------------
-- Given a location returns a table of poses of the form:
-- {
--   {
--     name = "Pose",
--     filename = "$(RootDir)\anims\Pose.xmd",
--     takename = "untitled",
--   },
-- }
------------------------------------------------------------------------------------------------------------------------
local listPoses = function(location)
  local poses = { }

  if anim.isResource(location) then
    local animations = anim.getResourceChildren(location)
    for animationResourceId in pairs(animations) do
      local animation = anim.getResourcePath(animationResourceId)
      local fileName = anim.getFileName(animationResourceId)

      -- if the resource has just been deleted then it will still be listed though it will no longer exist
      if anim.exists(animation) then
        local animName = fileName
        animation = utils.macroizeString(animation)
        local takes = anim.ls(animation)
        if table.getn(takes) == 1 then
          local takeName = string.sub(takes[1], string.len(animation) + 2)
          local pose = {
            name = animName,
            filename = animation,
            -- strip the animation component of the full take path
            takename = takeName,
          }

          table.insert(poses, pose)
        else
          for _, take in ipairs(takes) do
            local takeName = string.sub(take, string.len(animation) + 2)
            local pose = {
              -- for a file with multiple takes the name should be animName|takeName
              name = string.format("%s|%s", animName, takeName),
              filename = animation,
              -- strip the animation component of the full take path
              takename = takeName,
            }

            table.insert(poses, pose)
          end
        end
      end
    end
  end

  return poses
end

------------------------------------------------------------------------------------------------------------------------
local updatePosesLocationDirectoryControl = function(directoryControl, set)
  if not anim.isRigValid(set) then
    return
  end

  local location = anim.getRigPosesLocation(set)
  directoryControl:setValue(location)

  -- set the default directory for the control using either the root directory or the poses location
  local defaultLocation = utils.demacroizeString("$(RootDir)")
  if string.len(location) > 0 then
    local fullLocation = utils.demacroizeString(location)
    if app.directoryExists(fullLocation) then
      defaultLocation = fullLocation
    end
  end
  directoryControl:setDefaultDirectory(defaultLocation)
end

local rebuildPosesTreeControl = nil
local updatePoseComboBoxes = nil

------------------------------------------------------------------------------------------------------------------------
local onAddNewPose = function(location, set)
  local dlg = ui.createFileDialog{
    style = "save;prompt",
    caption = "Save pose as",
    wildcard = "XMD files|xmd",
    directory = utils.demacroizeString(location)
  }
  if dlg:show() then
    local filepath = dlg:getFullPath()
    local fullLocation = utils.demacroizeString(location)
    local result = anim.saveKinematicFrameAsXMD(set, "AssetManager", filepath)
  end
end

------------------------------------------------------------------------------------------------------------------------
local updateButtons = function(panel, set)
  if not anim.isRigValid(set) then
    return
  end

  local location = anim.getRigPosesLocation(set)
  local enable = anim.isResource(location) and anim.isKinematicPoseValid(set, "AssetManager")
  local addButton = panel:getChild("SavePosePutton")
  if enable then
    addButton:setOnClick(
      function(self)
        onAddNewPose(location, set)
      end
    )
  end
  addButton:enable(enable)
end

------------------------------------------------------------------------------------------------------------------------
rebuildPosesTreeControl = function(treeControl, set)  
  -- clear out the tree
  local rootItem = treeControl:getRoot()
  rootItem:clearChildren();

  -- Add the root item
  treeItemsToPoses = { }
  if not anim.isRigValid(set) then
    treeControl:setSize{width = -1, height = 25}
    treeControl:setEmptyOverlayLabel("The rig is not valid.")
    treeControl:enable(false)
    return
  end

  local location = anim.getRigPosesLocation(set)
  local poses = { }
  if anim.isResource(location) then
    poses = listPoses(location)
    if table.getn(poses) == 0 then
      treeControl:setSize{width = -1, height = 25}
      treeControl:setEmptyOverlayLabel("No poses in this location.")
    else
      treeControl:setSize{width = -1, height = -1}
      local locationItem = rootItem:addChild(location)
      locationItem:setImageIndex(1)
      
      -- Add all the poses
      for _, pose in ipairs(poses) do
        local treeItem = locationItem:addChild(pose.name)
        treeItemsToPoses[treeItem] = pose
      end
    end
  else
    treeControl:setSize{width = -1, height = 25}
    treeControl:setEmptyOverlayLabel("Location is not known by the asset manager.")
  end
  
  -- enable/disable if it is a valid location
  local enable = anim.isResource(location) and table.getn(poses) > 0 
  treeControl:enable(enable)
end

------------------------------------------------------------------------------------------------------------------------
local updateComboBoxSelection = function(comboBox, set)
  if not anim.isRigValid(set) then
    return
  end

  local poseName = comboBox:getName()

  -- get the current pose for this rig
  local location = anim.getRigPosesLocation(set)
  local poses = listPoses(location)

  local rigPose = anim.getRigPose(set, poseName)
  
  -- make sure that rigPose is always valid for the loop below
  if type(rigPose) ~= "table" then
    rigPose = {
      filename = "",
      takename = "",
    }
  end
  
  if type(rigPose.filename) ~= "string" then
    rigPose.filename = ""
  end

  if type(rigPose.takename) ~= "string" then
    rigPose.takename = ""
  end

  -- look in the selected poses
  local selection = -1
  for i, pose in ipairs(poses) do
    if pose.filename == rigPose.filename and pose.takename == rigPose.takename then
      selection = i
      break
    end
  end
  
  if selection ~= -1 then
    comboBox:setSelectedIndex(selection)
    comboBox:setError(false)
  elseif rigPose.filename == "" then
    comboBox:setSelectedItem(kNothingSelectedOption)
    comboBox:setSelectionString("")
    comboBox:setError(true)
  else
    comboBox:setSelectedItem(kOtherOption)
    comboBox:setSelectionString(rigPose.filename)
    comboBox:setError(false)
  end
end

------------------------------------------------------------------------------------------------------------------------
local getWildcardString = function()
  local wildcard = ""
  for i, value in ipairs(listAnimationFileExtensions()) do
    if items == "" then
      wildcard = string.format("%s files|%s", value, value)
    else
      wildcard = string.format("%s|%s files|%s", wildcard, value, value)
    end
  end
  return wildcard
end

------------------------------------------------------------------------------------------------------------------------
local updatePoseComboBoxContents = function(comboBox, set)
  if not anim.isRigValid(set) then
    return
  end

  local poseName = comboBox:getName()

  local location = anim.getRigPosesLocation(set)
  local poses = listPoses(location)

  -- populate the combo box
  local items = { }
  for i, pose in ipairs(poses) do
    table.insert(items, pose.name)
  end
  table.insert(items, kOtherOption)
  table.insert(items, kNothingSelectedOption)

  comboBox:setItems(items)

  local onComboBoxChanged = function(self)
    local pose = { filename = "", takename = "", }
    
    local selection = self:getSelectedIndex()
    if selection <= table.getn(poses) then
      pose = poses[selection]
      anim.setRigPose(set, poseName, pose)
    else
      local currentPose = anim.getRigPose(set, poseName)
      local currentPath = currentPose.filename
      local openFileDialog = ui.createFileDialog{
        caption = string.format("Choose \"%s\" file", poseName),
        wildcard = getWildcardString(),
        directory = utils.demacroizeString(currentPath),
      }

      if openFileDialog:show() then
        anim.setRigPose(set, poseName, openFileDialog:getFullPath())
      end
    end
    
    updateComboBoxSelection(self, set)
  end

  comboBox:setOnChanged(onComboBoxChanged)
  updateComboBoxSelection(comboBox, set)
end

------------------------------------------------------------------------------------------------------------------------
local updatePoseComboBoxes = function(component, set)
  for _, child in ipairs(component:getChildren()) do
    if child:getType() == "ComboBox" then
      updatePoseComboBoxContents(child, set)
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
local addSetAsPoseMenuItem = function(menu, menuLabel, set, poseName, thePose)
  local currentPose = anim.getRigPose(set, poseName)
  if    (anim.getResourceType(thePose) == "take")
    and (thePose ~= anim.getResourceId(currentPose)) then
    menu:addItem{
      label = menuLabel,
      onClick = function(self)
        anim.setRigPose(set, poseName, thePose)
      end,
    }
  end
end

------------------------------------------------------------------------------------------------------------------------
local addRemoveMenuItem = function(menu, menuLabel, thePose)
  local resourceId = anim.findFirstAncestor(thePose, "file")
  if resourceId ~= 0 then
    menu:addItem{
      label = menuLabel,
      onClick = function(self)
        local fullFilePath = anim.getResourcePath(resourceId)
        if app.fileExists(fullFilePath) then
          local message = string.format("Are you sure you want to permanatly delete\n\"%s\"?", anim.getFileName(resourceId)) 
          local result = ui.showMessageBox(message, "ok;cancel")
          if result == "ok" then
            os.remove(fullFilePath)
          end
        end
      end,
    }
  end
end

------------------------------------------------------------------------------------------------------------------------
local addShowInExplorerMenuItems = function(menu, menuLabel, resourceId)
  if not anim.getResourceIsMissing(resourceId) then
    menu:addItem{
      label = menuLabel,
      onClick = function(self)
        anim.showResourceInExplorer(resourceId)
      end,
    }
  end
end

------------------------------------------------------------------------------------------------------------------------
local onContextualMenu = function(menu, item)
  local set = getSelectedAssetManagerAnimSet()
  local location = anim.getRigPosesLocation(set)
  local selectedPose = treeItemsToPoses[item]

  addSetAsPoseMenuItem(menu, "Set as default pose", set, "DefaultPose", selectedPose)
  addSetAsPoseMenuItem(menu, "Set as guide pose", set, "GuidePose", selectedPose)
  if menu:countItems() > 0 then
    menu:addSeparator()
  end
  addShowInExplorerMenuItems(menu, "Show in explorer", selectedPose)
  addRemoveMenuItem(menu, "Delete file", selectedPose)
end

------------------------------------------------------------------------------------------------------------------------
local onSelectionChanged = function(self)
  local item = self:getSelectedItem()
  if item then
    local selectedPose = treeItemsToPoses[item]
    if type(selectedPose) == "table" then
      local fullTakePath = string.format("%s|%s", selectedPose.filename, selectedPose.takename)
      -- todo use fullTakePath when MORPH-9674 is fixed
      anim.selectTakeInAssetManager(selectedPose.filename)
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
local panelFunction = function(selectedSet, panel)
  panel:beginVSizer{ flags = "expand" }
  panel:setBorder(1)

    panel:beginHSizer{ flags = "expand" }
      panel:addStaticText{ text = "Location" }

      local directoryControl = panel:addDirectoryControl{
        name = "DirectoryControl",
        flags = "expand",
        onMacroize = utils.macroizeString,
        onDemacroize = utils.demacroizeString,
        directory = location,
        proportion = 1,
      }
      updatePosesLocationDirectoryControl(directoryControl, selectedSet)
      componentEditor.bindHelpToWidget(directoryControl, kPoseDirectoryControl)

    panel:endSizer()

    local treeControl = panel:addTreeControl{ 
      name = "ListControl", 
      size = { height = -1 },
      flags = "expand;hideRoot;sizeToContent", 
      onSelectionChanged = onSelectionChanged
    }
    treeControl:setOnContextualMenu(onContextualMenu)
    treeControl:setImageList(treeIcons, treeIcons[1]:getWidth(), treeIcons[1]:getHeight(), true)
   
    rebuildPosesTreeControl(treeControl, selectedSet)

    panel:addVSpacer(5)

    panel:beginFlexGridSizer{ cols = 2, flags = "expand", }
    panel:setFlexGridColumnExpandable(2)

    for i, pose in ipairs(requiredPoses) do
      panel:addStaticText{ text = string.format("%s", pose), proportion = 1 }

      local comboBox = panel:addComboBox{
        name = pose,
        items = poses,
        flags = "expand",
        truncation = "path",
        proportion = 1,
      }
      componentEditor.bindHelpToWidget(comboBox, kRequiredPosesHelpText[pose])
      updatePoseComboBoxContents(comboBox, selectedSet)
    end

    panel:endSizer()

    componentEditor.addSeparator(panel)
    local addButton = panel:addButton{name = "SavePosePutton", label = "Capture Asset Manager Pose", size = { width = 150 }}
    componentEditor.bindHelpToWidget(addButton, kAddPoseButtonHelpText)
    updateButtons(panel, selectedSet)

  panel:endSizer()

  local onDirectoryControlChanged = function(self)
    local component = self:getParent()
    local toolbar = component:getChild("Toolbar")
    local treeControl = component:getChild("ListControl")

    local location = self:getValue()
    anim.setRigPosesLocation(selectedSet, location)
    updatePosesLocationDirectoryControl(self, selectedSet)
    updateButtons(component, selectedSet)
    rebuildPosesTreeControl(treeControl, selectedSet)
    updatePoseComboBoxes(component, selectedSet)
  end
  directoryControl:setOnChanged(onDirectoryControlChanged)
end

------------------------------------------------------------------------------------------------------------------------
-- When a file has changed if it is in our list of poses then we should revalidate the component
if not mcn.inCommandLineMode() then
  if type(__posesComponentAnimationFileCallback) == "function" then
    unregisterEventHandler("mcAnimationFileCreated", __posesComponentAnimationFileCallback)
    unregisterEventHandler("mcAnimationFileDestroyed", __posesComponentAnimationFileCallback)
    unregisterEventHandler("mcRigPoseChanged", __posesComponentRigPoseCallback)
    unregisterEventHandler("mcRigPoseLocationChanged", __poseLocationChangedCallback)
  end

  __posesComponentAnimationFileCallback = function(resources)
    if lastValidatedLocation and isResourceInLocation(resources, lastValidatedLocation) then
      componentEditor.updateComponentValidity("Poses")

      if componentEditor.mainScrollPanel and componentEditor.getCurrentComponentName() == "Poses" then
        local selectedSet = getSelectedAssetManagerAnimSet()
        local component = componentEditor.mainScrollPanel
        local treeControl = component:getChild("ListControl")  
        rebuildPosesTreeControl(treeControl, selectedSet)

        local component = componentEditor.mainScrollPanel
        updatePoseComboBoxes(component, selectedSet)
      end
    end
  end
  
  __posesComponentRigPoseCallback = function()
    if componentEditor.mainScrollPanel and componentEditor.getCurrentComponentName() == "Poses" then
      local selectedSet = getSelectedAssetManagerAnimSet()
      
      local component = componentEditor.mainScrollPanel
      updatePoseComboBoxes(component, selectedSet)
    end
  end
  
  __poseLocationChangedCallback = function()
    if componentEditor.mainScrollPanel and componentEditor.getCurrentComponentName() == "Poses" then
      local selectedSet = getSelectedAssetManagerAnimSet()
      local component = componentEditor.mainScrollPanel
      local treeControl = component:getChild("ListControl")  
      rebuildPosesTreeControl(treeControl, selectedSet)

      local component = componentEditor.mainScrollPanel
      updatePoseComboBoxes(component, selectedSet)
    end
  end

  registerEventHandler("mcAnimationFileCreated", __posesComponentAnimationFileCallback)
  registerEventHandler("mcAnimationFileDestroyed", __posesComponentAnimationFileCallback)
  registerEventHandler("mcRigPoseChanged", __posesComponentRigPoseCallback)
  registerEventHandler("mcRigPoseLocationChanged", __poseLocationChangedCallback)
end

components.register("Poses",  {"Euphoria"}, validateFunction, panelFunction, nil,
  {
    CanUndo = function() return true end,
    CanRedo = function() return true end,
    Undo = function() nmx.Application.new():getSceneByName("AssetManager"):undo() end,
    Redo = function() nmx.Application.new():getSceneByName("AssetManager"):redo() end,
  })
