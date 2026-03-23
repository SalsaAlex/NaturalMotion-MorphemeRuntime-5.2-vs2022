------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2012 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- Rig Menu functions
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- check to see if it the animation set's rig has changes
-- returns true if the changes were saved or can be discarded
------------------------------------------------------------------------------------------------------------------------
local canChangeAnimationRig = function(set)
  if anim.hasRigChanged(set) then
    local oldRigPath = anim.getRigPath(set)

    -- ask to save changes
    shouldSave = ui.showMessageBox(
        string.format("The animation rig '%s' has unsaved changes.\nWould you like to save before changing rig?", oldRigPath),
        "yesno;cancel")

    if shouldSave == "cancel" then
      return false
    end

    -- keep attempting to save until the user cancels or discards changes.
    while shouldSave == "yes" and not anim.saveRig(set) do
      shouldSave = ui.showMessageBox(
        string.format("Failed trying to save animation rig '%s'.\nWould you like to retry?", oldRigPath),
        "yesno;cancel")
    end

    if shouldSave == "cancel" then
      return false
    end
  end

  return true
end

------------------------------------------------------------------------------------------------------------------------
-- Create a new animation rig and set it as the rig for the specified set
------------------------------------------------------------------------------------------------------------------------
local onNewAnimationRig = function()
  local set = getSelectedAssetManagerAnimSet()
  if canChangeAnimationRig(set) then
    -- find out where to save the new rig
    local saveDlg = ui.createFileDialog{ style = "save", caption = "Save Rig", wildcard = "Rig|mcarig" }
    if saveDlg:show() then
      local fullPath = saveDlg:getFullPath()
      local shouldSave = true

      -- if the file already exists then check we can overwrite it
      if app.fileExists(fullPath) then
        local path = utils.macroizeString(fullPath)
        local result = ui.showMessageBox(
          "'"..path.."' already exists.  Are you sure you want to overwrite it?",
          "ok;cancel")
        if result ~= "ok" then
          shouldSave = false
        end
      end

      -- create the new rig and set it as the rig for this set
      if shouldSave then
        local source = anim.getSelectedTakeInAssetManager()

        if anim.createRig(source.filename, fullPath) then
          anim.setRigPath(fullPath, set, true)
        end
      end
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
local onOpenAnimationRig = function()
  local set = getSelectedAssetManagerAnimSet()
  if canChangeAnimationRig(set) then
    -- get the currently loaded rig path or the default rig path if there is no current rig.
    local currentPath = anim.getPhysicsRigPath(set)
    if string.len(currentPath) == 0 then
      local defaultPath = preferences.get("DefaultAnimationRigFile")
      if type(defaultPath) == "string" then
        currentPath = defaultPath
      end
    end

    currentPath = utils.demacroizeString(currentPath)

    local openFileDialog = ui.createFileDialog{
      caption = "Open Animation Rig",
      wildcard = "Animation Rig|mcarig",
      path = currentPath,
    }
    if openFileDialog:show() then
      local fullPath = openFileDialog:getFullPath()
      anim.setRigPath(fullPath, set, true)
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
local onSaveAnimationRig = function()
  local set = getSelectedAssetManagerAnimSet()
  if not anim.saveRig(set) then
    ui.showErrorMessageBox(string.format("Error saving animation rig for set '%s'", set))
  end
end

------------------------------------------------------------------------------------------------------------------------
local onSaveAnimationFrame = function()
  local set = getSelectedAssetManagerAnimSet()
  if type(set) ~= "string" or string.len(set) == 0 then
    return
  end
  local kDisplayLowLODAnimSetPoseSaveWarning = "DisplayLowLODSetPoseSaveWarning"
  local showWarning = preferences.get(kDisplayLowLODAnimSetPoseSaveWarning)
  if type(showWarning) == "boolean" and showWarning == true then
    local selectedSetJointCount = anim.getRigSize(set)
    local hasLowerLod = false

    local parentAnimSet = getAnimSetParent(set)
    while (parentAnimSet ~= nil and hasLowerLod == false) do
      if anim.getRigSize(parentAnimSet) > selectedSetJointCount then
        hasLowerLod = true
      end
      parentAnimSet = getAnimSetParent(parentAnimSet)
    end

    if hasLowerLod then
      local message = "Saving a pose for this animation set will not contain the joints required for the" ..
          " parent set which is more detailed.\nDo you still wish to save this pose?"
      local result, ignore = ui.showMessageBox(message, "yesno;ignore")
      preferences.set{ name = kDisplayLowLODAnimSetPoseSaveWarning, value = not ignore}
      
      if result ~= "yes" then
        return
      end
    end
  end
  
  local saveDlg = ui.createFileDialog{
    style = "save;prompt",
    caption = "Save Animation Frame",
    wildcard = "XMD files|xmd"}

  if saveDlg:show() then
    if not anim.saveKinematicFrameAsXMD(set, "AssetManager", saveDlg:getFullPath()) then
      ui.showErrorMessageBox(string.format("Error saving animation frame for set '%s'", set))
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
-- check to see if the animation set's physics rig has changed.
-- returns true if the changes were saved or can be discarded
------------------------------------------------------------------------------------------------------------------------
local canChangePhysicsRig = function(set)
  if anim.hasPhysicsRigChanged(set) then
    local oldRigPath = anim.getPhysicsRigPath(set)

    -- ask to save changes
    shouldSave = ui.showMessageBox(
        string.format("The physics rig '%s' has unsaved changes.\nWould you like to save before changing rig?", oldRigPath),
        "yesno;cancel")

    if shouldSave == "cancel" then
      return false
    end

    -- keep attempting to save until the user cancels or discards changes.
    while shouldSave == "yes" and not anim.savePhysicsRig(set) do
      shouldSave = ui.showMessageBox(
        string.format("Failed trying to save physics rig '%s'.\nWould you like to retry?", oldRigPath),
        "yesno;cancel")
    end

    if shouldSave == "cancel" then
      return false
    end
  end

  return true
end

------------------------------------------------------------------------------------------------------------------------
local onNewPhysicsRig = function()
  local set = getSelectedAssetManagerAnimSet()
  if canChangePhysicsRig(set) then
    local scene = nmx.Application.new():getSceneByName("AssetManager")
    local hipJoint = nil

    -- check there is only one object selected
    local selectionList = nmx.SelectionList.new()
    scene:getSelectionList(selectionList)
    if selectionList:size() > 0 then
      local selection = nil
      for i=1,selectionList:size() do
        local node = selectionList:getNode(i)
        if node:is(nmx.sgTransformNode.ClassTypeId()) then
          if selection == nil then
            selection = node
          else
            ui.showErrorMessageBox("Please select a single joint in the animation rig before creating a physics rig.")
            return
          end
        end
      end
      -- check the selected object is a joint
      if nmx.JointNode.getJointNode(selection) then
        hipJoint = selection
      end
    end

    if hipJoint == nil then
      ui.showErrorMessageBox("Please select a joint in the animation rig before creating a physics rig.")
      return
    end

    local saveDlg = ui.createFileDialog{
      style = "save",
      caption = "Save Physics Rig",
      wildcard = "Physics Rig|mcprig" }

    if saveDlg:show() then
      local fullPath = saveDlg:getFullPath()
      local shouldSave = true
      if app.fileExists(fullPath) then
        local path = utils.macroizeString(fullPath)
        local result = ui.showMessageBox(
          "'"..path.."' already exists.  Are you sure you want to overwrite it?",
          "ok;cancel")
        if result ~= "ok" then
          shouldSave = false
        end
      end

      if shouldSave then
        local set = getSelectedAssetManagerAnimSet()

        local status, cbRef = scene:beginChangeBlock(getCurrentFileAndLine())
        if anim.createPhysicsRig(set, hipJoint:getName(), fullPath, false) then
          anim.setPhysicsRigPath(fullPath, set, true)
          validatePhysicsRigAndInformUser(set)
        end
        scene:endChangeBlock(cbRef, changeBlockInfo("Assetmanager.lua"))

        -- stops the physics rig thinking it has changed
        anim.savePhysicsRig(set)
      end
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
local onPhysicsRigMenuItemPoppedUp = function(menuItem)
  local selectedSet = getSelectedAssetManagerAnimSet()
  if type(selectedSet) == "string" and string.len(selectedSet) > 0 then
    local setType = anim.getAnimSetCharacterType(selectedSet)
    local supportsPhysicsRig = characterTypes.supportsRig(setType, "PhysicsRig")
    if supportsPhysicsRig then
      menuItem:enable(true)
      return
    end
  end
  
  menuItem:enable(false)
end

------------------------------------------------------------------------------------------------------------------------
local onRescalePhysicsRigMenuItemPoppedUp = function(menuItem)
  -- Rescaling is valid if one or more animation sets has a valid physics rig and a character type that supports
  -- physics rigs.
  --
  local enable = false

  local sets = listAnimSets()
  for _, set in ipairs(sets) do
    local characterType = anim.getAnimSetCharacterType(set)
    if characterTypes.supportsRig(characterType, "PhysicsRig") then
      if anim.isPhysicsRigValid(set) then
        enable = true
        break
      end
    end
  end

  menuItem:enable(enable)
end

------------------------------------------------------------------------------------------------------------------------
local onOpenPhysicsRig = function()
  local set = getSelectedAssetManagerAnimSet()
  if canChangePhysicsRig(set) then
    -- get the currently loaded rig path or the default rig path if there is no current rig.
    local currentPath = anim.getPhysicsRigPath(set)
    if string.len(currentPath) == 0 then
      local defaultPath = preferences.get("DefaultPhysicsRigFile")
      if type(defaultPath) == "string" then
        if string.len(defaultPath) == 0 then
          defaultPath = preferences.get("DefaultAnimationRigFile")
        end
      end

      if type(defaultPath) == "string" then
        currentPath = string.format("%s.mcprig", stripFilenameExtension(defaultPath))
      end
    end

    currentPath = utils.demacroizeString(currentPath)

    local openFileDialog = ui.createFileDialog{
      caption = "Open Physics Rig",
      wildcard = "Physics Rig|mcprig",
      path = currentPath,
    }
    if openFileDialog:show() then
      local fullPath = openFileDialog:getFullPath()
      anim.setPhysicsRigPath(fullPath, set, true)
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
local onSavePhysicsRig = function()
  local set = getSelectedAssetManagerAnimSet()
  if not anim.savePhysicsRig(set) then
    ui.showErrorMessageBox(string.format("Error saving physics rig for set '%s'", set))
  end
end

------------------------------------------------------------------------------------------------------------------------
-- given a data root node and a classTypeId this function will find all descendents of dataNodeRoot of type
-- classTypeId, get their corresponding sgTransformNode and add them to a selection list that is returned.
------------------------------------------------------------------------------------------------------------------------
local buildNodeSelectionList = function(dataNodeRoot, classTypeId)
  assert(dataNodeRoot)

  local selectionList = nmx.SelectionList.new()

  local app = nmx.Application:new()
  if not app:isNodeTypeRegistered(classTypeId) then
    return selectionList
  end

  local it = nmx.NodeIterator.new(dataNodeRoot, classTypeId)
  while it:next() do
    local node = it:node()
    local nodeInstance = node:getFirstInstance()
    local nodeTxInstance = nodeInstance:getParent()

    selectionList:add(nodeTxInstance)
  end

  return selectionList
end

------------------------------------------------------------------------------------------------------------------------
local canCreateJointLimitsForSelectedSet = function(preset)
  local app = nmx.Application:new()
  local assetManagerScene = app:getSceneByName("AssetManager")

  local selectedSet = getSelectedAssetManagerAnimSet()
  local rigRoot = anim.getRigDataRoot(assetManagerScene, selectedSet)

  local rigJoints = buildNodeSelectionList(rigRoot, nmx.JointNode.ClassTypeId())

  return app:canRunCommand("Core", "Create Limit", assetManagerScene, rigJoints, preset)
end

------------------------------------------------------------------------------------------------------------------------
local canCreateTwistSwingLimitsForSelectedSet = function()
  canCreateJointLimitsForSelectedSet("Twist Swing")
end

------------------------------------------------------------------------------------------------------------------------
-- This function will invoke the Core Create Joint Limit command with the given preset for all joints
-- in the currently selected asset manager animation set.
------------------------------------------------------------------------------------------------------------------------
local createJointLimitsForSelectedSet = function(preset)
  local app = nmx.Application:new()
  local assetManagerScene = app:getSceneByName("AssetManager")

  local selectedSet = getSelectedAssetManagerAnimSet()
  local rigRoot = anim.getRigDataRoot(assetManagerScene, selectedSet)

  local rigJoints = buildNodeSelectionList(rigRoot, nmx.JointNode.ClassTypeId())

  if app:canRunCommand("Core", "Create Joint Limit", assetManagerScene, rigJoints, preset) then
    app:runCommand("Core", "Create Joint Limit", assetManagerScene, rigJoints, preset)
  end
end

------------------------------------------------------------------------------------------------------------------------
local createTwistSwingLimitsForSelectedSet = function()
  createJointLimitsForSelectedSet("Twist Swing")
end

------------------------------------------------------------------------------------------------------------------------
-- This function will invoke the Physics Tools Create Joint Limits command with the given preset for all physics joints
-- in the currently selected asset manager animation set.
------------------------------------------------------------------------------------------------------------------------
local addPhysicsRigLimitsToSelectedSet = function(preset)
  local app = nmx.Application:new()
  local assetManagerScene = app:getSceneByName("AssetManager")

  local selectedSet = getSelectedAssetManagerAnimSet()
  local physicsRigRoot = anim.getPhysicsRigDataRoot(assetManagerScene, selectedSet)

  local physicsRigJoints = buildNodeSelectionList(physicsRigRoot, nmx.PhysicsJointNode.ClassTypeId())

  if app:canRunCommand("Physics Tools", "Create Joint Limits", assetManagerScene, physicsRigJoints, preset) then
    app:runCommand("Physics Tools", "Create Joint Limits", assetManagerScene, physicsRigJoints, preset)
  end
end

------------------------------------------------------------------------------------------------------------------------
local onAddPhysicsRigHardLimits = function()
  addPhysicsRigLimitsToSelectedSet("Hard Limit")
end

------------------------------------------------------------------------------------------------------------------------
local onAddPhysicsRigSoftLimits = function()
  addPhysicsRigLimitsToSelectedSet("Soft Limit")
end

------------------------------------------------------------------------------------------------------------------------
local onAddPhysicsRigRagdollLimits = function()
  addPhysicsRigLimitsToSelectedSet("Ragdoll Limit")
end

------------------------------------------------------------------------------------------------------------------------
addAssetManagerRigsMenu = function(menubar)
  local rigsmenu = menubar:addSubMenu{ name = "Rigs", label = "Rigs", }

  rigsmenu:addItem{ label = "New Animation Rig", onClick = function(self) onNewAnimationRig() end }
  if not mcn.isPhysicsDisabled() then
    rigsmenu:addItem{ 
      label = "New Physics Rig", 
      onClick = function(self) onNewPhysicsRig() end,
      onPoppedUp = onPhysicsRigMenuItemPoppedUp
    }
    rigsmenu:addSeparator()
  end

  rigsmenu:addItem{ label = "Open Animation Rig", onClick = function(self) onOpenAnimationRig() end }
  if not mcn.isPhysicsDisabled() then
    rigsmenu:addItem{ 
      label = "Open Physics Rig", 
      onClick = function(self) onOpenPhysicsRig() end,
      onPoppedUp = onPhysicsRigMenuItemPoppedUp
    }
  end

  rigsmenu:addSeparator()

  rigsmenu:addItem{ label = "Save Animation Rig", onClick = function(self) onSaveAnimationRig() end }
  rigsmenu:addItem{ label = "Save Animation Frame",
    onClick = function(self)
      onSaveAnimationFrame()
    end,
    onPoppedUp = function(self)
      local set = getSelectedAssetManagerAnimSet()
      local en = anim.isKinematicPoseValid(set, "AssetManager")
      self:enable(en)
    end
    }
  if not mcn.isPhysicsDisabled() then
    rigsmenu:addItem{ 
      label = "Save Physics Rig", 
      onClick = function(self) onSavePhysicsRig() end,
      onPoppedUp = onPhysicsRigMenuItemPoppedUp 
    }
  end

  rigsmenu:addSeparator()

  local animationLimitSubMenu = rigsmenu:addSubMenu{ label = "Add Animation Rig Limits", }
  animationLimitSubMenu:addItem{
    label = "Twist Swing",
    onPoppedUp = canCreateTwistSwingLimitsForSelectedSet,
    onClick = createTwistSwingLimitsForSelectedSet,
  }

  if not mcn.isPhysicsDisabled() then
    local physicsLimitSubMenu = rigsmenu:addSubMenu{ label = "Add Physics Rig Limits" }
    if nmx.Application.new():isNodeTypeRegistered("PhysicsTwistSwingNode") then
      physicsLimitSubMenu:addItem{ label = "Hard Limit", onClick = onAddPhysicsRigHardLimits }
    end

    if nmx.Application.new():isNodeTypeRegistered("PhysicsRagdollLimitNode") then
      physicsLimitSubMenu:addItem{ label = "Ragdoll Limit", onClick = onAddPhysicsRigRagdollLimits }
    end

    if not mcn.isEuphoriaDisabled() then
      physicsLimitSubMenu:addItem{ label = "Soft Limit", onClick = onAddPhysicsRigSoftLimits }
    end
  end
  
  
  rigsmenu:addItem{
    label = "Rescale Animation Rig Volumes",
    onClick = showRescaleAnimationRigDialog
  }
  
  if not mcn.isPhysicsDisabled() then
    rigsmenu:addItem{
      label = "Rescale Physics Rig",
      onClick = showRescalePhysicsDialog,
      onPoppedUp = onRescalePhysicsRigMenuItemPoppedUp
    }
  end

  rigsmenu:addSeparator()

  return rigsmenu
end
------------------------------------------------------------------------------------------------------------------------
