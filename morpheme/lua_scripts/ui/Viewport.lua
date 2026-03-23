-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/FollowJoint.lua"
require "luaAPI/MorphemeUnitAPI.lua"

------------------------------------------------------------------------------------------------------------------------
local enableIfNoNetworkRunning = function(menuItem)
  menuItem:enable(not mcn.isNetworkRunning())
end

------------------------------------------------------------------------------------------------------------------------
local onCreatePhysicsBodyMeshViewport = function(scene, dynamic, constrained)
  local dlg = ui.createFileDialog{
    style = "open;mustExist",
    caption = "Import Prop",
    wildcard = "XMD files|xmd" }

  if dlg:show() then
    local f = dlg:getFullPath()
    if not mcn.importMesh(f, dynamic, constrained) then
      error("Error importing file '" .. f .. "'")
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
local onEnvironmentSaveAs = function()
  local saveDlg = ui.createFileDialog{
    style = "save",
    caption = "Save Environment As",
    wildcard = "Morpheme Connect environment files|mcenv"
  }

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
      mcn.saveEnvironmentAs(fullPath)
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Viewport File menu
------------------------------------------------------------------------------------------------------------------------
local addEnvironmentMenu = function(menubar, scene)
  local filemenu = menubar:addSubMenu{
    name = "EnvironmentMenu",
    label = "Environment",
    onPoppedUp = function(self)
      self:enable(not mcn.isNetworkRunning())
    end,
  }

  filemenu:addItem{
    name = "New",
    label = "New",
    onClick = function()
      -- ask to save changes
      if mcn.hasEnvironmentChanged() then
        local shouldSave = ui.showMessageBox(
          "The Environment has unsaved changes.\nWould you like to save the changes first?",
          "yesno;cancel")
        if shouldSave == "yes" then
          if not mcn.saveEnvironment() then
            return
          end
        end
        if shouldSave == "cancel" then
          return
        end
      end
      mcn.clearEnvironment()
    end,
    onPoppedUp = function(menuItem)
      local networkRunning = mcn.isNetworkRunning()
      menuItem:enable(not networkRunning)
    end,
  }

  filemenu:addItem{
    name = "Open",
    label = "Open",
    onClick = function()
      -- ask to save changes
      if mcn.hasEnvironmentChanged() then
        local shouldSave = ui.showMessageBox(
          "The Environment has unsaved changes.\nWould you like to save the changes first?",
          "yesno;cancel")
        if shouldSave == "yes" then
          if not mcn.saveEnvironment() then
            return
          end
        end
        if shouldSave == "cancel" then
          return
        end
      end

      mcn.loadEnvironment()
    end,
    onPoppedUp = function(menuItem)
      local networkRunning = mcn.isNetworkRunning()
      menuItem:enable(not networkRunning)
    end,
  }

  filemenu:addSeparator()

  filemenu:addItem{
    name = "Save",
    label = "Save",
    onClick = function()
      mcn.saveEnvironment()
    end,
    onPoppedUp = function(menuItem)
      local environmentChanged = mcn.hasEnvironmentChanged()
      local networkRunning = mcn.isNetworkRunning()
      local enable = environmentChanged and not networkRunning
      menuItem:enable(enable)
    end,
  }

  filemenu:addItem{
    name = "SaveAs",
    label = "Save As",
    onClick = onEnvironmentSaveAs,
    onPoppedUp = function(menuItem)
      local environmentChanged = mcn.hasEnvironmentChanged()
      local networkRunning = mcn.isNetworkRunning()
      local enable = environmentChanged and not networkRunning
      menuItem:enable(enable)
    end,
  }
end

------------------------------------------------------------------------------------------------------------------------
-- Viewport Create menu
------------------------------------------------------------------------------------------------------------------------
local addCreateMenu = function(menubar, scene)
  local createMenu = menubar:addSubMenu{
    name = "CreateMenu",
    label = "Create",
    onPoppedUp = function(self)
      self:enable(not mcn.isNetworkRunning())
    end,
  }

  local addCreateShapeMenuItem = function(menu, name, constrained, typeName)
    typeName = typeName or name
    menu:addItem{
      label = name,
      onClick = function()
        local nmxApp = nmx.Application.new()
        local status, cbRef = scene:beginChangeBlock(getCurrentFileAndLine());
        local parent = mcn.getEnvironmentSceneRoot()

        local sceneParent
        local selectedItem

        local sl = nmx.SelectionList.new()
        sl:add(parent)

        local bodyPreset = "Free"
        if constrained then
          bodyPreset = "Constrained"
        end
        local commandReturn = nmxApp:runCommand("Physics Tools", "Create Environment Physics Body", scene, sl, bodyPreset)
        if commandReturn:asString() ~= "kSuccess" then
          app.error("\"Create Environment Physics Volume\" failed")
          return
        end

        scene:getSelectionList(sl) -- save `sl` for the eventual selection after adding the volume
        local returnSl = nmx.SelectionList.new(sl)

        -- If a constraint was created, we need to change the selection list to contain only the physics body.
        if constrained then
          -- Get the environment body created with the constraint
          local sgTransformNodeType = nmx.sgTransformNode.ClassTypeId()
          local constraintTx = returnSl:getLastSelectedNode(sgTransformNodeType)
          if constraintTx == nil then
            app.error("Cannot find newly created constraint in selection list. Rolling back")
            scene:rollback(cbRef)
            return
          end

          local bodyTx = constraintTx:getFirstChild(sgTransformNodeType)
          if bodyTx == nil then
            app.error("Cannot find newly created constraint's body sgTransform in selection list. Rolling back")
            scene:rollback(cbRef)
            return
          end

          returnSl:clear()
          returnSl:add(bodyTx)
        end

        commandReturn = nmxApp:runCommand("Physics Tools", "Create Physics Volume", scene, returnSl, typeName)
        if commandReturn:asString() ~= "kSuccess" then
          app.error("\"Create Physics Volume\" failed")
          scene:rollback(cbRef)
          return
        end

        scene:getSelectionList(returnSl)

        local volume = returnSl:getLastSelectedNode(nmx.PhysicsVolumeNode.ClassTypeId())
        if volume == nil then
          app.error("Cannot find newly created volume in selection list. Rolling back")
          scene:rollback(cbRef)
          return
        end

        local runtimeScaleFactor = preferences.get("RuntimeAssetScaleFactor")

        -- scale according to viewport scaling (so the new shape is properly visible)
        local shapeNode = volume:getShape() -- get the display shape node (ie Box/Capsule/SphereNode)
        if name == "Box" then
          shapeNode:setWidth(shapeNode:getWidth() / runtimeScaleFactor)
          shapeNode:setHeight(shapeNode:getHeight() / runtimeScaleFactor)
          shapeNode:setDepth(shapeNode:getDepth() / runtimeScaleFactor)
        elseif name == "Capsule" then
          shapeNode:setHeight(shapeNode:getHeight() / runtimeScaleFactor)
          shapeNode:setRadius(shapeNode:getRadius() / runtimeScaleFactor)
        elseif name == "Sphere" then
          shapeNode:setRadius(shapeNode:getRadius() / runtimeScaleFactor)
        end

        local worldUpAxis = preferences.get("WorldUpAxis")
        if worldUpAxis == "Z Axis" then
          local volumeTx = returnSl:getLastSelectedNode(nmx.TransformNode.ClassTypeId())
          if volumeTx == nil then
            app.error("Cannot find newly created volume TransformNode in selection list.")
          else
            -- generate a quaternion with a rotation of 90 degrees about the x-axis
            local xaxis = nmx.Vector3.new(1.0, 0.0, 0.0)
            local rotation = nmx.Quat.new(xaxis, math.pi / 2.0)

            -- rotate the object so it looks as if it is oriented the same regardless of the up axis
            volumeTx:setRotation(rotation)
          end
        end

        -- set the selection back to what was returned from create environment body command
        scene:setSelectionList(sl)

        scene:endChangeBlock(cbRef, changeBlockInfo("Create " .. name))
      end,
      onPoppedUp = function(menuItem)
        if mcn.isNetworkRunning() then
          menuItem:enable(false)
          return
        end

        -- Check the canRun for the create environment physics body command
        local parent = mcn.getEnvironmentSceneRoot()
        local sl = nmx.SelectionList.new()
        sl:add(parent)

        local bodyPreset = "Free"
        if constrained then bodyPreset = "Constrained" end
        local canRun = nmx.Application.new():canRunCommand("Physics Tools", "Create Environment Physics Body", scene, sl, bodyPreset)

        menuItem:enable(canRun)
      end
    }
  end

  local addCreateTriggerShapeMenuItem = function(menu, name, nodeType, oPU)
    menu:addItem{
      label = name,
      onClick = function()
        local status, cbRef = scene:beginChangeBlock(getCurrentFileAndLine());
        local parent = mcn.getEnvironmentSceneRoot()
        
        local triggerVolume = nmx.TriggerVolumeNode.createTriggerVolume(parent, nodeType, 1.0)
        
        if triggerVolume ~= nil then
          local sl = nmx.SelectionList.new()
          sl:add(triggerVolume)
          scene:setSelectionList(sl)
        end
        scene:endChangeBlock(cbRef, changeBlockInfo("Create " .. name))
      end,
      onPoppedUp = oPU
    }
  end
  
  addCreateShapeMenuItem(createMenu, "Box", false)
  addCreateShapeMenuItem(createMenu, "Capsule", false)
  addCreateShapeMenuItem(createMenu, "Sphere", false)
  
  addCreateTriggerShapeMenuItem(createMenu, "Trigger Box Volume", nmx.BoxNode.ClassTypeId(), enableIfNoNetworkRunning)
  addCreateTriggerShapeMenuItem(createMenu, "Trigger Sphere Volume", nmx.SphereNode.ClassTypeId(), enableIfNoNetworkRunning)
  
  createMenu:addItem{ label = "Mesh...",
    onClick = function() onCreatePhysicsBodyMeshViewport(scene, false, false) end,
    onPoppedUp = enableIfNoNetworkRunning
  }

  createMenu:addSeparator()

  if not mcn.isPhysicsDisabled() then
    local constrainedMenu = createMenu:addSubMenu{ label = "Constrained" }
    addCreateShapeMenuItem(constrainedMenu, "Box", true)
    addCreateShapeMenuItem(constrainedMenu, "Capsule", true)
    addCreateShapeMenuItem(constrainedMenu, "Sphere", true)
    constrainedMenu:addItem{ label = "Mesh...",
      onClick = function() onCreatePhysicsBodyMeshViewport(scene, true, true) end,
      onPoppedUp = enableIfNoNetworkRunning
    }
  end

  createMenu:addSeparator()

  createMenu:addItem{
    label = "Character Start Point",
    onClick = function()
      mcn.createCharacterStartPoint()
    end,
    onPoppedUp = enableIfNoNetworkRunning,
  }
end

------------------------------------------------------------------------------------------------------------------------
local getShowMenuTable = function ()
  local retTable = {
    -- Animation
    { name = "Animation",
      items = {
        { name = "Joints", nodes = {"JointNode", "MorphemeMainSkeletonNode"} },
        { name = "Character Controller", nodes = {"CharacterControllerNode"} },
      },
    },

    -- Physics
    { name = "Environment and Physics", availability = "Physics",
      items = {
        { name = "Volumes and Constraints",
          nodes = {"EnvironmentPhysicsConstraintNode", "PhysicsVolumeNode", "PhysicsBodyNode"}
        },
        { name = "Preview Volumes", nodes = {"PreviewPhysicsVolumeNode"}},
        { name = "Preview Joints", nodes = {"PreviewPhysicsJointNode"}},
        { name = "Preview Joint Limits", nodes = {"PhysicsJointLimitNode"}},
        { name = "Preview Soft Joint Limits", nodes = {"PhysicsSoftTwistSwingNode"}, availability = "Euphoria"},
        { name = "Preview Self Avoidance", nodes = {"SelfAvoidanceNode"}, availability = "Euphoria" },
        { name = "Ghost Volumes", nodes = {"PreviewGhostPhysicsVolumeNode"}},
        { name = "Ghost Joints", nodes = {"PreviewGhostPhysicsJointNode"}},
      },
    },

    -- Debug
    { name = "Debug",
      items = {
        { name = "Script Output", nodes = {"DebugOutputNode"} },
        { name = "Preview Skeleton", nodes = {"MorphemePreviewSkeletonNode"} },
        { name = "Low-level Euphoria", subMenu = true, availability = "Euphoria" },
     }
    }
  }

  -- append the "Other" section to the end of the show menu.
  local otherItem =
    { name = "Other",
      items = {
        { name = "Meshes", nodes = {"MeshNode"} },
        { name = "Character Start Point", nodes = {"CharacterStartPointNode"}},
      },
    }

  -- The physics body and volume can be created in the non-physics sku.
  if mcn.isPhysicsDisabled() then
    table.insert(otherItem["items"], { name = "Volumes", nodes = {"PhysicsVolumeNode", "PhysicsBodyNode"}})
  end
  table.insert(otherItem["items"], { name = "Trigger Volumes", nodes = {"TriggerVolumeNode"} })

  table.insert(retTable, otherItem)

  return retTable
end

------------------------------------------------------------------------------------------------------------------------
local kUnselectableNodes = {
  "JointNode",
  "MorphemeMainSkeletonNode",
  "CharacterControllerNode",
  "DebugOutputNode",
  "EnvironmentNode",
  "MorphemeSkeletonNode",
  "PhysicsVolumeNode",
  "PreviewPhysicsVolumeNode",
  "PreviewPhysicsJointNode",
  "PhysicsJointLimitNode",
  "PhysicsSoftTwistSwingNode",
  "SelfAvoidanceNode",
  "PreviewGhostPhysicsVolumeNode",
  "PreviewGhostPhysicsJointNode",
  "MorphemePreviewSkeletonNode",
  "MeshNode",
}

------------------------------------------------------------------------------------------------------------------------
local onEuphoriaLowLevel = function(menu)
  local mkToggle = function(theNode)
    return function()
      local db = theNode:getDatabase()
      local blockStatus, cbRef = db:beginChangeBlock(getCurrentFileAndLine())
      
        theNode:setIsOn(not theNode:getIsOn())
        local isOn = theNode:getIsOn()
        local visibility = theNode:getLimbVisibility();
        if not visibility:empty() then
          for i = 1, visibility:size() do
            visibility:set(i, isOn)
          end
          theNode:setLimbVisibility(visibility);
        end
      
      db:endChangeBlock(cbRef, changeBlockInfo("Toggle debug draw"))
    end
  end
  
  menu:clear()
  local app = nmx.Application.new()
  local roamingSettings = app:getRoamingUserSettings()
  local settingsNode = roamingSettings:getNodeFromPath("|EuphoriaSettings")
  local debugDrawSettingNodeId = app:lookupTypeId("DebugDrawSettingNode")
  local it = nmx.NodeIterator.new(settingsNode, debugDrawSettingNodeId)
  local hasItems = false
  while it:next() do
    hasItems = true 
    local theNode = it:node()
    menu:addCheckedItem{
      label = theNode:getName(),
      onClick = mkToggle(theNode),
      checked = theNode:getIsOn()
    }
  end
  
  if hasItems then
    menu:addSeparator()
  end
  
  local layoutManager = ui.getWindow("MainFrame|LayoutManager")
  local isShown = layoutManager:isShown("Runtime Debugging")
  if isShown then
    local window = layoutManager:getWindow("Runtime Debugging")
    isShown = window:isWindowSelected("Euphoria Debug Draw")
  end
  
  menu:addCheckedItem{
    label = "Euphoria Debug Draw",
    onClick = function()
      local window = layoutManager:showWindow{name = "Runtime Debugging", show = true}
      window:selectWindow("Euphoria Debug Draw")
    end,
    checked = isShown
 }
end

------------------------------------------------------------------------------------------------------------------------
-- 
------------------------------------------------------------------------------------------------------------------------
local enableToolButton = function (button, enable)
  if button then
    button:enable(enable)
  end
end

------------------------------------------------------------------------------------------------------------------------
local addSkinMenu = function(menubar)

  ----------------------------------------------------------------------------------------------------------------------
  local mkSelectSkin = function(startPoint, skinName)
    return function()
      setCharacterStartPoint(startPoint.name, {skin = skinName})
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  local addSkinsToMenu = function(menu, startPoint, prefix)
    local skins = anim.getRigSkins(startPoint.animationSet)
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
      local currentSkin = anim.getRigSkin(startPoint.skin, startPoint.animationSet)
      local currentSkinExists = currentSkin and app.fileExists(utils.demacroizeString(currentSkin.Path))
      for _, skinName in skinNames do
        local checked = currentSkinExists and skinName == currentSkin.Name
        menu:addCheckedItem{
          label = prefix .. skinName,
          checked = checked,
          onClick = mkSelectSkin(startPoint, skinName),
        }
      end
    end
  end
  
  ----------------------------------------------------------------------------------------------------------------------
  local buildMenu = function(menu)
    local startPoints = listCharacterStartPoints()
    local totalStartPoints = table.getn(startPoints)
    if totalStartPoints > 1 then
      for _, startPoint in startPoints do
        menu:addItem{ label = string.format("%s (%s)", startPoint.name, startPoint.animationSet), style = "subtitle" }
        addSkinsToMenu(menu, startPoint, "")
      end
    elseif totalStartPoints == 1 then
      for _, startPoint in startPoints do
        addSkinsToMenu(menu, startPoint, "")
      end
    else
      menu:addItem{ label = string.format("No character start point", selectedSet), enable = false }
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
-- Object creation functions.
------------------------------------------------------------------------------------------------------------------------
local gPlaybackBegin = nil
local gPlaybackEnd = nil
addViewport = function(layoutManager)
  local panel = layoutManager:addPanel{ name = "PreviewViewport", caption = "Preview" }

  panel:setBorder(0)
  panel:beginVSizer{ proportion = 1, flags = "expand" }
    local menubar = panel:addMenuBar{ name = "MenuBar", proportion = 0, flags = "expand" }
    panel:beginHSizer{ proportion = 1, flags = "expand" }
      panel:addHSpacer(2)
      panel:beginVSizer{ proportion = 0, flags = "expand" }
        panel:addVSpacer(4)
        local toolbar = panel:addToolBar{
          name = "ViewportToolBar",
          proportion = 0,
          orientation = "vertical",
        }
      panel:endSizer()
      panel:addHSpacer(2)
      local viewport = panel:addStockWindow{ type = "Viewport", name = "Viewport", proportion = 5, flags = "expand" }
    panel:endSizer()
  panel:endSizer()
  

  local scene = nmx.Application.new():getSceneByName("Network")

  addEnvironmentMenu(menubar, scene)

  local readOnly = function()
    return mcn.isNetworkRunning()
  end

  local viewportPanel = viewport:getChild("Viewport")
  viewportPanel:frame()

  local viewportEditExtraTools = function(editMenu)
    if not mcn.isPhysicsDisabled() then
      local rayCastTypes = rayCastTool.getRayCastTypes()
      if table.getn(rayCastTypes) > 0 then
        editMenu:addCheckedItem{
          label = "Ray Cast Tool",
          onClick = function()
            viewportPanel:setCurrentTool("RayCastTool")
          end,
          onPoppedUp = function(menuItem)
            menuItem:enable(mcn.isNetworkRunning())
            menuItem:setChecked(viewportPanel:getCurrentTool() == "RayCastTool")
          end
        }
      end

      editMenu:addCheckedItem{
        label = "Mouse Forces Tool",
        onClick = function()
          viewportPanel:setCurrentTool("MouseForcesTool")
        end,
        onPoppedUp = function(menuItem)
          menuItem:enable(mcn.isNetworkRunning())
          menuItem:setChecked(viewportPanel:getCurrentTool() == "MouseForcesTool")
        end
      }

      editMenu:addCheckedItem{
        label = "Impulse Tool",
        onClick = function()
          viewportPanel:setCurrentTool("ApplyForceTool")
        end,
        onPoppedUp = function(menuItem)
          menuItem:enable(mcn.isNetworkRunning())
          menuItem:setChecked(viewportPanel:getCurrentTool() == "ApplyForceTool")
        end
      }
    end
  end
  
  local envRoot = mcn.getEnvironmentSceneRoot()
  addEditMenu(menubar, scene, viewportPanel, true, readOnly, envRoot:getPath(scene:getRoot()), viewportEditExtraTools)

  addCreateMenu(menubar, scene)
  
  -- Colour modes in the display menu
  local colourModes = {}
  table.insert(colourModes, { MenuName = "Object Colour", ColourMode = "Object" })
  table.insert(colourModes, { MenuName = "Collision Colour", ColourMode = "Collision" })

  if not mcn.isEuphoriaDisabled() then
    table.insert(colourModes, { MenuName = "Euphoria Control Colour", ColourMode = "Euphoria" })
  end

  local addAdditionalFrameMenuItems = function(menu, viewportPanel)
    local viewport = viewportPanel:getViewport()
    menu:addItem{
    label = "Frame All",
    onClick = function()
      viewport:frame()
      -- Force a compute incase we are inside a change block
      nmx.Application.new():getSceneByName(viewport:getSceneName()):update()
    end
    }
  end
  
  local commonViewportPanel, viewMenu, dispMenu, showMenu, selectionMenu = addCommonViewportMenus(
    "Network",
    menubar,
    viewportPanel,
    scene,
    readOnly,
    getShowMenuTable(),
    kUnselectableNodes,
    colourModes,
    addAdditionalFrameMenuItems
  )
  
  local lowLevelEuphoria = showMenu:findSubMenu("Low-level Euphoria")
  if lowLevelEuphoria then
    lowLevelEuphoria:setOnPoppedUp(onEuphoriaLowLevel)
  end

  local rayCastToolButton = nil
  local mouseForcesTool = nil
  local applyForceTool = nil
  local viewportToolbarExtraTools = function(toolBar)
    if not mcn.isPhysicsDisabled() then
      local rayCastTypes = rayCastTool.getRayCastTypes()
      if table.getn(rayCastTypes) > 0 then
        rayCastToolButton = toolBar:addButton{
          name = "RayCastTool",
          image = app.loadImage("toolIcons\\ShotTool.png"),
          helpText = "Ray Cast Tool",
          onClick = function()
            viewportPanel:setCurrentTool("RayCastTool")
          end,
        }
        rayCastToolButton:enable(false)

        rayCastToolButton:setOnRightClick(
          function(self)
            local rayCastTypes = rayCastTool.getRayCastTypes()

            -- only pop up the menu if there are registered types
            --
            local count = table.getn(rayCastTypes)
            if count > 0 then
              local parent = self:getParent()
              local menu = parent:createPopupMenu()

              for i = 1, count do
                local rayCastType = rayCastTypes[i]

                local checked = (rayCastTool.getSelectedRayCastType() == rayCastType)
                menu:addCheckedItem{
                  label = rayCastType,
                  checked = checked,
                  onClick = function(self)
                    rayCastTool.setSelectedRayCastType(rayCastType)
                  end,
                }
              end

              local positionSelf = self:getPosition()
              local sizeSelf = self:getSize()
              menu:popup{
                x = positionSelf.x,
                y = positionSelf.y + sizeSelf.width,
              }
            end
          end
        )
      end

      mouseForcesTool = toolBar:addButton{
        name = "MouseForcesTool",
        label = "Mouse Forces Tool",
        helpText = "Mouse Forces Tool",
        image = app.loadImage("toolIcons\\MouseForcesTool.png"),
        flags = "expand;parentBackground",
        onClick = function()
          viewportPanel:setFocus()
          viewportPanel:setCurrentTool("MouseForcesTool")
        end,
      }
      mouseForcesTool:enable(false)

      applyForceTool = toolBar:addButton{
        name = "ApplyForceTool",
        label = "Impulse Tool",
        helpText = "Impulse Tool",
        image = app.loadImage("toolIcons\\ApplyForceTool.png"),
        flags = "expand;parentBackground",
        onClick = function()
          viewportPanel:setFocus()
          viewportPanel:setCurrentTool("ApplyForceTool")
        end,
      }
      applyForceTool:enable(false)

      local defaultOnToolChanged = viewportPanel:getOnToolChanged()
      viewportPanel:setOnToolChanged(function (tool)
        defaultOnToolChanged(tool)

        local networkRunning = mcn.isNetworkRunning()

        rayCastToolButton:setChecked(networkRunning and tool == "RayCastTool")
        mouseForcesTool:setChecked(networkRunning and tool == "MouseForcesTool")
        applyForceTool:setChecked(networkRunning and tool == "ApplyForceTool")
      end)
    end
  end

  addViewportToolbar(toolbar, scene, viewportPanel, viewportToolbarExtraTools)

  -- set the default options
  setToDefault(viewportPanel, "Environment Editing", kUnselectableNodes)

  -- set the default display mode to normal
  viewportPanel:setDisplayMode("Shaded")
  
  -- by default hide character controllers
  viewportPanel:setShouldRenderTypeByName("CharacterControllerNode", false)
  
  addSkinMenu(menubar)
  addPresetMenu(menubar, viewportPanel, scene, kUnselectableNodes)

  local toolbarPath = "MainFrame|LayoutManager|PreviewViewport|ViewportToolBar"
  local selectTool = ui.getWindow(toolbarPath .. "|SelectTool")
  local translateTool = ui.getWindow(toolbarPath .. "|TranslateTool")
  local rotateTool = ui.getWindow(toolbarPath .. "|RotateTool")
  local showManipsTool = ui.getWindow(toolbarPath .. "|ShowManipsTool")
  
  local lockSelection = ui.getWindow(toolbarPath .. "|LockSelection")
  local clickThroughSelection = ui.getWindow(toolbarPath .. "|ClickThroughSelection")

  return panel
end

-- store the tool the user is using when the network is started and restore
local toolStates = {
  editTool = "SelectTool",
  previewTool = "SelectTool",
}

registerEventHandler(
  "mcPlaybackBegin",
  function()
    local viewport = ui.getWindow("MainFrame|LayoutManager|PreviewViewport|Viewport|Viewport")
    
    if viewport then
      local toolbar = ui.getWindow("MainFrame|LayoutManager|PreviewViewport|ViewportToolBar")

      toolStates.editTool = viewport:getCurrentTool()
      viewport:setCurrentTool(toolStates.previewTool)

      enableToolButton(toolbar:getChild("SelectTool"), false)
      enableToolButton(toolbar:getChild("TranslateTool"), false)
      enableToolButton(toolbar:getChild("RotateTool"), false)
      enableToolButton(toolbar:getChild("ShowManipsTool"), false)

      enableToolButton(toolbar:getChild("RayCastTool"), true)
      enableToolButton(toolbar:getChild("MouseForcesTool"), true)
      enableToolButton(toolbar:getChild("ApplyForceTool"), true)

      enableToolButton(toolbar:getChild("LockSelection"), false)
      enableToolButton(toolbar:getChild("ClickThroughSelection"), false)
    end
  end
)

registerEventHandler(
  "mcPlaybackEnd",
  function()
    local viewport = ui.getWindow("MainFrame|LayoutManager|PreviewViewport|Viewport|Viewport")

    if viewport then
      local toolbar = ui.getWindow("MainFrame|LayoutManager|PreviewViewport|ViewportToolBar")
      
      toolStates.previewTool = viewport:getCurrentTool()
      viewport:setCurrentTool(toolStates.editTool)

      enableToolButton(toolbar:getChild("SelectTool"), true)
      enableToolButton(toolbar:getChild("TranslateTool"), true)
      enableToolButton(toolbar:getChild("RotateTool"), true)
      enableToolButton(toolbar:getChild("ShowManipsTool"), true)

      enableToolButton(toolbar:getChild("RayCastTool"), false)
      enableToolButton(toolbar:getChild("MouseForcesTool"), false)
      enableToolButton(toolbar:getChild("ApplyForceTool"), false)

      enableToolButton(toolbar:getChild("LockSelection"), true)
      enableToolButton(toolbar:getChild("ClickThroughSelection"), true)
    end
  end
)