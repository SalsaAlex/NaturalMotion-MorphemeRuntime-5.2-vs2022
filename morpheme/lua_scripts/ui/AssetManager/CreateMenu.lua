------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- Create menu functions
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- Adds a menu item for a given nmx command with optional preset
------------------------------------------------------------------------------------------------------------------------
local addCommandMenuItem = function(menu, label, namespace, command, preset)
  local menuitem = menu:addItem{
    label = label,
    onPoppedUp = function(self)
      local scene, selectionList = nmx.getSceneAndSelectionList("AssetManager")

      local enabled = false

      local app = nmx.Application.new()
      if preset then
        enabled = app:canRunCommand(namespace, command, scene, selectionList, preset)
      else
        enabled = app:canRunCommand(namespace, command, scene, selectionList)
      end

      self:enable(enabled)
    end,
    onClick = function(self)
      local scene, selectionList = nmx.getSceneAndSelectionList("AssetManager")

      local app = nmx.Application.new()
      if preset then
        app:runCommand(namespace, command, scene, selectionList, preset)
      else
        app:runCommand(namespace, command, scene, selectionList)
      end
    end,
  }

  return menuitem
end

------------------------------------------------------------------------------------------------------------------------
-- addAssetManagerCreateMenu
------------------------------------------------------------------------------------------------------------------------
addAssetManagerCreateMenu = function(menubar)
  local createMenu = menubar:addSubMenu{ label = "Create" }

  local jointLimitSubMenu = createMenu:addSubMenu{ label = "Joint Limit" }

  addCommandMenuItem(jointLimitSubMenu, "Twist Swing", "Core", "Create Joint Limit", "Twist Swing")

  if not mcn.isPhysicsDisabled() then
    createMenu:addSeparator()
    local physicsBodySubMenu = createMenu:addSubMenu{ label = "Physics Volume" }

    addCommandMenuItem(physicsBodySubMenu, "Box", "Physics Tools", "Create Physics Volume", "Box")
    addCommandMenuItem(physicsBodySubMenu, "Capsule", "Physics Tools", "Create Physics Volume", "Capsule")
    addCommandMenuItem(physicsBodySubMenu, "Sphere", "Physics Tools", "Create Physics Volume", "Sphere")

    local physicsJointLimitSubMenu = createMenu:addSubMenu{ label = "Physics Joint Limit" }

    addCommandMenuItem(physicsJointLimitSubMenu, "Hard Limit", "Physics Tools", "Create Joint Limits", "Hard Limit")
    addCommandMenuItem(physicsJointLimitSubMenu, "Soft Limit", "Physics Tools", "Create Joint Limits", "Soft Limit")
  end

  if not mcn.isEuphoriaDisabled() then
    createMenu:addSeparator()

    addCommandMenuItem(createMenu, "Interaction Proxy", "morpheme:connect", "Create Interaction Proxy", "Interaction Proxy")
  end

  return createMenu
end

------------------------------------------------------------------------------------------------------------------------
