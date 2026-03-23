------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2012 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- include other required scripts
------------------------------------------------------------------------------------------------------------------------
require [[ui/StaticUI.lua]]
require [[rigBuilder/CopyRig_GUI.lua]]
require [[rigBuilder/MiniTools.lua]]
require [[rigBuilder/CreateAnimRig.lua]]
require [[rigBuilder/CreatePhysicsRig.lua]]

local olderUserInitStaticUI = userInitStaticUI

------------------------------------------------------------------------------------------------------------------------
-- nil addRiggingToolsMenu(MenuBar mainMenuBar)
------------------------------------------------------------------------------------------------------------------------
local addRiggingToolsMenu = function(mainMenuBar)

  local riggingToolsMenu = mainMenuBar:addSubMenu{ name = "RiggingToolsMenu", label = "&Rigging Tools" }
  riggingToolsMenu:addItem{
    name = "CopyRigTool",
    label = "&Create R* Physics Rig",
    onClick = function(self)
      createCopyRigWindow()
    end,
  }
  riggingToolsMenu:addItem{
    name = "CreateAnimationRig",
    label = "&Create Default Anim Rig",
    onClick = function(self)
      creatAnimRigWindowFunc()
    end,
  }
  riggingToolsMenu:addItem{
    name = "CreatePhysicsRig",
    label = "&Create Default Physics Rig",
    onClick = function(self)
      createPhysicsRigWindowFunc()
    end,
  }
  riggingToolsMenu:addItem{
    name = "MiniTools",
    label = "&Mini Tools",
    onClick = function(self)
      miniToolsWindowFunc()
    end,
  }
  
  
end


------------------------------------------------------------------------------------------------------------------------
-- nil userInitStaticUI()
------------------------------------------------------------------------------------------------------------------------
userInitStaticUI = function()

  local mainFrame = ui.getWindow("MainFrame")
  local mainMenuBar = mainFrame:getChild("MainMenu")

  addRiggingToolsMenu(mainMenuBar)

  safefunc(olderUserInitStaticUI)
end
