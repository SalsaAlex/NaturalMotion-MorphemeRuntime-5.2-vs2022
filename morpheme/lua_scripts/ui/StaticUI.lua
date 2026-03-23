------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

-- ensure the scripts for the viewport api and creating the menubar are included
require "ui/MainMenuBar.lua"

local lockAnimationSetsButton = nil

------------------------------------------------------------------------------------------------------------------------
-- updateLockAnimationSetsButton
------------------------------------------------------------------------------------------------------------------------
local updateLockAnimationSetsButton = function()
  lockAnimationSetsButton:setChecked(anim.areAnimationSetsLocked())
end

------------------------------------------------------------------------------------------------------------------------
-- toggleAnimationSetsLocked
------------------------------------------------------------------------------------------------------------------------
local toggleAnimationSetsLocked = function()
  anim.setAnimationSetsLocked(not anim.areAnimationSetsLocked())
end

------------------------------------------------------------------------------------------------------------------------
-- createStaticUI
------------------------------------------------------------------------------------------------------------------------
createStaticUI = function()
  -- MainFrame is the only uniquely named window, added by the application automatically.
  local mainFrame = ui.getWindow("MainFrame")
  local linkImage = app.loadImage("Linked.png")
  local unlinkImage = app.loadImage("unlinked.png")

  -- Add windows vertically
  mainFrame:beginVSizer()

    -- First window is main menu bar which contains file, edit menus etc.
    safefunc(addMainMenuBar, mainFrame)

    -- Main layout manager
    mainFrame:addLayoutManager{ name = "LayoutManager", proportion = 1, flags = "expand" }

    -- Playback timeline
    mainFrame:addStockWindow{ type = "BufferedTimeline", proportion = 0, flags = "expand" }

    -- Script panel, status bar, animation set combo box and transport controls
    mainFrame:beginHSizer{ proportion = 0, flags = "expand" }
      mainFrame:addStockWindow{ type = "StatusBar", proportion = 1, flags = "expand" }
      mainFrame:addHSpacer(2)
      mainFrame:beginVSizer{ proportion = 0, flags = "expand" }
        mainFrame:addVSpacer(4)
        mainFrame:addStaticText{ text = "Animation Set:"}
      mainFrame:endSizer()
      mainFrame:addStockWindow{ type = "AnimSetComboBox", proportion = 0, flags = "centre" }

      lockAnimationSetsButton = mainFrame:addButton{
        name = "Link", label = "",
        image = unlinkImage, selectedImage = linkImage, flags = "expand;imageCheck",
        helpText = "Link asset manager animation set selection to runtime animation set selection",
        onClick = toggleAnimationSetsLocked,
        size = { width = linkImage:getWidth(), linkImage = linkImage:getHeight() },
      }

      lockAnimationSetsButton:setToolTip("Link runtime and asset manager animation set")
      updateLockAnimationSetsButton()
      mainFrame:addHSpacer(2)

      mainFrame:addStockWindow{ type = "TransportControls", proportion = 0, flags = "expand" }
    mainFrame:endSizer()

  mainFrame:endSizer()

  -- now init the user static ui if there is any.
  safefunc(userInitStaticUI)
end

if not mcn.inCommandLineMode() then
  registerEventHandler("mcAnimationSetsLockedChange", updateLockAnimationSetsButton)
  registerEventHandler("mcFileOpenEnd", updateLockAnimationSetsButton)
  registerEventHandler("mcFileNewEnd", updateLockAnimationSetsButton)
end
