------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "luaAPI/UIUtils.lua"
require "ui/PreferencesEditor/PreferencesAPI.lua"
require "ui/PreferencesEditor/SettingsPage.lua"

------------------------------------------------------------------------------------------------------------------------
-- updates the runtime targets preferences page on data changes
------------------------------------------------------------------------------------------------------------------------
local updatePreferencePage = function(panel)
  local frameBufferLimitBufferCheckBox = panel:getChild("FrameBufferLimitBufferCheckBox")
  local frameBufferLimitBuffer = preferences.get("FrameBufferLimitBuffer")
  frameBufferLimitBufferCheckBox:setChecked(frameBufferLimitBuffer)

  preferences.updateNumberWidget(panel, "FrameBufferLength")
end

------------------------------------------------------------------------------------------------------------------------
-- builds the settings preferences page
------------------------------------------------------------------------------------------------------------------------
local buildPreferencePage = function(panel)
  panel:beginVSizer{ flags = "expand", proportion = 1, }
    panel:addStaticText{
      name = "Heading",
      text = "Frame buffer settings",
      font = "bold",
      flags = "parentBackground;truncate;expand;decoration",
    }

    panel:addVSpacer(5)

    panel:addCheckBox{
      name = "FrameBufferLimitBufferCheckBox",
      label = "Limit buffer size",
      onChanged = function(self)
        local _, info = preferences.get("FrameBufferLimitBuffer")
        info.value = self:getChecked()
        preferences.set(info)
      end,
    }

    panel:beginHSizer{ }
      panel:addStaticText{ text = "Buffer" }

      preferences.addNumberWidget(panel, "FrameBufferLength")

      panel:addStaticText{ text = "Seconds" }
    panel:endSizer()

  panel:endSizer()
end

removePreferencesPage("FrameBuffer")
addPreferencesPage(
  "FrameBuffer",
  {
    title = "Frame buffer",
    parent = "Settings",
    create = buildPreferencePage,
    update = updatePreferencePage,
  }
)
