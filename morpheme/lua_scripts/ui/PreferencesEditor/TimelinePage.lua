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
-- updates format and framerate boxes
------------------------------------------------------------------------------------------------------------------------
local updateWidgetEnableState = function(panel)
  local formatWidget = panel:getChild("TimeDisplayFormatComboBox")
  local framerateWidget = panel:getChild("TimeDisplayFramerateTextBox")

  local timeDisplayUnits = preferences.get("TimeDisplayUnits")

  formatWidget:enable(true)

  local timeDisplayFormat = preferences.get("TimeDisplayFormat")
  if timeDisplayFormat == "Custom" then
    framerateWidget:enable(true)
  else
    framerateWidget:enable(false)
  end
end

------------------------------------------------------------------------------------------------------------------------
-- updates the runtime targets preferences page on data changes
------------------------------------------------------------------------------------------------------------------------
local updatePreferencePage = function(panel)
  preferences.updateEnumWidget(panel, "TimeDisplayUnits")
  preferences.updateEnumWidget(panel, "TimeDisplayFormat")
  preferences.updateNumberWidget(panel, "TimeDisplayFramerate")

  updateWidgetEnableState(panel)
end

------------------------------------------------------------------------------------------------------------------------
-- builds the settings preferences page
------------------------------------------------------------------------------------------------------------------------
local buildPreferencePage = function(panel)
  panel:beginVSizer{ flags = "expand", proportion = 1, }
    panel:addStaticText{
      name = "Heading",
      text = "Timeline",
      font = "bold",
      flags = "parentBackground;truncate;expand;decoration",
    }

    panel:beginFlexGridSizer{ flags = "expand", cols = 6 }
      panel:addStaticText{ text = "Units", }
      local unitsWidget = preferences.addEnumWidget(panel, "TimeDisplayUnits")

      panel:addStaticText{ text = "Format", }
      local formatWidget = preferences.addEnumWidget(panel, "TimeDisplayFormat")

      panel:addStaticText{ text = "FPS", }
      preferences.addNumberWidget(panel, "TimeDisplayFramerate", 1, 60)
    panel:endSizer()

  panel:endSizer()

  local onUnitsChanged = unitsWidget:getOnChanged()
  unitsWidget:setOnChanged(
    function(self)
      local panel = self:getParent()
      onUnitsChanged(self)
      updateWidgetEnableState(panel)
    end
  )

  local onFormatChanged = formatWidget:getOnChanged()
  formatWidget:setOnChanged(
    function(self)
      local panel = self:getParent()
      onFormatChanged(self)
      preferences.updateNumberWidget(panel, "TimeDisplayFramerate")
      updateWidgetEnableState(panel)
    end
  )
end

removePreferencesPage("Timeline")
addPreferencesPage(
  "Timeline",
  {
    title = "Timeline",
    parent = "Settings",
    create = buildPreferencePage,
    update = updatePreferencePage,
  }
)