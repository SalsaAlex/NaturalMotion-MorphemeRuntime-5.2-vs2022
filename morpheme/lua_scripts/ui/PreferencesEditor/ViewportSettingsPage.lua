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
  preferences.updateNumberWidget(panel, "CameraNearClipPlane")
  preferences.updateNumberWidget(panel, "CameraFarClipPlane")
  preferences.updateNumberWidget(panel, "GridSpacing")
  preferences.updateNumberWidget(panel, "GridDivisions")
  preferences.updateNumberWidget(panel, "GridNearFade")
  preferences.updateNumberWidget(panel, "GridFarFade")
  preferences.updateNumberWidget(panel, "JointDisplayScale")
  preferences.updateNumberWidget(panel, "LocatorDisplayScale")
end

------------------------------------------------------------------------------------------------------------------------
-- builds the settings preferences page
------------------------------------------------------------------------------------------------------------------------
local buildPreferencePage = function(panel)
  panel:beginVSizer{ flags = "expand", proportion = 1, }
    panel:addStaticText{
      name = "Heading",
      text = "3D views",
      font = "bold",
      flags = "parentBackground;truncate;expand;decoration",
    }

    panel:beginVSizer{
      flags = "group;expand",
      label = "Camera settings",
    }
      panel:addVSpacer(5)

      panel:beginFlexGridSizer{
        flags = "expand",
        cols = 4,
      }
      panel:setFlexGridColumnExpandable(4)

        panel:addStaticText{ text = "Near clip plane", }
        preferences.addNumberWidget(panel, "CameraNearClipPlane")

        panel:addStaticText{ text = "Far clip plane", }
        preferences.addNumberWidget(panel, "CameraFarClipPlane")

      panel:endSizer()

    panel:endSizer()

    panel:beginVSizer{
      flags = "group;expand",
      label = "Grid settings",
    }
      panel:addVSpacer(5)

      panel:beginFlexGridSizer{
        flags = "expand",
        cols = 4,
      }
      panel:setFlexGridColumnExpandable(4)

        panel:addStaticText{ text = "Spacing", }
        preferences.addNumberWidget(panel, "GridSpacing")

      panel:endSizer()

    panel:endSizer()

    panel:beginFlexGridSizer{
      flags = "expand",
      cols = 4,
    }
    panel:setFlexGridColumnExpandable(4)

      panel:addStaticText{ text = "Joint scale", }
      preferences.addNumberWidget(panel, "JointDisplayScale")
      
      panel:addStaticText{ text = "Locator scale", }
      preferences.addNumberWidget(panel, "LocatorDisplayScale")

    panel:endSizer()

  panel:endSizer()
end

removePreferencesPage("ViewportSettings")
addPreferencesPage(
  "ViewportSettings",
  {
    title = "3D views",
    parent = "Settings",
    create = buildPreferencePage,
    update = updatePreferencePage,
    collapse = true
  }
)
