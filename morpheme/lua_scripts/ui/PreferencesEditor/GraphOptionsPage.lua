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
  local graphAnimationZoomTimeFloatSlider = panel:getChild("GraphZoomAnimationTimeFloatSlider")
  local graphAnimationZoomTime = preferences.get("GraphZoomAnimationTime")
  graphAnimationZoomTimeFloatSlider:setValue(graphAnimationZoomTime)

  local graphMouseZoomToCursorComboBox = panel:getChild("GraphMouseZoomToCursorComboBox")
  local graphMouseZoomToCursor = preferences.get("GraphMouseZoomToCursor")
  if graphMouseZoomToCursor then
    graphMouseZoomToCursorComboBox:setSelectedItem("Zoom to cursor")
  else
    graphMouseZoomToCursorComboBox:setSelectedItem("Zoom to centre")
  end
end

------------------------------------------------------------------------------------------------------------------------
-- builds the settings preferences page
------------------------------------------------------------------------------------------------------------------------
local buildPreferencePage = function(panel)
  panel:beginVSizer{ flags = "expand", proportion = 1, }
    panel:addStaticText{
      name = "Heading",
      text = "Graph options",
      font = "bold",
      flags = "parentBackground;truncate;expand;decoration",
    }

    panel:addVSpacer(5)

    panel:beginFlexGridSizer{ flags = "expand", rows = 2, }
    panel:setFlexGridColumnExpandable(2)

      panel:addStaticText{ text = "Zoom animation time", }

      panel:addFloatSlider{
        name = "GraphZoomAnimationTimeFloatSlider",
        onChanged = function(self)
          local _, info = preferences.get("GraphZoomAnimationTime")
          info.value = self:getValue()
          preferences.set(info)
        end,
        flags = "expand",
        proportion = 1,
      }

      panel:addStaticText{ text = "Mouse wheel action", }

      panel:addComboBox{
        name = "GraphMouseZoomToCursorComboBox",
        items = {
          "Zoom to cursor",
          "Zoom to centre",
        },
        onChanged = function(self)
          local _, info = preferences.get("GraphMouseZoomToCursor")
          local item = self:getSelectedItem()
          if item == "Zoom to cursor" then
            info.value = true
          elseif item == "Zoom to centre" then
            info.value = false
          end
          preferences.set(info)
        end,
      }
    panel:endSizer()
  panel:endSizer()
end

removePreferencesPage("GraphOptions")
addPreferencesPage(
  "GraphOptions",
  {
    title = "Graph options",
    parent = "Settings",
    create = buildPreferencePage,
    update = updatePreferencePage,
  }
)