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

------------------------------------------------------------------------------------------------------------------------
-- updates the runtime targets preferences page on data changes
------------------------------------------------------------------------------------------------------------------------
local updatePreferencePage = function(panel)
  local loadMostRecentOnStartupCheckBox = panel:getChild("LoadMostRecentOnStartupCheckBox")
  local loadMostRecentOnStartup = preferences.get("LoadMostRecentOnStartup")
  loadMostRecentOnStartupCheckBox:setChecked(loadMostRecentOnStartup)

  local saveSessionOnNetworkErrorCheckBox = panel:getChild("SaveSessionOnNetworkErrorCheckBox")
  local saveSessionOnNetworkError = preferences.get("SaveSessionOnNetworkError")
  saveSessionOnNetworkErrorCheckBox:setChecked(saveSessionOnNetworkError)

  local displayNetworkValidationWarningsCheckBox = panel:getChild("DisplayNetworkValidationWarningsCheckBox")
  local displayNetworkValidationWarnings = preferences.get("DisplayNetworkValidationWarnings")
  displayNetworkValidationWarningsCheckBox:setChecked(displayNetworkValidationWarnings)
  
  local markupAnimationSynchronisationTypeComboBox = panel:getChild("MarkupAnimationSynchronisationType")
  
  local markupAnimationSynchronisationType = preferences.get("MarkupAnimationSynchronisationType")
  if markupAnimationSynchronisationType == "UpdateMarkupDurationPreserveClipRange" then
    markupAnimationSynchronisationTypeComboBox:setSelectedItem("Preserve Clip Range")
  elseif markupAnimationSynchronisationType == "UpdateMarkupDurationIgnoreClipRange" then
    markupAnimationSynchronisationTypeComboBox:setSelectedItem("Do Nothing")
  else
    markupAnimationSynchronisationTypeComboBox:setSelectedItem("Prompt User")
  end
end

------------------------------------------------------------------------------------------------------------------------
-- builds the settings preferences page
------------------------------------------------------------------------------------------------------------------------
local buildPreferencePage = function(panel)
  panel:beginVSizer{ flags = "expand", proportion = 1, }
    panel:addStaticText{
      name = "Heading",
      text = "Settings",
      font = "bold",
      flags = "parentBackground;truncate;expand;decoration",
    }

    panel:addVSpacer(5)

    panel:addCheckBox{
      name = "LoadMostRecentOnStartupCheckBox",
      label = "Load most recent network on startup",
      onChanged = function(self)
        local _, info = preferences.get("LoadMostRecentOnStartup")
        info.value = self:getChecked()
        preferences.set(info)
      end,
    }

    panel:addVSpacer(5)

    panel:addCheckBox{
      name = "SaveSessionOnNetworkErrorCheckBox",
      label = "Save session on network error",
      onChanged = function(self)
        local _, info = preferences.get("SaveSessionOnNetworkError")
        info.value = self:getChecked()
        preferences.set(info)
      end,
    }

    panel:addVSpacer(5)

    panel:addCheckBox{
      name = "DisplayNetworkValidationWarningsCheckBox",
      label = "Display network validation warnings",
      onChanged = function(self)
        local _, info = preferences.get("DisplayNetworkValidationWarnings")
        info.value = self:getChecked()
        preferences.set(info)
      end,
    }
    
    panel:beginVSizer{
      flags = "group;expand",
      label = "Animation Markup settings",
    }
      panel:beginHSizer{ flags = "expand" }

        panel:addStaticText{ text = "On markup duration change", }
        
        panel:addComboBox{
          name = "MarkupAnimationSynchronisationType",
          items = { "Preserve Clip Range", "Do Nothing", "Prompt User", },
          flags = "expand",
          proportion = 1,
          onChanged = function(self)
            local _, info = preferences.get("MarkupAnimationSynchronisationType")

            local selected = self:getSelectedItem()
            if selected == "Preserve Clip Range" then
              info.value = "UpdateMarkupDurationPreserveClipRange"
            elseif selected == "Do Nothing" then
              info.value = "UpdateMarkupDurationIgnoreClipRange"
            else
              info.value = "PromptUserForMarkupSynchronisationType"
            end

            preferences.set(info)
          end,
        }
        
      panel:endSizer()
    panel:endSizer()

  panel:endSizer()
end

removePreferencesPage("Settings")
addPreferencesPage(
  "Settings",
  {
    title = "Settings",
    create = buildPreferencePage,
    update = updatePreferencePage,
  }
)