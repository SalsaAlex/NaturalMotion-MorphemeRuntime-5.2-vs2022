------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- Refresh the BoneLookup ComboBox
------------------------------------------------------------------------------------------------------------------------

local addBonesToPanel = function()
  local bonesScrollPanel = ui.getWindow("MainFrame|LayoutManager|BoneLookup|BonesScrollPanel")
  local boneLookupCombo = ui.getWindow("MainFrame|LayoutManager|BoneLookup|BoneLookupCombo")

  if bonesScrollPanel == nil or boneLookupCombo == nil then
    return false
  end

  local animSets = listAnimSets()
  if animSets == nil then
    return false
  end

  bonesScrollPanel:clear()
  bonesScrollPanel:beginVSizer()
    local setName = animSets[boneLookupCombo:getSelectedIndex()]
    local rigSize = anim.getRigSize(setName)
    if rigSize > 0 then
      local channelNames = anim.getRigChannelNames(setName)
      if channelNames == nil then
        error("Unable to get channel names for the rig in the animation set: "..setName)
      end
      for i, v in ipairs(channelNames) do
        bonesScrollPanel:addStaticText{ text = i - 1 .. " : " .. v }
      end
    end
  bonesScrollPanel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- Refresh the BoneLookup ComboBox
------------------------------------------------------------------------------------------------------------------------

local refreshBoneLookupComboHandler = function(inParam)
  -- this callback can get different params based on how it was called
  -- ignore it in all cases
  local boneLookupCombo = ui.getWindow("MainFrame|LayoutManager|BoneLookup|BoneLookupCombo")
  if boneLookupCombo == nil then
    return false
  end
  local existingSets = listAnimSets()
  if existingSets ~= nil then
    boneLookupCombo:setItems(existingSets)
    boneLookupCombo:setSelectedIndex(1)
    addBonesToPanel()
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Utility window to lookup bone indices
------------------------------------------------------------------------------------------------------------------------
addBoneLookupWindow = function(layoutManager)
  local panel = layoutManager:addPanel{ name = "BoneLookup", caption = "Bone Lookup", flags = "expand", proportion = 1 }
  panel:beginVSizer{ flags = "expand", proportion = 1 }

    local animSets = listAnimSets()
    local boneLookupCombo = panel:addComboBox{ name = "BoneLookupCombo", flags = "expand", items = animSets }
    local bonesPanel = panel:addScrollPanel{ name = "BonesScrollPanel", flags = "expand;vertical", proportion = 1 }

    addBonesToPanel()

    boneLookupCombo:setOnChanged(
      function()
        addBonesToPanel()
      end
    )
  panel:endSizer()

  registerEventHandler("mcAnimationSetCreated", refreshBoneLookupComboHandler)
  registerEventHandler("mcAnimationSetDestroyed", refreshBoneLookupComboHandler)
  registerEventHandler("mcAnimationSetRenamed", refreshBoneLookupComboHandler)

end