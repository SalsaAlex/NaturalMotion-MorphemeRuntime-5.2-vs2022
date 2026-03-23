------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "luaAPI/ManifestUtils.lua"
require "ui/AttributeEditor/AttributeEditor.lua"

------------------------------------------------------------------------------------------------------------------------
-- adds the properties section for all transition types
-- a custom section is required as certain attributes must be disabled for
-- transitions to self and transitions from an active state
------------------------------------------------------------------------------------------------------------------------
attributeEditor.transitPropertiesDisplayInfoSection = function(panel, displayInfo, selection)
  if hasTransitionCategory(selection, "euphoria") then
    return
  end
  
  attributeEditor.logEnterFunc("attributeEditor.transitPropertiesDisplayInfoSection")

  -- check if the selection contains any referenced objects
  local rollup = panel:addRollup{
    label = displayInfo.title,
    flags = "mainSection",
    name = utils.getIdentifier(displayInfo.title)
  }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginHSizer")
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    rollPanel:addHSpacer(6)
    rollPanel:setBorder(1)

    attributeEditor.log("rollPanel:beginFlexGridSizer")
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      rollPanel:setFlexGridColumnExpandable(2)

      -- check if our current selection contains any transitions to self so the
      -- "Dead blend source on breakout" attribute can be disabled appropriately.
      local selectionContainsSelfTransition = false
      for _, transition in ipairs(selection) do
        if isSelfTransition(transition) then
          selectionContainsSelfTransition = true
          break
        end
      end

      -- check if our current selection contains any transitions from an ActiveState node so the
      -- "Breakout transition" attribute can be disabled appropriately.
      local selectionContainsTransitionFromActiveState = false
      for _, transition in ipairs(selection) do
        if isTransitionFromActiveState(transition) then
          selectionContainsTransitionFromActiveState = true
          break
        end
      end

      for _, attribute in ipairs(displayInfo.usedAttributes) do
        attributeEditor.log("adding control for attribute \"%s\"", attribute)
        local attributeDisplayName = getAttributeDisplayName(selection[1], attribute)
        local displayName = utils.getDisplayString(attributeDisplayName)

        attributeEditor.addAttributeLabel(rollPanel, displayName, selection, attribute)
        attributeEditor.addAttributeWidget(rollPanel, attribute, selection)
      end

    attributeEditor.log("rollPanel:endSizer")
    rollPanel:endSizer()

  attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.transitPropertiesDisplayInfoSection")
  return rollup
end