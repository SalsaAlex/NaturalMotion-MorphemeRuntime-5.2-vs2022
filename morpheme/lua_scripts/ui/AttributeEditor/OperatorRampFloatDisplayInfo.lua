------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "luaAPI/AttributeEditorUtils.lua"
require "ui/AttributeEditor/AttributeEditor.lua"

attributeEditor.operatorRampFloatClampingDisplayInfoSection = function(panel, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.operatorRampFloatClampingDisplayInfo")

  local rollup = panel:addRollup{ label = displayInfo.title, flags = "mainSection", name = utils.getIdentifier(displayInfo.title) }
  local rollPanel = rollup:getPanel()
  rollPanel:setBorder(1)

  attributeEditor.log("rollPanel:beginFlexGridSizer")
  rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
    rollPanel:setFlexGridColumnExpandable(2)

    attributeEditor.addAttributeLabel(rollPanel, "Enable", selection, "Enable")
    local enableWidget = attributeEditor.addAttributeWidget(rollPanel, "Enable", selection)

    attributeEditor.addAttributeLabel(rollPanel, "Minimum", selection, "Minimum")
    local minimumWidget = attributeEditor.addAttributeWidget(rollPanel, "Minimum", selection)

    attributeEditor.addAttributeLabel(rollPanel, "Maximum", selection, "Maximum")
    local maximumWidget = attributeEditor.addAttributeWidget(rollPanel, "Maximum", selection)

    local syncWidgetEnableState = function()
      local enableLimits = true

      local enable = getCommonAttributeValue(selection, "Enable")
      if enable == false then
        enableLimits = false
      end

      minimumWidget:enable(enableLimits)
      maximumWidget:enable(enableLimits)
    end

    syncWidgetEnableState()

    attributeEditor.log("creating data change context")
    local enabledContext = attributeEditor.createChangeContext()

    enabledContext:setObjects(selection)
    enabledContext:addAttributeChangeEvent("Enable")
    enabledContext:setAttributeChangedHandler(
      function(object, attr)
        syncWidgetEnableState()
      end
    )

  attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.operatorRampFloatClampingDisplayInfo")
  return rollPanel
end