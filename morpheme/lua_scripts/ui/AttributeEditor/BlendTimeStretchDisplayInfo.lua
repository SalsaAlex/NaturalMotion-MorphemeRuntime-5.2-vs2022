------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"

------------------------------------------------------------------------------------------------------------------------
-- Add's a display info section containing blend weights.
-- Use by BlendN, BlendNMatchEvents.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.blendTimeStretchDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.blendTimeStretchDisplayInfoSection")

  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "blendTimeStretchDisplayInfoSection" }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginVSizer")
  rollPanel:beginVSizer{ flags = "expand" }
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 0 }
      rollPanel:setFlexGridColumnExpandable(2)
      attributeEditor.addAttributeLabel(rollPanel, "Mode", selection, "TimeStretchMode")

      attributeEditor.addCustomComboBox{
        panel = rollPanel, objects = selection,
        attributes = { "PassThroughMode", },
        values = {
          ["None"] = function(selection)
            setCommonAttributeValue(selection, "TimeStretchMode", 0)
          end,
          ["Match Events"] = function(selection)
            setCommonAttributeValue(selection, "TimeStretchMode", 1)
          end,
        },
        order = { "None", "Match Events" },
        syncValueWithUI = function(combo, selection)
          local value = getCommonAttributeValue(selection, "TimeStretchMode")
          if value ~= nil then
            combo:setIsIndeterminate(false)
            if value == 0 then
              combo:setSelectedItem("None")
            else
              combo:setSelectedItem("Match Events")
            end
          else
            combo:setIsIndeterminate(true)
          end
        end,
      }
    rollPanel:endSizer()

  attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.blendTimeStretchDisplayInfoSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Add's a display info section for Match Events Properties that rely on the TimeStretchMode to control
-- their enable state.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.matchEventsPropertiesDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.matchEventsPropertiesDisplayInfoSection")

  local rollup = attributeEditor.addSimpleAttributeSection(
    rollContainer,
    displayInfo.title,
    selection,
    displayInfo.usedAttributes)

  if not rollup then
    return
  end

  rollup:expand(false)

  -- if the node type doesn't have the TimeStretchMode then this section shouldn't be used but make
  -- sure it is handled gracefully if the attribute does not exist
  if attributeExists(selection[1], "TimeStretchMode") then
    local hasReference = containsReference(selection)

    -- when the TimeStretchMode is set to None then these attributes are relevant so the
    -- corresponding widgets should be disabled.
    local updateSectionEnableState = function()
      attributeEditor.logEnterFunc("updateSectionEnableState")

      local enableSection = true

      local timeStretchMode = getCommonAttributeValue(selection, "TimeStretchMode")
      if not timeStretchMode or timeStretchMode == 0 then
        enableSection = false
      end

      -- the children of a rollup are the header panel and the main panel so
      -- the widgets and text labels to disable are the grand children of the rollup
      local children = rollup:getChildren()
      for _, child in ipairs(children) do
        local grandChildren = child:getChildren()
        for _, grandChild in ipairs(grandChildren) do
          grandChild:enable(enableSection and not hasReference)
        end
      end

      attributeEditor.logExitFunc("updateSectionEnableState")
    end

    local enabledContext = attributeEditor.createChangeContext()
    enabledContext:setObjects(selection)
    enabledContext:addAttributeChangeEvent("TimeStretchMode")
    enabledContext:setAttributeChangedHandler(
      function(object, attr)
        updateSectionEnableState()
      end
    )

    updateSectionEnableState()
  end

  attributeEditor.logExitFunc("attributeEditor.matchEventsPropertiesDisplayInfoSection")
end
