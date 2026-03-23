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
attributeEditor.blendPassThroughDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.blendPassThroughDisplayInfoSection")

  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "blendPassThroughDisplayInfoSection" }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginVSizer")
  rollPanel:beginVSizer{ flags = "expand" }
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 0 }
      rollPanel:setFlexGridColumnExpandable(2)
      attributeEditor.addAttributeLabel(rollPanel, "Mode", selection, "PassThroughMode")

      attributeEditor.addCustomComboBox{
        panel = rollPanel, objects = selection,
        attributes = { "PassThroughMode", },
        values = {
          ["Source 0"] = function(selection)
            setCommonAttributeValue(selection, "PassThroughMode", 0)
          end,
          ["Source 1"] = function(selection)
            setCommonAttributeValue(selection, "PassThroughMode", 1)
          end,
          ["None"] = function(selection)
            setCommonAttributeValue(selection, "PassThroughMode", 2)
          end,
        },
        order = { "None", "Source 0", "Source 1", },
        syncValueWithUI = function(combo, selection)
          local value = getCommonAttributeValue(selection, "PassThroughMode")
          if value ~= nil then
            combo:setIsIndeterminate(false)
            if value == 0 then
              combo:setSelectedItem("Source 0")
            elseif value == 1 then
              combo:setSelectedItem("Source 1")
            else
              combo:setSelectedItem("None")
            end
          else
            combo:setIsIndeterminate(true)
          end
        end,
      }
    rollPanel:endSizer()

  attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.blendPassThroughDisplayInfoSection")
end