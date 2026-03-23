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
-- Add's a display info section containing blend modes.
-- Used by Blend2.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.blendModeDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.blendModeDisplayInfoSection")

  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "blendModeDisplayInfoSection" }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginVSizer")
  rollPanel:beginVSizer{ flags = "expand" }
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 0 }
      rollPanel:setFlexGridColumnExpandable(2)
      
      -- Position
      attributeEditor.addAttributeLabel(rollPanel, "Position", selection, "PositionBlendMode")
      attributeEditor.addCustomComboBox{
        panel = rollPanel, objects = selection,
        attributes = { "PositionBlendMode", },
        values = {
          ["Interpolate"] = function(selection)
            setCommonAttributeValue(selection, "PositionBlendMode", 0)
          end,
          ["Add"] = function(selection)
            setCommonAttributeValue(selection, "PositionBlendMode", 1)
          end
        },
        order = { "Interpolate", "Add" },
        syncValueWithUI = function(combo, selection)
          local value = getCommonAttributeValue(selection, "PositionBlendMode")
          if value ~= nil then
            combo:setIsIndeterminate(false)
            if value == 0 then
              combo:setSelectedItem("Interpolate")
            elseif value == 1 then
              combo:setSelectedItem("Add")
            end
          else
            combo:setIsIndeterminate(true)
          end
        end,
      }
      
      -- Rotation
      attributeEditor.addAttributeLabel(rollPanel, "Rotation", selection, "RotationBlendMode")
      attributeEditor.addCustomComboBox{
        panel = rollPanel, objects = selection,
        attributes = { "RotationBlendMode", },
        values = {
          ["Interpolate"] = function(selection)
            setCommonAttributeValue(selection, "RotationBlendMode", 0)
          end,
          ["Add"] = function(selection)
            setCommonAttributeValue(selection, "RotationBlendMode", 1)
          end,
        },
        order = { "Interpolate", "Add" },
        syncValueWithUI = function(combo, selection)
          local value = getCommonAttributeValue(selection, "RotationBlendMode")
          if value ~= nil then
            combo:setIsIndeterminate(false)
            if value == 0 then
              combo:setSelectedItem("Interpolate")
            elseif value == 1 then
              combo:setSelectedItem("Add")
            end
          else
            combo:setIsIndeterminate(true)
          end
        end,
      }
    rollPanel:endSizer()

  attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.blendModeDisplayInfoSection")
end
