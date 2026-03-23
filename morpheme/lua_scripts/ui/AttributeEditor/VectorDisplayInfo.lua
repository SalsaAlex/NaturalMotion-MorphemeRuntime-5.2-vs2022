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
-- Add's a display info section for vectors.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.vectorDisplayInfoSection = function(rollContainer, displayInfo, selection, set)
  attributeEditor.logEnterFunc("attributeEditor.vectorDisplayInfoSection")

  -- first add the ui for the section
  attributeEditor.log("rollContainer:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "vectorDisplayInfoSection" }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginHSizer")
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }

    rollPanel:addHSpacer(6)
    rollPanel:setBorder(1)

    attributeEditor.addVectorAttributeWidget(rollPanel, displayInfo.usedAttributes, selection, set)

  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.vectorDisplayInfoSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Adds a vectorAttributeWidget consisting of three individual floats.
------------------------------------------------------------------------------------------------------------------------

attributeEditor.addVectorAttributeWidget = function(rollPanel, usedAttributes, selection, set)
  attributeEditor.logEnterFunc("attributeEditor.addVectorAttributeWidget")

  local widgets = { }
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }

    local attrPaths = { }
    local objectCount = table.getn(selection)
    table.setn(attrPaths, objectCount)
    local widgetNames = { "x", "y", "z", "w", "i", "j", "k" }
    for i, attr in ipairs(usedAttributes) do

      for j = 1, objectCount do
        attrPaths[j] = string.format("%s.%s", selection[j], attr)
      end

      local widget = rollPanel:addAttributeWidget{
        attributes = attrPaths,
        set = set,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function(self)
          attributeEditor.setHelpText(self:getAttributeHelp())
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }
      widgets[widgetNames[i]] = widget
    end

  rollPanel:endSizer()
  attributeEditor.logExitFunc("attributeEditor.addVectorAttributeWidget")
  widgets.enable = function(widgetTbl, value)
    for obj, val in widgetTbl do
      if (type(val) ~= "function") then
        val:enable(value)
      end
    end
  end
  return widgets
end

