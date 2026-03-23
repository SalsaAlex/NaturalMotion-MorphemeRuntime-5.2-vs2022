------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"

local animSetDisplayFunc = function(panel, objects, attributes, set)
  attributeEditor.logEnterFunc("animSetDisplayFunc")

  attributeEditor.log("rollPanel:beginVSizer")
  panel:beginVSizer{ flags = "expand" }

    local attribute = attributes[1]

    local attrs = { }
    for i, object in pairs(objects) do
      local current = string.format("%s.%s", object, attribute)
      table.insert(attrs, current)
    end

    local setChannelNames = anim.getRigChannelNames(set)

    attributeEditor.log("rollPanel:addAttributeWidget")
    panel:addChannelEditor{
      attributes = attrs,
      flags = "expand",
      proportion = 1,
      set = set,
      labels = setChannelNames,
      onMouseEnter = function(self)
        attributeEditor.setHelpText(self:getAttributeHelp())
      end,
      onMouseLeave = function()
        attributeEditor.clearHelpText()
      end
    }

  attributeEditor.log("rollPanel:endSizer")
  panel:endSizer()

  attributeEditor.logExitFunc("animSetDisplayFunc")
end

------------------------------------------------------------------------------------------------------------------------
-- Add's a display info section containing channel names.
-- Use by ClosestAnim, FeatherBlend2 and FilterTransforms.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.channelNameDisplayInfoSection = function(rollContainer, displayInfo, selection)

  attributeEditor.logEnterFunc("attributeEditor.channelNameDisplayInfoSection")

  attributeEditor.log("rollContainer:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "channelNameDisplayInfoSection" }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginVSizer")
  rollPanel:beginVSizer{ flags = "expand" }

    local showLabels = true
    if table.getn(displayInfo.usedAttributes) == 1 then
      showLabels = false
    end

    local attrs = { }
    for i, object in ipairs(selection) do
      for j, attr in ipairs(displayInfo.usedAttributes) do
        local current = string.format("%s.%s", object, attr)
        table.insert(attrs, current)
        attributeEditor.log("adding \"%s\" to attribute list", current)
      end
    end

    attributeEditor.log("rollPanel:addAttributeWidget")
    rollPanel:addAnimationSetWidget{
      attributes = attrs,
      flags = "expand",
      proportion = 1,
      displayFunc = animSetDisplayFunc
    }

  attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.channelNameDisplayInfoSection")

end

