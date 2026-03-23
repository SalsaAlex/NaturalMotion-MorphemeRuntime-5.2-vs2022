------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

-- intComboBoxes for Physics part index option

------------------------------------------------------------------------------------------------------------------------
-- Adds a physicsPartIndexSection.
-- Used by OperatorPhysicsInfo
------------------------------------------------------------------------------------------------------------------------
attributeEditor.physicsPartIndexSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.physicsPartIndexSection")

  local title = displayInfo.title
  attributeEditor.log("rollContainer:addRollup")
  local rollup = rollContainer:addRollup{ label = title, flags = "mainSection", name = utils.getIdentifier(title) }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginHSizer")
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    rollPanel:addHSpacer(6)

    local HelpText = getAttributeHelpText(selection[1], "PartIndex")
    attributeEditor.addStaticTextWithHelp(rollPanel, "Part  ", HelpText)

    if anim.isPhysicsRigValid() then
      local physicsParts = anim.getPhysicsRigChannelNames(selectedSet)

      local options = {}
      for i, v in pairs(physicsParts) do
        options[i-1] =  v
      end

      attributeEditor.addIntAttributeCombo{
        panel = rollPanel,
        objects = selection,
        attribute = "PartIndex",
        helpText = HelpText,
        values = options,
        errorText = "Invalid Part"
      }
    else
      local attrPaths = { }

      for i, object in ipairs(selection) do
        table.insert(attrPaths, string.format("%s.PartIndex", object))
      end

      rollPanel:addAttributeWidget{
        attributes = attrPaths,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function(self)
          attributeEditor.setHelpText(HelpText)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }
    end

  attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.physicsPartIndexSection")
end
