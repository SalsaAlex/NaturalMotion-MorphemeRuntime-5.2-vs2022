------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

-- boolComboBoxes for Physics transition condition options

------------------------------------------------------------------------------------------------------------------------
-- Adds a physicsInUseSection.
-- Used by PhysicsInUse Condition
------------------------------------------------------------------------------------------------------------------------
attributeEditor.physicsInUseSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.physicsInUseSection")

    -- first add the ui for the section
  rollContainer:beginHSizer{ flags = "expand", proportion = 0 }
  attributeEditor.log("rollContainer:beginHSizer")

    local HelpText = getAttributeHelpText(selection[1], "OnPhysicsInUse")
    attributeEditor.addStaticTextWithHelp(rollContainer, "State  ", HelpText)

    attributeEditor.log("attributeEditor.addBoolAttributeCombo")
    attributeEditor.addBoolAttributeCombo{
      panel = rollContainer,
      objects = selection,
      attribute = "OnPhysicsInUse",
      falseValue = "Animation Only",
      trueValue ="Physics & Animation",
      helpText = HelpText,
      flags = "expand",
      proportion = 1
    }
  rollContainer:endSizer()
  attributeEditor.logExitFunc("attributeEditor.physicsInUseSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Adds a physicsAvailableSection.
-- Used by physicsAvailableSection Condition
------------------------------------------------------------------------------------------------------------------------
attributeEditor.physicsAvailableSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.physicsAvailableSection")

    -- first add the ui for the section
  rollContainer:beginHSizer{ flags = "expand", proportion = 0 }
  attributeEditor.log("rollContainer:beginHSizer")

    local HelpText = getAttributeHelpText(selection[1], "OnPhysicsAvailable")
    attributeEditor.addStaticTextWithHelp(rollContainer, "Physics  ", HelpText)

    attributeEditor.log("attributeEditor.addBoolAttributeCombo")
    attributeEditor.addBoolAttributeCombo{
      panel = rollContainer,
      objects = selection,
      attribute = "OnPhysicsAvailable",
      falseValue = "Out Of Resources",
      trueValue ="Available",
      helpText = HelpText
    }
  rollContainer:endSizer()
  attributeEditor.logExitFunc("attributeEditor.physicsAvailableSection")
end

