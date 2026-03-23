------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"
require "luaAPI/IKCalculationAPI.lua"

------------------------------------------------------------------------------------------------------------------------
-- Add's a twoBoneIkHingeAxisSection
-- Used by TwoBoneIK
------------------------------------------------------------------------------------------------------------------------
local perAnimSetTwoBoneIkHingeAxisSection = function(panel, selection, attributes, set)
  attributeEditor.logEnterFunc("perAnimSetTwoBoneIkHingeAxisSection")

  panel:setBorder(1)
  attributeEditor.log("panel:beginHSizer")
  panel:beginVSizer{ flags = "expand", proportion = 1 }

    local calculateHelpText = "Automatically calculate the Hinge Axis based on the axis in the current rig."

    panel:addHSpacer(6)
    panel:setBorder(1)
    local rotAxisAttrs = { "MidJointRotationAxisX", "MidJointRotationAxisY", "MidJointRotationAxisZ" }
    attributeEditor.addVectorAttributeWidget(panel, rotAxisAttrs, selection, set)
    panel:addHSpacer(6)
    local calculateButton = panel:addButton {
      label = "Calculate",
      onMouseEnter = function()
        attributeEditor.setHelpText(calculateHelpText)
      end,
      onMouseLeave = function()
        attributeEditor.clearHelpText()
      end
    }
    if containsReference(selection) then
      calculateButton:enable(false)
    end

    calculateButton:setOnClick(
      function(self)
        local endJointName = getCommonAttributeValue(selection, "EndJointName", set)
        local axis = calculateHingeAxis(set, endJointName)
        setCommonAttributeValue(selection, "MidJointRotationAxisX", axis:getX(), set)
        setCommonAttributeValue(selection, "MidJointRotationAxisY", axis:getY(), set)
        setCommonAttributeValue(selection, "MidJointRotationAxisZ", axis:getZ(), set)
        -- Deselect Flip Hinge which is assumed off by Calculate
        setCommonAttributeValue(selection, "FlipMidJointRotationDirection", false, set)
      end
    )

  panel:endSizer()

  attributeEditor.logExitFunc("perAnimSetTwoBoneIkHingeAxisSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Add's a twoBoneIkHingeAxisSection
-- Used by TwoBoneIK
------------------------------------------------------------------------------------------------------------------------
attributeEditor.twoBoneIkHingeAxisSection = function(panel, displayInfo, selection)
  local attrs = checkSetOrderAndBuildAttributeList(selection, displayInfo.usedAttributes)

  local rollup = panel:addRollup{
    label = displayInfo.title,
    flags = "mainSection",
    name = string.format("twoBoneIkHingeAxisSection")
  }
  local rollPanel = rollup:getPanel()

  rollPanel:beginVSizer{ flags = "expand" }
    rollPanel:addAnimationSetWidget{
      attributes = attrs,
      displayFunc = perAnimSetTwoBoneIkHingeAxisSection,
      flags = "expand",
      proportion = 1,
    }
  rollPanel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- Adds a twoBoneIkReferenceAxisSection
-- Used by TwoBoneIK
------------------------------------------------------------------------------------------------------------------------

local perAnimSetTwoBoneIkReferenceAxisSection = function(panel, selection, attributes, set)
  attributeEditor.logEnterFunc("perAnimSetTwoBoneIkReferenceAxisSection")

  -- first add the ui for the section
  attributeEditor.log("panel:beginHSizer")
  panel:beginVSizer{ flags = "expand", proportion = 1 }
    panel:addHSpacer(6)
    panel:setBorder(1)

    attributeEditor.log("panel:beginHSizer")
    panel:beginHSizer{ flags = "expand", proportion = 1 }

      local swivelHelpText = [[
Change the swivel angle offset from source to a specified reference plane.
When source is selected the swivel angle control parameter value is relative to the source animation value.
When reference plane is selected the swivel angle is relative to a specified plane.
]]

      attributeEditor.addStaticTextWithHelp(panel, "Reference Frame", swivelHelpText)
      attributeEditor.log("panel:addStaticText")

      panel:addHSpacer(6)
      local planeVectorWidget = nil
      local spaceCombo = nil

      attributeEditor.addBoolAttributeCombo{
        panel = panel,
        objects = selection,
        attribute = "UseReferenceAxis",
        set = set,
        trueValue = "Plane",
        falseValue = "Source",
        helpText = swivelHelpText
      }

    attributeEditor.log("panel:endSizer")
    panel:endSizer()

    attributeEditor.log("panel:beginHSizer")
    panel:beginHSizer{ flags = "expand", proportion = 1 }

    panel:addHSpacer(8)
    attributeEditor.log("panel:addStaticText")

    local planeHelpText = [[
The normal of the plane used with the swivel angle value.  For instance, to keep the elbow of an arm chain in the XZ plane specify the plane as <0, 1, 0> and set the swivel angle to 0.
]]

    attributeEditor.addStaticTextWithHelp(panel, "Plane", planeHelpText)

    panel:addHSpacer(7)
    local rayStartAttrs = { "MidJointReferenceAxisX", "MidJointReferenceAxisY", "MidJointReferenceAxisZ" }
    planeVectorWidget = attributeEditor.addVectorAttributeWidget(panel, rayStartAttrs, selection, set)

    attributeEditor.log("panel:endSizer")
    panel:endSizer()

    attributeEditor.log("panel:beginHSizer")
    panel:beginHSizer{ flags = "expand", proportion = 1 }

      panel:addHSpacer(6)
      local referencePlaneText = [[
Change the space of the plane.
Character space is centered on the trajectory and shares the character's orientation in the world.  IK Root space is the parent of the root joint's space.
]]

      attributeEditor.log("panel:addStaticText")
      attributeEditor.addStaticTextWithHelp(panel, "Space", referencePlaneText)

      panel:addHSpacer(6)

      attributeEditor.log("panel:addComboBox")

      spaceCombo = attributeEditor.addBoolAttributeCombo{
        panel = panel,
        objects = selection,
        attribute = "GlobalReferenceAxis",
        set = set,
        trueValue = "Character",
        falseValue ="IK Root",
        helpText = swivelHelpText
      }

      attributeEditor.log("panel:endSizer")
    panel:endSizer()

  attributeEditor.log("panel:endSizer")
  panel:endSizer()

------------------------------------------------------------------------------------------------------------------------
-- Enable controls based on the value of UseReferenceAxis
------------------------------------------------------------------------------------------------------------------------
  local syncUIWithAttributes = function()
    attributeEditor.logEnterFunc("syncUIWithAttributes")
    local useReferenceValue = getCommonAttributeValue(selection, "UseReferenceAxis", set)
    if useReferenceValue ~= nil then
      planeVectorWidget:enable(useReferenceValue)
      spaceCombo:enable(useReferenceValue)
    else
      planeVectorWidget:enable(false)
      spaceCombo:enable(false)
    end
    attributeEditor.logExitFunc("syncUIWithAttributes")
  end

  local changeContext = attributeEditor.createChangeContext()
  changeContext:setObjects(selection)
  changeContext:addAttributeChangeEvent("UseReferenceAxis")
  changeContext:setAttributeChangedHandler(
    function(object, attr)
      attributeEditor.logEnterFunc("changeContext attributeChangedHandler")
      syncUIWithAttributes()
      attributeEditor.logExitFunc("changeContext attributeChangedHandler")
    end
  )

  syncUIWithAttributes()

  attributeEditor.logExitFunc("perAnimSetTwoBoneIkReferenceAxisSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Adds a twoBoneIkReferenceAxisSection
-- Used by TwoBoneIK
------------------------------------------------------------------------------------------------------------------------
attributeEditor.twoBoneIkReferenceAxisSection = function(panel, displayInfo, selection)
  local attrs = checkSetOrderAndBuildAttributeList(selection, displayInfo.usedAttributes)

  local rollup = panel:addRollup{
    label = displayInfo.title,
    flags = "mainSection",
    name = string.format("twoBoneIkReferenceAxisSection")
  }
  local rollPanel = rollup:getPanel()

  rollPanel:beginVSizer{ flags = "expand" }
    rollPanel:addAnimationSetWidget{
      attributes = attrs,
      displayFunc = perAnimSetTwoBoneIkReferenceAxisSection,
      flags = "expand",
      proportion = 1,
    }
  rollPanel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
local perAnimSetFlipHingeSection = function(panel, selection, attributes, set)
  attributeEditor.logEnterFunc("perAnimSetFlipHingeSection")

    local flipPaths = { }
    for i, object in ipairs(selection) do
      table.insert(flipPaths, string.format("%s.FlipMidJointRotationDirection", object))
    end

    local flipHingeHelpText = getAttributeHelpText(selection[1], "FlipMidJointRotationDirection")

    panel:beginHSizer{ flags = "expand", proportion = 1 }
      panel:setBorder(1)

      attributeEditor.addStaticTextWithHelp(panel, "Flip Hinge", flipHingeHelpText)
      panel:addHSpacer(2)
      panel:addAttributeWidget{
        attributes = flipPaths,
        flags = "expand",
        proportion = 1,
        set = set,
        onMouseEnter = function()
          attributeEditor.setHelpText(flipHingeHelpText)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

  panel:endSizer()
  attributeEditor.logExitFunc("perAnimSetFlipHingeSection")

end

------------------------------------------------------------------------------------------------------------------------
-- Add's a twoBoneIkEndEffectorSection
-- Used by TwoBoneIK
------------------------------------------------------------------------------------------------------------------------
attributeEditor.twoBoneIkPropertiesSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.twoBoneIkPropertiesSection")

  local swivelHelpText = [[Change the orientation of the end effector.
Source World uses the end effector world space orientation specified by the source animation, and is commonly used to maintain ground and environment contacts.
Free follows the input target orientation, or leaves the orientation unchanged if no target orientation is connected.
]]

  --If turned on, assume that input target position and orientation were given relative to the character root at
  -- the previous update, and should be corrected for motion since then.
  local updateDeltasHelpText = [[
Choose if the target was specified relative to the previous or current frame.
When previous frame is selected the target will be offset by the character root motion.
When current frame is selected the target will not be modified by the character root motion.

This attribute only applies when the Target Frame is set to Character Space.]]

  local targetSpaceHelpText = [[The coordinate frame of the effector target control parameters, which can either be World Space or Character Space.]]

  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "twoBoneIkPropertiesSection" }
  local rollPanel = rollup:getPanel()

  rollPanel:beginVSizer{ flags = "expand", proportion = 1 }
    -- first add the ui for the section
    attributeEditor.log("rollPanel:beginFlexGridSizer")
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 0 }
      rollPanel:setFlexGridColumnExpandable(2)
      rollPanel:setBorder(1)

      attributeEditor.log("rollPanel:addStaticText")
      attributeEditor.addStaticTextWithHelp(rollPanel, "End Orientation", swivelHelpText)

      attributeEditor.log("attributeEditor.addBoolAttributeCombo")
      attributeEditor.addBoolAttributeCombo{
        panel = rollPanel,
        objects = selection,
        attribute = "KeepEndEffOrientation",
        trueValue = "Source World",
        falseValue = "Free",
        helpText = swivelHelpText
      }

      attributeEditor.log("rollPanel:addStaticText")
      attributeEditor.addStaticTextWithHelp(rollPanel, "Target Frame", targetSpaceHelpText)

      attributeEditor.log("attributeEditor.addBoolAttributeCombo")
      attributeEditor.addBoolAttributeCombo{
        panel = rollPanel,
        objects = selection,
        attribute = "WorldSpaceTarget",
        trueValue = "World Space",
        falseValue = "Character Space",
        helpText = targetSpaceHelpText
      }

      attributeEditor.log("rollPanel:addStaticText")
      attributeEditor.addStaticTextWithHelp(rollPanel, "Target Update", updateDeltasHelpText)

      attributeEditor.log("attributeEditor.addBoolAttributeCombo")
      attributeEditor.addBoolAttributeCombo{
        panel = rollPanel,
        objects = selection,
        attribute = "UpdateTargetByDeltas",
        falseValue = "Current Frame",
        trueValue = "Previous Frame",
        helpText = updateDeltasHelpText
      }

    attributeEditor.log("rollPanel:endSizer")
    rollPanel:endSizer()

   local attrs = checkSetOrderAndBuildAttributeList(selection, { "FlipMidJointRotationDirection" })
   rollPanel:addAnimationSetWidget{
      attributes = attrs,
      displayFunc = perAnimSetFlipHingeSection,
      flags = "expand",
      proportion = 1,
    }

  rollPanel:endSizer()
  attributeEditor.logExitFunc("attributeEditor.twoBoneIkEndEffectorSection")
end

