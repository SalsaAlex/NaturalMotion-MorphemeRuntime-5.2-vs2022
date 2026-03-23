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
-- Adds a SoftKeyFrameAccelerationLimitsInfoSection
-- Used by SoftKeyFrame
------------------------------------------------------------------------------------------------------------------------
local addSoftKeyFrameAccelerationLimitsInfoSection = function(panel, selection, attributes, set)
  attributeEditor.logEnterFunc("attributeEditor.addSoftKeyFrameAccelerationLimitsInfoSection")
      
  local maxAngularAccelerationWidget, maxLinearAccelerationWidget

  attributeEditor.log("panel:beginVSizer")
  panel:beginVSizer{ flags = "expand" }

    -- Linear acceleration limit checkbox.
    panel:beginHSizer{ flags = "expand" }
      panel:addHSpacer(6)
      local useMaxLinearAccelerationLabel = attributeEditor.addAttributeLabel(panel, "Linear", selection, "UseMaxAcceleration")
      local useMaxLinearAccelerationCheckbox = panel:addCheckBox{ flags = "expand" }
      bindWidgetToAttribute(useMaxLinearAccelerationCheckbox, selection, "UseMaxAcceleration", set)
      attributeEditor.bindAttributeHelpToWidget(useMaxLinearAccelerationCheckbox, selection, "UseMaxAcceleration")
    panel:endSizer()

    panel:addVSpacer(3)

    -- Linear acceleration limit.
    panel:beginHSizer{ flags = "expand" }
      panel:addHSpacer(6)
      local maxLinearAccelerationLabel = attributeEditor.addAttributeLabel(panel, "", selection, "MaxAcceleration")
      maxLinearAccelerationWidget = attributeEditor.addAttributeWidget(panel, "MaxAcceleration", selection, set)
      maxLinearAccelerationWidget:enable( true )
    panel:endSizer()

    panel:addVSpacer(3)

    -- Angular acceleration limit checkbox.
    panel:beginHSizer{ flags = "expand" }
      panel:addHSpacer(6)
      local useMaxAngularAccelerationLabel = attributeEditor.addAttributeLabel(panel, "Angular", selection, "UseMaxAngularAcceleration")
      local useMaxAngularAccelerationCheckbox = panel:addCheckBox{ flags = "expand" }
      bindWidgetToAttribute(useMaxAngularAccelerationCheckbox, selection, "UseMaxAngularAcceleration", set)
      attributeEditor.bindAttributeHelpToWidget(useMaxAngularAccelerationCheckbox, selection, "UseMaxAngularAcceleration")
    panel:endSizer()

    panel:addVSpacer(3)

    -- Angular acceleration limit.
    panel:beginHSizer{ flags = "expand" }
      panel:addHSpacer(6)
      local angularAccelerationLimitLabel = attributeEditor.addAttributeLabel(panel, "", selection, "MaxAngularAcceleration")
      maxAngularAccelerationWidget = attributeEditor.addAttributeWidget(panel, "MaxAngularAcceleration", selection, set)
      maxAngularAccelerationWidget:enable( true )
    panel:endSizer()

  attributeEditor.log("panel:endVSizer")
  panel:endSizer()

  ----------------------------------------------------------------------------------------------------------------------
  -- update the interface to reflect the current attributes.
  ----------------------------------------------------------------------------------------------------------------------
  local syncInterface = function()
    attributeEditor.logEnterFunc("syncInterface")

    local useMaxAcceleration = getCommonAttributeValue(selection, "UseMaxAcceleration")

    if useMaxAcceleration ~= nil then
      if useMaxAcceleration then
        maxLinearAccelerationWidget:enable(not hasReference)
      else
        maxLinearAccelerationWidget:enable(false)
      end
    else
      maxLinearAccelerationWidget:enable(false)
    end

    local useMaxAngularAcceleration = getCommonAttributeValue(selection, "UseMaxAngularAcceleration")

    if useMaxAngularAcceleration ~= nil then
      if useMaxAngularAcceleration then
        maxAngularAccelerationWidget:enable(not hasReference)
      else
        maxAngularAccelerationWidget:enable(false)
      end
    else
      maxAngularAccelerationWidget:enable(false)
    end

    attributeEditor.logExitFunc("syncInterface")
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- this data change context ensures the ui reflects any changes that happen via script, or through undo redo.
  ----------------------------------------------------------------------------------------------------------------------

  local changeContext = attributeEditor.createChangeContext()

  changeContext:setObjects(selection)
  changeContext:addAttributeChangeEvent("UseMaxAcceleration")
  changeContext:addAttributeChangeEvent("UseMaxAngularAcceleration")

  changeContext:setAttributeChangedHandler(
    function(object, attr)
      attributeEditor.logEnterFunc("changeContext attributeChangedHandler")
      syncInterface()
      attributeEditor.logExitFunc("changeContext attributeChangedHandler")
    end
  )

  syncInterface()
  attributeEditor.logExitFunc("attributeEditor.addSoftKeyFrameAccelerationLimitsInfoSection")
end


------------------------------------------------------------------------------------------------------------------------
-- Add's a display info section containing per anim set acceleration limits attributes.
-- Use by SoftKeyFrame.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.SoftKeyFrameAccelerationLimitsInfoSection = function(panel, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.SoftKeyFrameAccelerationLimitsInfoSection")

  attributeEditor.log("rollContainter:addRollup")
  local rollup = panel:addRollup{ label = displayInfo.title, flags = "mainSection", name = "SoftKeyFrameAccelerationLimitsInfoSection" }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginVSizer")
  rollPanel:beginVSizer{ flags = "expand" }

    attributeEditor.addAnimationSetWidget(rollPanel, displayInfo.usedAttributes, selection, addSoftKeyFrameAccelerationLimitsInfoSection)

  attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.SoftKeyFrameAccelerationLimitsInfoSection")
end

--------------------------------------------------------------------------------------------------------------------------
-- Adds a SoftKeyFrameGravityCompensationInfoSection
-- Used by SoftKeyFrame
------------------------------------------------------------------------------------------------------------------------
attributeEditor.SoftKeyFrameGravityCompensationInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.SoftKeyFrameGravityCompensationInfoSection")

  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "SoftKeyFrameGravityCompensationInfoSection" }
  local rollPanel = rollup:getPanel()
    -- first add the ui for the section
  attributeEditor.log("rollPanel:beginHSizer")
  rollPanel:beginVSizer{ flags = "expand", proportion = 1 }
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    local gravityCompensation = { }
    for i, object in ipairs(selection) do
      table.insert(gravityCompensation, string.format("%s.GravityCompensation", object))
    end
    attributeEditor.log("rollPanel:addAttributeWidget")
    rollPanel:addHSpacer(6)
    rollPanel:addAttributeWidget{
      attributes = gravityCompensation,
      flags = "expand",
      proportion = 1,
      onMouseEnter = function()
        attributeEditor.setHelpText(getAttributeHelpText(selection[1], "GravityCompensation"))
      end,
      onMouseLeave = function()
        attributeEditor.clearHelpText()
      end
    }
  rollPanel:endSizer()
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.SoftKeyFrameGravityCompensationInfoSection")
end
------------------------------------------------------------------------------------------------------------------------
-- Add's a SoftKeyFrameCharacterControllerInfoSection
-- Used by SoftKeyFrame
------------------------------------------------------------------------------------------------------------------------
attributeEditor.SoftKeyFrameCharacterControllerInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.SoftKeyFrameCharacterControllerInfoSection")

  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "SoftKeyFrameCharacterControllerInfoSection" }
  local rollPanel = rollup:getPanel()
    -- first add the ui for the section
  attributeEditor.log("rollPanel:beginHSizer")
  rollPanel:beginVSizer{ flags = "expand", proportion = 1 }
    rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
      rollPanel:addHSpacer(6)
      attributeEditor.log("rollPanel:addStaticText")
        rollPanel:addStaticText{
          text = "Radius",
          onMouseEnter = function()
            attributeEditor.setHelpText(getAttributeHelpText(selection[1], "ControllerRadiusFraction"))
          end,
          onMouseLeave = function()
            attributeEditor.clearHelpText()
          end
        }

      local Radius = { }
      for i, object in ipairs(selection) do
        table.insert(Radius, string.format("%s.ControllerRadiusFraction", object))
      end

      rollPanel:addHSpacer(6)
      attributeEditor.log("rollPanel:addAttributeWidget")
      rollPanel:addAttributeWidget{
        attributes = Radius,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function()
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "ControllerRadiusFraction"))
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

    rollPanel:endSizer()
    rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
      rollPanel:addHSpacer(6)
      attributeEditor.log("rollPanel:addStaticText")
        rollPanel:addStaticText{
          text = "Height",
          onMouseEnter = function()
            attributeEditor.setHelpText(getAttributeHelpText(selection[1], "ControllerHeightFraction"))
          end,
          onMouseLeave = function()
            attributeEditor.clearHelpText()
          end
        }

      local Height = { }
      for i, object in ipairs(selection) do
        table.insert(Height, string.format("%s.ControllerHeightFraction", object))
      end

      rollPanel:addHSpacer(6)
      attributeEditor.log("rollPanel:addAttributeWidget")
      rollPanel:addAttributeWidget{
        attributes = Height,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function()
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "ControllerHeightFraction"))
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

    rollPanel:endSizer()
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.SoftKeyFrameCharacterControllerInfoSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Add's a SoftKeyFrameAnchorInfoSection
-- Used by SoftKeyFrame
------------------------------------------------------------------------------------------------------------------------
attributeEditor.SoftKeyFrameAnchorInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.SoftKeyFrameAnchorInfoSection")

  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "SoftKeyFrameAnchorInfoSection" }
  local rollPanel = rollup:getPanel()
    -- first add the ui for the section
  attributeEditor.log("rollPanel:beginHSizer")
  rollPanel:beginVSizer{ flags = "expand", proportion = 1 }
    rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
      rollPanel:addHSpacer(6)
      attributeEditor.log("rollPanel:addStaticText")
      rollPanel:addStaticText{
          text = "Method",
          onMouseEnter = function()
            attributeEditor.setHelpText(getAttributeHelpText(selection[1], "UseRootAsAnchor"))
          end,
          onMouseLeave = function()
            attributeEditor.clearHelpText()
          end
        }
      rollPanel:addHSpacer(6)
      attributeEditor.log("attributeEditor.addBoolAttributeCombo")
      attributeEditor.addBoolAttributeCombo{
        panel = rollPanel,
        objects = selection,
        attribute = "UseRootAsAnchor",
        falseValue = "Default",
        trueValue ="Character (WS)",
        helpText = getAttributeHelpText(selection[1], "UseRootAsAnchor")
      }
    rollPanel:endSizer()
  rollPanel:endSizer()
  rollup:expand(false)

  attributeEditor.logExitFunc("attributeEditor.SoftKeyFrameAnchorInfoSection")
end


------------------------------------------------------------------------------------------------------------------------
-- Add's a display info section containing physX 3 joint compliance attributes.for a single anim set.
-- Used by ActiveAnimation, SoftKeyframeAndActiveAnimation and SoftKeyFrame.
------------------------------------------------------------------------------------------------------------------------
local addSoftKeyFrameJointComplianceDisplayInfoSection = function(panel, selection, attributes, set)
  attributeEditor.logEnterFunc("attributeEditor.addSoftKeyFrameJointComplianceDisplayInfoSection")

  local widgetTable = {}

  attributeEditor.log("panel:beginVSizer")
  panel:beginVSizer{ flags = "expand" }

    attributeEditor.log("panel:beginFlexGridSizer")
    panel:beginFlexGridSizer{ cols = 3, flags = "expand", proportion = 1 }
      panel:setFlexGridColumnExpandable(3)

      for _, attributeName in ipairs(attributes) do
        -- Create numeric slider widget for each attribute.
        panel:addHSpacer(6)
        
        local name = utils.getDisplayString(getAttributeDisplayName(selection[1], attributeName))
        attributeEditor.addAttributeLabel(panel, name, selection, attributeName)
        sliderWidget = attributeEditor.addAttributeWidget(panel, attributeName, selection, set)
        sliderWidget:enable( true )

        -- Map widget to attribute name in an associative array.
        widgetTable[attributeName] = sliderWidget
      end

    attributeEditor.log("panel:endSizer")
    panel:endSizer()

  attributeEditor.log("panel:endSizer")
  panel:endSizer()
  
  attributeEditor.logExitFunc("attributeEditor.addSoftKeyFrameJointComplianceDisplayInfoSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Add's a display info section containing physX 3 joint compliance attributes.for all anim sets.
-- Used by ActiveAnimation, SoftKeyframeAndActiveAnimation and SoftKeyFrame.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.SoftKeyFrameJointComplianceDisplayInfoSection = function(rollContainer, displayInfo, selection)

  attributeEditor.logEnterFunc("attributeEditor.SoftKeyFrameJointComplianceDisplayInfoSection")

  if preferences.get("PhysicsEngine") == "PhysX3" then
    attributeEditor.log("rollContainer:addRollup")
      local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "SoftKeyFrameJointComplianceDisplayInfoSection" }
      local rollPanel = rollup:getPanel()

      attributeEditor.addAnimationSetWidget(rollPanel, displayInfo.usedAttributes, selection, addSoftKeyFrameJointComplianceDisplayInfoSection)

    attributeEditor.log("rollPanel:endSizer")
    rollPanel:endSizer()
  end

  attributeEditor.logExitFunc("attributeEditor.SoftKeyFrameJointComplianceDisplayInfoSection")
end
