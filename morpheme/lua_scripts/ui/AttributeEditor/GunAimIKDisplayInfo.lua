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
-- Per animation set gun section
------------------------------------------------------------------------------------------------------------------------
local perAnimSetGunSection = function(panel, selection, attributes, set)
  panel:setBorder(1)
  panel:beginHSizer{ flags = "expand", proportion = 1 }
    panel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      panel:setFlexGridColumnExpandable(2)

      -- gun joint attribute
      attributeEditor.addAttributeLabel(panel, "Gun Joint", selection, "GunJointName")
      attributeEditor.addAttributeWidget(panel, "GunJointName", selection, set)

      -- brace joint attribute
      attributeEditor.addAttributeLabel(panel, "Brace", selection, "GunBindJointName")
      attributeEditor.addAttributeWidget(panel, "GunBindJointName", selection, set)

      -- stock pivot vector
      local gunPivotAttrs = { "GunPivotOffsetX", "GunPivotOffsetY", "GunPivotOffsetZ" }
      attributeEditor.addAttributeLabel(panel, "Stock Position", selection, gunPivotAttrs[1])
      attributeEditor.addVectorAttributeWidget(panel, gunPivotAttrs, selection, set)

      -- barrel position
      local gunBarrelAttrs = { "GunBarrelOffsetX", "GunBarrelOffsetY", "GunBarrelOffsetZ" }
      attributeEditor.addAttributeLabel(panel, "Barrel Position", selection, gunBarrelAttrs[1])
      attributeEditor.addVectorAttributeWidget(panel, gunBarrelAttrs, selection, set)

      -- gun barrel vector
      local gunPointingAttrs = { "GunPointingVectorX", "GunPointingVectorY", "GunPointingVectorZ" }
      attributeEditor.addAttributeLabel(panel, "Barrel Vector", selection, gunPointingAttrs[1])
      attributeEditor.addVectorAttributeWidget(panel, gunPointingAttrs, selection, set)

    panel:endSizer()
  panel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- Per animation set body section
------------------------------------------------------------------------------------------------------------------------
local perAnimSetBodySection = function(panel, selection, attributes, set)
  panel:setBorder(1)
  panel:beginHSizer{ flags = "expand", proportion = 1 }
    panel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      panel:setFlexGridColumnExpandable(2)

      -- root combo
      attributeEditor.addAttributeLabel(panel, "Root", selection, "SpineRootJointName")
      attributeEditor.addAttributeWidget(panel, "SpineRootJointName", selection, set)

      -- bias slider
      attributeEditor.addAttributeLabel(panel, "Bias", selection, "SpineBias")
      local biasWidget = panel:addFloatSlider{ flags = "expand", proportion = 1, min = -1, max = 1, }

    panel:endSizer()
  panel:endSizer()

  -- check if the selection contains any referenced objects
  local enable = not containsReference(selection)
  biasWidget:enable(enable)

  -- this function syncs the ui with the current attribute values
  local syncUIWithAttributes = function()
    local bias = getCommonAttributeValue(selection, "SpineBias", set)
    if bias ~= nil then
      biasWidget:setIsIndeterminate(false)
      biasWidget:setValue(bias)
    else
      biasWidget:setIsIndeterminate(true)
    end
  end

  local helpText = getAttributeHelpText(selection[1], "SpineBias")
  -- set mouse enter/leave and on changed for bias slider.
  biasWidget:setOnMouseEnter(function() attributeEditor.setHelpText(helpText) end)
  biasWidget:setOnMouseLeave(function() attributeEditor.clearHelpText() end)
  biasWidget:setOnChanged(
    function(self)
      local value = self:getValue()
      setCommonAttributeValue(selection, "SpineBias", value, set)
      syncUIWithAttributes()
    end
  )

  -- this data change context ensures the ui reflects any changes that happen to selected
  -- up axis attributes through script or undo redo are reflected in the custom ui.
  local changeContext = attributeEditor.createChangeContext()
  changeContext:setObjects(selection)
  changeContext:addAttributeChangeEvent("SpineBias")
  changeContext:setAttributeChangedHandler(function(object, attr) syncUIWithAttributes() end)

  -- set the initial state of the ui
  syncUIWithAttributes()
end

------------------------------------------------------------------------------------------------------------------------
-- perAnimSetArmSection
------------------------------------------------------------------------------------------------------------------------
local perAnimSetArmSection = function(armType, panel, selection, attributes, set)
  assert(type(armType) == "string")
  assert(armType == "Primary" or armType == "Secondary")

  panel:setBorder(1)
  panel:beginHSizer{ flags = "expand", proportion = 1 }
    panel:beginVSizer{ flags = "expand", proportion = 1 }

      -- only the secondary arm has the use arm attribute and if this is not checked
      -- then all other controls for the secondary arm should be disabled.
      local enable = true
      if armType == "Secondary" then
        panel:beginHSizer{ flags = "expand" }

          enable = getCommonAttributeValue(selection, "UseSecondaryArm", set) or false
          attributeEditor.addAttributeLabel(panel, "Use Secondary Arm", selection, "UseSecondaryArm")
          attributeEditor.addAttributeWidget(panel, "UseSecondaryArm", selection, set)

        panel:endSizer()
        panel:addVSpacer(4)
      end

      panel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
        panel:setFlexGridColumnExpandable(2)

        -- add wrist section
        local wristJointName = string.format("%sWristJointName", armType)
        local wristJointText = attributeEditor.addAttributeLabel(panel, "Wrist", selection, wristJointName)
        local wristJointWidget = attributeEditor.addAttributeWidget(panel, wristJointName, selection, set)

        -- add hinge axis section
        local hingeAxisNames = {
          string.format("%sArmHingeAxisX", armType),
          string.format("%sArmHingeAxisY", armType),
          string.format("%sArmHingeAxisZ", armType),
        }

        local hingeAxisText = attributeEditor.addAttributeLabel(panel, "Hinge Axis", selection, hingeAxisNames[1])
        local hingeAxisWidget = attributeEditor.addVectorAttributeWidget(panel, hingeAxisNames, selection, set)

        panel:addStretchSpacer{ proportion = 1 }
        panel:beginHSizer{ expand = true, flags = "right" }
          local calculateButton = panel:addButton{ label = "Calculate", size = { width = 74, }, }
        panel:endSizer()

        -- flip wrist
        local flipWristName = string.format("Flip%sHinge", armType)
        local flipHingeText = attributeEditor.addAttributeLabel(panel, "Flip Hinge", selection, flipWristName)
        local flipHingeWidget = attributeEditor.addAttributeWidget(panel, flipWristName, selection, set)

      panel:endSizer()
    panel:endSizer()
  panel:endSizer()

  -- set calculate hinge button help text and click functions
  local calculateHelpText =
    string.format("Automatically calculate the %s arm hinge axis based on the axis in the current rig.", string.lower(armType))
  calculateButton:setOnMouseEnter(function() attributeEditor.setHelpText(calculateHelpText) end)
  calculateButton:setOnMouseLeave(function() attributeEditor.clearHelpText() end)
  calculateButton:setOnClick(
    function()
      local endJoint = getCommonAttributeValue(selection, wristJointName, set)
      local axis = calculateHingeAxis(set, endJoint)
      setCommonAttributeValue(selection, hingeAxisNames[1], axis:getX(), set)
      setCommonAttributeValue(selection, hingeAxisNames[2], axis:getY(), set)
      setCommonAttributeValue(selection, hingeAxisNames[3], axis:getZ(), set)
      -- Deselect Flip Hinge which is assumed off by Calculate
      if armType == "Primary" then
        setCommonAttributeValue(selection, "FlipPrimaryHinge", false, set)
      elseif armType == "Secondary" then
        setCommonAttributeValue(selection, "FlipSecondaryHinge", false, set)
      end
    end
  )

  local enableControls = function(enable)
    local enable = enable and not containsReference(selection)
    wristJointText:enable(enable)
    wristJointWidget:enable(enable)
    hingeAxisText:enable(enable)
    hingeAxisWidget:enable(enable)
    calculateButton:enable(enable)
    flipHingeText:enable(enable)
    flipHingeWidget:enable(enable)
  end

  -- if this is the secondary joint section then set the change context to enable the ui depending on
  -- the UseSecondaryArm attribute.
  if armType == "Secondary" then
    local changeContext = attributeEditor.createChangeContext()
    changeContext:setObjects(selection)
    changeContext:addAttributeChangeEvent("UseSecondaryArm")
    changeContext:setAttributeChangedHandler(
      function(object, attr)
        local enable = getCommonAttributeValue(selection, "UseSecondaryArm", animSet) or false
        enableControls(enable)
      end
    )
  end

  enableControls(enable)
end

------------------------------------------------------------------------------------------------------------------------
-- Per animation set properties section
------------------------------------------------------------------------------------------------------------------------
local perAnimSetPropertiesSection = function(panel, selection, attributes, set)
  panel:setBorder(1)
  panel:beginHSizer{ flags = "expand", proportion = 1 }
    panel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      panel:setFlexGridColumnExpandable(2)

      -- Update frame
      attributeEditor.addAttributeLabel(panel, "Update Frame", selection, "UpdateTargetByDeltas")
      attributeEditor.addBoolAttributeCombo{
        panel = panel,
        objects = selection,
        attribute = "UpdateTargetByDeltas",
        trueValue = "Previous",
        falseValue ="Current",
        helpText = getAttributeHelpText(selection[1], "UpdateTargetByDeltas"),
        set = set,
      }

    panel:endSizer()
  panel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- Adds the GunAimIK gun section to the attribute editor
------------------------------------------------------------------------------------------------------------------------
attributeEditor.gunAimIkGunDisplayInfoSection = function(panel, displayInfo, selection)
  local attrs = checkSetOrderAndBuildAttributeList(selection, displayInfo.usedAttributes)

  local rollup = panel:addRollup{
    label = displayInfo.title,
    flags = "mainSection",
    name = string.format("GunAimIkGunDisplayInfoSection")
  }
  local rollPanel = rollup:getPanel()

  rollPanel:beginVSizer{ flags = "expand" }
    rollPanel:addAnimationSetWidget{
      attributes = attrs,
      displayFunc = perAnimSetGunSection,
      flags = "expand",
      proportion = 1,
    }
  rollPanel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- Adds the GunAimIK body section to the attribute editor
------------------------------------------------------------------------------------------------------------------------
attributeEditor.gunAimIkBodyDisplayInfoSection = function(panel, displayInfo, selection)
  local attrs = checkSetOrderAndBuildAttributeList(selection, displayInfo.usedAttributes)

  local rollup = panel:addRollup{
    label = displayInfo.title,
    flags = "mainSection",
    name = string.format("GunAimIkBodyDisplayInfoSection")
  }
  local rollPanel = rollup:getPanel()

  rollPanel:beginVSizer{ flags = "expand" }
    rollPanel:addAnimationSetWidget{
      attributes = attrs,
      displayFunc = perAnimSetBodySection,
      flags = "expand",
      proportion = 1,
    }
  rollPanel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- Adds the GunAimIK Primary or Secondary arm sections to the attribute editor
------------------------------------------------------------------------------------------------------------------------
attributeEditor.gunAimIkArmDisplayInfoSection = function(panel, displayInfo, selection)
  -- first work out which arm this section is for using the title of the rollup section
  local armType = string.find(displayInfo.title, "Primary") and "Primary" or nil
  armType = armType or string.find(displayInfo.title, "Secondary") and "Secondary" or nil
  assert(armType, "displayInfo.title for GunAimIK arm section must contain either the word \"Primary\" or \"Secondary\"")

  local attrs = checkSetOrderAndBuildAttributeList(selection, displayInfo.usedAttributes)

  local rollup = panel:addRollup{
    label = displayInfo.title,
    flags = "mainSection",
    name = string.format("GunAimIk%sArmDisplayInfoSection", armType)
  }
  local rollPanel = rollup:getPanel()

  rollPanel:beginVSizer{ flags = "expand" }
    rollPanel:addAnimationSetWidget{
      attributes = attrs,
      displayFunc = function(panel, selection, attributes, set)
        perAnimSetArmSection(armType, panel, selection, attributes, set)
      end,
      flags = "expand",
      proportion = 1,
    }
  rollPanel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- Adds the GunAimIK properties section to the attribute editor
------------------------------------------------------------------------------------------------------------------------
attributeEditor.gunAimIkPropertiesDisplayInfoSection = function(panel, displayInfo, selection)
  local attrs = checkSetOrderAndBuildAttributeList(selection, displayInfo.usedAttributes)

  local rollup = panel:addRollup{
    label = displayInfo.title,
    flags = "mainSection",
    name = string.format("GunAimIkPropertiesDisplayInfoSection")
  }
  local rollPanel = rollup:getPanel()

  rollPanel:beginVSizer{ flags = "expand" }
    rollPanel:addAnimationSetWidget{
      attributes = attrs,
      displayFunc = perAnimSetPropertiesSection,
      flags = "expand",
      proportion = 1,
    }
  rollPanel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- Adds the GunAimIK general section to the attribute editor
------------------------------------------------------------------------------------------------------------------------
attributeEditor.gunAimIkGeneralDisplayInfoSection = function(panel, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.gunAimIkGeneralDisplayInfoSection")

  local targetSpaceHelpText = [[The coordinate frame of the target control parameter, which can either be World Space or Character Space.]]

  -- add the ui for the section
  attributeEditor.log("panel:addRollup")
  local rollup = panel:addRollup{
    label = displayInfo.title,
    flags = "mainSection",
    name = "gunAimIkGeneralDisplayInfoSection" }
  local rollPanel = rollup:getPanel()

  -- add widgets
  attributeEditor.log("rollPanel:beginHSizer")
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    rollPanel:setBorder(1)

    attributeEditor.log("rollPanel:beginFlexGridSizer")
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      rollPanel:setFlexGridColumnExpandable(2)

      -- Target Space
      attributeEditor.addAttributeLabel(rollPanel, "Target Frame", selection, "WorldSpaceTarget")
      attributeEditor.addBoolAttributeCombo{
        panel = rollPanel,
        objects = selection,
        attribute = "WorldSpaceTarget",
        trueValue = "World Space",
        falseValue = "Character Space",
        helpText = targetSpaceHelpText
      }

      -- Keep Upright
      attributeEditor.addAttributeLabel(rollPanel, "Keep Gun Upright", selection, "KeepUpright")
      attributeEditor.addAttributeWidget(rollPanel, "KeepUpright", selection)

      -- Apply Joint Limits
      attributeEditor.addAttributeLabel(rollPanel, "Apply Joint Limits", selection, "ApplyJointLimits")
      attributeEditor.addAttributeWidget(rollPanel, "ApplyJointLimits", selection)

      attributeEditor.log("rollPanel:endSizer")
    rollPanel:endSizer()

    attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.gunAimIkGeneralDisplayInfoSection")
end
