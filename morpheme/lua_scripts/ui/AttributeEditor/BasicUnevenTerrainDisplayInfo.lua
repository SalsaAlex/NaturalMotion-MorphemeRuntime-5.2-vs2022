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
-- Per animation set basic uneven terrain hips section
------------------------------------------------------------------------------------------------------------------------
local perAnimSetHipsSection = function(panel, selection, attributes, set)
  panel:setBorder(1)
  panel:beginHSizer{ flags = "expand", proportion = 1 }
    panel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      panel:setFlexGridColumnExpandable(2)

      -- hips joint attribute
      attributeEditor.addAttributeLabel(panel, "Hips Joint", selection, "HipsName")
      attributeEditor.addAttributeWidget(panel, "HipsName", selection, set)

      -- HipsHeightControlEnable
      attributeEditor.addAttributeLabel(panel, "Use Hips Height Control", selection, "HipsHeightControlEnable")
      attributeEditor.addAttributeWidget(panel, "HipsHeightControlEnable", selection, set)

    panel:endSizer()
  panel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- perAnimSetLegSection
------------------------------------------------------------------------------------------------------------------------
local perAnimSetLegSection = function(panel, selection, attributes, set)
  -- check if the selection contains any referenced objects
  local hasReference = containsReference(selection)

  local calculateHelpText = "Automatically calculate the Knee Axis based on the normal to the plane of the leg bones (select a suitable frame of animation in the animation browser)."

  panel:setBorder(1)
  panel:beginVSizer{ flags = "expand", proportion = 1 }

    -- shared leg attributes
    panel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 0 }
      panel:setFlexGridColumnExpandable(2)

      -- ball joint enable
      local EnableBallJointText = attributeEditor.addAttributeLabel(panel, "Specify Ball", selection, "BallJointEnable")
      local EnableBallJointWidget = attributeEditor.addAttributeWidget(panel, "BallJointEnable", selection, set)

      -- toe joint enable
      local EnableToeJointText = attributeEditor.addAttributeLabel(panel, "Specify Toe", selection, "ToeJointEnable")
      local EnableToeJointWidget = attributeEditor.addAttributeWidget(panel, "ToeJointEnable", selection, set)

      -- StraightestLegFactor
      local straightestLegFactorText = attributeEditor.addAttributeLabel(panel, "Straightest Leg Factor", selection, "StraightestLegFactor")
      local straightestLegFactorWidget = attributeEditor.addAttributeWidget(panel, "StraightestLegFactor", selection, set)

    panel:endSizer()
    panel:addVSpacer(10)

    -- left leg specific attributes
    panel:beginVSizer{ label = "Left", flags = "expand;group", proportion = 1 }
    panel:addVSpacer(10)

      panel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      panel:setFlexGridColumnExpandable(2)
        -- Ankle joint
        local LeftAnkleJointText = attributeEditor.addAttributeLabel(panel, "Ankle", selection, "LeftAnkleName")
        local LeftAnkleJointWidget = attributeEditor.addAttributeWidget(panel, "LeftAnkleName", selection, set)

        -- Ball joint
        local LeftBallJointText = attributeEditor.addAttributeLabel(panel, "Ball", selection, "LeftBallName")
        local LeftBallJointWidget = attributeEditor.addAttributeWidget(panel, "LeftBallName", selection, set)

        -- Toe joint
        local LeftToeJointText = attributeEditor.addAttributeLabel(panel, "Toe", selection, "LeftToeName")
        local LeftToeJointWidget = attributeEditor.addAttributeWidget(panel, "LeftToeName", selection, set)

        -- Knee rotation axis
        local LeftKneeRotationAxisAttrs = {
                          string.format("LeftKneeRotationAxisX", legType),
                          string.format("LeftKneeRotationAxisY", legType),
                          string.format("LeftKneeRotationAxisZ", legType),
                          }
        attributeEditor.addAttributeLabel(panel, "Knee Axis", selection, LeftKneeRotationAxisAttrs[1])
        attributeEditor.addVectorAttributeWidget(panel, LeftKneeRotationAxisAttrs, selection, set)

        -- Calculate Knee rotation axis
        panel:addHSpacer(6)
        local calculateLeftKneeAxisButton = panel:addButton {
          label = "Calculate",
          onMouseEnter = function()
            attributeEditor.setHelpText(calculateHelpText)
          end,
          onMouseLeave = function()
            attributeEditor.clearHelpText()
          end
        }
        calculateLeftKneeAxisButton:enable(not hasReference)

        calculateLeftKneeAxisButton:setOnClick(
          function(self)
            -- For multiselect to work we have to iterate through selected objects
            for i, selectedObject in selection do
              if getType(selectedObject) == "BasicUnevenTerrain" then
                local ankleJointName = getAttribute(selectedObject, "LeftAnkleName", set)
                if ankleJointName ~= nil then
                  local axis = calculateHingeAxis(set, ankleJointName)
                  setAttribute(selectedObject .. ".LeftKneeRotationAxisX", axis:getX(), set)
                  setAttribute(selectedObject .. ".LeftKneeRotationAxisY", axis:getY(), set)
                  setAttribute(selectedObject .. ".LeftKneeRotationAxisZ", axis:getZ(), set)

                  -- Deselect Flip Knee which is assumed off by Calculate
                  setAttribute(selectedObject .. ".LeftFlipKneeRotationDirection", false, set)
                end
              end
            end
          end
        )

        -- Flip Knee rotation direction
        local flipLeftKneeRotationDirectionName = string.format("LeftFlipKneeRotationDirection", legType)
        local flipLeftKneeRotationDirectionText = attributeEditor.addAttributeLabel(panel, "Flip Knee Axis", selection, flipLeftKneeRotationDirectionName)
        local flipLeftKneeRotationDirectionWidget = attributeEditor.addAttributeWidget(panel, flipLeftKneeRotationDirectionName, selection, set)

      panel:endSizer()

    panel:endSizer()
    panel:addVSpacer(10)

    -- right leg specific attributes
    panel:beginVSizer{ label = "Right", flags = "expand;group", proportion = 1 }
      panel:addVSpacer(10)

      panel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      panel:setFlexGridColumnExpandable(2)
        -- Ankle joint
        local RightAnkleJointText = attributeEditor.addAttributeLabel(panel, "Ankle", selection, "RightAnkleName")
        local RightAnkleJointWidget = attributeEditor.addAttributeWidget(panel, "RightAnkleName", selection, set)

        -- Ball joint
        local RightBallJointText = attributeEditor.addAttributeLabel(panel, "Ball", selection, "RightBallName")
        local RightBallJointWidget = attributeEditor.addAttributeWidget(panel, "RightBallName", selection, set)

        -- Toe joint
        local RightToeJointText = attributeEditor.addAttributeLabel(panel, "Toe", selection, "RightToeName")
        local RightToeJointWidget = attributeEditor.addAttributeWidget(panel, "RightToeName", selection, set)

        -- Knee rotation axis
        local RightKneeRotationAxisAttrs = {
                          string.format("RightKneeRotationAxisX", legType),
                          string.format("RightKneeRotationAxisY", legType),
                          string.format("RightKneeRotationAxisZ", legType),
                          }
        attributeEditor.addAttributeLabel(panel, "Knee Axis", selection, RightKneeRotationAxisAttrs[1])
        attributeEditor.addVectorAttributeWidget(panel, RightKneeRotationAxisAttrs, selection, set)

        -- Calculate Knee rotation axis
        panel:addHSpacer(6)
        local calculateRightKneeAxisButton = panel:addButton {
          label = "Calculate",
          onMouseEnter = function()
            attributeEditor.setHelpText(calculateHelpText)
          end,
          onMouseLeave = function()
            attributeEditor.clearHelpText()
          end
        }
        calculateRightKneeAxisButton:enable(not hasReference)

        calculateRightKneeAxisButton:setOnClick(
          function(self)
            -- For multiselect to work we have to iterate through selected objects
            for i, selectedObject in selection do
              if getType(selectedObject) == "BasicUnevenTerrain" then
                local ankleJointName = getAttribute(selectedObject, "RightAnkleName", set)
                if ankleJointName ~= nil then
                  local axis = calculateHingeAxis(set, ankleJointName)
                  setAttribute(selectedObject .. ".RightKneeRotationAxisX", axis:getX(), set)
                  setAttribute(selectedObject .. ".RightKneeRotationAxisY", axis:getY(), set)
                  setAttribute(selectedObject .. ".RightKneeRotationAxisZ", axis:getZ(), set)

                  -- Deselect Flip Knee which is assumed off by Calculate
                  setAttribute(selectedObject .. ".RightFlipKneeRotationDirection", false, set)
                end
              end
            end
          end
        )

        -- Flip Knee rotation direction
        local flipRightKneeRotationDirectionName = string.format("RightFlipKneeRotationDirection", legType)
        local flipRightKneeRotationDirectionText = attributeEditor.addAttributeLabel(panel, "Flip Knee Axis", selection, flipRightKneeRotationDirectionName)
        local flipRightKneeRotationDirectionWidget = attributeEditor.addAttributeWidget(panel, flipRightKneeRotationDirectionName, selection, set)

      panel:endSizer()
    panel:endSizer()

  panel:endSizer()

  ----------------------------------------------------------------------------------------------------------------------
  -- enable LowerHeightBound based on the value of FixGroundPenetration
  -- enable ToeLowerHeightBound based on the value of FixToeGroundPenetration
  ----------------------------------------------------------------------------------------------------------------------
  local syncBallJointInterfaceFromAttributes = function()
    attributeEditor.logEnterFunc("syncBallJointInterfaceFromAttributes")

    local enableBallJoint = getCommonAttributeValue(selection, "BallJointEnable", set)
    local enableToeJoint = getCommonAttributeValue(selection, "ToeJointEnable", set)

    if enableBallJoint ~= nil then
      RightBallJointWidget:enable(enableBallJoint and not hasReference)
      LeftBallJointWidget:enable(enableBallJoint and not hasReference)
    end

    if enableToeJoint ~= nil then
      RightToeJointWidget:enable(enableToeJoint and not hasReference)
      LeftToeJointWidget:enable(enableToeJoint and not hasReference)
    end

    attributeEditor.logExitFunc("syncBallJointInterfaceFromAttributes")
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- this data change context ensures the ui reflects any changes that happen through script or undo redo.
  local changeContext = attributeEditor.createChangeContext()
  changeContext:setObjects(selection)
  changeContext:addAttributeChangeEvent("BallJointEnable")
  changeContext:addAttributeChangeEvent("ToeJointEnable")

  changeContext:setAttributeChangedHandler(
    function(object, attr)
      attributeEditor.logEnterFunc("changeContext attributeChangedHandler")

      syncBallJointInterfaceFromAttributes()

      attributeEditor.logExitFunc("changeContext attributeChangedHandler")
    end
  )

  syncBallJointInterfaceFromAttributes()

end

------------------------------------------------------------------------------------------------------------------------
-- Per animation set basic uneven terrain limits section
------------------------------------------------------------------------------------------------------------------------
local perLimitsSetSection = function(panel, selection, attributes, set)
  -- check if the selection contains any referenced objects
  local hasReference = containsReference(selection)

  panel:setBorder(1)
  panel:beginVSizer{ flags = "expand", proportion = 1 }
    -- hips specific limits
    panel:beginVSizer{ label = "Hips Limits", flags = "expand;group", proportion = 0 }
    panel:addVSpacer(10)

      panel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 0 }
      panel:setFlexGridColumnExpandable(2)

        -- HipsPosVelLimitEnable
        attributeEditor.addAttributeLabel(panel, "Pos Vel Limit Enable", selection, "HipsPosVelLimitEnable")
        local HipsPosVelLimitEnableWidget = attributeEditor.addAttributeWidget(panel, "HipsPosVelLimitEnable", selection, set)

        -- HipsPosVelLimit
        attributeEditor.addAttributeLabel(panel, "Pos Vel Limit", selection, "HipsPosVelLimit")
        local HipsPosVelLimitWidget = attributeEditor.addAttributeWidget(panel, "HipsPosVelLimit", selection, set)

        -- HipsPosAccelLimitEnable
        attributeEditor.addAttributeLabel(panel, "Pos Accel Limit Enable", selection, "HipsPosAccelLimitEnable")
        local HipsPosAccelLimitEnableWidget = attributeEditor.addAttributeWidget(panel, "HipsPosAccelLimitEnable", selection, set)

        -- HipsPosAccelLimit
        attributeEditor.addAttributeLabel(panel, "Pos Accel Limit", selection, "HipsPosAccelLimit")
        local HipsPosAccelLimitWidget = attributeEditor.addAttributeWidget(panel, "HipsPosAccelLimit", selection, set)
      panel:endSizer()

    panel:endSizer()

    -- leg specific limits
    panel:beginVSizer{ label = "Ankle Limits", flags = "expand;group", proportion = 1 }
      panel:addVSpacer(10)

      panel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      panel:setFlexGridColumnExpandable(2)

        ---------------------------
        -- Leg IK end joint limit clamping

        -- AnklePosVelLimitEnable
        local anklePosVelLimitEnableName = "AnklePosVelLimitEnable"
        local anklePosVelLimitEnableText = attributeEditor.addAttributeLabel(panel, "Pos Vel Limit Enable", selection, anklePosVelLimitEnableName)
        local anklePosVelLimitEnableWidget = attributeEditor.addAttributeWidget(panel, anklePosVelLimitEnableName, selection, set)

        -- AnklePosVelLimit
        local anklePosVelLimitName = "AnklePosVelLimit"
        local anklePosVelLimitText = attributeEditor.addAttributeLabel(panel, "Pos Vel Limit", selection, anklePosVelLimitName)
        local anklePosVelLimitWidget = attributeEditor.addAttributeWidget(panel, anklePosVelLimitName, selection, set)

        -- AnklePosAccelLimitEnable
        local anklePosAccelLimitEnableName = "AnklePosAccelLimitEnable"
        local anklePosAccelLimitEnableText = attributeEditor.addAttributeLabel(panel, "Pos Accel Limit Enable", selection, anklePosAccelLimitEnableName)
        local anklePosAccelLimitEnableWidget = attributeEditor.addAttributeWidget(panel, anklePosAccelLimitEnableName, selection, set)

        -- AnklePosAccelLimit
        local anklePosAccelLimitName = "AnklePosAccelLimit"
        local anklePosAccelLimitText = attributeEditor.addAttributeLabel(panel, "Pos Accel Limit", selection, anklePosAccelLimitName)
        local anklePosAccelLimitWidget = attributeEditor.addAttributeWidget(panel, anklePosAccelLimitName, selection, set)

        -- AnkleAngVelLimitEnable
        local ankleAngVelLimitEnableName = "AnkleAngVelLimitEnable"
        local ankleAngVelLimitEnableText = attributeEditor.addAttributeLabel(panel, "Ang Vel Enable", selection, ankleAngVelLimitEnableName)
        local ankleAngVelLimitEnableWidget = attributeEditor.addAttributeWidget(panel, ankleAngVelLimitEnableName, selection, set)

        -- AnkleAngVelLimit
        local ankleAngVelLimitName = "AnkleAngVelLimit"
        local ankleAngVelLimitText = attributeEditor.addAttributeLabel(panel, "Ang Vel Limit", selection, ankleAngVelLimitName)
        local ankleAngVelLimitWidget = attributeEditor.addAttributeWidget(panel, ankleAngVelLimitName, selection, set)

        -- AnkleAngAccelLimitEnable
        local ankleAngAccelLimitEnableName = "AnkleAngAccelLimitEnable"
        local ankleAngAccelLimitEnableText = attributeEditor.addAttributeLabel(panel, "Ang Accel Enable", selection, ankleAngAccelLimitEnableName)
        local ankleAngAccelLimitEnableWidget = attributeEditor.addAttributeWidget(panel, ankleAngAccelLimitEnableName, selection, set)

        -- AnkleAngAccelLimit
        local ankleAngAccelLimitName = "AnkleAngAccelLimit"
        local ankleAngAccelLimitText = attributeEditor.addAttributeLabel(panel, "Ang Accel Limit", selection, ankleAngAccelLimitName)
        local ankleAngAccelLimitWidget = attributeEditor.addAttributeWidget(panel, ankleAngAccelLimitName, selection, set)

      panel:endSizer()
    panel:endSizer()

    -- foot specific limits
    panel:beginVSizer{ label = "Foot Limits", flags = "expand;group", proportion = 0 }
    panel:addVSpacer(10)

      panel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 0 }
      panel:setFlexGridColumnExpandable(2)

        -- FootLiftingHeightLimit
        local footLiftingHeightLimitName = "FootLiftingHeightLimit"
        local footLiftingHeightLimitText = attributeEditor.addAttributeLabel(panel, "Lifting Height Limit", selection, footLiftingHeightLimitName)
        local footLiftingHeightLimitWidget = attributeEditor.addAttributeWidget(panel, footLiftingHeightLimitName, selection, set)

        -- UseGroundPenetrationFixup
        local useGroundPenetrationFixupName = "UseGroundPenetrationFixup"
        local useGroundPenetrationFixupText = attributeEditor.addAttributeLabel(panel, "Ground Penetration Fixup", selection, useGroundPenetrationFixupName)
        local useGroundPenetrationFixupWidget = attributeEditor.addAttributeWidget(panel, useGroundPenetrationFixupName, selection, set)

        -- UseTrajectorySlopeAlignment
        local useTrajectorySlopeAlignmentName = "UseTrajectorySlopeAlignment"
        local useTrajectorySlopeAlignmentText = attributeEditor.addAttributeLabel(panel, "Trajectory Alignment", selection, useTrajectorySlopeAlignmentName)
        local useTrajectorySlopeAlignmentWidget = attributeEditor.addAttributeWidget(panel, useTrajectorySlopeAlignmentName, selection, set)

        -- FootAlignToSurfaceAngleLimit
        local footAlignToSurfaceAngleLimitName = "FootAlignToSurfaceAngleLimit"
        local footAlignToSurfaceAngleLimitText = attributeEditor.addAttributeLabel(panel, "Surface Angle Limit", selection, footAlignToSurfaceAngleLimitName)
        local footAlignToSurfaceAngleLimitWidget = attributeEditor.addAttributeWidget(panel, footAlignToSurfaceAngleLimitName, selection, set)

        -- FootAlignToSurfaceMaxSlopeAngle
        local footAlignToSurfaceMaxSlopeAngleName = "FootAlignToSurfaceMaxSlopeAngle"
        local footAlignToSurfaceMaxSlopeAngleText = attributeEditor.addAttributeLabel(panel, "Surface Max Slope Angle", selection, footAlignToSurfaceMaxSlopeAngleName)
        local footAlignToSurfaceMaxSlopeAngleWidget = attributeEditor.addAttributeWidget(panel, footAlignToSurfaceMaxSlopeAngleName, selection, set)

      panel:endSizer()
    panel:endSizer()

  panel:endSizer()

  ----------------------------------------------------------------------------------------------------------------------
  -- enable text boxes based on the value of governing booleans
  ----------------------------------------------------------------------------------------------------------------------
  local syncLimitsInterfaceFromAttributes = function()
    attributeEditor.logEnterFunc("syncLimitsInterfaceFromAttributes")

    -- Hips Limits
    local HipsHeightControlEnable = getCommonAttributeValue(selection, "HipsHeightControlEnable", set)
    if HipsHeightControlEnable and not hasReference then

      HipsPosVelLimitEnableWidget:enable(true)
      local HipsPosVelLimitEnable = getCommonAttributeValue(selection, "HipsPosVelLimitEnable", set)
      if HipsPosVelLimitEnable ~= nil then
        HipsPosVelLimitWidget:enable(HipsPosVelLimitEnable)
      end

      HipsPosAccelLimitEnableWidget:enable(true)
      local HipsPosAccelLimitEnable = getCommonAttributeValue(selection, "HipsPosAccelLimitEnable", set)
      if HipsPosAccelLimitEnable ~= nil then
        HipsPosAccelLimitWidget:enable(HipsPosAccelLimitEnable)
      end
    else
        HipsPosVelLimitEnableWidget:enable(false)
        HipsPosVelLimitWidget:enable(false)
        HipsPosAccelLimitEnableWidget:enable(false)
        HipsPosAccelLimitWidget:enable(false)
    end

    -- Ankle Limits
    local AnklePosVelLimitEnable = getCommonAttributeValue(selection, "AnklePosVelLimitEnable", set)
    if AnklePosVelLimitEnable ~= nil then
      anklePosVelLimitWidget:enable(AnklePosVelLimitEnable and not hasReference)
    end

    local AnklePosAccelLimitEnable = getCommonAttributeValue(selection, "AnklePosAccelLimitEnable", set)
    if AnklePosAccelLimitEnable ~= nil then
      anklePosAccelLimitWidget:enable(AnklePosAccelLimitEnable and not hasReference)
    end

    local AnkleAngVelLimitEnable = getCommonAttributeValue(selection, "AnkleAngVelLimitEnable", set)
    if AnkleAngVelLimitEnable ~= nil then
      ankleAngVelLimitWidget:enable(AnkleAngVelLimitEnable and not hasReference)
    end

    local AnkleAngAccelLimitEnable = getCommonAttributeValue(selection, "AnkleAngAccelLimitEnable", set)
    if AnkleAngAccelLimitEnable ~= nil then
      ankleAngAccelLimitWidget:enable(AnkleAngAccelLimitEnable and not hasReference)
    end

    attributeEditor.logExitFunc("syncLimitsInterfaceFromAttributes")
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- this data change context ensures the ui reflects any changes that happen through script or undo redo.
  local changeContext = attributeEditor.createChangeContext()
  changeContext:setObjects(selection)
  changeContext:addAttributeChangeEvent("HipsHeightControlEnable")
  changeContext:addAttributeChangeEvent("HipsPosVelLimitEnable")
  changeContext:addAttributeChangeEvent("HipsPosAccelLimitEnable")
  changeContext:addAttributeChangeEvent("AnklePosVelLimitEnable")
  changeContext:addAttributeChangeEvent("AnklePosAccelLimitEnable")
  changeContext:addAttributeChangeEvent("AnkleAngVelLimitEnable")
  changeContext:addAttributeChangeEvent("AnkleAngAccelLimitEnable")

  changeContext:setAttributeChangedHandler(
    function(object, attr)
      attributeEditor.logEnterFunc("changeContext attributeChangedHandler")

      syncLimitsInterfaceFromAttributes()

      attributeEditor.logExitFunc("changeContext attributeChangedHandler")
    end
  )

  syncLimitsInterfaceFromAttributes()
end

------------------------------------------------------------------------------------------------------------------------
-- Adds the basic uneven terrain hips section to the attribute editor
------------------------------------------------------------------------------------------------------------------------
attributeEditor.basicUnevenTerrainHipsDisplayInfoSection = function(panel, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.basicUnevenTerrainHipsDisplayInfoSection")

  local attrs = checkSetOrderAndBuildAttributeList(selection, displayInfo.usedAttributes)

  local rollup = panel:addRollup{
    label = displayInfo.title,
    flags = "mainSection",
    name = string.format("BasicUnevenTerrainHipsDisplayInfoSection")
  }
  local rollPanel = rollup:getPanel()

  rollPanel:beginVSizer{ flags = "expand" }
    rollPanel:addAnimationSetWidget{
      attributes = attrs,
      displayFunc = perAnimSetHipsSection,
      flags = "expand",
      proportion = 1,
    }
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.basicUnevenTerrainHipsDisplayInfoSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Adds the basic uneven terrain leg sections to the attribute editor
------------------------------------------------------------------------------------------------------------------------
attributeEditor.basicUnevenTerrainLegDisplayInfoSection = function(panel, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.basicUnevenTerrainLegDisplayInfoSection")

  local attrs = checkSetOrderAndBuildAttributeList(selection, displayInfo.usedAttributes)

  local rollup = panel:addRollup{
    label = displayInfo.title,
    flags = "mainSection",
    name = "BasicUnevenTerrainLegDisplayInfoSection",
  }
  local rollPanel = rollup:getPanel()

  rollPanel:beginVSizer{ flags = "expand" }
    rollPanel:addAnimationSetWidget{
      attributes = attrs,
      displayFunc = perAnimSetLegSection,
      flags = "expand",
      proportion = 1,
    }
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.basicUnevenTerrainLegDisplayInfoSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Adds the basic uneven terrain leg sections to the attribute editor
------------------------------------------------------------------------------------------------------------------------
attributeEditor.basicUnevenTerrainLimitsDisplayInfoSection = function(panel, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.basicUnevenTerrainLimitsDisplayInfoSection")

  local attrs = checkSetOrderAndBuildAttributeList(selection, displayInfo.usedAttributes)

  local rollup = panel:addRollup{
    label = displayInfo.title,
    flags = "mainSection",
    name = "basicUnevenTerrainLimitsDisplayInfoSection",
  }
  local rollPanel = rollup:getPanel()

  rollPanel:beginVSizer{ flags = "expand" }
    rollPanel:addAnimationSetWidget{
      attributes = attrs,
      displayFunc = perLimitsSetSection,
      flags = "expand",
      proportion = 1,
    }
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.basicUnevenTerrainLimitsDisplayInfoSection")
end
