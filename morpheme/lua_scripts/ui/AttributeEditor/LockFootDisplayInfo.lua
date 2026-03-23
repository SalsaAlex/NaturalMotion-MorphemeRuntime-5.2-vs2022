------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"

local perAnimSetLockFootJointAxisDisplayInfoSection = function(panel, selection, attributes, set)
  attributeEditor.logEnterFunc("perAnimSetLockFootJointAxisDisplayInfoSection")

  local scene = nmx.Application.new():getSceneByName("AssetManager")
  local kneeVectorLabel = nil
  local kneeVectorWidget = { }
  local ballVectorLabel = nil
  local ballVectorWidget = { }
  local ballVectorCalculateButton = nil

  -- check if the selection contains any referenced objects
  local hasReference = containsReference(selection)

  -- first add the ui for the section
  attributeEditor.log("panel:beginHSizer")
  panel:beginHSizer{ flags = "expand", proportion = 1 }
    panel:addHSpacer(6)
    panel:setBorder(1)

    attributeEditor.log("panel:beginFlexGridSizer")
    panel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      panel:setFlexGridColumnExpandable(2)

      local useBallJointAttrPaths = { }
      for i, object in ipairs(selection) do
        table.insert(useBallJointAttrPaths, string.format("%s.UseBallJoint", object))
      end

      attributeEditor.addAttributeLabel(panel, "Use ball", selection, "UseBallJoint")
      panel:addAttributeWidget{
        attributes = useBallJointAttrPaths,
        flags = "expand",
        proportion = 1,
        set = set,
        onMouseEnter = function(self)
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "UseBallJoint"))
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      ballVectorLabel = attributeEditor.addAttributeLabel(panel, "Ball", selection, "BallRotationAxisX")
      local ballAttrs = { "BallRotationAxisX", "BallRotationAxisY", "BallRotationAxisZ" }
      ballVectorWidget = attributeEditor.addVectorAttributeWidget(panel, ballAttrs, selection, set)

      panel:addHSpacer(6)
      local calculateBallAxisHelpText = "Automatically calculate the Ball Axis by assuming hinging in a direction parallel to the ground.  Select an animation frame in the animation browser that has the relevant foot flat on the ground."
      ballVectorCalculateButton = panel:addButton {
        label = "Calculate",
        onMouseEnter = function()
          attributeEditor.setHelpText(calculateBallAxisHelpText)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }
      if containsReference(selection) then
        ballVectorCalculateButton:enable(false)
      end

      ballVectorCalculateButton:setOnClick(
        function(self)
          -- For multiselect to work we have to iterate through selected objects
          for i, selectedObject in selection do
            if getType(selectedObject) == "LockFoot" then

              -- Retrieve up vector
              local worldUpAxis = preferences.get("WorldUpAxis")
              local upVector = nmx.Vector3.new(0, 0, 0)
              if worldUpAxis == "Y Axis" then
                upVector:set(0.0, 1.0, 0.0)
              else
                upVector:set(0.0, 0.0, 1.0)
              end

              local useBallJoint = getAttribute(selectedObject, "UseBallJoint", set)
              if useBallJoint then
                local ballJointName = getAttribute(selectedObject, "BallName", set)
                local toeJointName = getAttribute(selectedObject, "ToeName", set)
                if ballJointName ~= nil and toeJointName ~= nil then
                  local ballIndex = anim.getRigChannelIndex(ballJointName, set)
                  local ankleIndex = anim.getParentBoneIndex(ballIndex, set)
                  ankleJointName = anim.getRigChannelNames(set)[ankleIndex + 1]

                  local axis = calculateLiftAxis(set, ankleJointName, ballJointName, toeJointName, upVector)
                  setAttribute(selectedObject .. ".BallRotationAxisX", axis:getX(), set)
                  setAttribute(selectedObject .. ".BallRotationAxisY", axis:getY(), set)
                  setAttribute(selectedObject .. ".BallRotationAxisZ", axis:getZ(), set)
                end
              end
            end
          end
        end
      )

      kneeVectorLabel = attributeEditor.addAttributeLabel(panel, "Knee", selection, "KneeRotationAxisX")
      local kneeAttrs = { "KneeRotationAxisX", "KneeRotationAxisY", "KneeRotationAxisZ" }
      kneeVectorWidget = attributeEditor.addVectorAttributeWidget(panel, kneeAttrs, selection, set)

      panel:addHSpacer(6)
      local calculateHelpText = "Automatically calculate the Knee Axis based on the normal to the plane of the leg bones (select a suitable frame of animation in the animation browser)."
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
          -- For multiselect to work we have to iterate through selected objects
          for i, selectedObject in selection do
            if getType(selectedObject) == "LockFoot" then
              local useBallJoint = getAttribute(selectedObject, "UseBallJoint", set)
              local ankleJointName = getAttribute(selectedObject, "AnkleName", set)
              if useBallJoint ~= nil and useBallJoint then
                local ballJointName = getAttribute(selectedObject, "BallName", set)
                if ballJointName ~= nil then
                  local ballIndex = anim.getRigChannelIndex(ballJointName, set)
                  local ankleIndex = anim.getParentBoneIndex(ballIndex, set)
                  ankleJointName = anim.getRigChannelNames(set)[ankleIndex + 1]
                end
              end
              if ankleJointName ~= nil then
                local axis = calculateHingeAxis(set, ankleJointName)
                setAttribute(selectedObject .. ".KneeRotationAxisX", axis:getX(), set)
                setAttribute(selectedObject .. ".KneeRotationAxisY", axis:getY(), set)
                setAttribute(selectedObject .. ".KneeRotationAxisZ", axis:getZ(), set)
              end
              -- Deselect Flip Knee which is assumed off by Calculate
              setAttribute(selectedObject .. ".FlipKneeRotationDirection", false, set)
            end
          end
        end
      )

      attributeEditor.addAttributeLabel(panel, "Flip Knee", selection, "FlipKneeRotationDirection")
      local flipKneeRotationDirectionAttrPaths = { }
      for i, object in ipairs(selection) do
        table.insert(flipKneeRotationDirectionAttrPaths, string.format("%s.FlipKneeRotationDirection", object))
      end

      panel:addAttributeWidget{
        attributes = flipKneeRotationDirectionAttrPaths,
        flags = "expand",
        proportion = 1,
        set = set,
        onMouseEnter = function(self)
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "FlipKneeRotationDirection"))
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      attributeEditor.log("panel:endSizer")
    panel:endSizer()

    attributeEditor.log("panel:endSizer")
  panel:endSizer()

  -- these prevent callbacks firing off other callbacks causing infinite loops
  local enableContextEvents = true
  local enableUISetAttribute = true

  ----------------------------------------------------------------------------------------------------------------------
  -- this function enables/disables the BallRotationAxis vector UI based on the value of the UseBallJoint attribute.
  ----------------------------------------------------------------------------------------------------------------------
  local syncInterfaceFromAttributes = function()
    attributeEditor.logEnterFunc("syncInterfaceFromAttributes")

    local useBallJointValue = getCommonAttributeValue(selection, "UseBallJoint")

    if useBallJointValue == nil then
      --not all objects have the same value
    else
      -- all objects have the same value
      ballVectorLabel:enable(useBallJointValue)
      ballVectorWidget:enable(useBallJointValue and not hasReference)
      ballVectorCalculateButton:enable(useBallJointValue)
    end

    attributeEditor.logExitFunc("syncInterfaceFromAttributes")
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- this data change context ensures the ui reflects any changes that happen to selected
  -- lock foot nodes through script or undo redo are reflected in the custom ui.
  local changeContext = attributeEditor.createChangeContext()
  changeContext:setObjects(selection)
  changeContext:addAttributeChangeEvent("UseBallJoint")

  ------------------------------------------------------------------------------------------------------------------------
  -- this function is called whenever UseBallJoint is changed via script, or through undo and redo.
  -- calls syncBallRotationAxisUIWithUseBallJointValue to enable/disable the BallRotationAxis UI.
  ------------------------------------------------------------------------------------------------------------------------
  changeContext:setAttributeChangedHandler(
    function(object, attr)
      attributeEditor.logEnterFunc("changeContext attributeChangedHandler")

      if enableContextEvents then
        enableUISetAttribute = false
        syncInterfaceFromAttributes()
        enableUISetAttribute = true
      end

      attributeEditor.logExitFunc("changeContext attributeChangedHandler")
    end
  )

  -- set the initial state of the BallRotationAxisUI
  syncInterfaceFromAttributes()

  ------------------------------------------------------------------------------------------------------------------------

  attributeEditor.logExitFunc("perAnimSetLockFootJointAxisDisplayInfoSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Adds a lockFootJointAxisDisplayInfoSection.
-- Used by LockFoot.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.lockFootJointAxisDisplayInfoSection = function(panel, displayInfo, selection)
  local attrs = checkSetOrderAndBuildAttributeList(selection, displayInfo.usedAttributes)

  local rollup = panel:addRollup{
    label = displayInfo.title,
    flags = "mainSection",
    name = string.format("lockFootJointAxisDisplayInfoSection")
  }
  local rollPanel = rollup:getPanel()

  rollPanel:beginVSizer{ flags = "expand" }
    rollPanel:addAnimationSetWidget{
      attributes = attrs,
      displayFunc = perAnimSetLockFootJointAxisDisplayInfoSection,
      flags = "expand",
      proportion = 1,
    }
  rollPanel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- Add's a lockFootGroundPlaneDisplayInfoSection.
-- Used by LockFoot.
------------------------------------------------------------------------------------------------------------------------
local perAnimSetLockFootGroundPlaneDisplayInfoSection = function(panel, selection, attributes, set)
  attributeEditor.logEnterFunc("perAnimSetLockFootGroundPlaneDisplayInfoSection")

  local scene = nmx.Application.new():getSceneByName("AssetManager")
  local lowerHeightBoundsCalculateButton = nil

  -- check if the selection contains any referenced objects
  local hasReference = containsReference(selection)

  -- first add the ui for the section
  attributeEditor.log("panel:beginHSizer")
  panel:beginHSizer{ flags = "expand", proportion = 1 }
    panel:addHSpacer(6)
    panel:setBorder(1)

    attributeEditor.log("panel:beginFlexGridSizer")
    panel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      panel:setFlexGridColumnExpandable(2)

      attributeEditor.addAttributeLabel(panel, "Fix penetration", selection, "FixGroundPenetration")
      attributeEditor.addAttributeWidget(panel, "FixGroundPenetration", selection, set)

      local useBallJoint = getCommonAttributeValue(selection, "UseBallJoint", set)

      local ankleLowerHeightBoundText = "Ankle lower bound"
      local lowerHeightBoundText = "Ball lower bound"
      if not useBallJoint then
        ankleLowerHeightBoundText = "(Unused)"
        lowerHeightBoundText = "Ankle lower bound"
      end

      local ankleLowerHeightBoundLabel = attributeEditor.addAttributeLabel(panel, ankleLowerHeightBoundText, selection, "AnkleLowerHeightBound")
      local ankleLowerHeightBoundWidget = attributeEditor.addAttributeWidget(panel, "AnkleLowerHeightBound", selection, set)

      local lowerHeightBoundLabel = attributeEditor.addAttributeLabel(panel, lowerHeightBoundText, selection, "LowerHeightBound")
      local lowerHeightBoundWidget = attributeEditor.addAttributeWidget(panel, "LowerHeightBound", selection, set)

      local fixToeGroundPenetrationLabel = attributeEditor.addAttributeLabel(panel, "Fix toe penetration", selection, "FixToeGroundPenetration")
      local fixToeGroundPenetrationWidget = attributeEditor.addAttributeWidget(panel, "FixToeGroundPenetration", selection, set)

      local toeLowerHeightBoundLabel = attributeEditor.addAttributeLabel(panel, "Toe lower bound", selection, "ToeLowerHeightBound")
      local toeLowerHeightBoundWidget = attributeEditor.addAttributeWidget(panel, "ToeLowerHeightBound", selection, set)

      panel:addHSpacer(6)
      local calculateHelpText = "Automatically calculate the height bounds based on the current animation.  Select an animation frame in the animation browser where the relevant foot is flat on the ground."
      local lowerHeightBoundsCalculateButton = panel:addButton{
        label = "Calculate",
        onMouseEnter = function()
          attributeEditor.setHelpText(calculateHelpText)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }
      if containsReference(selection) then
        lowerHeightBoundsCalculateButton:enable(false)
      end

      lowerHeightBoundsCalculateButton:setOnClick(
        function(self)
          -- For multiselect to work we have to iterate through selected objects
          for i, selectedObject in selection do
            if getType(selectedObject) == "LockFoot" then

              -- Retrieve up vector
              local worldUpAxis = preferences.get("WorldUpAxis")
              local upVector = nmx.Vector3.new(0, 0, 0)
              if worldUpAxis == "Y Axis" then
                upVector:set(0.0, 1.0, 0.0)
              else
                upVector:set(0.0, 0.0, 1.0)
              end

              local useBallJoint = getAttribute(selectedObject, "UseBallJoint", set)
              local ankleJointName = getAttribute(selectedObject, "AnkleName", set)
              if useBallJoint ~= nil and useBallJoint then
                local ballJointName = getAttribute(selectedObject, "BallName", set)
                local ballIndex = anim.getRigChannelIndex(ballJointName, set)
                local ankleIndex = anim.getParentBoneIndex(ballIndex, set)
                ankleJointName = anim.getRigChannelNames(set)[ankleIndex + 1]
              end

              -- Set Ankle height bound
              if ankleJointName ~= nil then
                local height = calculateJointHeight(set, ankleJointName, upVector)
                setAttribute(selectedObject .. ".AnkleLowerHeightBound", height, set)
                if not useBallJoint then
                  setAttribute(selectedObject .. ".LowerHeightBound", height, set)
                end
              end

              -- Set Ball height bound
              if useBallJoint ~=  nil and useBallJoint then
                local ballJointName = getAttribute(selectedObject, "BallName", set)
                local height = calculateJointHeight(set, ballJointName, upVector)
                setAttribute(selectedObject .. ".LowerHeightBound", height, set)
              end

              -- Set Toe height bound
              if getAttribute(selectedObject, "FixToeGroundPenetration", set) then
                local toeJointName = getAttribute(selectedObject, "ToeName", set)
                local height = calculateJointHeight(set, toeJointName, upVector)
                setAttribute(selectedObject .. ".ToeLowerHeightBound", height, set)
              end
            end
          end
        end

      ) -- End of setOnClick() for the calculate button

      attributeEditor.log("panel:endSizer")
    panel:endSizer()

    attributeEditor.log("panel:endSizer")
  panel:endSizer()

  -- these prevent callbacks firing off other callbacks causing infinite loops
  local enableContextEvents = true
  local enableUISetAttribute = true

  ----------------------------------------------------------------------------------------------------------------------
  -- enable LowerHeightBound based on the value of FixGroundPenetration
  -- enable ToeLowerHeightBound based on the value of FixToeGroundPenetration
  ----------------------------------------------------------------------------------------------------------------------
  local syncInterfaceFromAttributes = function()
    attributeEditor.logEnterFunc("syncInterfaceFromAttributes")

    local useBallJoint = getCommonAttributeValue(selection, "UseBallJoint", set) or false

    local ankleLowerHeightBoundText = "(Unused)"
    local lowerHeightBoundText = "Ankle lower bound"
    if useBallJoint then
      ankleLowerHeightBoundText = "Ankle lower bound"
      lowerHeightBoundText = "Ball lower bound"
    end

    local fixGroundPenetration = getCommonAttributeValue(selection, "FixGroundPenetration", set) or false

    ankleLowerHeightBoundLabel:enable(useBallJoint and fixGroundPenetration)
    ankleLowerHeightBoundLabel:setLabel(ankleLowerHeightBoundText)
    ankleLowerHeightBoundWidget:enable(useBallJoint and fixGroundPenetration and not hasReference)

    lowerHeightBoundLabel:enable(fixGroundPenetration)
    lowerHeightBoundLabel:setLabel(lowerHeightBoundText)
    lowerHeightBoundWidget:enable(fixGroundPenetration and not hasReference)

    fixToeGroundPenetrationLabel:enable(fixGroundPenetration)
    fixToeGroundPenetrationWidget:enable(fixGroundPenetration and not hasReference)

    lowerHeightBoundsCalculateButton:enable(fixGroundPenetration and not hasReference)

    local fixToeGroundPenetration = getCommonAttributeValue(selection, "FixToeGroundPenetration", set) or false

    toeLowerHeightBoundLabel:enable(fixGroundPenetration and fixToeGroundPenetration)
    toeLowerHeightBoundWidget:enable(fixGroundPenetration and fixToeGroundPenetration and not hasReference)

    panel:rebuild()

    attributeEditor.logExitFunc("syncInterfaceFromAttributes")
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- this data change context ensures the ui reflects any changes that happen through script or undo redo.
  local changeContext = attributeEditor.createChangeContext()
  changeContext:setObjects(selection)
  changeContext:addAttributeChangeEvent("UseBallJoint")
  changeContext:addAttributeChangeEvent("FixGroundPenetration")
  changeContext:addAttributeChangeEvent("FixToeGroundPenetration")

  changeContext:setAttributeChangedHandler(
    function(object, attr)
      attributeEditor.logEnterFunc("changeContext attributeChangedHandler")

      if enableContextEvents then
        enableUISetAttribute = false
        syncInterfaceFromAttributes()
        enableUISetAttribute = true
      end

      attributeEditor.logExitFunc("changeContext attributeChangedHandler")
    end
  )

  -- set the initial state of the ui
  syncInterfaceFromAttributes()

  ------------------------------------------------------------------------------------------------------------------------

  attributeEditor.logExitFunc("perAnimSetLockFootGroundPlaneDisplayInfoSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Adds a lockFootJointAxisDisplayInfoSection.
-- Used by LockFoot.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.lockFootGroundPlaneDisplayInfoSection = function(panel, displayInfo, selection)
  local attrs = checkSetOrderAndBuildAttributeList(selection, displayInfo.usedAttributes)

  local rollup = panel:addRollup{
    label = displayInfo.title,
    flags = "mainSection",
    name = string.format("lockFootGroundPlaneDisplayInfoSection")
  }
  local rollPanel = rollup:getPanel()

  rollPanel:beginVSizer{ flags = "expand" }
    rollPanel:addAnimationSetWidget{
      attributes = attrs,
      displayFunc = perAnimSetLockFootGroundPlaneDisplayInfoSection,
      flags = "expand",
      proportion = 1,
    }
  rollPanel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- Adds a lockFootOrientationDisplayInfoSection.
-- Used by LockFoot.
------------------------------------------------------------------------------------------------------------------------

local perAnimSetLockFootOrientationDisplayInfoSection = function(panel, selection, attributes, set)
  attributeEditor.logEnterFunc("perAnimSetLockFootOrientationDisplayInfoSection")

  local scene = nmx.Application.new():getSceneByName("AssetManager")
  local footLevelVectorLabel = nil
  local footLevelVectorWidget = { }
  local footLevelVectorCalculateButton = nil
  local footPivotResistanceLabel = nil
  local footPivotResistanceWidget = { }

  -- check if the selection contains any referenced objects
  local hasReference = containsReference(selection)

  -- first add the ui for the section
  attributeEditor.log("panel:beginHSizer")
  panel:beginHSizer{ flags = "expand", proportion = 1 }
    panel:addHSpacer(6)
    panel:setBorder(1)

    attributeEditor.log("panel:beginFlexGridSizer")
    panel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      panel:setFlexGridColumnExpandable(2)

      local fixFootOrientationAttrPaths = { }
      for i, object in ipairs(selection) do
        table.insert(fixFootOrientationAttrPaths, string.format("%s.FixFootOrientation", object))
      end

      local fixFootOrientationHelp = getAttributeHelpText(selection[1], "FixFootOrientation")

      panel:addStaticText{
        text = "Fix Foot Orientation",
        onMouseEnter = function()
          attributeEditor.setHelpText(fixFootOrientationHelp)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      panel:addAttributeWidget{
        attributes = fixFootOrientationAttrPaths,
        flags = "expand",
        proportion = 1,
        set = set,
        onMouseEnter = function(self)
          attributeEditor.setHelpText(fixFootOrientationHelp)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      footLevelVectorLabel = panel:addStaticText{
        text = "Foot Level Vector",
        onMouseEnter = function()
          attributeEditor.setHelpText(getAttributeHelpText(selection[1], "FootLevelVectorX"))
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      local footLevelVectorAttrs = { "FootLevelVectorX", "FootLevelVectorY", "FootLevelVectorZ" }
      footLevelVectorWidget = attributeEditor.addVectorAttributeWidget(panel, footLevelVectorAttrs, selection, set)

      panel:addHSpacer(6)
      local calculateHelpText = "Automatically calculate the foot level vector using the current animation.  Select an animation frame in the animation browser where the relevant foot is flat on the ground.  Requires a ball joint to be specified, or for the feet to be pointing in the +Z direction."
      footLevelVectorCalculateButton = panel:addButton {
        label = "Calculate",
        onMouseEnter = function()
          attributeEditor.setHelpText(calculateHelpText)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }
      if containsReference(selection) then
        footLevelVectorCalculateButton:enable(false)
      end

      footLevelVectorCalculateButton:setOnClick(

        function(self)

          -- For multiselect to work we have to iterate through selected objects
          for i, selectedObject in selection do
            if getType(selectedObject) == "LockFoot" then

              -- Retrieve up vector
              local worldUpAxis = preferences.get("WorldUpAxis")
              local upVector = nmx.Vector3.new(0, 0, 0)
              if worldUpAxis == "Y Axis" then
                upVector:set(0.0, 1.0, 0.0)
              else
                upVector:set(0.0, 0.0, 1.0)
              end
              
              local useBallJoint = getAttribute(selectedObject, "UseBallJoint", set)
              local ankleJointName = getAttribute(selectedObject, "AnkleName", set)
              local ballJointName = nil
              if useBallJoint ~= nil and useBallJoint then
                ballJointName = getAttribute(selectedObject, "BallName", set)
                local ballIndex = anim.getRigChannelIndex(ballJointName, set)
                local ankleIndex = anim.getParentBoneIndex(ballIndex, set)
                ankleJointName = anim.getRigChannelNames(set)[ankleIndex + 1]
              end

              if ankleJointName ~= nil then
                local levelVector = calculateLevelVector(set, ankleJointName, ballJointName, upVector)
                setAttribute(selectedObject .. ".FootLevelVectorX", levelVector:getX(), set)
                setAttribute(selectedObject .. ".FootLevelVectorY", levelVector:getY(), set)
                setAttribute(selectedObject .. ".FootLevelVectorZ", levelVector:getZ(), set)
              end
            end
          end
        end

      ) -- End of setOnClick() for the calculate button

      local footPivotResistanceAttrPaths = { }
      for i, object in ipairs(selection) do
        table.insert(footPivotResistanceAttrPaths, string.format("%s.FootPivotResistance", object))
      end

      local footPivotResistanceHelp = getAttributeHelpText(selection[1], "FootPivotResistance")

      footPivotResistanceLabel = panel:addStaticText{
        text = "Foot Pivot Friction",
        onMouseEnter = function()
          attributeEditor.setHelpText(footPivotResistanceHelp)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      footPivotResistanceWidget = panel:addAttributeWidget{
        attributes = footPivotResistanceAttrPaths,
        flags = "expand",
        proportion = 1,
        set = set,
        onMouseEnter = function(self)
          attributeEditor.setHelpText(footPivotResistanceHelp)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      attributeEditor.log("panel:endSizer")
    panel:endSizer()

    attributeEditor.log("panel:endSizer")
  panel:endSizer()

  -- these prevent callbacks firing off other callbacks causing infinite loops
  local enableContextEvents = true
  local enableUISetAttribute = true

  ----------------------------------------------------------------------------------------------------------------------
  -- this function enables/disables the FootLevelVector vector UI based on the value of the FixFootOrientation attribute.
  ----------------------------------------------------------------------------------------------------------------------
  local syncInterfaceFromAttributes = function()
    attributeEditor.logEnterFunc("syncInterfaceFromAttributes")

    local fixFootOrientationJointValue = getCommonAttributeValue(selection, "FixFootOrientation", set)

    if fixFootOrientationJointValue == nil then
      --not all objects have the same value
    else
      -- all objects have the same value
      footLevelVectorLabel:enable(fixFootOrientationJointValue)
      footLevelVectorWidget:enable(fixFootOrientationJointValue and not hasReference)
      footLevelVectorCalculateButton:enable(fixFootOrientationJointValue)
      footPivotResistanceLabel:enable(fixFootOrientationJointValue)
      footPivotResistanceWidget:enable(fixFootOrientationJointValue)
    end

    attributeEditor.logExitFunc("syncInterfaceFromAttributes")
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- this data change context ensures the ui reflects any changes that happen to selected
  -- lock foot nodes through script or undo redo are reflected in the custom ui.
  local changeContext = attributeEditor.createChangeContext()
  changeContext:setObjects(selection)
  changeContext:addAttributeChangeEvent("FixFootOrientation")

  ------------------------------------------------------------------------------------------------------------------------
  -- this function is called whenever FixFootOrientation is changed via script, or through undo and redo.
  ------------------------------------------------------------------------------------------------------------------------
  changeContext:setAttributeChangedHandler(
    function(object, attr)
      attributeEditor.logEnterFunc("changeContext attributeChangedHandler")

      if enableContextEvents then
        enableUISetAttribute = false
        syncInterfaceFromAttributes()
        enableUISetAttribute = true
      end

      attributeEditor.logExitFunc("changeContext attributeChangedHandler")
    end
  )

  -- set the initial state of the FootLevelVector UI
  syncInterfaceFromAttributes()

  ------------------------------------------------------------------------------------------------------------------------

  attributeEditor.logExitFunc("attributeEditor.lockFootOrientationDisplayInfoSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Adds a lockFootOrientationDisplayInfoSection.
-- Used by LockFoot.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.lockFootOrientationDisplayInfoSection = function(panel, displayInfo, selection)
  local attrs = checkSetOrderAndBuildAttributeList(selection, displayInfo.usedAttributes)

  local rollup = panel:addRollup{
    label = displayInfo.title,
    flags = "mainSection",
    name = string.format("lockFootOrientationDisplayInfoSection")
  }
  local rollPanel = rollup:getPanel()

  rollPanel:beginVSizer{ flags = "expand" }
    rollPanel:addAnimationSetWidget{
      attributes = attrs,
      displayFunc = perAnimSetLockFootOrientationDisplayInfoSection,
      flags = "expand",
      proportion = 1,
    }
  rollPanel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- Adds the LockFoot general section to the attribute editor
------------------------------------------------------------------------------------------------------------------------
attributeEditor.footlockGeneralDisplayInfoSection = function(panel, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.footlockGeneralDisplayInfoSection")

  -- add the ui for the section
  attributeEditor.log("panel:addRollup")
  local rollup = panel:addRollup{
    label = displayInfo.title,
    flags = "mainSection",
    name = "footlockGeneralDisplayInfoSection" }
  local rollPanel = rollup:getPanel()

  -- add widgets
  attributeEditor.log("rollPanel:beginHSizer")
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    rollPanel:setBorder(1)

    attributeEditor.log("rollPanel:beginFlexGridSizer")
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      rollPanel:setFlexGridColumnExpandable(2)

      -- Lock Vertical Motion
      attributeEditor.addAttributeLabel(rollPanel, "Lock Vertical Motion", selection, "LockVerticalMotion")
      attributeEditor.addAttributeWidget(rollPanel, "LockVerticalMotion", selection)

      -- Track Character Controller
      attributeEditor.addAttributeLabel(rollPanel, "Track Character Controller", selection, "TrackCharacterController")
      attributeEditor.addAttributeWidget(rollPanel, "TrackCharacterController", selection)

      attributeEditor.log("rollPanel:endSizer")
    rollPanel:endSizer()

    attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.footlockGeneralDisplayInfoSection")
end
