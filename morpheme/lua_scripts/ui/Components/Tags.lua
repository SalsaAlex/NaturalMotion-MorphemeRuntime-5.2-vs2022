------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

local kDeleteCrossImage = app.loadImage("deleteCross.png")
local kAddImage = app.loadImage("additem.png")
local kTrajectoryHelpText = [[An animation's trajectory joint describes a character's fundamental movement in the world, defining the overall rotation and translation that the animation will apply to the character. Typically, you will want this bone to follow the character around directly below the character root.]]
local kRootHelpText = [[The character root joint is used to define the root of a character skeleton, which determines a character's orientation in the world.]]
local kTrajectoryHelperHelpText = [[These are parts that are used to help the runtime calculate a position/orientation for the trajectory joint when that joint is not controlled by animation. Typically set this on the feet.]]
local kNotSetText = "--- Not Set ---"

-- This table is indexed by the filename of the rig. It stores if a new trajectory helper has been added to the rig. This is used to determine
-- if the "new" unset item is added to the list of rollups.
local newCalculationHelper = { }

------------------------------------------------------------------------------------------------------------------------
local animSetHasPhysicsUI = function(selectedSet)
  local selectedTypeName = anim.getAnimSetCharacterType(selectedSet)
  if (selectedTypeName == "Physics" or selectedTypeName == "Euphoria") then
    if anim.isPhysicsRigValid() and anim.getPhysicsRigSize(selectedSet) then
      return true
    end
  end

  return false
end

------------------------------------------------------------------------------------------------------------------------
local getPhysicsRigChannelNames = function(selectedSet)
  local result = anim.getPhysicsRigChannelNames(selectedSet)
  result[1] = kNotSetText
  return result
end

------------------------------------------------------------------------------------------------------------------------
local validateFunction = function(selectedSet, isCurrentComponent)
  if not anim.isRigValid() then
    return "Inactive"
  end

  -- Validate the hip and trajectory
  local markupData = anim.getRigMarkupData(selectedSet)
  if markupData.hipIndex == 0 or markupData.trajectoryIndex == 0 then
    return "Error"
  end
  
  -- Validate the trajectory helpers
  local hasPhysicsUI = animSetHasPhysicsUI(selectedSet)
  if hasPhysicsUI then
    trajectoryHelpers = anim.getTrajectoryCalculationHelpers(selectedSet)
    if trajectoryHelpers then
      for _, joint in ipairs(trajectoryHelpers) do
        if joint == 0 then
          return "Error"
        end
      end
    end
  end
  
  return "Ok"
end

------------------------------------------------------------------------------------------------------------------------
-- return the index of an item in a list
local indexOf = function(itemList, item)
  for i, anItem in itemList do
    if anItem == item then 
      return i
    end
  end
  
  return 0
end

------------------------------------------------------------------------------------------------------------------------
-- This returns a all the elements in 'srceList' with all the elements that exist in 'orderList' first and in the
-- same order as 'orderList'
local copyElementsInOrder = function(srceList, orderList)
  local result = { }
  
  for _, item in ipairs(orderList) do
    local pos = indexOf(srceList, item)
    if pos > 0 then
      table.insert(result, item)
      table.remove(srceList, pos)
    end
  end
  
  for _, item in ipairs(srceList) do
    table.insert(result, item)
  end
  
  return result
end

------------------------------------------------------------------------------------------------------------------------
local panelFunction = function(selectedSet, panel)
  
  local hipsCombo = nil
  local trajectoryCombo = nil
  local trajectoryHelperRollup = nil
  local trajectoryHelpers = { }
  local addTrajectoryHelper = { }
  
  local updateUI
  local updateAnimationUI
  local updateTrajectoryHelpersUI
  
  ----------------------------------------------------------------------------------------------------------------------
  updateAnimationUI = function()
    local markupData = anim.getRigMarkupData(selectedSet)
    local channelNames = anim.getRigChannelNames(selectedSet)
    channelNames[1] = kNotSetText

    if hipsCombo then
      hipsCombo:setItems(channelNames)
      hipsCombo:setSelectedIndex(markupData.hipIndex + 1)
      hipsCombo:setError(markupData.hipIndex == 0)
      componentEditor.bindHelpToWidget(hipsCombo, kRootHelpText)
    end
    
    if trajectoryCombo then
      trajectoryCombo:setItems(channelNames)
      trajectoryCombo:setSelectedIndex(markupData.trajectoryIndex + 1)
      trajectoryCombo:setError(markupData.trajectoryIndex == 0)
      componentEditor.bindHelpToWidget(trajectoryCombo, kTrajectoryHelpText)
    end
  end
  
  ----------------------------------------------------------------------------------------------------------------------
  local addTrajectoryHelper = function(self)
    newCalculationHelper[anim.getPhysicsRigPath(selectedSet)] = true
    updateUI()
  end
  
  ----------------------------------------------------------------------------------------------------------------------
  local mkDeleteTrajectoryHelper = function(arrayIndex, jointIndex)
  
    return function(self)
      if jointIndex == 0 then
        newCalculationHelper[anim.getPhysicsRigPath(selectedSet)] = nil
        updateUI()
      else
        table.remove(trajectoryHelpers, arrayIndex)
        anim.setTrajectoryCalculationHelpers(trajectoryHelpers, selectedSet)
      end
    end
  end
  
  ----------------------------------------------------------------------------------------------------------------------
  local mkChangeTrajectoryHelper = function(arrayIndex, jointIndex)
  
    return function(self)
      if jointIndex == 0 then
        newCalculationHelper[anim.getPhysicsRigPath(selectedSet)] = nil
      end
      
      local channelNames = anim.getRigChannelNames(selectedSet)
      local chosenJointName = self:getSelectedItem()
      local chosenJoint = indexOf(channelNames, chosenJointName) - 1

      for i, joint in ipairs(trajectoryHelpers) do
        if i ~= arrayIndex and chosenJoint == joint then
          local message = string.format("\"%s\" is already a trajectory helper.", chosenJointName)
          ui.showMessageBox(message, "ok")
          return;
        end
      end

      trajectoryHelpers[arrayIndex] = chosenJoint
      anim.setTrajectoryCalculationHelpers(trajectoryHelpers, selectedSet)
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  updateTrajectoryHelpersUI = function()
    local hasPhysicsUI = animSetHasPhysicsUI(selectedSet)

    trajectoryHelperRollup:setShown(hasPhysicsUI)

    local rollPanel = trajectoryHelperRollup:getPanel()
    rollPanel:clear()

    if hasPhysicsUI then
      local physicsChannelNames = anim.getPhysicsRigChannelNames(selectedSet)
      table.insert(physicsChannelNames, 1, kNotSetText)
      
      local channelNames = anim.getRigChannelNames(selectedSet)

      trajectoryHelpers = copyElementsInOrder(anim.getTrajectoryCalculationHelpers(selectedSet), trajectoryHelpers)
      if newCalculationHelper[anim.getPhysicsRigPath(selectedSet)] then
        table.insert(trajectoryHelpers, 1, 0)
      end
     
      rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand" }
        rollPanel:setFlexGridColumnExpandable(1)
        
        trajectoryHelperWidgets = { }
        local hasCharacterWorldSpaceJoint = false;
        for i, joint in ipairs(trajectoryHelpers) do
          hasCharacterWorldSpaceJoint = hasCharacterWorldSpaceJoint or joint == 0
          local jointName = channelNames[joint + 1]
          local trajectoryCombo = rollPanel:addComboBox{flags = "expand", proportion = 0, items = physicsChannelNames}
          componentEditor.bindHelpToWidget(trajectoryCombo, kTrajectoryHelperHelpText)
          local deleteButton = rollPanel:addButton{image = kDeleteCrossImage, size = { width = 16 ,  height = 16 }, onClick = mkDeleteTrajectoryHelper(i, joint) }
          componentEditor.bindHelpToWidget(deleteButton, "Delete this trajectory helper.")

          trajectoryCombo:setSelectedItem(jointName)
          trajectoryCombo:setOnChanged(mkChangeTrajectoryHelper(i, joint))
          trajectoryCombo:setError(joint == 0)
        end
        
        rollPanel:addHSpacer(1)
        local addTrajectoryHelper = rollPanel:addButton{image = kAddImage, size = { width = 16 ,  height = 16 }, onClick = addTrajectoryHelper }
        addTrajectoryHelper:enable(not hasCharacterWorldSpaceJoint)
        componentEditor.bindHelpToWidget(addTrajectoryHelper, "Add a trajectory helper.\n\n" .. kTrajectoryHelperHelpText)
      rollPanel:endSizer()
    end
  end
  
  ----------------------------------------------------------------------------------------------------------------------
  updateUI = function()
    componentEditor.panel:freeze()
    
    updateAnimationUI()
    updateTrajectoryHelpersUI()
    
    -- force the componentEditor.panel to rebuild
    componentEditor.panel:rebuild()
  end

  ----------------------------------------------------------------------------------------------------------------------
  local animationTagsSection = function()

    -- Animation Tags
    local rollup = panel:addRollup{label = "Animation Tags", flags = "mainSection;expand", name = "AnimationTags" }
    local rollPanel = rollup:getPanel()

    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand" }
    rollPanel:setFlexGridColumnExpandable(2)

      rollPanel:addStaticText{ text = "Character Root"}
      hipsCombo = rollPanel:addComboBox{name = "characterRoot", flags = "expand", proportion = 0}

      rollPanel:addStaticText{ text = "Trajectory"}
      trajectoryCombo = rollPanel:addComboBox{name = "trajectory", flags = "expand", proportion = 0}
      
    rollPanel:endSizer()

    hipsCombo:setOnChanged(
      function(self)
        local markup = {hipIndex = self:getSelectedIndex() - 1}
        anim.setRigMarkupData(markup, selectedSet)
      end)
      
    trajectoryCombo:setOnChanged(
      function(self)
        local markup = {trajectoryIndex = self:getSelectedIndex() - 1}
        anim.setRigMarkupData(markup, selectedSet)
      end)
  end

  ----------------------------------------------------------------------------------------------------------------------
  local trajectoryHelpersSection = function()
    trajectoryHelpers = anim.getTrajectoryCalculationHelpers(selectedSet)
    trajectoryHelperRollup = panel:addRollup{label = "Trajectory Helpers", flags = "mainSection;expand", name = "TrajectoryHelpers" }
    
    -- Note: the updateTrajectoryHelpersUI populates the trajectoryHelperRollup
  end

  ----------------------------------------------------------------------------------------------------------------------
  animationTagsSection()
  trajectoryHelpersSection()

  componentEditor.addOnRigChanged(updateUI)
  componentEditor.addOnPhysicsRigChanged(updateUI)
  
  updateAnimationUI()
  updateTrajectoryHelpersUI()
end

components.register("Tags",  {"Animation", "Physics", "Euphoria"}, validateFunction, panelFunction, nil,
  {
    CanUndo = function() return true end,
    CanRedo = function() return true end,
    Undo = function() nmx.Application.new():getSceneByName("AssetManager"):undo() end,
    Redo = function() nmx.Application.new():getSceneByName("AssetManager"):redo() end,
  })
