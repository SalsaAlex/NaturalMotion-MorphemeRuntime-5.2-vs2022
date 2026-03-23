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
attributeEditor.activeStateDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.ActiveStateDisplayInfoSection")

  local kCheckBoxBlank = 0
  local kCheckBoxTicked = 1
  local kCheckBoxIndeterminate = 2
  local kRadioBlank = 3
  local kRadioTicked = 4
  local kRadioIndeterminate = 5

  local stateMachineTree = nil
  local stateLookupTable = nil

  local enableContextEvents = true

  ----------------------------------------------------------------------------------------------------------------------
  local compareNoCase = function(a, b)
    return string.lower(b) > string.lower(a)
  end

  ----------------------------------------------------------------------------------------------------------------------
  local rebuildStateMachineTree = function()

    -- delete all the elements in the tree
    local rootItem = stateMachineTree:getRoot()
    rootItem:clearChildren();

    -- add the States item
    local allStates = rootItem
    allStates:setColumnCheckbox(1, kCheckBoxBlank)
    allStates:setColumnValue(2, "States")

    -- add the Nodes item
    local allNodes = rootItem:addChild()
    allNodes:setColumnCheckbox(1, kCheckBoxBlank)
    allNodes:setColumnValue(2, "Nodes")

    -- add the Transitions item
    local allTransits = rootItem:addChild()
    allTransits:setColumnCheckbox(1, kCheckBoxBlank)
    allTransits:setColumnValue(2, "Transitions")

    -- add all the nodes and transitions in the current state machine
    local stateMachine = getParent(selection[1])
    if stateMachine ~= nil then
      local children = listChildren(stateMachine)
      table.sort(children, compareNoCase)
      for i, v in ipairs(children) do
        local manifestType, objectType = getType(v)
        local _, shortName = splitNodePath(v)
        if objectType == "StateMachineNode" then
          if manifestType == "StateMachine" or manifestType == "PhysicsStateMachine"
          or manifestType == "BlendTree" or manifestType == "PhysicsBlendTree" then
            local childNode = allNodes:addChild()
            childNode:setColumnCheckbox(1, kCheckBoxBlank)
            childNode:setColumnValue(2, shortName)
            childNode:setUserDataString(v)
          end
        elseif objectType == "Transition" then
          local childTransit = allTransits:addChild()
          childTransit:setColumnCheckbox(1, kCheckBoxBlank)
          childTransit:setColumnValue(2, shortName)
          childTransit:setUserDataString(v)
        end
      end
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- set the visibility of the state machine tree based on the value of the AllStates attribute
  local shouldShowStateMachineTree = function()
    attributeEditor.logEnterFunc("shouldShowStateMachineTree")

    local shouldShow = false
    local allStates = getCommonAttributeValue(selection, "AllStates")
    if allStates ~= nil and not allStates then
      -- Only show the state machine if all the currently selected objects have specific substates
      shouldShow = true
    end

    attributeEditor.logExitFunc("shouldShowStateMachineTree")
    return shouldShow
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- set the visibility of the state machine tree based on the value of the AllStates attribute
  local syncStateMachineTreeVisibility = function()
    attributeEditor.logEnterFunc("syncStateMachineTreeVisibility")

    local changed = stateMachineTree:setShown(shouldShowStateMachineTree())
    if changed then
      attributeEditor.editorWindow:rebuild()
    end

    attributeEditor.logExitFunc("syncStateMachineTreeVisibility")
  end

  ----------------------------------------------------------------------------------------------------------------------
  local rebuildStateLookupTable = function()

    stateLookupTable = { }
    for _, object in ipairs(selection) do
      local objectSubStates = { }
      local attrSubStates = getAttribute(object, "States")
      for i, subState in ipairs(attrSubStates) do
        objectSubStates[subState] = true
      end
      stateLookupTable[object] = objectSubStates
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  local getCommonCheckBoxValue = function(subState)

    local checkBoxValue = nil

    for _, subStates in pairs(stateLookupTable) do
      if subStates[subState] then
        if checkBoxValue == nil then
          checkBoxValue = kCheckBoxTicked
        elseif checkBoxValue ~= kCheckBoxTicked then
          checkBoxValue = kCheckBoxIndeterminate
          break
        end
      else
        if checkBoxValue == nil then
          checkBoxValue = kCheckBoxBlank
        elseif checkBoxValue ~= kCheckBoxBlank then
          checkBoxValue = kCheckBoxIndeterminate
          break
        end
      end
    end

    return checkBoxValue
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- set the state machine check boxes based on the value of the SubStates attribute
  local syncStateMachineTreeCheckBoxes = nil
  syncStateMachineTreeCheckBoxes = function(treeItem)

    local checkBoxStatus = nil

    treeItem = treeItem or stateMachineTree:getRoot()
    local numChildren = treeItem:getNumChildren()
    if numChildren > 0 then
      for i = 1, numChildren do
        local childItem = treeItem:getChild(i)
        syncStateMachineTreeCheckBoxes(childItem)
        local childCheckBoxStatus = childItem:getColumnCheckbox(1)
        if not checkBoxStatus then
          checkBoxStatus = childCheckBoxStatus
        elseif checkBoxStatus ~= childCheckBoxStatus then
          checkBoxStatus = kCheckBoxIndeterminate
        end
      end
    else
      local subState = treeItem:getUserDataString()
      checkBoxStatus = getCommonCheckBoxValue(subState)
    end

    if checkBoxStatus then
      treeItem:setColumnCheckbox(1, checkBoxStatus)
    else
      treeItem:setColumnCheckbox(1, kCheckBoxBlank)
    end

  end

  ----------------------------------------------------------------------------------------------------------------------
  local addSubStates = nil
  addSubStates = function(lookupTable, subStates, treeItem)

    local numChildren = treeItem:getNumChildren()
    if numChildren > 0 then
      for i = 1, numChildren do
        local childItem = treeItem:getChild(i)
        addSubStates(lookupTable, subStates, childItem)
      end
    else
      local subState = treeItem:getUserDataString()
      if string.len(subState) > 0 then
        if not lookupTable[subState] then
          table.insert(subStates, subState)
        end
      end
    end

  end

  ----------------------------------------------------------------------------------------------------------------------
  local removeSubStates = nil
  removeSubStates = function(lookupTable, subStates, treeItem)

    local numChildren = treeItem:getNumChildren()
    if numChildren > 0 then
      for i = 1, numChildren do
        local childItem = treeItem:getChild(i)
        removeSubStates(lookupTable, subStates, childItem)
      end
    else
      local subState = treeItem:getUserDataString()
      if string.len(subState) > 0 then
        if lookupTable[subState] then
          -- non-optimal linear search
          for i, v in ipairs(subStates) do
            if v == subState then
              table.remove(subStates, i)
              break
            end
          end
        end
      end
    end

  end

  ----------------------------------------------------------------------------------------------------------------------
  -- An item has been chosen in the tree - update the underlying attributes
  local onCheckboxChanged = function(treeCtrl, treeItem)
    attributeEditor.logEnterFunc("onCheckboxChanged")

    enableContextEvents = false

    local checkBoxStatus = treeItem:getColumnCheckbox(1)
    if checkBoxStatus == kCheckBoxTicked then
      for _, object in ipairs(selection) do
        local subStates = getAttribute(object, "States")
        addSubStates(stateLookupTable[object], subStates, treeItem)
        setAttribute(object .. ".States", subStates)
      end
    elseif checkBoxStatus == kCheckBoxBlank then
      for _, object in ipairs(selection) do
        local subStates = getAttribute(object, "States")
        removeSubStates(stateLookupTable[object], subStates, treeItem)
        setAttribute(object .. ".States", subStates)
      end
    end

    rebuildStateLookupTable()
    syncStateMachineTreeCheckBoxes()
    enableContextEvents = true

    attributeEditor.logExitFunc("onCheckboxChanged")
  end

  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "stateSetDisplayInfo" }
  local rollPanel = rollup:getPanel()
  rollup:expand(false)

  -- first add the ui for the section
  attributeEditor.log("rollPanel:beginVSizer")
  rollPanel:beginVSizer{ flags = "expand", proportion = 1 }

    attributeEditor.log("rollPanel:beginHSizer")
    rollPanel:beginHSizer{ cols = 2, flags = "expand", proportion = 0 }

      local helpText = getAttributeHelpText(selection[1], "AllStates")

      local attrPaths = { }
      for i, object in ipairs(selection) do
        table.insert(attrPaths, string.format("%s.AllStates", object))
      end

      rollPanel:addHSpacer(5)

      rollPanel:addStaticText{
        text = "Any State",
        onMouseEnter = function()
          attributeEditor.setHelpText(helpText)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      rollPanel:addHSpacer(5)

      rollPanel:addAttributeWidget{
        attributes = attrPaths,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function(self)
          attributeEditor.setHelpText(helpText)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

    rollPanel:endSizer()
    attributeEditor.log("rollPanel:endHSizer")

    attributeEditor.log("rollPanel:beginHSizer")
    rollPanel:beginHSizer{ cols = 1, flags = "expand", proportion = 0 }

      rollPanel:addHSpacer(5)

      stateMachineTree = rollPanel:addTreeControl{
        name = "StateMachineTree",
        flags = "expand",
        size = { height = 200 },
        proportion = 1,
        numColumns = 2,
        treeControlColumn = 2,
        onCheckboxChanged = onCheckboxChanged
      }

      stateMachineTree:setColumnWidth(1, 20)
      -- Set the visibility of the SM tree here. 
      -- This avoids a breaking rebuild in the call to syncStateMachineTreeVisibility below
      stateMachineTree:setShown(shouldShowStateMachineTree())

    rollPanel:endSizer()
    attributeEditor.log("rollPanel:endHSizer")

  rollPanel:endSizer()
  attributeEditor.log("rollPanel:endVSizer")
  
  ----------------------------------------------------------------------------------------------------------------------
  local changeContext = attributeEditor.createChangeContext()

  changeContext:setObjects(selection)
  changeContext:addAttributeChangeEvent("AllStates")
  changeContext:addAttributeChangeEvent("States")
  changeContext:setAttributeChangedHandler(
    function(object, attr)
      attributeEditor.logEnterFunc("changeContext attributeChangedHandler")
      if enableContextEvents then
        if attr == "AllStates" then
          syncStateMachineTreeVisibility()
        elseif attr == "States" then
          rebuildStateLookupTable()
          syncStateMachineTreeCheckBoxes()
        end
      end
      attributeEditor.logExitFunc("changeContext attributeChangedHandler")
    end
  )

  rebuildStateMachineTree()
  rebuildStateLookupTable()
  syncStateMachineTreeVisibility()
  syncStateMachineTreeCheckBoxes()

  ----------------------------------------------------------------------------------------------------------------------

  attributeEditor.logExitFunc("attributeEditor.ActiveStateDisplayInfoSection")
end
