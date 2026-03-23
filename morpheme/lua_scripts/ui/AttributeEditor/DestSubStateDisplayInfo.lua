------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"

local kDefaultStateGarnish = " " .. string.char(8226) -- bullet is used as a suffix to denote the default state

----------------------------------------------------------------------------------------------------------------------
local stableCaseInsensitiveCompare = function(a, b)
  local a_upper = string.lower(a)
  local b_upper = string.lower(b)

  if a_upper == b_upper then
    return b > a
  end

  return b_upper > a_upper
end

------------------------------------------------------------------------------------------------------------------------
attributeEditor.destSubStateDisplayInfo = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.destSubStateDisplayInfo")

  local stateMachinePanel
  local stateMachineTree -- the tree control that shows all the substates
  local subStateCheckBox
  local subStateLabel

  -- these prevent callbacks firing off other callbacks causing infinite loops
  local enableContextEvents = true
  local enableUISetAttribute = true

  ----------------------------------------------------------------------------------------------------------------------
  -- Returns the state machine at the end of a transition
  local kRebuildEditor = false
  local kDontRebuildEditor = true

  local showStateMachineTree = function(show, dontRebuildEditor)
    local changed = stateMachineTree:setShown(show)
    if changed and (not dontRebuildEditor) then
      local editor = attributeEditor.editorWindow
      if editor then
        editor:rebuild()
      end
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- Returns the state machine at the end of a transition

  local findTransitionEnd = function(transition)
    local connections = listConnections{ Object = transition , Upstream = false, Downstream = true }
    for i, object in connections do
      local _, objectType = getType(object);
      if objectType == "StateMachineNode" then
        return object
      end
    end

    return nil
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- Returns if a table has only one element
  local hasOneElement = function(t)
    local count = 0
    for i in t do
     count = count + 1
     if count > 1 then
       return false
     end
    end

    return count == 1
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- Returns a table of destination substates
  local findSelectedSubStates = function(selection, endPoint)
    local subStates = { }
    local subState

    for i, object in selection do
      local _, objectType = getType(object);
      if objectType == "Transition" then
        subState = getAttribute(object, "DestinationSubState")

        -- if the substate is not set then this impies that the root is set
        if subState == "" or subState == nil then
          subState = endPoint
        end
        subStates[subState] = true
       end
    end

    if not hasOneElement(subStates) then
      subState = nil
    end

    return subStates, subState
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- Returns the end points (destinations) of all the transitions in the selection
  local findTransitionEndPoint = function(selection)
    local endPoint = nil
    for i, object in selection do
      local _, objectType = getType(object);
      if objectType == "Transition" then
        local newEndPoint = findTransitionEnd(object)
        if endPoint == nil then
          endPoint = newEndPoint
        elseif endPoint ~= newEndPoint then
          return nil
        end
      end
    end

    return endPoint
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- Expands all the parents below an item
  local expandParents = function(item)
    item = item:getParent()
    while item do
      item:expand()
      item = item:getParent()
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- Collapses all the items in a tree
  local collapseTree
  collapseTree = function(rootItem)
    rootItem:collapse()
    local numChildren = rootItem:getNumChildren()
    for i = 1, numChildren do
      local childItem = rootItem:getChild(i)
      collapseTree(childItem)
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- Clears out any ticks that have been set indicating chosen items
  local clearChosenItemsInTree
  clearChosenItemsInTree = function(rootItem)
    rootItem:setColumnCheckbox(1, 3) -- no check
    local numChildren = rootItem:getNumChildren()
    for i = 1, numChildren do
      local childItem = rootItem:getChild(i)
      clearChosenItemsInTree(childItem)
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  local hasStates = function(node)
    local children = listChildren(node)
    if table.getn(children) == 0 then
      return false
    end

    for i, v in ipairs(children) do
      local _, objectType = getType(v)
      if objectType ~= "StateMachineNode" and objectType ~= "Transition" then
        return false
      end
    end

    return true
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- Returns the default state of an node - if the node is a BlendTree it returns nil
  local safeGetDefaultState = function(node)
    local name, objectType = getType(node)
    local result = nil
    if (name == "BlendTree" or name == "PhysicsBlendTree") then
      local children = listChildren(node)
      if table.getn(children) == 1 then
       local child = children[1]
       if hasStates(child) then
         result = getDefaultState(child)
       end
      end
    elseif objectType == "BlendNode" then
      if hasStates(node) then
       result = getDefaultState(node)
      end
    elseif objectType == "StateMachineNode" then
      result = getDefaultState(node)
    elseif objectType == "StateMachine"  or objectType == "PhysicsStateMachine" then
      result = getDefaultState(node)
    end

    return result
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- This will traverse through all of the items in a tree from a given item checking the default states to find the implied
  -- target that is the target that is implied by the default states
  local findUltimateTarget = nil
  findUltimateTarget = function(rootItem)
    local v = rootItem:getUserDataString()
    local defaultState = safeGetDefaultState(v)
    local numChildren = rootItem:getNumChildren()
    if numChildren > 0 then
      if defaultState or (rootItem == stateMachineTree:getRoot()) then
        for i = 1, numChildren do
          local childItem = rootItem:getChild(i)
          v = childItem:getUserDataString()
          if v == defaultState or (not childItem:isEnabled()) then
            return findUltimateTarget(childItem)
          end
        end
      end
    end

    return rootItem
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- Mark the items in the tree that show the chosen items
  local chooseItemsInTree
  chooseItemsInTree = function(rootItem, selectedItems, expandInTree)
    local v = rootItem:getUserDataString()
    if selectedItems[v] then
      if hasOneElement(selectedItems) then
        rootItem:setColumnCheckbox(1, 4) -- check
        if expandInTree then
          stateMachineTree:ensureItemVisible(rootItem)
          stateMachineTree:selectItem(rootItem)
        end
        stateMachineTree:styleItemAsTarget(findUltimateTarget(rootItem), "adornTarget")
      else
        stateMachineTree:styleItemAsTarget(nil)
        rootItem:setColumnCheckbox(1, 5) -- indeterminate
      end

      if expandInTree then
        expandParents(findUltimateTarget(rootItem))
      end
    end

    local numChildren = rootItem:getNumChildren()
    for i = 1, numChildren do
      local childItem = rootItem:getChild(i)
      chooseItemsInTree(childItem, selectedItems, expandInTree)
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  local addSubStateToTree
  addSubStateToTree = function(nodePath, rootItem, nodeToTreeItem, defaultState)
    local isDefined = (nodeToTreeItem[nodePath] ~= nil) and (nodeToTreeItem[nodePath] ~= true)
    if not isDefined then
      local path, nodeName = splitNodePath(nodePath)
      local treeItem = addSubStateToTree(path, rootItem, nodeToTreeItem)
      
      local childItem = treeItem:addChild()
      local enable = nodeToTreeItem[nodePath] ~= nil
      if nodePath == defaultState then
        childItem:setColumnValue(2, nodeName .. kDefaultStateGarnish)
      else
        childItem:setColumnValue(2, nodeName)
      end
      childItem:enable(enable)
      childItem:setUserDataString(nodePath)
      nodeToTreeItem[nodePath] = childItem
    end
    
    return nodeToTreeItem[nodePath]
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- An item has been chosen in the tree - update the underlying attributes
  local onItemChosen = function(tree, itemActivated)
    if not itemActivated:isEnabled() then
      return
    end

    enableContextEvents = false
    enableUISetAttribute = false

    local subState = itemActivated:getUserDataString()
    local endPoint = findTransitionEndPoint(selection)

    if endPoint then

      -- If the substate is the same as the end point of the transition we
      -- do not set a substate i.e. we set it to ""
      if subState == endPoint then
        subState = ""
      end

      -- Set the attributes of all the transitions in the selection
      for i, object in selection do
        local _, objectType = getType(object);
        if objectType == "Transition" then
          setAttribute(object .. ".DestinationSubState", subState)
        end
      end

      local rootItem = stateMachineTree:getRoot()
      local selectedSubStates = findSelectedSubStates(selection, endPoint)

      clearChosenItemsInTree(rootItem)
      chooseItemsInTree(rootItem, selectedSubStates)

      tree:selectItem(itemActivated)
    end

    enableContextEvents = true
    enableUISetAttribute = true
  end

  ----------------------------------------------------------------------------------------------------------------------
  local rebuildStateMachineTree = function()

    -- Delete all the elements in the tree
    local rootItem = stateMachineTree:getRoot()
    rootItem:clearChildren();

    local endPoint = findTransitionEndPoint(selection)
    if endPoint ~= nil then
      local _, shortName = splitNodePath(endPoint)

      -- get a ordered list and a set of all the valid nodes
      local nodeToTreeItem = { }
      local validNodeList = listValidAttributeValues(selection[1], "DestinationSubState")
      table.sort(validNodeList, stableCaseInsensitiveCompare)
      for _, attr in validNodeList do
        nodeToTreeItem[attr] = true
      end
      
      rootItem:setColumnValue(2, shortName)
      rootItem:setUserDataString(endPoint)
      rootItem:setColumnCheckbox(1, 3)
      nodeToTreeItem[endPoint] = rootItem

      -- loop throgh all of the valid attributes and add them to the tree
      for _, path in validNodeList do
        local defaultState = safeGetDefaultState(splitNodePath(path))
        addSubStateToTree(path, rootItem, nodeToTreeItem, defaultState)
      end
      
      -- Choose the items in the tree
      local subStates, subState = findSelectedSubStates(selection, endPoint)
      clearChosenItemsInTree(rootItem)
      collapseTree(rootItem)
      rootItem:expand()

      chooseItemsInTree(rootItem, subStates, true)
    end

    return endPoint
  end

  ----------------------------------------------------------------------------------------------------------------------
  local syncUIWithAttributes = function(dontRebuild)
    attributeEditor.logEnterFunc("syncUIWithAttributes")

    if type(subStateCheckBox) == "userdata" then
      local endPoint = rebuildStateMachineTree()
      if endPoint == nil then
        -- At this point have different end-points this means that it is not possible
        -- to show the tree.
        showStateMachineTree(false, dontRebuild)
        local _, subState = findSelectedSubStates(selection, "")

        subStateCheckBox:enable(subState ~= "")
        subStateLabel:enable(subState ~= "")

        if subState == nil then
          subStateCheckBox:setChecked(2) -- conflict
        elseif subState == "" then
          subStateCheckBox:setChecked(0) -- off
        else
          subStateCheckBox:setChecked(1) -- on
        end

      else
        local subStates, subState = findSelectedSubStates(selection, endPoint)
        subStateCheckBox:setChecked(not (subState == endPoint))
        showStateMachineTree(not (subState == endPoint), dontRebuild)
      end
    end
    attributeEditor.logExitFunc("syncUIWithAttributes")
  end

  ----------------------------------------------------------------------------------------------------------------------
  local subStateCheckboxChanged = function(checkBox)
    if checkBox:getChecked() then
      local endPoint = findTransitionEndPoint(selection)

      -- we were in the indeterminate state
      if endPoint == nil then
        checkBox:setChecked(0)
        if ui.showMessageBox("Do you want to remove the \"Destination Sub State\" from all the selected transitions?", "ok;cancel") == "ok" then
          checkBox:setChecked(false)
        else
          checkBox:setChecked(2)
          return
       end
      end
    end

    enableContextEvents = false
    enableUISetAttribute = false

    if (checkBox:getChecked()) then
      rebuildStateMachineTree()
      showStateMachineTree(true, kRebuildEditor)
    else
      -- clear all of the sub-states
      for i, object in selection do
        local _, objectType = getType(object);
        if objectType == "Transition" then
          setAttribute(object .. ".DestinationSubState", "")
        end
      end

      showStateMachineTree(false, kRebuildEditor)
    end

    enableContextEvents = true
    enableUISetAttribute = true
  end

  attributeEditor.onNodeCreateDestroy = function() syncUIWithAttributes() end
  attributeEditor.onNodeRename = function() syncUIWithAttributes() end
  attributeEditor.onEdgeConnectionChange = function() syncUIWithAttributes() end

  ----------------------------------------------------------------------------------------------------------------------
  -- this data change context ensures the ui reflects any changes that happen via script,
  -- or through undo redo, are reflected in the custom ui.
  ------------------------------------------------------------------------------------------------------------------------
  local changeContext = attributeEditor.createChangeContext()
  changeContext:setObjects(selection)
  changeContext:addAttributeChangeEvent("DestinationSubState")

  ------------------------------------------------------------------------------------------------------------------------
  -- this function is called whenever ReversibleTransit is changed via script, or through undo redo.
  ------------------------------------------------------------------------------------------------------------------------
  changeContext:setAttributeChangedHandler(
    function(object, attr)
      attributeEditor.logEnterFunc("changeContext attributeChangedHandler")

      if enableContextEvents then
        enableUISetAttribute = false
        syncUIWithAttributes(kRebuildEditor)
        enableUISetAttribute = true
      end

      attributeEditor.logExitFunc("changeContext attributeChangedHandler")
    end
  )

  local helpText = "An optional override for transitioning into sub-states other than the defaults."

  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "destSubStateDisplayInfo" }
  local rollPanel = rollup:getPanel()
  rollup:expand(false)

  -- first add the ui for the section
  attributeEditor.log("rollPanel:beginHSizer")
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    rollPanel:addHSpacer(6)

    attributeEditor.log("rollPanel:beginVSizer")
    rollPanel:beginVSizer{ flags = "expand", proportion = 1 }

      attributeEditor.log("rollPanel:beginHSizer")
      rollPanel:beginHSizer{ cols = 2, flags = "expand", proportion = 0 }
        subStateLabel = rollPanel:addStaticText{ text = "Sub State" }
        attributeEditor.bindHelpToWidget(subStateLabel, helpText)

        rollPanel:addHSpacer(5)
        subStateCheckBox = rollPanel:addCheckBox()
        attributeEditor.bindHelpToWidget(subStateCheckBox, helpText)
      rollPanel:endSizer()
      attributeEditor.log("rollPanel:endHSizer")

      rollPanel:addHSpacer(6)

      attributeEditor.log("rollPanel:beginHSizer")
      rollPanel:beginHSizer{ cols = 1, flags = "expand", proportion = 0 }

        stateMachineTree = rollPanel:addTreeControl{
          name = "StateMachineTree",
          flags = "expand;hideRoot",
          size = { height = 200 },
          proportion = 1,
          numColumns = 2,
          treeControlColumn = 2,
          onItemActivated = onItemChosen,
          onCheckboxChanged = onItemChosen
        }

        stateMachineTree:setColumnWidth(1, 20)
      rollPanel:endSizer()
      attributeEditor.log("rollPanel:endHSizer")

    rollPanel:endSizer()
    attributeEditor.log("rollPanel:endVSizer")

  rollPanel:endSizer()
  attributeEditor.log("rollPanel:endHSSizer")

  subStateCheckBox:setOnChanged(subStateCheckboxChanged)

  -- set the initial state of the ui
  syncUIWithAttributes(kDontRebuildEditor)

  ------------------------------------------------------------------------------------------------------------------------

  attributeEditor.logExitFunc("attributeEditor.destSubStateDisplayInfo")
end
