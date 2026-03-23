------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"

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
attributeEditor.conditionSubStateDisplayInfo = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.conditionSubStateDisplayInfo")

  local stateMachinePanel
  local stateMachineTree -- the tree control that shows all the substates

  -- these prevent callbacks firing off other callbacks causing infinite loops
  local enableContextEvents = true
  local enableUISetAttribute = true

  ----------------------------------------------------------------------------------------------------------------------
  -- Returns the state machine at the end of a transition
  local kRebuildEditor = false
  local kDontRebuildEditor = true

  ----------------------------------------------------------------------------------------------------------------------
  -- Returns the state machine at the end of a transition

  local findTransitionStart = function(condition)
    local transition = getParent(condition)
    local connections = listConnections{ Object = transition , Upstream = true, Downstream = false }
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
  local findSelectedSubStates = function(selection, startPoint)
    local subStates = { }
    local subState

    for i, object in selection do
      local _, objectType = getType(object);
      if objectType == "Condition" then
        subState = getAttribute(object, "Node")

        -- if the substate is not set then this impies that the root is set
        if subState ~= "" and subState ~= nil then
          subStates[subState] = true
        end
       end
    end

    if not hasOneElement(subStates) then
      subState = nil
    end

    return subStates, subState
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- Returns the start points (sources) of all the transiions in the selection
  local findTransitionStartPoints = function(selection)
    local startPoints = { };
    local lastEndPoint = nil
    for i, object in selection do
      local _, objectType = getType(object);
      if objectType == "Condition" then
        lastEndPoint = findTransitionStart(object)
        if lastEndPoint then
          startPoints[lastEndPoint] = true;
        end
      end
    end

    if not hasOneElement(startPoints) then
      return startPoints, nil
    end

    return startPoints, lastEndPoint
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
        stateMachineTree:styleItemAsTarget(rootItem, "adornTarget")
      else
        stateMachineTree:styleItemAsTarget(nil)
        rootItem:setColumnCheckbox(1, 5) -- indeterminate
      end

      if expandInTree then
        expandParents(rootItem)
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
  addSubStateToTree = function(nodePath, rootItem, nodeToTreeItem)
    local isDefined = (nodeToTreeItem[nodePath] ~= nil) and (nodeToTreeItem[nodePath] ~= true)
    if not isDefined then
      local path, nodeName = splitNodePath(nodePath)
      local treeItem = addSubStateToTree(path, rootItem, nodeToTreeItem)
      
      local childItem = treeItem:addChild()
      local enable = nodeToTreeItem[nodePath] ~= nil
      childItem:setColumnValue(2, nodeName)
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
    local _, startPoint = findTransitionStartPoints(selection)

    if startPoint then

      -- If the substate is the same as the end point of the transition we
      -- do not set a substate i.e. we set it to ""
      if subState == startPoint then
        subState = ""
      end

      -- Set the attributes of all the transitions in the selection
      for i, object in selection do
        local _, objectType = getType(object);
        if objectType == "Condition" then
          setAttribute(object .. ".Node", subState)
        end
      end

      local rootItem = stateMachineTree:getRoot()
      local selectedSubStates = findSelectedSubStates(selection, startPoint)

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

    local startPoints, startPoint = findTransitionStartPoints(selection)
    if startPoint ~= nil then
      local _, shortName = splitNodePath(startPoint)

      -- get a ordered list and a set of all the valid nodes
      local nodeToTreeItem = { }
      local validNodeList = listValidAttributeValues(selection[1], "Node")
      table.sort(validNodeList, stableCaseInsensitiveCompare)
      for _, attr in validNodeList do
        nodeToTreeItem[attr] = true
      end
      
      rootItem:setColumnValue(2, shortName)
      rootItem:setUserDataString(startPoint)
      rootItem:setColumnCheckbox(1, 3)
      nodeToTreeItem[startPoint] = rootItem

      -- loop throgh all of the valid attributes and add them to the tree
      for _, path in validNodeList do
        addSubStateToTree(path, rootItem, nodeToTreeItem)
      end
      
      -- Choose the items in the tree
      local subStates, subState = findSelectedSubStates(selection, startPoint)
      clearChosenItemsInTree(rootItem)
      collapseTree(rootItem)
      rootItem:expand()

      chooseItemsInTree(rootItem, subStates, true)
    end

    return startPoint
  end
  ----------------------------------------------------------------------------------------------------------------------
  local syncUIWithAttributes = function(dontRebuild)
    attributeEditor.logEnterFunc("syncUIWithAttributes")

    local startPoint = rebuildStateMachineTree()
    if startPoint == nil then
      -- At this point have different end-points this means that it is not possible
      -- to show the tree.
      local _, subState = findSelectedSubStates(selection, "")

    else
      local subStates, subState = findSelectedSubStates(selection, startPoint)
    end

    attributeEditor.logExitFunc("syncUIWithAttributes")
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

  local helpText = "The sub state node."

  attributeEditor.log("rollContainer:beginVSizer")
  rollContainer:beginVSizer{ flags = "expand", name = "Destination", proportion = 1 }

    stateMachineTree = rollContainer:addTreeControl{
      name = "ConditionSubstateTree",
      flags = "expand;hideRoot",
      size = { height = 200 },
      proportion = 1,
      numColumns = 2,
      treeControlColumn = 2,
      onItemActivated = onItemChosen,
      onCheckboxChanged = onItemChosen
    }

    stateMachineTree:setColumnWidth(1, 20)

  rollContainer:endSizer()
  attributeEditor.log("rollContainer:endVSizer")

  -- set the initial state of the ui
  syncUIWithAttributes(kDontRebuildEditor)

 ------------------------------------------------------------------------------------------------------------------------

  attributeEditor.logExitFunc("attributeEditor.conditionSubStateDisplayInfo")
end

