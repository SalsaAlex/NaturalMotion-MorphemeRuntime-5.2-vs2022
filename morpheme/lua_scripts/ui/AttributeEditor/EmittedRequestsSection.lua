------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"
require "luaAPI/AttributeEditorUtils.lua"
require "luaAPI/ManifestUtils.lua"

local kNotSpecifiedString = "Not Specified"
local kClearAllString = "Clear All"
local kBroadcastString = "- Broadcast - "
local kBroadcastLongString = "- Broadcast - (all state machines)"

------------------------------------------------------------------------------------------------------------------------
-- Poses a dialog with a tree displaying all the possible state machines that can be chose as a target
local poseStateMachineDialog = function(targetAttr, selection)
  local panel, okButton
  
  local dlg -- the dialog
  local treeControl -- the tree control
  local nodeToTreeItem = {}  -- A table mapping items (by path) to tree items

  ----------------------------------------------------------------------------------------------------------------------
  -- Add an item (specified by path) to the tree. This will add any required parent items
  local addPathToStateMachineTree
  addPathToStateMachineTree = function(nodepath)
    if nodepath == "" then
      return treeControl:getRoot()
    end
    
    local treeItem = nodeToTreeItem[nodepath]
    if treeItem == nil then
      local path, state = splitNodePath(nodepath)
      local rootItem = addPathToStateMachineTree(path)
      treeItem = rootItem:addChild()
      treeItem:setUserDataString(nodepath)
      treeItem:setColumnValue(2, state)
      local theType = getType(nodepath)
      treeItem:enable(theType == "StateMachine" or theType == "PhysicsStateMachine")
      nodeToTreeItem[nodepath] = treeItem
    end
    
    return treeItem
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- Mark the items in the tree that show the chosen items
  local findChosenItem
  findChosenItem = function(rootItem)
    if rootItem:getColumnCheckbox(1) == 4 then
      return rootItem:getUserDataString()
    end
    
    local numChildren = rootItem:getNumChildren()
    for i = 1, numChildren do
      local item = findChosenItem(rootItem:getChild(i))
      if item ~= nil then
        return item
      end
    end
    
    return nil
  end
  
  ----------------------------------------------------------------------------------------------------------------------
  -- Mark the items in the tree that show the chosen items
  local chooseItemInTree
  chooseItemInTree = function(rootItem, selectedItemPath)
    local rootPath = rootItem:getUserDataString()
    if selectedItemPath ~= nil and rootPath == selectedItemPath then
      rootItem:setColumnCheckbox(1, 4) -- check
      treeControl:selectItem(rootItem)
    else
      rootItem:setColumnCheckbox(1, 3) -- don't check
    end
    
    local numChildren = rootItem:getNumChildren()
    for i = 1, numChildren do
      chooseItemInTree(rootItem:getChild(i), selectedItemPath)
    end
  end
  
  ----------------------------------------------------------------------------------------------------------------------
  -- An item has been chosen in the tree
  local inItemChosen = false
  local onCheckboxChanged = function(tree, itemActivated)
    if not inItemChosen then
      inItemChosen = true
      if itemActivated:isEnabled() then
        chooseItemInTree(treeControl:getRoot(), itemActivated:getUserDataString())
      end
      inItemChosen = false
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- An item has been chosen in the tree
  local onSelectionChanged = function(tree)
    local selectedItem = tree:getSelectedItem()
    if selectedItem then
      onCheckboxChanged(tree, selectedItem)
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- called when an item has been chosen and the dialog is diismissed
  local dissmissOk = function(self)
    local chosenStateMachine = findChosenItem(treeControl:getRoot())
    if chosenStateMachine then
      setCommonAttributeValue(selection, targetAttr, chosenStateMachine)
    end
    dlg:hide()
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- called the dilog is dismissed and cancelled
  local dissmissCancel = function(self)
    dlg:hide()
  end

  ----------------------------------------------------------------------------------------------------------------------
  local rebuildStateMachineTree = function()

    nodeToTreeItem = {}
    local rootItem = treeControl:getRoot()
    rootItem:clearChildren();
    local broadcastItem = rootItem:addChild()
    broadcastItem:setColumnValue(2, kBroadcastLongString)
    broadcastItem:setBold(true)
    local stateMachines = ls("StateMachine")
    local hasRootState = false;
    for _, path in stateMachines do
      addPathToStateMachineTree(path) 
    end
    
    local targetValue = getPathFromWeakReferenceValue(getCommonAttributeValue(selection, targetAttr))
    if targetValue == "" then
      broadcastItem:setColumnCheckbox(1, 4) -- check
    elseif targetValue~= nil then
      local treeItem = nodeToTreeItem[targetValue]
      if treeItem ~= nil then
        broadcastItem:setColumnCheckbox(1, 4) -- check
      end
    end
  end


  ----------------------------------------------------------------------------------------------------------------------
  dlg = ui.getWindow("RequestStateMachineDialog")
  if not dlg then
    dlg = ui.createModalDialog{
      name = "RequestStateMachineDialog",
      caption = "Choose a State Machine",
      resize = true,
      size = { width = 200, height = -1 },
    }

    dlg:beginVSizer{ flags = "expand", proportion = 1 }
    
      -- tree control
      treeControl = dlg:addTreeControl{
        name = "StateMachineTree",
        flags = "expand;hideRoot;stripe",
        size = { height = 200 },
        proportion = 1,
        numColumns = 2,
        treeControlColumn = 2
      }

      -- ok & cancel buttons
      dlg:beginHSizer{ flags = "right", proportion = 0 }
        okButton = dlg:addButton{ name = "OKButton", label = "OK", size = { width = 74 } }
        dlg:addButton{ name = "CancelButton", label = "Cancel", size = { width = 74 }, onClick = dissmissCancel, }
      dlg:endSizer()

    dlg:endSizer()
  else
    treeControl = dlg:getChild("StateMachineTree")
    okButton = dlg:getChild("OKButton")
  end

  treeControl:setOnSelectionChanged(onSelectionChanged)
  treeControl:setOnCheckboxChanged(onCheckboxChanged)
  treeControl:setOnItemActivated(dissmissOk)
  okButton:setOnClick(dissmissOk)

  dlg:freeze()

  rebuildStateMachineTree();

  dlg:rebuild()
  
  local targetValue = getPathFromWeakReferenceValue(getCommonAttributeValue(selection, targetAttr))
  chooseItemInTree(treeControl:getRoot(), targetValue)
  dlg:setSize{ width = 400, height = -1 }
  dlg:show()
end

----------------------------------------------------------------------------------------------------------------------
-- Add a combo box for requests. This has two additional elements "Not Specified" and "Clear All". The "Clear All"
-- option is unusual in that it actually reflects a "Clear All" setting in actionAttr.
local addCustomRequestsComboBox = function(panel, requestAttr, actionAttr, targetAttr, selection)
  attributeEditor.logEnterFunc("attributeEditor.customComboBox")
  
  local customComboBox
  
  --------------------------------------------------------------------------------------------------------------------
  -- Flags to stop an infinte loop of ui code calling context change code calling ui code and so on.
  local enableContextEvents = true
  local enableUISetAttribute = true

  --------------------------------------------------------------------------------------------------------------------
  -- A pair of lists containing items in the combo and the functions that are called when an item is chosen
  local comboValueNames = { }
  local comboValueOnChangeFn = { }
    
  --------------------------------------------------------------------------------------------------------------------
  -- returns a function that sets a value in the requestAttr
  local mkSetRequestFunction = function(value)
    return function(selection)
      undoBlock(function()
        setCommonAttributeValue(selection, requestAttr, value)
        local actionValue = getCommonAttributeValue(selection, actionAttr)
        if actionValue == kClearAllString then
          setCommonAttributeValue(selection, actionAttr, "Set")
        end
      end)
    end
  end

  --------------------------------------------------------------------------------------------------------------------
  -- rebuild the list of elements that apear in the combo box
  local rebuildComboList = function()
    comboValueNames = { }
    comboValueOnChangeFn = { }

    -- Define the extra options "NotSpecified" and "ClearAll"
    comboValueNames[1] = kNotSpecifiedString
    comboValueOnChangeFn[1] = function(selection)
        undoBlock(function()
          setCommonAttributeValue(selection, requestAttr, "")
          setCommonAttributeValue(selection, actionAttr, "Set")
          setCommonAttributeValue(selection, targetAttr, "")
        end)
      end
    
    comboValueNames[2] = kClearAllString
      comboValueOnChangeFn[2] = function(selection)
        undoBlock(function()
          setCommonAttributeValue(selection, requestAttr, "")
          setCommonAttributeValue(selection, actionAttr, kClearAllString)
        end)
      end
    
    -- Add all the requests
    local allRequests = ls("Request")
    local firstElementOffset = table.getn(comboValueNames)
    for i, nodepath in ipairs(allRequests) do
      local path, request = splitNodePath(nodepath)
      table.insert(comboValueNames, request)
      comboValueOnChangeFn[i + firstElementOffset] = mkSetRequestFunction(nodepath)
    end
    
    customComboBox:setItems(comboValueNames)
  end
   
  --------------------------------------------------------------------------------------------------------------------
  -- sync the UI with the underlying attribute values
  local syncUIFunction = function() 
    rebuildComboList()
    local requestValue = getCommonAttributeValue(selection, requestAttr)
    local actionValue = getCommonAttributeValue(selection, actionAttr)
    local requestIndeterminate = requestValue == nil or (requestValue == "" and actionValue == nil)
    customComboBox:setIsIndeterminate(requestIndeterminate)
    if requestValue then
      if requestValue == "" and actionValue == kClearAllString then
        customComboBox:setSelectedIndex(2)
      elseif requestValue == "" then
        customComboBox:setSelectedIndex(1)
      else
        local path, request = splitNodePath(requestValue)
        customComboBox:setSelectedItem(request)
      end
    end
  end

  --------------------------------------------------------------------------------------------------------------------
  -- this function is called whenever custom combo box is changed through the ui
  local onChanged = function(self)
    if enableUISetAttribute then
      -- prevent the change context callbacks from firing off
      enableContextEvents = false
        local selectedIndex = self:getSelectedIndex()
        local onValueChangeFunction = comboValueOnChangeFn[selectedIndex]
        onValueChangeFunction(selection)
        syncUIFunction(self, selection)
      enableContextEvents = true
    end
  end
  
  --------------------------------------------------------------------------------------------------------------------
  local onEnter = function(self, text)
    if text == kClearAllString then
      customComboBox:setSelectedItem(kClearAllString)
    elseif text == kNotSpecifiedString then
      customComboBox:setSelectedItem(kNotSpecifiedString)
    else
      local allRequests = ls("Request")
      for _, requestPath in allRequests do
       local _, request = splitNodePath(requestPath)
       if request == text then
          customComboBox:setSelectedItem(text)
          setCommonAttributeValue(selection, requestAttr, requestPath)
          return
        end
      end
      
      local requestPath = create("Request", text)
      local path, name = splitNodePath(requestPath)
      customComboBox:setSelectedItem(name)
      setCommonAttributeValue(selection, requestAttr, requestPath)
   end
  end
  
  attributeEditor.log("panel:addComboBox")
  customComboBox = panel:addComboBox{ flags = "expand",  proportion = 1, onEnter = onEnter}
  customComboBox:enable(not containsReference(selection))
  customComboBox:setEditable(true)
  customComboBox:setOnChanged(onChanged)

  attributeEditor.log("creating change context for attributes")
  local changeContext = attributeEditor.createChangeContext()
  changeContext:setObjects(selection)
  changeContext:addAttributeChangeEvent(requestAttr)
  changeContext:addAttributeChangeEvent(actionAttr)

  --------------------------------------------------------------------------------------------------------------------
  -- this function is called whenever attributes are changed via script, or through undo redo.
  changeContext:setAttributeChangedHandler(
    function(object, attr)
      attributeEditor.logEnterFunc("changeContext attributeChangedHandler")

      if enableContextEvents then
        enableUISetAttribute = false
          syncUIFunction()
        enableUISetAttribute = true
      end

      attributeEditor.logExitFunc("changeContext attributeChangedHandler")
    end
  )

  syncUIFunction()
  attributeEditor.addRequestCreateDestroy(syncUIFunction)
  attributeEditor.addRequestRename(syncUIFunction)
  attributeEditor.logExitFunc("attributeEditor.customComboBox")
  return customComboBox
end

----------------------------------------------------------------------------------------------------------------------
local addTargetWidgets = function(panel, requestAttr, actionAttr, targetAttr, selection)

  panel:beginHSizer{ flags = "expand", proportion = 1 }
    local targetWidget = panel:addTextBox{ flags = "expand", proportion = 1 }
    
    targetWidget:setReadOnly(true)
    local moreButton = panel:addButton{
      label = "...",
      size = { width = 16, height = 17 },
      onClick = function()
        poseStateMachineDialog(targetAttr, selection)
      end
    }
  panel:endSizer()
  
  local isReferencedList = containsReference(selection)

  -- Update Widget from the Attributes
  local updateWidget = function(self)
    local requestValue = getCommonAttributeValue(selection, requestAttr)
    local actionValue = getCommonAttributeValue(selection, actionAttr)
    local targetValue = getPathFromWeakReferenceValue(getCommonAttributeValue(selection, targetAttr))
    local requestIndeterminate = requestValue == nil or (requestValue == "" and actionValue == nil)
    local enableWidget = (not requestIndeterminate) 
                     and (requestValue ~= "" or actionValue == kClearAllString) 
                     and (not isReferencedList) 

    targetWidget:setIsIndeterminate(value == nil)
    if targetValue == "" then
      targetWidget:setValue(kBroadcastString)
    elseif targetValue ~= nil then
      targetWidget:setValue(targetValue)
    end
    targetWidget:enable(enableWidget)
    moreButton:enable(enableWidget)
  end

  -- create the change context
  local context = attributeEditor.createChangeContext()
  context:setObjects(selection)
  context:addAttributeChangeEvent(requestAttr)
  context:addAttributeChangeEvent(actionAttr)
  context:addAttributeChangeEvent(targetAttr)
  context:setAttributeChangedHandler(updateWidget)

  -- and update the widget so that to start with it displays the current value
  -- and is enabled appropriately
  updateWidget()
  
end

------------------------------------------------------------------------------------------------------------------------
-- this adds three widgets for 'request', 'action', and 'target' attributes and manages them abd
-- their interrelationship
attributeEditor.addRequestWidgets = function(panel, requestAttr, actionAttr, targetAttr, selection)
  
  ----------------------------------------------------------------------------------------------------------------------
  local mkSetActionFunction = function(value)
    return function(selection)
      setCommonAttributeValue(selection, actionAttr, value)
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  local syncActionComboBoxUI = function(combo, selection)
    local requestValue = getCommonAttributeValue(selection, requestAttr)
    local actionValue = getCommonAttributeValue(selection, actionAttr)
    local requestIndeterminate = requestValue == nil or (requestValue == "" and actionValue == nil)
    local enableCombo = (not requestIndeterminate) 
                    and (requestValue ~= "") 
                    and (actionValue == nil or actionValue ~= kClearAllString)
                    and (not containsReference(selection))
 
   combo:enable(enableCombo)
    if enableCombo then
      combo:setIsIndeterminate(actionValue == nil)
      if actionValue ~= nil then
        local lookup = { ["Set"] = 1, ["Clear"] = 2 }
        local index = lookup[actionValue]
        if index then
          combo:setSelectedItem(actionValue)
        end
      end
    else
      combo:setSelectedItem("Clear")
    end
  end
  
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.logEnterFunc("attributeEditor.addRequestWidgets")
  
    -- Requests combo box
    addCustomRequestsComboBox(panel, requestAttr, actionAttr, targetAttr, selection)

    -- Action combo box
    local actionComboBox = attributeEditor.addCustomComboBox{
      panel = panel, objects = selection,
      attributes = { requestAttr, actionAttr },
      values = {
        ["Set"] = mkSetActionFunction("Set"),
        ["Clear"] = mkSetActionFunction("Clear"),
      },
      size = { width = 80 },
      syncValueWithUI = syncActionComboBoxUI
    }
         
    -- Target wifgets (text field + button)
    addTargetWidgets(panel, requestAttr, actionAttr, targetAttr, selection) 
   
  attributeEditor.logEnterFunc("attributeEditor.addRequestWidgets")
end

------------------------------------------------------------------------------------------------------------------------
-- Setting up a section that deals only with emitting requests.
attributeEditor.operatorEmitRequestSection = function(spec, rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.operatorEmitRequestSection")
  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "blendWeights" }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginVSizer")
  rollPanel:beginVSizer{ flags = "expand" }

    attributeEditor.log("rollPanel:beginFlexGridSizer")
    rollPanel:beginFlexGridSizer{ cols = 3, flags = "expand", proportion = 1 }
      local oldBorder = rollPanel:getBorder()
      rollPanel:setBorder(0)
      rollPanel:setFlexGridColumnExpandable(3)
      
      -- Add the labels
      rollPanel:addStaticText{text = spec.request.text}
      rollPanel:addStaticText{text = spec.action.text}
      rollPanel:addStaticText{text = spec.target.text}
      
      -- Add the widgets
      for i = spec.first, spec.last do
        local requestAttr = spec.request.name .. i
        local actionAttr = spec.action.name .. i
        local targetAttr = spec.target.name .. i
        attributeEditor.addRequestWidgets(rollPanel, requestAttr, actionAttr, targetAttr, selection)
      end
      
      rollPanel:setBorder(oldBorder)
    rollPanel:endSizer()

    attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.operatorEmitRequestSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Setting up a section that emits requests when a control param value comparison is true.
attributeEditor.operatorEmitRequestAndCPSection = function(spec, rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.operatorEmitRequestAndCPSection")
  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "blendWeights" }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginVSizer")
  rollPanel:beginVSizer{ flags = "expand" }

    attributeEditor.log("rollPanel:beginFlexGridSizer")
    rollPanel:beginFlexGridSizer{ cols = 5, flags = "expand", proportion = 1 }
      local oldBorder = rollPanel:getBorder()
      rollPanel:setBorder(0)
      rollPanel:setFlexGridColumnExpandable(5)
      
      -- Add the labels
      rollPanel:addStaticText{text = spec.triggerVal.text}
      rollPanel:addStaticText{text = spec.comparison.text}
      rollPanel:addStaticText{text = spec.request.text}
      rollPanel:addStaticText{text = spec.action.text}
      rollPanel:addStaticText{text = spec.target.text}
      
      -- Add the widgets
      for i = spec.first, spec.last do
        local triggerValAttr = spec.triggerVal.name .. i
        attributeEditor.addAttributeWidget(rollPanel, triggerValAttr, selection, set)
               
        local comparisonAttr = spec.comparison.name .. i
        attributeEditor.addIntAttributeCombo{
          panel = rollPanel,
          objects = selection,
          attribute = comparisonAttr,
          values = { [">"] = ">", ["<"] = "<", [">="] = ">=", ["<="] = "<=" },
          set = set,
          helpText = "Set your comparison choice here.",
          order = { ">", "<", ">=", "<=" },
        }
               
        local requestAttr = spec.request.name .. i
        local actionAttr = spec.action.name .. i
        local targetAttr = spec.target.name .. i
        attributeEditor.addRequestWidgets(rollPanel, requestAttr, actionAttr, targetAttr, selection)
      end
      
      rollPanel:setBorder(oldBorder)
    rollPanel:endSizer()

    attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.operatorEmitRequestAndCPSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Setting up a section that emits requests when a specified sampled user data event is encountered.
attributeEditor.emitRequestOnDiscreteEventSection = function(spec, rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.emitRequestOnDiscreteEventSection")
  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "blendWeights" }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginVSizer")
  rollPanel:beginVSizer{ flags = "expand" }

    attributeEditor.log("rollPanel:beginFlexGridSizer")
    rollPanel:beginFlexGridSizer{ cols = 4, flags = "expand", proportion = 1 }
      local oldBorder = rollPanel:getBorder()
      rollPanel:setBorder(0)
      rollPanel:setFlexGridColumnExpandable(4)
      
      -- Add the labels
      rollPanel:addStaticText{text = spec.event.text}
      rollPanel:addStaticText{text = spec.request.text}
      rollPanel:addStaticText{text = spec.action.text}
      rollPanel:addStaticText{text = spec.target.text}
      
      -- Add the widgets
      for i = spec.first, spec.last do
        local requestAttr = spec.event.name .. i
        attributeEditor.addAttributeWidget(rollPanel, requestAttr, selection, set)
        
        local requestAttr = spec.request.name .. i
        local actionAttr = spec.action.name .. i
        local targetAttr = spec.target.name .. i
        attributeEditor.addRequestWidgets(rollPanel, requestAttr, actionAttr, targetAttr, selection)
      end
      
      rollPanel:setBorder(oldBorder)
    rollPanel:endSizer()

    attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.emitRequestOnDiscreteEventSection")
end

------------------------------------------------------------------------------------------------------------------------
attributeEditor.emitBehaviourRequestSection = function(spec, rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.emitBehaviourRequestSection")
  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "emitBehaviourRequest" }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginVSizer")
  rollPanel:beginVSizer{ flags = "expand" }

    attributeEditor.log("rollPanel:beginFlexGridSizer")
      rollPanel:beginFlexGridSizer{ cols = 4, flags = "expand", proportion = 1 }
      local oldBorder = rollPanel:getBorder()
      rollPanel:setBorder(0)
      rollPanel:setFlexGridColumnExpandable(4)
          
      -- Add the labels
      rollPanel:addStaticText{text = spec.id.text}
      rollPanel:addStaticText{text = spec.request.text}
      rollPanel:addStaticText{text = spec.action.text}
      rollPanel:addStaticText{text = spec.target.text}

      -- Add the widgets
      for i = spec.first, spec.last do
        rollPanel:addStaticText{text = spec.id.requests[i+1]}
        local requestAttr = spec.request.name .. i
        local actionAttr = spec.action.name .. i
        local targetAttr = spec.target.name .. i
        attributeEditor.addRequestWidgets(rollPanel, requestAttr, actionAttr, targetAttr, selection)
      end
          
    rollPanel:setBorder(oldBorder)
    rollPanel:endSizer()

    attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.emitBehaviourRequestSection")
end