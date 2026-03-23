------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: PAGE
--| name: ComponentEditorAPI
--| desc:
--|   Lua-scripted functions related to the component editor.
------------------------------------------------------------------------------------------------------------------------
componentEditor = { }
listMapping = { }
listRollupHeading = nil

------------------------------------------------------------------------------------------------------------------------
local componentList = nil
local statusIcons = {
  app.loadImage("InactiveIcon.png"),
  app.loadImage("TickIcon.png"),
  app.loadImage("WarningIcon.png"),
  app.loadImage("ErrorIcon.png")
}
local iconLookup = { Inactive = 1, Ok = 2, Warning = 3, Error = 4 }

local desiredOrder = {
  "Tags",
  "Templates",
  "Limbs",
  "Poses",
  "Retargeting"
}

------------------------------------------------------------------------------------------------------------------------
componentEditor.getStatusIcon = function(status)
  local argType = type(status)
  if argType ~= "number" then
    if argType == "string" then
      status = iconLookup[status]
    else
      app.error("Could not get status icon, bad parameter passed")
    end
  end

  return statusIcons[status]
end

------------------------------------------------------------------------------------------------------------------------
componentEditor.clear = function()
  if componentEditor.dataChangeEventContexts then
     componentEditor.clearChangeContexts()
  end
  componentEditor.dataChangeEventContexts = { }

  componentEditor.selectedComponentName = nil
  componentEditor.panel = nil
  componentEditor.mainScrollPanel = nil
  componentEditor.helpPanel = nil
  componentEditor.helpText = nil
  listMapping = { }
end

------------------------------------------------------------------------------------------------------------------------
local checkComponentPanelValid = function()
  -- check if the attribute editor panel exists or has changed
  local panel = ui.getWindow("MainFrame|LayoutManager|AttributeEditor|ComponentAttributeEditor")
  if panel == nil or componentEditor.panel ~= panel then
    componentEditor.clear()
  end
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table componentEditor.suspendUpdates()
--| brief:
--|   Prevents the component editor from updating while ui is rebuilt or changed. This
--|   helps to prevent flickering when many controls are changed causing resizing. Must be
--|   called as pair with componentEditor.resumeUpdates().
--|
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
componentEditor.suspendUpdates = function()
  local editor = componentEditor.panel
  editor:freeze()
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table componentEditor.resumeUpdates()
--| brief:
--|   Causes the component editor to continue updating after a call to componentEditor.resumeUpdates.
--|   This helps to prevent flickering when many controls are changed causing resizing. Must be
--|   called as pair with componentEditor.suspendUpdates().
--|
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
componentEditor.resumeUpdates = function()
  local editor = componentEditor.panel
  editor:rebuild()
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: nil componentEditor.setHelpText(string helpText)
--| brief:
--|   Sets the help text displayed in the bottom of the component editor to the
--|   string specified.
--|
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
componentEditor.setHelpText = function(string)
  if componentEditor.helpText then
    if string then
      componentEditor.helpText:setValue(string)
    else
      componentEditor.helpText:setValue("")
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table componentEditor.clearHelpText()
--| brief:
--|   Clears the text displayed at the bottom of the component editor.
--|
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
componentEditor.clearHelpText = function()
  if componentEditor.helpText then
    componentEditor.helpText:setValue("")
  end
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: ComboBox componentEditor.bindHelpToWidget(Window widget, string helpText)
--| brief: Adds the help text to a widget
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
componentEditor.bindHelpToWidget = function(widget, helpText)
 widget:setOnMouseEnter(
    function(self)
      componentEditor.setHelpText(helpText)
    end
  )
  widget:setOnMouseLeave(
    function(self)
      componentEditor.clearHelpText()
    end
  )
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: Window componentEditor.addSeparator(Panel panel)
--| signature: Window componentEditor.addSeparator(Panel panel, integer height)
--| brief:
--|   Adds a horizontal separator.
--|
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
componentEditor.addSeparator = function(panel, height)
  local vSpace = 2
  if height ~= nil then
    vSpace = height/2
  end
  panel:addVSpacer(vSpace)
  local separator = panel:addPanel{ flags = "expand", size = { height = 1 }, proportion = 0 }
  separator:setBackgroundColour("dialogDarkestTint")
  panel:addVSpacer(vSpace)
end


------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: nil componentEditor.setChangeFunction(string changeID, function newFunc, table events)
--| brief:
--|   Adds a set of event handlers tagged by an changeID.
--|
--| environments: GlobalEnv
--| page: ComponentEditorAPI
componentEditor.currentChangeFunction = { }
componentEditor.setChangeFunction = function(changeID, newFunc, events)

  -- remove the old observer
  if componentEditor.currentChangeFunction then
    local details = componentEditor.currentChangeFunction[changeID]
    for _, v in ipairs(events) do
      unregisterEventHandler(v, details.func, true)  
    end
  end
  
  -- register the new observer
  componentEditor.currentChangeFunction[changeID] = {events = events,  func = newFunc }
  for _,v in ipairs(events) do
    registerEventHandler(v, newFunc)  
  end
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: nil componentEditor.clearChangeFunctions()
--| brief:
--|   Clears any tagged event handling functions.
--|
--| environments: GlobalEnv
--| page: ComponentEditorAPI
componentEditor.clearChangeFunctions = function()
  for _, details in componentEditor.currentChangeFunction do
    for _,v in ipairs(details.events) do
      local result = unregisterEventHandler(v, details.func)
      if not result then
        app.warning("failed to unregister previously registered event handler for " .. v)
      end
    end
  end  
  componentEditor.currentChangeFunction = { }
end

  ------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: nil componentEditor.reset()
--| brief:
--|   Clears and rebuilds the component editor.
--|
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
componentEditor.reset = function(storeState)
  componentEditor.clearChangeFunctions()
  local component = componentEditor.component
  local panel = componentEditor.panel
  if panel then
    local selectedSet = getSelectedAssetManagerAnimSet()

    if componentEditor.mainScrollPanel and (storeState == nil or storeState == true) then
      component.currentState = { }
      componentEditor.mainScrollPanel:storeState(component.currentState, true)
    end

    componentEditor.clearOnRigChanged()
    componentEditor.clearOnPhysicsRigChanged()
    panel:suspendLayout()
    panel:freeze()
    panel:clear()

    panel:beginVSizer{ flags = "expand", proportion = 1 }

      -- header panel
      if component and component.headerFunction then
        component.headerFunction(selectedSet, panel)
      end

      -- scroll panel
      componentEditor.mainScrollPanel = panel:addScrollPanel{name = "ScrollPanel", flags = "expand;vertical", proportion = 8}
      componentEditor.mainScrollPanel:beginVSizer{ flags = "expand" }

      if component and component.panelFunction then
        local status, message = pcall(component.panelFunction, selectedSet, componentEditor.mainScrollPanel)
        if not status then
          app.error(message)
          componentEditor.mainScrollPanel:addStaticText{text = "Error in component: " .. component.name}
        end
      elseif component then
      else
        componentEditor.mainScrollPanel:addStaticText{text = "Empty Selection"}
      end

      componentEditor.mainScrollPanel:endSizer()

      -- help panel
      componentEditor.helpPanel = panel:addPanel{name = "HelpPanel", flags = "expand", proportion = 2 }
      componentEditor.helpPanel:beginVSizer{ flags = "expand", proportion = 1 }
        componentEditor.helpText = componentEditor.helpPanel:addTextControl{ name = "TextBox", flags = "expand;noscroll", proportion = 1 }
        componentEditor.helpText:setReadOnly(true)
      componentEditor.helpPanel:endSizer()
    panel:endSizer()

    -- restore rollup settings etc.
    if component.currentState then
      componentEditor.mainScrollPanel:restoreState(component.currentState, true, false)
    end

    panel:resumeLayout()
    panel:rebuild()
  end
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: nil componentEditor.getCurrentComponentName()
--| brief:
--|   Returns the name of the currently selected component.
--|
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
componentEditor.getCurrentComponentName = function()
  return componentEditor.selectedComponentName
end

------------------------------------------------------------------------------------------------------------------------
componentEditor.setCurrentComponent = function(component)
  local oldComponent = componentEditor.component
  if oldComponent ~= component then
    if oldComponent and componentEditor.mainScrollPanel then
      oldComponent.currentState = { }
      componentEditor.mainScrollPanel:storeState(oldComponent.currentState, true)
    end
    
    componentEditor.component = component
    componentEditor.reset(false)
  end
end

------------------------------------------------------------------------------------------------------------------------
componentEditor.getComponentListItem = function(component)
  return listMapping[component]
end

------------------------------------------------------------------------------------------------------------------------
componentEditor.onRetargetingOffsetsCreated = function(component)
  for _, func in ipairs(componentEditor.onRetargetingOffsetsCreatedFunctions) do
    func()
  end
end

------------------------------------------------------------------------------------------------------------------------
componentEditor.onRetargetingOffsetsDeleted = function(component)
  for _, func in ipairs(componentEditor.onRetargetingOffsetsDeletedFunctions) do
    func()
  end
end

------------------------------------------------------------------------------------------------------------------------
componentEditor.onBodyMappingChanged = function(component)
  for _, func in ipairs(componentEditor.onBodyMappingChangedFunctions) do
    func()
  end
end

------------------------------------------------------------------------------------------------------------------------
componentEditor.onAnimationSetsChange = function(component)
  for _, func in ipairs(componentEditor.onAnimationSetsChangeFunctions) do
    func()
  end
end

------------------------------------------------------------------------------------------------------------------------
componentEditor.onAnimationSetsModified = function(component)
  for _, func in ipairs(componentEditor.onAnimationSetsModifiedFunctions) do
    func()
  end
end

------------------------------------------------------------------------------------------------------------------------
componentEditor.onRigChanged = function(component)
  for _, func in ipairs(componentEditor.onRigChangedFunctions) do
    func()
  end
end

------------------------------------------------------------------------------------------------------------------------
componentEditor.onPhysicsRigChanged = function(component)
  for _, func in ipairs(componentEditor.onPhysicsRigChangedFunctions) do
    func()
  end
end

----------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table componentEditor.createChangeContext()
--| brief:
--|   Creates a data change context and adds it to the attribute editors list of data
--|   change contexts.
--|
--| environments: GlobalEnv
--| page: ComponentEditorAPI
----------------------------------------------------------------------------------------------------------------------
componentEditor.createChangeContext = function()
  local context = createDataChangeEventContext()
  table.insert(componentEditor.dataChangeEventContexts, context)
  return context
end

----------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table componentEditor.clearChangeContexts()
--| brief:
--|   Clears any current change contexts that are registered with the attribute editor.
--|   Automatically called when the selection changes so change context are only kept for
--|   the current selection.
--|
--| environments: GlobalEnv
--| page: ComponentEditorAPI
----------------------------------------------------------------------------------------------------------------------
componentEditor.clearChangeContexts = function()
  for i, context in componentEditor.dataChangeEventContexts do
    deleteDataChangeEventContext(context)
  end
  componentEditor.dataChangeEventContexts = { }
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table componentEditor.addOnRetargetingOffsetsCreated(function callback)
--| brief: This registers a callback that will be processed when retarget offsets are created.
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
componentEditor.addOnRetargetingOffsetsCreated = function(func)
  table.insert(componentEditor.onRetargetingOffsetsCreatedFunctions, func)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table componentEditor.clearOnRetargetingOffsetsCreated()
--| brief: Clears any onRetargetingOffsetsCreated functions.
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
componentEditor.clearOnRetargetingOffsetsCreated = function()
  componentEditor.onRetargetingOffsetsCreatedFunctions = { }
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table componentEditor.addOnRetargetingOffsetsDeleted(function callback)
--| brief: This registers a callback that will be processed when retarget offsets are deleted.
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
componentEditor.addOnRetargetingOffsetsDeleted = function(func)
  table.insert(componentEditor.onRetargetingOffsetsDeletedFunctions, func)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table componentEditor.clearOnRetargetingOffsetsDeleted()
--| brief: Clears any onRetargetingOffsetsDeleted functions.
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
componentEditor.clearOnRetargetingOffsetsDeleted = function()
  componentEditor.onRetargetingOffsetsDeletedFunctions = { }
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table componentEditor.addOnBodyMappingChanged(function callback)
--| brief: This registers a callback that will be processed when the body mapping changes.
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
componentEditor.addOnBodyMappingChanged = function(func)
  table.insert(componentEditor.onBodyMappingChangedFunctions, func)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table componentEditor.clearOnBodyMappingChanged()
--| brief: Clears any onBodyMappingChanged functions.
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
componentEditor.clearOnBodyMappingChanged = function()
  componentEditor.onBodyMappingChangedFunctions = { }
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table componentEditor.addOnAnimationSetsChange(function callback)
--| brief: This registers a callback that will be processed when the body mapping changes.
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
componentEditor.addOnAnimationSetsChange = function(func)
  table.insert(componentEditor.onAnimationSetsChangeFunctions, func)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table componentEditor.clearOnAnimationSetsChange()
--| brief: Clears any onAnimationSetsChange functions.
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
componentEditor.clearOnAnimationSetsChange = function()
  componentEditor.onAnimationSetsChangeFunctions = { }
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table componentEditor.addOnAnimationSetsModified(function callback)
--| brief: This registers a callback that will be processed when the body mapping changes.
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
componentEditor.addOnAnimationSetsModified = function(func)
  table.insert(componentEditor.onAnimationSetsModifiedFunctions, func)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table componentEditor.clearOnAnimationSetsModified()
--| brief: Clears any onAnimationSetsModified functions.
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
componentEditor.clearOnAnimationSetsModified = function()
  componentEditor.onAnimationSetsModifiedFunctions = { }
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table componentEditor.addOnRigChanged(function callback)
--| brief: This registers a callback that will be processed when the rig changes.
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
componentEditor.addOnRigChanged = function(func)
  table.insert(componentEditor.onRigChangedFunctions, func)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table componentEditor.clearOnRigChanged()
--| brief: Clears any onRigChanged functions.
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
componentEditor.clearOnRigChanged = function()
  componentEditor.onRigChangedFunctions = { }
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table componentEditor.addOnPhysicsRigChanged(function callback)
--| brief: This registers a callback that will be processed when the physics rig changes.
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
componentEditor.addOnPhysicsRigChanged = function(func)
  table.insert(componentEditor.onPhysicsRigChangedFunctions, func)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table componentEditor.clearOnPhysicsRigChanged()
--| brief: Clears any onPhysicsRigChanged functions.
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
componentEditor.clearOnPhysicsRigChanged = function()
  componentEditor.onPhysicsRigChangedFunctions = { }
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table componentEditor.updateComponentValidity(object name)
--| brief: Updates a specific component's validity.
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
componentEditor.updateComponentValidity = function(name)
  if type(name) ~= "string" or string.len(name) == 0 then
    app.error("Could not update component, invalid component name")
    return
  end

  if componentEditor.panel then
    local selectedSet = getSelectedAssetManagerAnimSet()
    local selectedTypeName = anim.getAnimSetCharacterType(selectedSet)

    local mapping = listMapping[name]
    local component = components.get(name)
    if mapping and component then
      if component.validationFunction then
        -- update this component
        local isCurrentComponent = (name == componentEditor.component.name)

        local validity = component.validationFunction(selectedSet, isCurrentComponent)
        local currentComponentValidity = mapping:getImageIndex()
        local componentValidity = iconLookup[validity]

        if componentValidity ~= currentComponentValidity then
          mapping:setImageIndex(componentValidity)

          -- need to get the validity of all the components combined to set the heading icon
          if listRollupHeading then
            local summaryValidity = componentValidity
            for n, item in pairs(listMapping) do
              -- ignore the component we've just updated
              if n ~= name then
                local validity = item:getImageIndex()
                if validity > summaryValidity then
                  summaryValidity = validity
                end
              end
            end

            if summaryValidity == iconLookup["Ok"] then
              listRollupHeading:setIcon(nil)
            else
              listRollupHeading:setIcon(statusIcons[summaryValidity])
            end
          end
        end
      end
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Validation.
------------------------------------------------------------------------------------------------------------------------
local validateComponents = function()
  if componentEditor.panel then
    local summaryValidity = iconLookup["Ok"]
    local selectedSet = getSelectedAssetManagerAnimSet()
    local selectedTypeName = anim.getAnimSetCharacterType(selectedSet)
    for name, item in pairs(listMapping) do
      local component = components.get(name)
      local validity
      if component.validationFunction then
        local selectedSet = getSelectedAssetManagerAnimSet()
        local isCurrentComponent = name == componentEditor.component.name
        validity = iconLookup[component.validationFunction(selectedSet, isCurrentComponent)]
      else
        validity = iconLookup["Error"]
      end

      item:setImageIndex(validity)
      if validity > summaryValidity then
        summaryValidity = validity
      end
    end

    if listRollupHeading then
      if summaryValidity == iconLookup["Ok"] then
        listRollupHeading:setIcon(nil)
      else
        listRollupHeading:setIcon(statusIcons[summaryValidity])
      end
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table componentEditor.updateAllComponentsValidity()
--| brief: Updates a all components's validity.
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
componentEditor.updateAllComponentsValidity = function()
  validateComponents()
end

local ignoreSelectionChange = false
local componentListSelectionChanged = function(panel)
  if not ignoreSelectionChange then
    local selection = componentList:getSelectedItem()
    if selection then
      local component = components.get(selection:getValue())
      componentEditor.setCurrentComponent(component)
      componentEditor.selectedComponentName = component.name
    else
      componentEditor.setCurrentComponent(nil)
      componentEditor.selectedComponentName = nil
    end
  end
end

local rebuildComponentList = function()
  if componentList then
    local selectedSet = getSelectedAssetManagerAnimSet()
    local selectedTypeName = anim.getAnimSetCharacterType(selectedSet)
    local currentComponents = components.listForType(selectedTypeName)
    local lastSelectedComponent = componentEditor.selectedComponentName

    local rootItem = componentList:getRoot()
    rootItem:clearChildren()

    listMapping = { }
    local listItemToSelect = nil

    local orderedMap = { }
    local orderedComponents = { }
    for i, name in pairs(desiredOrder) do
      orderedMap[name] = i
      orderedComponents[i] = 0
    end

    -- copy the current components.
    -- items with dummy values will be in order,
    -- everything else will appear at the end.
    for _, n in ipairs(currentComponents) do
      local index = orderedMap[n.name]
      if orderedComponents[index] ~= nil then
        orderedComponents[index] = n
      else
        table.insert(orderedComponents, n)
      end
    end

    for _, n in pairs(orderedComponents) do
      -- the ordered list contains unused items set to 0. ignore those values.
      if type(n) == "table" then
        listMapping[n.name] = rootItem:addChild(n.name)

        if type(n.componentFunction) == "function" then
          n.componentFunction(selectedSet, listMapping[n.name])
        end

        if n.name == lastSelectedComponent then
          listItemToSelect = listMapping[n.name]
        end
      end
    end

    validateComponents()
    if listItemToSelect then
      local component = components.get(listItemToSelect:getValue())
      componentEditor.setCurrentComponent(components.get(lastSelectedComponent))
      ignoreSelectionChange = true
      componentList:selectItem(listItemToSelect)
      ignoreSelectionChange = false
    end
    
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Add the attribut editor.
------------------------------------------------------------------------------------------------------------------------
addComponentAttributeEditor = function(contextualPanel, forContext)
  componentEditor.clear()
  componentEditor.panel = contextualPanel:addPanel{ 
    name = "ComponentAttributeEditor", 
    caption = "Components", 
    forContext = forContext 
  }

  componentEditor.reset()
  rebuildComponentList()
  return componentEditor.panel
end

------------------------------------------------------------------------------------------------------------------------
-- Component section.
------------------------------------------------------------------------------------------------------------------------
addAssetManagerComponentSection = function(panel)
  componentEditor.clear()

  panel:beginVSizer{ flags = "expand"}
    panel:setBorder(0)
    listRollupHeading = panel:addRollupHeading{ 
      label = "Components", 
      proportion = 1, 
      flags = "advanced;expand;border;onlyShowIconWhenClosed", 
      name = "Components" 
    }

    componentList = panel:addTreeControl{
      name = "ComponentList",
      size = { height = -1 },
      numColumns = 2,
      flags = "sizeToContent;expand;hideRoot;hideExpansionBoxes",
    }
    
    listRollupHeading:setDisclosedWindow(componentList)
  panel:endSizer()

  componentList:setImageList(statusIcons, statusIcons[1]:getWidth(), statusIcons[1]:getHeight(), true)
  componentList:setOnSelectionChanged(componentListSelectionChanged)

  rebuildComponentList()
end

------------------------------------------------------------------------------------------------------------------------
canComponentEditorRedo = function()
  if type(componentEditor.component.undoFunctions.CanRedo) == "function" then
    return componentEditor.component.undoFunctions.CanRedo()
  end
  return false
end

------------------------------------------------------------------------------------------------------------------------
canComponentEditorUndo = function()
  if type(componentEditor.component.undoFunctions.CanUndo) == "function" then
    return componentEditor.component.undoFunctions.CanUndo()
  end
  return false
end

------------------------------------------------------------------------------------------------------------------------
componentEditorRedo = function()
  if type(componentEditor.component.undoFunctions.Redo) == "function" then
    componentEditor.component.undoFunctions.Redo()
  end
end

------------------------------------------------------------------------------------------------------------------------
componentEditorUndo = function()
  if type(componentEditor.component.undoFunctions.Undo) == "function" then
    componentEditor.component.undoFunctions.Undo()
  end
end

------------------------------------------------------------------------------------------------------------------------
if not componentEditor.registered then
  componentEditor.onRigChangedFunctions = { }
  componentEditor.onPhysicsRigChangedFunctions = { }
  componentEditor.onRetargetingOffsetsCreatedFunctions = { }
  componentEditor.onRetargetingOffsetsDeletedFunctions = { }
  componentEditor.onBodyMappingChangedFunctions = { }
  componentEditor.onAnimationSetsChangeFunctions = { }
  componentEditor.onAnimationSetsModifiedFunctions = { }

  local rigChangeHandler = function()
    validateComponents()
    safefunc(componentEditor.onRigChanged)
  end

  local retargetingOffsetsCreatedHandler = function()
    validateComponents()
    safefunc(componentEditor.onRetargetingOffsetsCreated)
  end

  local retargetingOffsetsDeletedHandler = function()
    validateComponents()
    safefunc(componentEditor.onRetargetingOffsetsDeleted)
  end

  local bodyMappingChangedHandler = function()
    validateComponents()
    safefunc(componentEditor.onBodyMappingChanged)
  end

  local animationSetsChangeHandler = function()
    validateComponents()
    safefunc(componentEditor.onAnimationSetsChange)
  end

  local animationSetsModifiedHandler = function()
    validateComponents()
    safefunc(componentEditor.onAnimationSetsModified)
  end

  local physicsRigChangeHandler = function()
    validateComponents()
    safefunc(componentEditor.onPhysicsRigChanged)
  end

  if not mcn.inCommandLineMode() then
    registerEventHandler("mcAnimationSetSelectionChange", rebuildComponentList)
    registerEventHandler("mcAnimationSetCharacterTypeModified", rebuildComponentList)
    registerEventHandler("mcRigChange", rigChangeHandler)
    registerEventHandler("mcPhysicsRigChange", physicsRigChangeHandler)
    registerEventHandler("mcFileCloseEnd", checkComponentPanelValid)
    registerEventHandler("mcPreferencesChanged", validateComponents)
    registerEventHandler("mcRetargetingOffsetsCreated", retargetingOffsetsCreatedHandler)
    registerEventHandler("mcRetargetingOffsetsDeleted", retargetingOffsetsDeletedHandler)
    registerEventHandler("mcBodyMappingChanged", bodyMappingChangedHandler)
    registerEventHandler("mcAnimationSetsChange", animationSetsChangeHandler)
    registerEventHandler("mcAnimationSetsModified", animationSetsModifiedHandler)
  end

  componentEditor.registered = true
end
