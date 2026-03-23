------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "luaAPI/utils.lua"

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: PAGE
--| name: AttributeEditorAPI
--| desc:
--|   Lua-scripted functions related to the attribute editor.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- Called the first time this script is sourced unless attributeEditor.registered is manually
-- set to false.
------------------------------------------------------------------------------------------------------------------------
if not attributeEditor.registered then
  attributeEditor = {
    displayInfo = { },
    dataChangeEventContexts = { },
    markupChangedFunctions = { },
    editorWindow = nil,
    helpWindow = nil
  }

  if not mcn.inCommandLineMode() then
    --------------------------------------------------------------------------------------------------------------------
    -- Having a wrapper handler function ensures this script can be rerun
    -- without registering duplicate event handlers.
    --------------------------------------------------------------------------------------------------------------------
    local selectionChangeEventHandler = function()
      safefunc(attributeEditor.onSelectionChange)
      -- prints a new line to make the loginfo easier to read
      safefunc(attributeEditor.log)
    end

    local markupChangedEventHandler = function()
      safefunc(attributeEditor.onMarkupChanged)
      -- prints a new line to make the loginfo easier to read
      safefunc(attributeEditor.log)
    end

    local animationFileChangedEventHandler = function(resourceSet)
      safefunc(attributeEditor.onAnimationFileChanged, resourceSet)
      -- prints a new line to make the loginfo easier to read
      safefunc(attributeEditor.log)
    end

    -- make sure all change contexts are cleared
    local fileCloseEndHandler = function()
      safefunc(attributeEditor.clearChangeContexts)
    end

    -- make sure all change contexts are cleared
    local referenceChangedHandler = function(nodePath)
      safefunc(attributeEditor.onReferenceChanged, nodePath)
    end

    registerEventHandler("mcSelectionChange", selectionChangeEventHandler)
    registerEventHandler("mcCurrentGraphChange", selectionChangeEventHandler)
    registerEventHandler("mcAnimationSetCreated", selectionChangeEventHandler)
    registerEventHandler("mcAnimationSetDestroyed", selectionChangeEventHandler)
    registerEventHandler("mcAnimationTakeChange", markupChangedEventHandler)
    registerEventHandler("mcAnimationFileChange", animationFileChangedEventHandler)
    registerEventHandler("mcFileCloseEnd", fileCloseEndHandler)
    registerEventHandler("mcReferenceChanged", referenceChangedHandler)

    --------------------------------------------------------------------------------------------------------------------
    -- Having a wrapper handler function ensures this script can be rerun
    -- without registering duplicate event handlers.
    --------------------------------------------------------------------------------------------------------------------
    local renameEventHandler = function(oldName, newName)
      attributeEditor.logEnterFunc("renameEventHandler()")

      local selection = ls("Selection")
      for i, object in selection do
        if object == newName then
          attributeEditor.editorWindow:addIdleCallback(
            function()
              -- force the attribute editor ui to update on idle.
              -- this is not recommended, but in a small number of cases
              -- is the only way to make the ui behave correctly.
              safefunc(attributeEditor.onNameChange)
            end
          )
          break
        end
      end

      attributeEditor.logExitFunc("renameEventHandler()")
    end

    registerEventHandler("mcNodeRenamed", renameEventHandler)
    registerEventHandler("mcEdgeRenamed", renameEventHandler)

    --------------------------------------------------------------------------------------------------------------------
    -- Having a wrapper handler function ensures this script can be rerun
    -- without registering duplicate event handlers.
    --------------------------------------------------------------------------------------------------------------------
    local conditionCreateDestroyHandler = function()
      safefunc(attributeEditor.log, "condition created or destroyed")
      safefunc(attributeEditor.onConditionCreateDestroy)
      -- prints a new line to make the loginfo easier to read
      safefunc(attributeEditor.log)
    end

    registerEventHandler("mcConditionCreated", conditionCreateDestroyHandler)
    registerEventHandler("mcConditionDestroyed", conditionCreateDestroyHandler)

    --------------------------------------------------------------------------------------------------------------------
    -- Having a wrapper handler function ensures this script can be rerun
    -- without registering duplicate event handlers.
    --------------------------------------------------------------------------------------------------------------------
    local flowEdgeCreateDestroyHandler = function()
      safefunc(attributeEditor.log, "edge created or destroyed")
      safefunc(attributeEditor.onFlowEdgeCreateDestroy)
      -- prints a new line to make the loginfo easier to read
      safefunc(attributeEditor.log)
    end

    registerEventHandler("mcFlowEdgeCreated", flowEdgeCreateDestroyHandler)
    registerEventHandler("mcFlowEdgeDestroyed", flowEdgeCreateDestroyHandler)

    --------------------------------------------------------------------------------------------------------------------
    local nodeCreateDestroyHandler = function()
      safefunc(attributeEditor.log, "node created or destroyed")
      safefunc(attributeEditor.onNodeCreateDestroy)
      -- prints a new line to make the loginfo easier to read
      safefunc(attributeEditor.log)
    end

    registerEventHandler("mcNodeCreated", nodeCreateDestroyHandler)
    registerEventHandler("mcNodeDestroyed", nodeCreateDestroyHandler)

    --------------------------------------------------------------------------------------------------------------------
    local nodeRenameHandler = function()
      safefunc(attributeEditor.log, "node renamed")
      safefunc(attributeEditor.onNodeRename)
      -- prints a new line to make the loginfo easier to read
      safefunc(attributeEditor.log)
    end

    registerEventHandler("mcNodeRenamed", nodeRenameHandler)

    --------------------------------------------------------------------------------------------------------------------
    local requestCreateDestroyHandler = function()
      safefunc(attributeEditor.log, "Request created or destroyed")
      safefunc(attributeEditor.onRequestCreateDestroy)
      -- prints a new line to make the loginfo easier to read
      safefunc(attributeEditor.log)
    end

    registerEventHandler("mcRequestCreated", requestCreateDestroyHandler)
    registerEventHandler("mcRequestDestroyed", requestCreateDestroyHandler)

    --------------------------------------------------------------------------------------------------------------------
    local requestRenameHandler = function()
      safefunc(attributeEditor.log, "Request renamed")
      safefunc(attributeEditor.onRequestRename)
      -- prints a new line to make the loginfo easier to read
      safefunc(attributeEditor.log)
    end

    registerEventHandler("mcRequestRenamed", requestRenameHandler)

    --------------------------------------------------------------------------------------------------------------------
    local edgeConnectionChangeHandler = function()
      safefunc(attributeEditor.log, "edge connection change")
      safefunc(attributeEditor.onEdgeConnectionChange)
      -- prints a new line to make the loginfo easier to read
      safefunc(attributeEditor.log)
    end

    registerEventHandler("mcEdgeConnectionChanged", edgeConnectionChangeHandler)

  end

  attributeEditor.registered = true
else
  -- called when this script is re-sourced.
end

------------------------------------------------------------------------------------------------------------------------
-- only setup the functions if morpheme:connect is not in command line mode
------------------------------------------------------------------------------------------------------------------------
if not mcn.inCommandLineMode() then
  ----------------------------------------------------------------------------------------------------------------------
  -- used to indent log messages
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.logDepthIndent = " "

  -- if this script was rerun this preserves whether logging is enabled
  local previousLoggingEnabled = safefunc(attributeEditor.loggingEnabled)

  -- local variable is quicker to access than a global variable when performing enabled
  -- checks in the log functions.
  local loggingEnabled = false

  -- if the previousLoggingEnabled is not nil then this script has been run before
  -- so reset loggingEnabled to it's previous value.
  if previousLoggingEnabled then
    loggingEnabled = previousLoggingEnabled
  end

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: nil attributeEditor.enableLogging()
  --| brief:
  --|   Enables attribute editor logging to the script output window for any messages
  --|   output by the attributeEditor.log function.
  --|
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.enableLogging = function()
    loggingEnabled = true
  end

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: boolean attributeEditor.loggingEnabled()
  --| brief:
  --|   Is attribute editor logging enabled.
  --|
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.loggingEnabled = function()
    return loggingEnabled
  end

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: nil attributeEditor.disableLogging()
  --| brief:
  --|   Disables attribute editor logging.
  --|
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.disableLogging = function()
    loggingEnabled = false
  end

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table attributeEditor.log(string message, ...)
  --| brief:
  --|   A printf style logging function for attribute editor code to help debug it.
  --|
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.log = function(msg, ...)
    if loggingEnabled then
      if type(msg) == "string" and string.len(msg) > 0 then
        local message = string.format(msg, unpack(arg))
        local output = string.format("AE:%s%s", attributeEditor.logDepthIndent, message)
        print(output)
      elseif string.len(attributeEditor.logDepthIndent) == 1 then
        print("")
      end
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table attributeEditor.logEnterFunc(string funcname)
  --| brief:
  --|   Logs the entering of a function and also increases the log indent until a matching
  --|   attributeEditor.logExitFunc(funcname) is called. Logging is enabled by setting
  --|   attributeEditor.logDepthIndent = true.
  --|
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.logEnterFunc = function(funcname)
    if loggingEnabled then
      attributeEditor.log("entering %s", funcname)
      attributeEditor.logDepthIndent = string.format("%s ", attributeEditor.logDepthIndent)
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table attributeEditor.logExitFunc(string funcname)
  --| brief:
  --|   Logs the exit of a function and also decreases the log indent, should be used in
  --|   conjunction with a matching attributeEditor.logEnterFunc(funcname) is called.
  --|   Logging is enabled by setting attributeEditor.logDepthIndent = true.
  --|
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.logExitFunc = function(funcname)
    if loggingEnabled then
      attributeEditor.logDepthIndent =
        string.sub(attributeEditor.logDepthIndent, 0, string.len(attributeEditor.logDepthIndent) - 1)
      attributeEditor.log("exiting %s", funcname)
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- the name of the currently active object displayed in the attribute editor, used
  -- in conjunction with renaming.
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.activeObjectName = ""

  ----------------------------------------------------------------------------------------------------------------------
  -- the type of the currently active object displayed in the attribute editor used
  -- for restoring the state of the ui for specific object types
  -- note: the type will be an empty string for an empty selection or a multi type selection
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.activeObjectType = ""

  ----------------------------------------------------------------------------------------------------------------------
  -- table that preserves the view state of the attribute editor for each type of object.
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.objectTypeUIState = { }

  ----------------------------------------------------------------------------------------------------------------------
  -- table used to store the currently used/displayed attributes so the editor knows
  -- which attributes to display at the end.
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.usedAttributes = { }

  ----------------------------------------------------------------------------------------------------------------------
  -- table used to store the file changed functions
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.animationFileChangedFunctions = { }

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table attributeEditor.addOnMarkupChanged(function callback)
  --| brief: This registers a callback that will be processed when markup changes.
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.addAnimationFileChanged = function(onFileChanged)
    table.insert(attributeEditor.animationFileChangedFunctions, onFileChanged)
  end

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: attributeEditor.clearOnAnimationFileChanged()
  --| brief: Clears any onAnimationFileChanged functions.
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.clearOnAnimationFileChanged = function()
    attributeEditor.animationFileChangedFunctions = { }
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- table used to store the file changed functions
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.requestCreateDestroyFunctions = { }

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table attributeEditor.addRequestCreateDestroy(function callback)
  --| brief: This registers a callback that will be processed when a request is created or destroyed.
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.addRequestCreateDestroy = function(onFileChanged)
    table.insert(attributeEditor.requestCreateDestroyFunctions, onFileChanged)
  end

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: attributeEditor.clearOnRequestCreateDestroy()
  --| brief: Clears any onRequestCreateDestroy functions.
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.clearOnRequestCreateDestroy = function()
    attributeEditor.requestCreateDestroyFunctions = { }
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- table used to store the file changed functions
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.requestRenameFunctions = { }

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table attributeEditor.addRequestRename(function callback)
  --| brief: This registers a callback that will be processed when a request name changes.
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.addRequestRename = function(onFileChanged)
    table.insert(attributeEditor.requestRenameFunctions, onFileChanged)
  end

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: attributeEditor.clearOnRequestRename()
  --| brief: Clears any onRequestRename functions.
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.clearOnRequestRename = function()
    attributeEditor.requestRenameFunctions = { }
  end


  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table attributeEditor.clearOnMarkupChanged()
  --| brief: Clears any onMarkupChanged functions.
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.clearOnMarkupChanged = function()
    attributeEditor.markupChangedFunctions = { }
  end

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table attributeEditor.createChangeContext()
  --| brief:
  --|   Creates a data change context and adds it to the attribute editors list of data
  --|   change contexts.
  --|
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.createChangeContext = function()
    local context = createDataChangeEventContext()
    table.insert(attributeEditor.dataChangeEventContexts, context)
    return context
  end

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table attributeEditor.clearChangeContexts()
  --| brief:
  --|   Clears any current change contexts that are registered with the attribute editor.
  --|   Automatically called when the selection changes so change context are only kept for
  --|   the current selection.
  --|
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.clearChangeContexts = function()
    for i, context in attributeEditor.dataChangeEventContexts do
      deleteDataChangeEventContext(context)
    end
    attributeEditor.dataChangeEventContexts = { }
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- returns a table of type tables each containing the type and the objects of that type
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.getTypeTable = function(objects)
    local uniqueTypes = { }
    local typeTable = { }

    for i, nodeName in ipairs(objects) do
      local nodeType = getType(nodeName)
      local tableEntry = uniqueTypes[nodeType]
      if not tableEntry then
        tableEntry = { type = nodeType, nodes = { } }
        uniqueTypes[nodeType] = tableEntry
        table.insert(typeTable, tableEntry)
      end
      table.insert(tableEntry.nodes, nodeName)
    end

    return typeTable
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- this function is called an animation changes
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.onAnimationFileChanged = function(resourceSet, b)
    for i, fileChangedChanged in ipairs(attributeEditor.animationFileChangedFunctions) do
      fileChangedChanged(resourceSet)
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- this function is called when a reference file changes
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.onReferenceChanged = function(resourceSet, thePath)
    attributeEditor.onSelectionChange()
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- this function is called when the markup changes
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.onMarkupChanged = function()
    for i, markupChangedChanged in ipairs(attributeEditor.markupChangedFunctions) do
      markupChangedChanged()
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- this function is called when a request is created or destroyed
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.onRequestCreateDestroy = function()
    for i, func in ipairs(attributeEditor.requestCreateDestroyFunctions) do
      func()
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- this function is called when a request is renamed
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.onRequestRename = function()
    for i, func in ipairs(attributeEditor.requestRenameFunctions) do
      func()
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- this function is called when the network selection changes, the attribute panel
  -- is cleared and rebuilt for the new selection
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.onSelectionChange = function()
    attributeEditor.logEnterFunc("attributeEditor.onSelectionChange()")

    attributeEditor.log("clearing all existing attribute editor change contexts")
    attributeEditor.clearChangeContexts()
    attributeEditor.clearOnMarkupChanged()
    attributeEditor.clearOnAnimationFileChanged()
    attributeEditor.clearOnRequestCreateDestroy()
    attributeEditor.clearOnRequestRename()

    if not attributeEditor.editorWindow then
      attributeEditor.log("could not find window: \"attributeEditor.editorWindow\"")
      attributeEditor.logExitFunc("attributeEditor.onSelectionChange()")
      return
    end

    local scrollPanel = attributeEditor.editorWindow:getChild("AttributePanel")
    local panel = scrollPanel:getChild("AttributeSwitchablePanel")
    if not panel then
      attributeEditor.log("could not find child: \"AttributeSwitchablePanel\"")
      attributeEditor.logExitFunc("attributeEditor.onSelectionChange()")
      return
    end

    panel:suspendLayout()
    attributeEditor.log("storing ui state for node type \"%s\"", attributeEditor.activeObjectType)
    panel:storeState(attributeEditor.objectTypeUIState, true)

    attributeEditor.log("setting attributeEditor.activeObjectName to \"\"")
    attributeEditor.activeObjectName = ""

    attributeEditor.log("setting attributeEditor.onConditionCreateDestroy to nil")
    attributeEditor.onConditionCreateDestroy = nil

    attributeEditor.log("setting attributeEditor.onFlowEdgeCreateDestroy to nil")
    attributeEditor.onFlowEdgeCreateDestroy = nil

    attributeEditor.log("setting attributeEditor.onNodeCreateDestroy to nil")
    attributeEditor.onNodeCreateDestroy = nil

    attributeEditor.log("setting attributeEditor.onNodeRename to nil")
    attributeEditor.onNodeRename = nil

    attributeEditor.log("setting attributeEditor.onEdgeConnectionChange to nil")
    attributeEditor.onEdgeConnectionChange = nil

    attributeEditor.log("setting attributeEditor.activeObjectType to \"\"")
    attributeEditor.activeObjectType = ""

    attributeEditor.log("clearing attributeEditor.usedAttributes table")
    attributeEditor.usedAttributes = { }

    local selection = ls("Selection")
    local selectionCount = table.getn(selection)

    attributeEditor.log("freezing attributeEditor.editorWindow")
    attributeEditor.editorWindow:freeze()
    attributeEditor.log("clearing attributeEditor.editorWindow")

    local encounteredError = false

    -- Always clear the subpanel to clear out any old node attribute widgets...
    local subPanel = panel:getChild("Selection")
    if subPanel then
      subPanel:clear()
    end

    -- respond to number of elements in selection
    if selectionCount == 0 then
      -- respond to an empty selection
      attributeEditor.log("handling empty selection")

      local emptySelPanel = panel:getChild("EmptySelection")
      if not emptySelPanel then
        emptySelPanel = panel:addPanel{
          name = "EmptySelection",
          flags = "expand",
          proportion = 1
        }
      end

      safefunc(attributeEditor.doEmptySelection, emptySelPanel)
      panel:setCurrentPanel(emptySelPanel)

    else
      local typeTable = attributeEditor.getTypeTable(selection)

      if not subPanel then
        subPanel = panel:addPanel{
          name = "Selection",
          flags = "expand",
          proportion = 1
        }
      end

      if table.getn(typeTable) == 1 then
        -- only one type selected overall, select as single type
        attributeEditor.log("handling single type selection")

        local nodeType = typeTable[1].type
        attributeEditor.log("setting attributeEditor.activeObjectType to \"%s\"", nodeType)
        attributeEditor.activeObjectType = nodeType

        local status, message = pcall(safefunc, attributeEditor.doSingleTypeSelection, subPanel, selection)
        -- if there was an error in a script handle it gracefully and clear the panel of all controls
        if not status then
          app.error(message)
          attributeEditor.log("error handling single type selection: %s", message)
          encounteredError = true
        end
      else
        -- multi-select many types
        attributeEditor.log("handling multiple type selection")

        local status, message = pcall(safefunc, attributeEditor.doMultiTypeSelection, subPanel, selection, typeTable)
        -- if there was an error in a script handle it gracefully and clear the panel of all controls
        if not status then
          app.error(message)
          attributeEditor.log("error handling multiple type selection: %s", message)
          encounteredError = true
        end
      end

      panel:setCurrentPanel(subPanel)

    end

    if not encounteredError then

      -- attributeEditor.editorWindow:rebuild() checks and calls thaw if necessary
      attributeEditor.log("rebuilding attributeEditor.editorWindow")
      local panelState = attributeEditor.objectTypeUIState
      if type(panelState) == "table" then
        attributeEditor.log("restoring ui state for node type \"%s\"", attributeEditor.activeObjectType)
        panel:restoreState(panelState, true, false)
      end

      panel:resumeLayout()
      attributeEditor.editorWindow:rebuild()

      attributeEditor.clearHelpText()
    else
      attributeEditor.reset()
    end

    attributeEditor.logExitFunc("attributeEditor.onSelectionChange()")
  end

  attributeEditor.onNameChange = function()
    attributeEditor.logEnterFunc("attributeEditor.onNameChange()")

    if not attributeEditor.editorWindow then
      attributeEditor.log("could not find window: \"attributeEditor.editorWindow\"")
      attributeEditor.logExitFunc("attributeEditor.rebuild()")
      return
    end

    -- save and restore the focused Window - we do this by path as the
    -- actual window will be destroyed
    local focusedChild = attributeEditor.editorWindow:findFocusedChild();

    if focusedChild == null then
      attributeEditor.onSelectionChange()
    else
      local focusedPath = focusedChild:getPath()
      attributeEditor.onSelectionChange()
      focusedChild = ui.getWindow(focusedPath)
      if focusedChild ~= null then
        focusedChild:setFocus{ navigation = true }
      end
    end

    attributeEditor.logExitFunc("attributeEditor.onNameChange()")
  end

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table attributeEditor.suspendUpdates()
  --| brief:
  --|   Prevents the attribute editor from updating while ui is rebuilt or changed. This
  --|   helps to prevent flickering when many controls are changed causing resizing. Must be
  --|   called as pair with attributeEditor.resumeUpdates().
  --|
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.suspendUpdates = function()
    attributeEditor.logEnterFunc("attributeEditor.suspendUpdates()")
    local editor = attributeEditor.editorWindow
    editor:freeze()
    attributeEditor.logExitFunc("attributeEditor.suspendUpdates()")
  end

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table attributeEditor.resumeUpdates()
  --| brief:
  --|   Causes the attribute editor to continue updating after a call to attributeEditor.resumeUpdates.
  --|   This helps to prevent flickering when many controls are changed causing resizing. Must be
  --|   called as pair with attributeEditor.suspendUpdates().
  --|
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.resumeUpdates = function()
  attributeEditor.logEnterFunc("attributeEditor.resumeUpdates()")
    local editor = attributeEditor.editorWindow
    editor:rebuild()
    attributeEditor.logExitFunc("attributeEditor.resumeUpdates()")
  end

------------------------------------------------------------------------------------------------------------------------
  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: nil attributeEditor.setHelpText(string helpText)
  --| brief:
  --|   Sets the help text displayed in the bottom of the attribute editor to the
  --|   string specified.
  --|
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.setHelpText = function(string)
    if attributeEditor.helpWindow then
      local helptext = attributeEditor.helpWindow:getChild("TextBox")
      if helptext then
        if string then
          helptext:setValue(string)
        else
          helptext:setValue("")
        end
      end
    end
  end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: textWidget attributeEditor.addStaticTextWithHelp(Panel panel, string staticText, string helpText)
--| brief:
--|   Adds static text to the given panel with onMouseEnter and onMouseLeave functions to display the specified help text.
--|
--| environments: GlobalEnv
--| page: AttributeEditorAPI
------------------------------------------------------------------------------------------------------------------------
  attributeEditor.addStaticTextWithHelp = function(panel, staticText, helpText)
    local spacingText = panel:addStaticText{
      text = staticText,
      onMouseEnter = function()
        attributeEditor.setHelpText(helpText)
      end,
      onMouseLeave = function()
        attributeEditor.clearHelpText()
      end
    }

    return spacingText
  end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table attributeEditor.clearHelpText()
--| brief:
--|   Clears the text displayed at the bottom of the attribute editor.
--|
--| environments: GlobalEnv
--| page: AttributeEditorAPI
------------------------------------------------------------------------------------------------------------------------
  attributeEditor.clearHelpText = function()
    if attributeEditor.helpWindow then
      local helptext = attributeEditor.helpWindow:getChild("TextBox")
      if helptext then
        helptext:setValue("")
      end
    end
  end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table attributeEditor.registerDisplayInfo(string type, table displayInfo)
--| brief:
--|   Registers a new display info table for a specific manifest type. Used for custom
--|   attribute editor display. Each entry into the displayInfo table should contain
--|   a title, a table of usedAttributes and a displayFunction.
--|
--| environments: GlobalEnv
--| page: AttributeEditorAPI
------------------------------------------------------------------------------------------------------------------------
  attributeEditor.registerDisplayInfo = function(type, info)
    attributeEditor.displayInfo[type] = info
  end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table attributeEditor.unregisterDisplayInfo(string type)
--| brief:
--|   Unregisters a displayInfo entry for a specific type.
--|
--| environments: GlobalEnv
--| page: AttributeEditorAPI
------------------------------------------------------------------------------------------------------------------------
  attributeEditor.unregisterDisplayInfo = function(type)
    attributeEditor.displayInfo[type] = nil
  end

----------------------------------------------------------------------------------------------------------------------
-- display func for attributeEditor.addCompoundAttributeSection attribute widget
----------------------------------------------------------------------------------------------------------------------
  local animSetDisplayFunc = function(panel, objects, attributes, set)
    attributeEditor.logEnterFunc("attributeEditor.addCompoundAttributeSection displayFunc")

    panel:setBorder(1)

    local showLabels = table.getn(attributes) > 1

    if showLabels then
      attributeEditor.log("panel:beginFlexGridSizer")
      panel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      panel:setFlexGridColumnExpandable(2)
    else
      attributeEditor.log("panel:beginVSizer")
      panel:beginVSizer{ flags = "expand", proportion = 1 }
    end

      for i, attribute in attributes do
        attributeEditor.log("adding control for attribute \"%s\"", attribute)

        if showLabels then
          local name = utils.getDisplayString(getAttributeDisplayName(objects[1], attribute))
          attributeEditor.addAttributeLabel(panel, name, objects, attribute)
        end
        attributeEditor.addAttributeWidget(panel, attribute, objects, set)
      end

    attributeEditor.log("panel:endSizer")
    panel:endSizer()

    attributeEditor.logExitFunc("attributeEditor.addCompoundAttributeSection displayFunc")
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- display func for attributeEditor.addCompoundAttributeSection attribute widget
  ----------------------------------------------------------------------------------------------------------------------
  local animSetDisplayWithLabelsFunc = function(panel, objects, attributes, set)
    attributeEditor.logEnterFunc("attributeEditor.addCompoundAttributeSection displayFunc")

    panel:setBorder(1)

    attributeEditor.log("panel:beginFlexGridSizer")
    panel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      panel:setFlexGridColumnExpandable(2)

      for i, attribute in attributes do
        attributeEditor.log("adding control for attribute \"%s\"", attribute)

        local label = utils.getDisplayString(getAttributeDisplayName(objects[1], attribute))
        local helpText = getAttributeHelpText(objects[1], attribute)
        
        local staticText = panel:addStaticText{
          name = string.format("%sText", attribute),
          text = label,
          flags = "expand",
          align = "right",
          onMouseEnter = function()
            attributeEditor.setHelpText(helpText)
          end,
          onMouseLeave = function()
            attributeEditor.clearHelpText()
          end
        }
        
        attributeEditor.addAttributeWidget(panel, attribute, objects, set)
      end

    attributeEditor.log("panel:endSizer")
    panel:endSizer()

    attributeEditor.logExitFunc("attributeEditor.addCompoundAttributeSection displayFunc")
  end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: Rollup attributeEditor.addSimpleAttributeSection(RollupContainer container, string title, table objects, table attrs)
--| brief:
--|   Adds a simple section of attributes with the title specified and the names. Returns
--|   the newly added rollup section.
--|
--| environments: GlobalEnv
--| page: AttributeEditorAPI
------------------------------------------------------------------------------------------------------------------------
  attributeEditor.addSimpleAttributeSection = function(rollContainer, title, objects, attrs, set)
    attributeEditor.logEnterFunc("attributeEditor.addSimpleAttributeSection")

    if rollContainer == nil then
      attributeEditor.log("error: rollContainer == nil")
      attributeEditor.logExitFunc("attributeEditor.addSimpleAttributeSection")
      return
    end

    if type(rollContainer.addRollup) ~= "function" then
      attributeEditor.log(
        "error: type(rollContainer.addRollup) is \"%s\" expected \"function\"",
        type(rollContainer.addRollup)
      )
      attributeEditor.logExitFunc("attributeEditor.addSimpleAttributeSection")
      return
    end

    if type(title) ~= "string" then
      attributeEditor.log("error: type(title) is \"%s\" expected \"string\"", type(title))
      attributeEditor.logExitFunc("attributeEditor.addSimpleAttributeSection")
      return
    end

    if type(objects) ~= "table" then
      attributeEditor.log("error: type(objects) is \"%s\" expected \"table\"", type(objects))
      attributeEditor.logExitFunc("attributeEditor.addSimpleAttributeSection")
      return
    end

    if type(attrs) ~= "table" then
      attributeEditor.log("error: type(attrs) is \"%s\" expected \"table\"", type(attrs))
      attributeEditor.logExitFunc("attributeEditor.addSimpleAttributeSection")
      return
    end

    if type(set) ~= "string" then
      local sets = listAnimSets()
      set = sets[1]
    end

    local objectCount = table.getn(objects)
    if objectCount > 0 and table.getn(attrs) > 0 then
      attributeEditor.log("adding simple section \"%s\"", title)

      attributeEditor.log("rollContainer:addRollup")
      local rollup = rollContainer:addRollup{ label = title, flags = "mainSection", name = utils.getIdentifier(title) }
      local rollPanel = rollup:getPanel()

      attributeEditor.log("rollPanel:beginHSizer")
      rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
        rollPanel:addHSpacer(6)

        -- Add attributes into flex grid sizer
        local standardAttrs = { }
        local animSetAttrs = { }
        for i, attr in ipairs(attrs) do
          local _, info = getAttributeType(objects[1], attr)
          if info.isSet then
            table.insert(animSetAttrs, attr)
          else
            table.insert(standardAttrs, attr)
          end
        end

        rollPanel:setBorder(1)
        if table.getn(standardAttrs) > 0 then
          attributeEditor.log("rollPanel:beginFlexGridSizer")
          rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
            rollPanel:setFlexGridColumnExpandable(2)

            for _, attr in ipairs(standardAttrs) do
              attributeEditor.log("adding control for attribute \"%s\"", attr)
              local name = utils.getDisplayString(getAttributeDisplayName(objects[1], attr))
              attributeEditor.addAttributeLabel(rollPanel, name, objects, attr)
              attributeEditor.addAttributeWidget(rollPanel, attr, objects)
            end

          attributeEditor.log("rollPanel:endSizer")
          rollPanel:endSizer()
        end

        if table.getn(animSetAttrs) > 0 then
          rollPanel:beginVSizer{ flags = "expand", proportion = 1 }
          local attrPaths = { }
          table.setn(attrPaths, objectCount)
          for _, attr in ipairs(animSetAttrs) do
            attributeEditor.log("adding control for attribute \"%s\"", attr)
            for i, object in ipairs(objects) do
              attrPaths[i] = string.format("%s.%s", object, attr)
            end

            rollPanel:addAnimationSetWidget{
              attributes = attrPaths,
              flags = "expand",
              proportion = 0,
              displayFunc = animSetDisplayWithLabelsFunc,
            }
          end
          rollPanel:endSizer()
        end

      attributeEditor.log("rollPanel:endSizer")
      rollPanel:endSizer()

      attributeEditor.logExitFunc("attributeEditor.addSimpleAttributeSection")
      return rollup
    end

    attributeEditor.logExitFunc("attributeEditor.addSimpleAttributeSection")
    return
  end
  
  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: Rollup attributeEditor.addCompoundAttributeSection(RollupContainer container, string title, table objects, table attrs)
  --| brief:
  --|   Adds a compound anim set section of attributes with the title specified and the names. Returns
  --|   the newly added rollup section.
  --|
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.addCompoundAttributeSection = function(rollContainer, title, objects, attrs)
    attributeEditor.logEnterFunc("attributeEditor.addCompoundAttributeSection")

    if rollContainer == nil then
      attributeEditor.log("error: rollContainer == nil")
      attributeEditor.logExitFunc("attributeEditor.addCompoundAttributeSection")
      return
    end

    if type(rollContainer.addRollup) ~= "function" then
      attributeEditor.log(
        "error: type(rollContainer.addRollup) is \"%s\" expected \"function\"",
        type(rollContainer.addRollup)
      )
      attributeEditor.logExitFunc("attributeEditor.addCompoundAttributeSection")
      return
    end

    if type(title) ~= "string" then
      attributeEditor.log("error: type(title) is \"%s\" expected \"string\"", type(title))
      attributeEditor.logExitFunc("attributeEditor.addCompoundAttributeSection")
      return
    end

    if type(objects) ~= "table" then
      attributeEditor.log("error: type(objects) is \"%s\" expected \"table\"", type(objects))
      attributeEditor.logExitFunc("attributeEditor.addCompoundAttributeSection")
      return
    end

    if type(attrs) ~= "table" then
      attributeEditor.log("error: type(attrs) is \"%s\" expected \"table\"", type(attrs))
      attributeEditor.logExitFunc("attributeEditor.addCompoundAttributeSection")
      return
    end

    local objectCount = table.getn(objects)
    if objectCount > 0 and table.getn(attrs) > 0 then
      attributeEditor.log("adding compound section \"%s\"", title)

      attributeEditor.log("rollContainer:addRollup")
      local rollup = rollContainer:addRollup{ label = title, flags = "mainSection", name = "CompoundAttributeSection" }
      local rollPanel = rollup:getPanel()

      attributeEditor.log("rollPanel:beginVSizer")
      rollPanel:beginVSizer{ flags = "expand" }

        local attrPaths = { }
        for i, object in ipairs(objects) do
          for j, attr in ipairs(attrs) do
            local current = string.format("%s.%s", object, attr)
            table.insert(attrPaths, current)
            attributeEditor.log("adding \"%s\" to attribute list", current)
          end
        end

        attributeEditor.log("rollPanel:addAttributeWidget")
        rollPanel:addAnimationSetWidget{
          attributes = attrPaths,
          displayFunc = animSetDisplayFunc,
          flags = "expand",
          proportion = 1
        }

      attributeEditor.log("rollPanel:endSizer")
      rollPanel:endSizer()

      attributeEditor.logExitFunc("attributeEditor.addCompoundAttributeSection")
      return rollup
    end

    attributeEditor.logExitFunc("attributeEditor.addCompoundAttributeSection")
    return
  end

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: Rollup attributeEditor.addCustomAttributeSection(RollupContainer container, string title, table objects, table attrs)
  --| brief:
  --|   Adds a simple section of custom attributes with the title specified and the names. Returns
  --|   the newly added rollup section.
  --|
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.addCustomAttributeSection = function(rollContainer, title, objects, attrs)
    attributeEditor.logEnterFunc("attributeEditor.addCustomAttributeSection")

    if rollContainer == nil then
      attributeEditor.log("error: rollContainer == nil")
      attributeEditor.logExitFunc("attributeEditor.addCustomAttributeSection")
      return
    end

    if type(rollContainer.addRollup) ~= "function" then
      attributeEditor.log(
        "error: type(rollContainer.addRollup) is \"%s\" expected \"function\"",
        type(rollContainer.addRollup)
      )
      attributeEditor.logExitFunc("attributeEditor.addCustomAttributeSection")
      return
    end

    if type(title) ~= "string" then
      attributeEditor.log("error: type(title) is \"%s\" expected \"string\"", type(title))
      attributeEditor.logExitFunc("attributeEditor.addCustomAttributeSection")
      return
    end

    if type(objects) ~= "table" then
      attributeEditor.log("error: type(objects) is \"%s\" expected \"table\"", type(objects))
      attributeEditor.logExitFunc("attributeEditor.addCustomAttributeSection")
      return
    end

    if type(attrs) ~= "table" then
      attributeEditor.log("error: type(attrs) is \"%s\" expected \"table\"", type(attrs))
      attributeEditor.logExitFunc("attributeEditor.addCustomAttributeSection")
      return
    end

    local objectCount = table.getn(objects)
    if objectCount > 0 then
      attributeEditor.log("adding custom attribute section \"%s\"", title)

      attributeEditor.log("rollContainer:addRollup")
      local rollup = rollContainer:addRollup{ label = title, flags = "mainSection", name = "CustomAttributeSection" }
      local rollPanel = rollup:getPanel()

      local rebuildCustomAttributePanel = function(rollPanel, objects, attrs)
        attributeEditor.logEnterFunc("rebuildCustomAttributePanel")

        local setAttrs = { }
        local nonSetAttrs = { }
        for i, attr in ipairs(attrs) do
          local path = string.format("%s.%s", objects[1], attr)
          local type, info = getAttributeType(path)
          if info.isSet then
            table.insert(setAttrs, attr)
          else
            table.insert(nonSetAttrs, attr)
          end
        end

        attributeEditor.log("rollPanel:beginVSizer")
        rollPanel:beginVSizer{ flags = "expand", proportion = 1 }

          attributeEditor.log("adding custom properties button")
          local editButton = rollPanel:addButton{ label = "Edit properties", flags = "left", onClick = showEditAttributeDialog }
          if containsReference(objects) then
            editButton:enable(false)
          end

          attributeEditor.log("adding non animation set attributes")
          if table.getn(nonSetAttrs) > 0 then

            attributeEditor.log("rollPanel:beginHSizer")
            rollPanel:beginHSizer{ flags = "expand" }

              rollPanel:addHSpacer(6)
              rollPanel:setBorder(1)

              attributeEditor.log("rollPanel:beginFlexGridSizer")
              rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
                rollPanel:setFlexGridColumnExpandable(2)

                -- Add attributes into flex grid sizer
                local attrPaths = { }
                table.setn(attrPaths, objectCount)
                for i, attr in ipairs(nonSetAttrs) do

                  attributeEditor.log("adding control for attribute \"%s\"", attr)
                  local name = utils.getDisplayString(attr)

                  attributeEditor.log("rollPanel:addStaticText")
                  rollPanel:addStaticText{ text = name, align = "right", flags = "expand" }

                  for j = 1, objectCount do
                    attrPaths[j] = string.format("%s.%s", objects[j], attr)
                    attributeEditor.log("adding \"%s\" to attribute list", attrPaths[j])
                  end

                  attributeEditor.log("rollPanel:addAttributeWidget")
                  rollPanel:addAttributeWidget{ attributes = attrPaths, flags = "expand", proportion = 1 }

                end

              attributeEditor.log("rollPanel:endFlexGridSizer")
              rollPanel:endSizer()

            attributeEditor.log("rollPanel:endHSizer")
            rollPanel:endSizer()

          end

          attributeEditor.log("adding animation set attributes")
          if table.getn(setAttrs) > 0 then

            attributeEditor.log("rollPanel:beginHSizer")
            rollPanel:beginHSizer{ flags = "expand" }

              rollPanel:addHSpacer(6)
              rollPanel:setBorder(1)

              local setAttrPaths = { }
              for i, object in ipairs(objects) do
                for j, attr in ipairs(setAttrs) do
                  local current = string.format("%s.%s", object, attr)
                  table.insert(setAttrPaths, current)
                  attributeEditor.log("adding \"%s\" to attribute list", current)
                end
              end

              attributeEditor.log("rollPanel:addAttributeWidget")
              rollPanel:addAnimationSetWidget{
                attributes = setAttrPaths,
                displayFunc = animSetDisplayWithLabelsFunc,
                flags = "expand",
                proportion = 1
              }

            attributeEditor.log("rollPanel:endHSizer")
            rollPanel:endSizer()

          end

        attributeEditor.log("rollPanel:endVSizer")
        rollPanel:endSizer()

        attributeEditor.logExitFunc("rebuildCustomAttributePanel")
      end

      local changeContext = attributeEditor.createChangeContext()
      changeContext:setObjects(objects)
      for i, attr in ipairs(attrs) do
        changeContext:addAttributeChangeEvent(attr)
      end

      -- build a table of common custom attributes
      local rebuildCommonNonManifestAttributes = function(objects)
        local attrs = listAttributes(objects[1])
        local nonManifestAttributes = { }
        for i, attr in pairs(attrs) do
          if not attributeEditor.usedAttributes[attr] then
            if not isManifestAttribute(string.format("%s.%s", objects[1], attr)) then
              table.insert(nonManifestAttributes, attr)
            end
          end
        end
        local commonNonManifestAttributes = getCommonAttributes(objects, nonManifestAttributes)
        table.sort(commonNonManifestAttributes)
        return commonNonManifestAttributes
      end

      -- rebuild the custom attribute panel whenever attributes are added
      changeContext:setAttributeAddedHandler(
        function(object, attr)
          attributeEditor.logEnterFunc("changeContext attributeAddedHandler")

          attributeEditor.editorWindow:freeze()
          rollPanel:clear()
          local attrs = rebuildCommonNonManifestAttributes(objects)
          rebuildCustomAttributePanel(rollPanel, objects, attrs)
          attributeEditor.editorWindow:rebuild()

          changeContext:clearAllAttributeChangeEvents()
          for i, attr in ipairs(attrs) do
            changeContext:addAttributeChangeEvent(attr)
          end

          attributeEditor.logExitFunc("changeContext attributeAddedHandler")
        end
      )

      -- rebuild the custom attribute panel whenever attributes are removed
      changeContext:setAttributeRemovedHandler(
        function(object, attr)
          attributeEditor.logEnterFunc("changeContext attributeRemovedHandler")

          attributeEditor.editorWindow:freeze()
          rollPanel:clear()
          local attrs = rebuildCommonNonManifestAttributes(objects)
          rebuildCustomAttributePanel(rollPanel, objects, attrs)
          attributeEditor.editorWindow:rebuild()

          changeContext:clearAllAttributeChangeEvents()
          for i, attr in ipairs(attrs) do
            changeContext:addAttributeChangeEvent(attr)
          end

          attributeEditor.logExitFunc("changeContext attributeRemovedHandler")
        end
      )

      rebuildCustomAttributePanel(rollPanel, objects, attrs)

      if table.getn(attrs) == 0 then
        rollup:expand(false)
      end

      attributeEditor.logExitFunc("attributeEditor.addCustomAttributeSection")
      return rollup
    end

    attributeEditor.logExitFunc("attributeEditor.addCustomAttributeSection")
    return
  end

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: nil attributeEditor.bindHelpToWidget(Window widget, string helpText)
  --| brief: Adds the help text to a widget
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.bindHelpToWidget = function(widget, helpText)
   widget:setOnMouseEnter(
      function(self)
        attributeEditor.setHelpText(helpText)
      end
    )
    widget:setOnMouseLeave(
      function(self)
        attributeEditor.clearHelpText()
      end
    )
  end

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: nil attributeEditor.bindAttributeHelpToWidget(Window widget, table selection, string attribute)
  --| brief: Adds the help text of a particular attribute to a widget
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.bindAttributeHelpToWidget = function(widget, selection, attribute)
   local helpText = getAttributeHelpText(selection[1], attribute)
   attributeEditor.bindHelpToWidget(widget, helpText)
  end

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: Window attributeEditor.addSeparator(Panel panel)
  --| signature: Window attributeEditor.addSeparator(Panel panel, integer height)
  --| brief:
  --|   Adds a horizontal separator.
  --|
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.addSeparator = function(panel, height)
    local vSpace = 2
    if height ~= nil then
      vSpace = height/2
    end
    panel:addVSpacer(vSpace)
    local separator = panel:addPanel{ flags = "expand", size = { height = 1 }, proportion = 0 }
    separator:setBackgroundColour("dialogDarkestTint")
    panel:addVSpacer(vSpace)
  end

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: Window attributeEditor.addAttributeLabel(Panel panel, string label, table selection, string attribute, string font)
  --| brief:
  --|   Adds an attribute label with help text to the panel given. For example calling
  --|   attributeEditor.addAttributeLabel(panel, "My Label", selection, "Attribute")
  --|   would return a static text control with the name "AttributeText" the help text
  --|   would be set the same as calling attributeEditor.setHelpText(getAttributeHelpText(selection[1], "Attribute"))
  --|   font is an optional parameter that allows you to set the font of the object.
  --|
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.addAttributeLabel = function(panel, label, selection, attribute, font)
    local helpText = getAttributeHelpText(selection[1], attribute)
    local staticText = panel:addStaticText{
      name = string.format("%sText", attribute),
      text = label,
      onMouseEnter = function()
        attributeEditor.setHelpText(helpText)
      end,
      onMouseLeave = function()
        attributeEditor.clearHelpText()
      end
    }
    if font ~= nil then
      staticText:setFont(font)
    end

    return staticText
  end

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: Window attributeEditor.addAttributeWidget(Panel panel, string attribute, table selection, string set)
  --| signature: Window attributeEditor.addAttributeWidget(Panel panel, table attributePaths, table selection, string set)
  --| brief:
  --|   Adds an attribute widget to the panel given by calling panel:addAttributeWidget, also sets the
  --|   help text for the widget. For non-animation set attributes the set parameter is optional otherwise
  --|   an animation set must be given.
  --|
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.addAttributeWidget = function(panel, attribute, selection, set)
  
    local attrPaths = { }
    if type(attribute) == "table" then
      -- we've been passed in some attribute paths
      attrPaths = attribute
      -- pull the name of the attribute from the first element
      _, attribute = splitAttributePath(attrPaths[1])
    elseif type(attribute) == "string" then
      -- build the table of attribute paths
      for _, object in ipairs(selection) do
        table.insert(attrPaths, string.format("%s.%s", object, attribute))
      end
    else
      -- Passed in a type that we don't know how to deal with
      assert(false)
    end
    

    local widget = panel:addAttributeWidget{
      name = string.format("%sWidget", attribute),
      attributes = attrPaths,
      flags = "expand",
      set = set,
      proportion = 1,
    }
    attributeEditor.bindAttributeHelpToWidget(widget, selection, attribute)
    assert(widget)
    return widget
  end

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: Window attributeEditor.addAttributeResetButton(Panel panel, string label, string attribute, table selection, string set)
  --| signature: Window attributeEditor.addAttributeResetButton(Panel panel, string label, table attributePaths, table selection, string set)
  --| brief:
  --|   This adds a reset button that will reset a given attribute, or attributes to their factory defaults
  --|
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.addAttributeResetButton = function(panel, label, attributes, selection, set)
    local widget

    --------------------------------------------------------------------------------------------------------------------
    local resetToDefaults = function()
      if type(attributes) == "table" then
        undoBlock(function()
          for _, attr in ipairs(attributes) do
            local defaultValue = getAttributeDefault(selection[1], attr)
            setCommonAttributeValue(selection, attr, defaultValue, set)
          end
        end)
      else
        local defaultValue = getAttributeDefault(selection[1], attributes)
        setCommonAttributeValue(selection, attributes, defaultValue, set)
      end
    end

   ---------------------------------------------------------------------------------------------------------------------
   local updateWidgets = function()
      local enable = false
      if type(attributes) == "table" then
        for _, attr in ipairs(attributes) do
          local currentValue = getCommonAttributeValue(selection, attr, set)
          if currentValue == nil or currentValue ~= getAttributeDefault(selection[1], attr) then
            enable = true
            break
          end
        end
      else
        local currentValue = getCommonAttributeValue(selection, attributes, set)
        enable = currentValue == nil or currentValue ~= getAttributeDefault(selection[1], attributes)
      end
      widget:enable(enable)
    end

    widget = panel:addButton{
        label = label, 
        size = { width = 54 },
        onClick = resetToDefaults
    }
    
    -- Create a change context
    local context = attributeEditor.createChangeContext()
    context:setObjects(selection)
    if type(attributes) == "table" then
      for _, attr in ipairs(attributes) do
        context:addAttributeChangeEvent(attr)
      end
    else
      context:addAttributeChangeEvent(attributes)
    end
    context:setAttributeChangedHandler(updateWidgets)
    updateWidgets()

    return widget
  end

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: ComboBox attributeEditor.addCustomComboBox(table params)
  --| brief:
  --|   Creates a custom combo box using a specialized syntax.
  --| desc:
  --|   Syntax Example:
  --|  <codeblock>
  --|   attributeEditor.addCustomComboBox{
  --|     panel = rollPanel,
  --|     objects = selection,
  --|     attributes = { "AttributeToSync", "AnotherAttributeToSync" },
  --|     values = { yes = function(selection) print("In Yes") end,
  --|               no = function(selection) print("In No")end },
  --|     syncValueWithUI = function(combo, selection) combo:setSelectedItem("yes") end,
  --|     order = { "no", "yes" } -- optional
  --|   }
  --|  </codeblock>
  --|
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.addCustomComboBox = function(params)
    attributeEditor.logEnterFunc("attributeEditor.customComboBox")
    local syncUIFunction = function(combo, selection) end
    if type(params.syncValueWithUI) == "function" then
      attributeEditor.log("found valid function \"syncValueWithUI\" in params")
      syncUIFunction = params.syncValueWithUI
    end

    local attributes = { }
    if type(params.attributes)  == "table" then
      attributeEditor.log("found valid table \"attributes\" in params")
      attributes = params.attributes
    end

    local objects = { }
    if type(params.objects)  == "table" then
      attributeEditor.log("found valid table \"objects\" in params")
      objects = params.objects
    end

    local comboValueNames = { }
    local comboValueOnChangeFn = { }
    if type(params.order) == "table" then
      for _, comboValueName in ipairs(params.order) do
        local onChangeFunction = params.values[comboValueName]
        if type(onChangeFunction) == "function" then
          attributeEditor.log("adding ordered combo option %s", comboValueName)
          table.insert(comboValueNames, comboValueName)
          comboValueOnChangeFn[comboValueName] = onChangeFunction
        end
      end
    else
      for comboValueName, onChangeFunction in pairs(params.values) do
        attributeEditor.log("adding combo option %s", comboValueName)
        table.insert(comboValueNames, comboValueName)
        comboValueOnChangeFn[comboValueName] = onChangeFunction
      end
    end

    local helpText = ""
    if type(params.helpText) == "string" then
      helpText = params.helpText
    end

    attributeEditor.log("panel:addComboBox")
    local customComboBox = params.panel:addComboBox{
      flags = "expand",
      proportion = 1,
      items = comboValueNames,
      onMouseEnter = function(self)
        attributeEditor.setHelpText(helpText)
      end,
      onMouseLeave = function()
        attributeEditor.clearHelpText()
      end
    }

    if containsReference(objects) then
      customComboBox:enable(false)
    end

    -- these are to stop an infinte loop of ui code calling context change code calling ui code and so on.
    local enableContextEvents = true
    local enableUISetAttribute = true

    --------------------------------------------------------------------------------------------------------------------
    -- this function is called whenever custom combo box is changed through the ui
    --------------------------------------------------------------------------------------------------------------------
    customComboBox:setOnChanged(
      function(self)
        attributeEditor.logEnterFunc("customComboBoxAuto:setOnChanged")

        if enableUISetAttribute then
          -- prevent the change context callbacks from firing off
          enableContextEvents = false
          local selectedItem = self:getSelectedItem()
          local onValueChangeFunction = comboValueOnChangeFn[selectedItem]
          onValueChangeFunction(objects)
          syncUIFunction(self, objects)
          enableContextEvents = true
        end

        attributeEditor.logExitFunc("customComboBoxAuto:setOnChanged")
      end
    )

    attributeEditor.log("creating change context for attributes")
    local changeContext = attributeEditor.createChangeContext()
    changeContext:setObjects(objects)
    for _, attrName in ipairs(attributes) do
      changeContext:addAttributeChangeEvent(attrName)
    end

    --------------------------------------------------------------------------------------------------------------------
    -- this function is called whenever attributes are changed via script, or through undo redo.
    --------------------------------------------------------------------------------------------------------------------
    changeContext:setAttributeChangedHandler(
      function(object, attr)
        attributeEditor.logEnterFunc("changeContext attributeChangedHandler")

        if enableContextEvents then
          enableUISetAttribute = false
            syncUIFunction(customComboBox, objects)
          enableUISetAttribute = true
        end

        attributeEditor.logExitFunc("changeContext attributeChangedHandler")
      end
    )

    syncUIFunction(customComboBox, objects)

    attributeEditor.logExitFunc("attributeEditor.customComboBox")
    return customComboBox
  end

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: ComboBox attributeEditor.addBoolAttributeCombo(table params)
  --| brief:
  --|   Create a combo box using a specialized syntax
  --| desc:
  --|  <codeblock>
  --|  Create a combo box using a specialized syntax
  --|  attributeEditor.addBoolAttributeCombo{ panel = rollPanel, objects = selection,
  --|     attribute = "AttributeToConnect",
  --|     trueValue = "I'm true", falseValue ="I'm false",
  --|     helpText = "This is some help",
  --|     set = "DefaultSet", -- optional
  --|     syncValueWithUI = "function", -- optional
  --|  }
  --|  </codeblock>
  --|
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.addBoolAttributeCombo = function(params)
    attributeEditor.logEnterFunc("attributeEditor.boolAttributeCombo")

    local attributeList = { }
    table.insert(attributeList, params.attribute)
    local valuesTable = { }
    local selection = params.objects
    local trueValueString = params.trueValue
    local falseValueString = params.falseValue
    local syncValueWithUI = params.syncValueWithUI
    local set = params.set

    valuesTable[trueValueString] = function(selection)
      setCommonAttributeValue(selection, attributeList[1], true, params.set)
    end

    valuesTable[falseValueString] = function(selection)
      setCommonAttributeValue(selection, attributeList[1], false, params.set)
    end

    if params.helpText == nil then
      params.helpText = getAttributeHelpText(selection[1], params.attribute)
    end

    attributeEditor.logExitFunc("attributeEditor.boolAttributeCombo")
    return attributeEditor.addCustomComboBox{
      panel = params.panel,
      objects = params.objects,
      attributes = attributeList,
      helpText = params.helpText,
      values = valuesTable,
      syncValueWithUI = function(combo, selection)
        attributeEditor.logEnterFunc("boolAttributeCombo syncValueWithUI")

        local value = getCommonAttributeValue(selection, attributeList[1], set)
        if value ~= nil then
          if value then
            attributeEditor.log("boolAttributeCombo comboBox:setSelectedItem(\"%s\")", trueValueString)
            combo:setSelectedItem(trueValueString)
            combo:setIsIndeterminate(false)
          else
            attributeEditor.log("boolAttributeCombo comboBox:setSelectedItem(\"%s\")", falseValueString)
            combo:setSelectedItem(falseValueString)
            combo:setIsIndeterminate(false)
          end
        else
          attributeEditor.log("boolAttributeCombo comboBox:setSelectedItem(\"%s\")", "{ blank }")
          combo:setSelectedItem("")
          combo:setIsIndeterminate(true)
        end
        if syncValueWithUI then
          syncValueWithUI()
        end
        attributeEditor.logExitFunc("boolAttributeCombo syncValueWithUI")
      end
    }
  end

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: ComboBox attributeEditor.addIntAttributeCombo(table params)
  --| brief:
  --|   Create a combo box using a specialized syntax
  --|   attributeEditor.addIntAttributeCombo{ panel = rollPanel, objects = selection,
  --|     attribute = "AttributeToConnect",
  --|     values = { [0] = "Apples", [1] = "Pears" }
  --|     helpText = "This is some help",
  --|     order = { "Pears", "Apples" } -- optional
  --|     set = "DefaultSet", -- optional
  --|     syncValueWithUI = "function", -- optional
  --|     errorText = "", -- optional
  --|   }
  --|
  --|   if an order is not specified items will be ordered by ascending value - so that
  --|   in the case of values = { [4] = "Onions", [2] = "Carrots" } "carrots" will be the
  --|   first value.
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.addIntAttributeCombo = function(params)
    attributeEditor.logEnterFunc("attributeEditor.addIntAttributeCombo")

    local attributeList = { }
    table.insert(attributeList, params.attribute)
    local valuesTable = { }
    local valuesLookup = { }
    local selection = params.objects
    local syncValueWithUI = params.syncValueWithUI
    local set = params.set

    local makeSelectionFunction = function(i)
      return function(selection)
        setCommonAttributeValue(selection, attributeList[1], i, params.set)
      end
    end

    for i, text in pairsByKeys(params.values) do
      valuesTable[text] = makeSelectionFunction(i)
      valuesLookup[i] = text
    end

    attributeEditor.logExitFunc("attributeEditor.boolAttributeCombo")
    return attributeEditor.addCustomComboBox {
      panel = params.panel,
      objects = params.objects,
      attributes = attributeList,
      helpText = params.helpText,
      order = params.order,
      values = valuesTable,
      syncValueWithUI = function(combo, selection)
        attributeEditor.logEnterFunc("intAttributeCombo syncValueWithUI")

        local value = getCommonAttributeValue(selection, attributeList[1], set)
        if value == nil then
          combo:setIsIndeterminate(true)
        else
          combo:setIsIndeterminate(false)

          local lookupVal = valuesLookup[value]
          if lookupVal == nil then
            combo:setError(true)
            if params.errorText then
              combo:setValue(params.errorText)
            end
          else
            combo:setError(false)
            combo:setSelectedItem(lookupVal)
          end
        end

        if syncValueWithUI then
          syncValueWithUI()
        end

        attributeEditor.logExitFunc("intAttributeCombo syncValueWithUI")
      end
    }
  end

 ------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: ComboBox attributeEditor.addStringAttributeCombo(table params)
--| brief:
--|   Create a combo box using a specialized syntax
--|   attributeEditor.addIntAttributeCombo{ panel = rollPanel, objects = selection,
--|   attribute = "AttributeToConnect",
--|   values = { "Apples", "Pears" }
--|   helpText = "This is some help",
--|   set = "DefaultSet", -- optional
--|   syncValueWithUI = "function", -- optional
--|   }
--|
--| environments: GlobalEnv
--| page: AttributeEditorAPI
------------------------------------------------------------------------------------------------------------------------
  attributeEditor.addStringAttributeCombo = function(params)
    attributeEditor.logEnterFunc("attributeEditor.addIntAttributeCombo")

    local attributeList = { }
    table.insert(attributeList, params.attribute)
    local valuesTable = { }
    local selection = params.objects
    local syncValueWithUI = params.syncValueWithUI
    local set = params.set

    local makeSelectionFunction = function(i)
      return function(selection)
        setCommonAttributeValue(selection, attributeList[1], i, params.set)
      end
    end

    for i, text in pairsByKeys(params.values) do
      valuesTable[text] = makeSelectionFunction(text)
    end

    return attributeEditor.addCustomComboBox {
      panel = params.panel,
      objects = params.objects,
      attributes = attributeList,
      helpText = params.helpText,
      order = params.values,
      values = valuesTable,
      syncValueWithUI = function(combo, selection)
        local value = getCommonAttributeValue(selection, attributeList[1], set)
        if value == nil then
          combo:setIsIndeterminate(true)
        else
          combo:setIsIndeterminate(false)
          combo:setSelectedItem(value)
        end
        if syncValueWithUI then
          syncValueWithUI()
        end
      end
    }
  end

------------------------------------------------------------------------------------------------------------------------
  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: Window attributeEditor.addCollisionSetWidget(Panel panel, string attribute, table selection, string set)
  --| signature: Window attributeEditor.addCollisionSetWidget(Panel panel, table attributePaths, table selection, string set)
  --| brief:
  --|   Adds a collision set attribute widget to the panel given by calling panel:addCollisionSetWidget, also sets the
  --|   help text for the widget. For non-animation set attributes the set parameter is optional otherwise
  --|   an animation set must be given.
  --|
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.addCollisionSetWidget = function(panel, attribute, selection, set)
    local index = 0
    local values = { }
    local order = { }

    if set == nil then
      set = getSelectedAssetManagerAnimSet()
    end

    if anim.isPhysicsRigValid(set) then
      local app = nmx.Application.new()
      local scene = app:getSceneByName("AssetManager")
      if scene then
        local rigDataRoot = anim.getPhysicsRigDataRoot(scene, set)
        if rigDataRoot then
          local setIt = nmx.NodeIterator.new(rigDataRoot, nmx.CollisionSetNode.ClassTypeId())
          if setIt:next() then
            local node = setIt:node():getFirstChild()
            while node do
              local name = string.format("%i (%s)", index, node:getName())
              values[index] = name
              table.insert(order, name)
              node = node:getNextSibling()

              index = index + 1
            end
          end
        end
      end
    end

    attributeEditor.addIntAttributeCombo{
      panel = panel,
      attribute = attribute,
      objects = selection,
      set = set,
      values = values,
      order = order,
      helpText = getAttributeHelpText(selection[1], attribute)
    }
  end

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: nil attributeEditor.addBasicConditionInfoSection()
  --| brief: adds a list of conditions to the addtribue editor
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.addBasicConditionInfoSection = function(panel, objects, attributes)
    attributeEditor.logEnterFunc("attributeEditor.addBasicConditionInfoSection")

    panel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 0 }
      panel:setFlexGridColumnExpandable(2)
      for i, attribute in ipairs(attributes) do
        local name = utils.getDisplayString(getAttributeDisplayName(objects[1], attribute))
        attributeEditor.addAttributeLabel(panel, name, objects, attribute)
        attributeEditor.addAttributeWidget(panel, attribute, objects, set)
      end
    panel:endSizer()

    attributeEditor.logExitFunc("attributeEditor.addBasicConditionInfoSection")
  end

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: nil attributeEditor.reset()
  --| brief:
  --|   Clears and rebuilds the attribute editor.
  --|
  --| environments: GlobalEnv
  --| page: AttributeEditorAPI
  ----------------------------------------------------------------------------------------------------------------------
  attributeEditor.reset = function()
    attributeEditor.logDepthIndent = " "
    attributeEditor.logEnterFunc("resetAttributeEditor()")

    if attributeEditor.editorWindow then
      attributeEditor.editorWindow:suspendLayout()
      attributeEditor.editorWindow:freeze()
      attributeEditor.editorWindow:clear()

      attributeEditor.editorWindow:beginVSizer{ flags = "expand", proportion = 1 }
        attributeEditor.log("adding the attribute controls ScrollPanel")

        local scrollPanel = attributeEditor.editorWindow:addScrollPanel{
          name = "AttributePanel",
          flags = "expand;vertical",
          proportion = 8
        }
        
        scrollPanel:addSwitchablePanel{
          name = "AttributeSwitchablePanel",
          flags = "expand",
          proportion = 1,
        }

        -- this panel contains the help text box
        attributeEditor.log("adding the help text Panel")
        local footer = attributeEditor.editorWindow:addPanel{
          name = "HelpPanel",
          flags = "expand",
          proportion = 2
        }
        attributeEditor.helpWindow = footer

        attributeEditor.log("adding the help text control")
        footer:beginVSizer{ flags = "expand", proportion = 1 }
          local helptext = footer:addTextControl{ name = "TextBox", flags = "expand;noscroll", proportion = 1 }
          helptext:setReadOnly(true)
        footer:endSizer()

      attributeEditor.editorWindow:endSizer()

      attributeEditor.editorWindow:resumeLayout()
      attributeEditor.editorWindow:rebuild()
    end

    attributeEditor.logExitFunc("resetAttributeEditor()")
    attributeEditor.log()
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- add the attribute editor panel to the layout manager and add the controls
  ----------------------------------------------------------------------------------------------------------------------
  addAttributeEditor = function(contextualPanel, forContext)
    attributeEditor.logEnterFunc("addAttributeEditor()")

    attributeEditor.editorWindow = contextualPanel:addPanel{ name = "AttributeEditor", caption = "Attribute Editor", forContext = forContext }
    attributeEditor.reset()

    -- repopulate with the current selection
    attributeEditor.onSelectionChange()

    attributeEditor.logExitFunc("addAttributeEditor()")
  end

  -- ensures that running this script resets the attribute editor, make sure this is run
  -- last so it sources all the funcs before calling them
  attributeEditor.reset()

  -- repopulate with the current selection
  attributeEditor.onSelectionChange()
else
  -- morpheme:connect is in command line mode
end
