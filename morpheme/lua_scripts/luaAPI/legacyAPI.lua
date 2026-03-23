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
--| LUAHELP: PAGE
--| name: DeprecatedAPI
--| title: Deprecated Functions
--| desc: Supports deprecated Morpheme:Connect script commands through Lua.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- Report deprecated warning for LegacyAPI functions.  Reports on function, line no, and file
-- that called function if possible.
------------------------------------------------------------------------------------------------------------------------
local deprecatedWarning = function(oldName, newName, version)
  oldName = oldName or "(unknown)"
  version = version or "(unknown)"

  -- get function name and line called from 3 up on the callstack.
  local debugInfo = debug.getinfo(3, "nSl")
  local funcname = debugInfo.name
  local line = debugInfo.currentline

  -- parse source file returned to see if it can be displayed (@ prefixes filename).
  local filename = nil
  local source = debugInfo.source
  local filenameToken = string.find(source, "@")
  if filenameToken == 1 then
    filename = string.sub(source, 2)
  end

  if filename and line then
    local location = "file " .. filename .. ", line " .. line
    if funcname then
      location = location .. ", function " .. funcname
    end
    if newName then
      app.warning("Deprecated " .. oldName .. " was superseded by ".. newName .. " in version " .. version .. ".  Called from " .. location)
    else
      app.warning("Deprecated " .. oldName .. " was removed in version " .. version .. ".  Called from " .. location)
    end
  else
    if newName then
      app.warning("Deprecated " .. oldName .. " was superseded by ".. newName .. " in version " .. version)
    else
      app.warning("Deprecated " .. oldName .. " was removed in version " .. version)
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string createNode(string Type, string ParentPath)
--| signature: string createNode(string Type, string ParentPath, string Name)
--| signature: string createNode(table tbl)
--| brief:
--|   Creates a node of the given type with the specified parent.
--|   Returns the fully qualified name of the new node
--| param: tbl table alternative calling convention using table of arguments.
--| tparam: string Type
--| tparam: string ParentPath
--| tparam: string Name
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
createNode = function(arg1, arg2, arg3)
  deprecatedWarning("createNode()", "create()", "1.2")

  if type(arg1) == "table" then
    local args = arg1
    arg1 = args["Type"]
    arg2 = args["ParentPath"]
    arg3 = args["Name"]
  end
  local Type = arg1
  local ParentPath = arg2
  local Name = arg3

  if not Type then
    error("createNode expecting valid Type")
  end

  if not ParentPath then
    error("createNode expecting valid ParentPath")
  end

  return create(Type, ParentPath, Name)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string createFilterNode(string Type, string ParentPath)
--| signature: string createFilterNode(string Type, string ParentPath, string Name)
--| signature: string createFilterNode(table tbl)
--| brief:
--|   Creates a node of the given type with the specified parent.
--|   Returns the fully qualified name of the new node
--| param: tbl table alternative calling convention using table of arguments.
--| tparam: string Type
--| tparam: string ParentPath
--| tparam: string Name
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
createFilterNode = function(arg1, arg2, arg3)
  deprecatedWarning("createFilterNode()", "create()", "1.2")

  if type(arg1) == "table" then
    local args = arg1
    arg1 = args["Type"]
    arg2 = args["ParentPath"]
    arg3 = args["Name"]
  end
  local Type = arg1
  local ParentPath = arg2
  local Name = arg3

  if not Type then
    error("createFilterNode expecting valid Type")
  end

  if not ParentPath then
    error("createFilterNode expecting valid ParentPath")
  end

  return create(Type, ParentPath, Name)
end


------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string registerReference(string name, table referenceTable)
--| brief:
--|   Register a reference file with the connect manifest.
--|   This function should be replaced by registerNetworkFile(string Type, table networkFileTable)
--|
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
registerReference = function(name, referenceTable)
  deprecatedWarning("registerReference()", "registerNetworkFile()", "3.5")

  if type(referenceTable) ~= "table" then
    -- referenceTable should be a table
    return 
  end
  
  referenceTable["type"] = "ReferenceDefault"

  return registerNetworkFile(name, referenceTable)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string createStateMachine(string ParentPath)
--| signature: string createStateMachine(string ParentPath, string Name)
--| signature: string createStateMachine(table tbl)
--| brief:
--|   Creates a State Machine with the specified parent.
--|   Returns the fully qualified name of the new State Machine.
--| param: tbl table alternative calling convention using table of arguments.
--| tparam: string Type
--| tparam: string ParentPath
--| tparam: string Name
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
createStateMachine = function(arg1, arg2)
  deprecatedWarning("createStateMachine()", "create()", "1.2")

  if type(arg1) == "table" then
    local args = arg1
    arg1 = args["ParentPath"]
    arg2 = args["Name"]
  end
  local ParentPath = arg1
  local Name = arg2

  if not ParentPath then
    error("createStateMachine expecting valid ParentPath")
  end

  return create("StateMachine", ParentPath, Name)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string createBlendTree(string ParentPath)
--| signature: string createBlendTree(string ParentPath, string Name)
--| signature: string createBlendTree(table tbl)
--| brief:
--|   Creates a Blend Tree with the specified parent.
--|   Returns the fully qualified name of the new Blend Tree.
--| param: tbl table alternative calling convention using table of arguments.
--| tparam: string Type
--| tparam: string ParentPath
--| tparam: string Name
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
createBlendTree = function(arg1, arg2)
  deprecatedWarning("createBlendTree()", "create()", "1.2")

  if type(arg1) == "table" then
    local args = arg1
    arg1 = args["ParentPath"]
    arg2 = args["Name"]
  end
  local ParentPath = arg1
  local Name = arg2

  if not ParentPath then
    error("createBlendTree expecting valid ParentPath")
  end

  return create("BlendTree", ParentPath, Name)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string createTransition(string Type, string SourcePath, string DestPath)
--| signature: string createTransition(string Type, string SourcePath, string DestPath, string Name)
--| signature: string createTransition(table tbl)
--| brief:
--|   Creates a transition between the source and destination nodes
--|   Returns the fully qualified name of the new transition.
--| param: tbl table alternative calling convention using table of arguments.
--| tparam: string Type
--| tparam: string SourcePath
--| tparam: string DestPath
--| tparam: string Name
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
createTransition = function(arg1, arg2, arg3, arg4)
  deprecatedWarning("createTransition()", "create()", "1.2")

  if type(arg1) == "table" then
    local args = arg1
    arg1 = args["Type"]
    arg2 = args["SourcePath"]
    arg3 = args["DestPath"]
    arg4 = args["Name"]
  end
  local Type = arg1
  local SourcePath = arg2
  local DestPath = arg3
  local Name = arg4

  if not Type then
    error("createTransition expecting valid node Type")
  end

  if not SourcePath then
    error("createTransition expecting valid SourcePath")
  end

  if not DestPath then
    error("createTransition expecting valid DestPath")
  end

  return create(Type, SourcePath, DestPath, Name)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string createCondition(string Type, string TransitionPath)
--| signature: string createCondition(string Type, string TransitionPath, string Name)
--| signature: string createCondition(table tbl)
--| brief:
--|   Creates an condition of the given type for the specified transition.
--|   Returns the fully qualified name of the new condition.
--| param: tbl table alternative calling convention using table of arguments.
--| tparam: string Type
--| tparam: string TransitionPath
--| tparam: string Name
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
createCondition = function(arg1, arg2, arg3)
  deprecatedWarning("createCondition()", "create()", "1.2")

  if type(arg1) == "table" then
    local args = arg1
    arg1 = args["Type"]
    arg2 = args["TransitionPath"]
    arg3 = args["Name"]
  end
  local Type = arg1
  local TransitionPath = arg2
  local Name = arg3

  if not Type then
    error("createCondition expecting valid Type")
  end

  if not TransitionPath then
    error("createCondition expecting valid TransitionPath")
  end

  return create(Type, TransitionPath, Name)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string createControlParameter(string DataType)
--| signature: string createControlParameter(string DataType, string Name)
--| signature: string createControlParameter(table tbl)
--| brief:
--|   Type must be one of: Float, Vector3, Vector4, Boolean or Quaternion
--|   Creates an control parameter of the specified type.
--|   Returns the fully qualified name of the new Control Parameter.
--| param: tbl table alternative calling convention using table of arguments.
--| tparam: string DataType
--| tparam: string Name
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
createControlParameter = function(arg1, arg2)
  deprecatedWarning("createControlParameter()", "create()", "1.2")

  if type(arg1) == "table" then
    local args = arg1
    arg1 = args["DataType"]
    arg2 = args["Name"]
  end
  local DataType = arg1
  local Name = arg2

  if not DataType then
    error("createControlParameter expecting valid DataType")
  end

  return create("ControlParameter", DataType, Name)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string createRequest(string Name)
--| signature: string createRequest(string Name, integer Id)
--| brief:
--|   Creates a new request with a runtime Id of 'Id' (if the id is not already in use)
--|   Returns the fully qualified name of the new request object.
--|
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
createRequest = function(arg1, arg2)
  deprecatedWarning("createRequest()", "create()", "1.2")

  if type(arg1) == "table" then
    local args = arg1
    arg1 = args["Name"]
    arg2 = args["Id"]
  end
  local Name = arg1
  local Id = arg2

  if not Name then
    error("createRequest expecting valid Name")
  end

  return create("Request", Name, Id)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean addRuntimeTarget(string name, string description, string APPath, string RTPath, string IPAddress, integer port)
--| brief:
--|   Adds a new runtime target that the application can connect to.
--| desc:
--|   name - the name of the target
--|   description - the description of the target
--|   APPath - the absolute path to the asset processor to use
--|   RTPath - the absolute path to the runtime target to use. If this is "" then morpheme:connect will not attempt to start the target automatically
--|   IPAddress - the ip address of the target
--|   port - the port to connect on.
--|
--|   Returns false if the target wasn't added.
--|
--|   Note: Paths must only use '/' or '\\' to seperate directories
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
addRuntimeTarget = function(name, description, APPath, RTPath, IPAddress, port)
  deprecatedWarning("addRuntimeTarget()", "target.add()", "1.2")
  return target.add(name, description, APPath, RTPath, IPAddress, port)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean removeRuntimeTarget(string name)
--| brief:
--|   Removes a runtime target.
--|   name - the name of the target to remove
--|   Returns true if the target is successfully removed
--|
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
removeRuntimeTarget = function(name)
  deprecatedWarning("removeRuntimeTarget()", "target.remove()", "1.2")
  return target.remove(name)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table listRuntimeTargets()
--| brief:
--|   Returns a list of the available runtime targets.
--|
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
listRuntimeTargets = function()
  deprecatedWarning("listRuntimeTargets()", "target.ls()", "1.2")
  return target.ls(name)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean selectRuntimeTarget(string name)
--| brief:
--|   Select which runtime target to connect to.
--| desc:
--|   name - the name of the target to connect to
--|   Returns false if the names target is not found.
--|
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
selectRuntimeTarget = function(name)
  deprecatedWarning("selectRuntimeTarget()", "target.select()", "1.2")
  return target.select(name)
end

------------------------------------------------------------------------------------------------------------------------
getRequestID = function(path)
  deprecatedWarning("getRequestID()", "target.getRequestID()", "1.2")
  return target.getRequestID(name)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table getTransformChannels(string Path)
--| signature: table getTransformChannels(string animTakeAttribute)
--| brief: Returns a table that contains the rig bone indices of transform channels output by the node or animation take.
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
getTransformChannels = function(arg)
  deprecatedWarning("getTransformChannels()", "anim.getTransformChannels()", "1.2")
  return anim.getTransformChannels(arg)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table getEvents(string Path)
--| brief: Returns a table that contains the event data for this node
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
getEvents = function(path)
  deprecatedWarning("getEvents()", "anim.getEvents()", "1.2")
  return anim.getEvents(arg)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: nil selectTakeInBrowser(TakeTable animationTake)
--| brief: Select the animation take selected in the asset manager
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
selectTakeInBrowser = function(animationTake)
  deprecatedWarning("selectTakeInBrowser()", "anim.selectTakeInAssetManager()", "1.2")
  return anim.selectTakeInAssetManager(animationTake)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: float getTakeDuration(TakeTable animationTake)
--| brief: Get the duration of the given animation take
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
getTakeDuration = function(animationTake)
  deprecatedWarning("getTakeDuration()", "anim.getTakeDuration()", "1.2")
  return anim.getTakeDuration(animationTake)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table getTakeMarkupData(TakeTable animationTake)
--| brief: Get the markup data for the given animation take.
--|
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
getTakeMarkupData = function(animationTake)
  deprecatedWarning("getTakeMarkupData()", "anim.getTakeMarkupData()", "1.2")
  return anim.getTakeMarkupData(animationTake)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table getRigChannelNames()
--| brief: returns the number of channels in the rig
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
getRigChannelNames = function()
  deprecatedWarning("getRigChannelNames()", "anim.getRigChannelNames()", "1.2")
  return anim.getRigChannelNames()
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: integer getRigSize()
--| brief:
--|   returns the number of channels in the rig
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
getRigSize = function()
  deprecatedWarning("getRigSize()", "anim.getRigSize()", "1.2")
  return anim.getRigSize()
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table getRigHierarchy()
--| brief: get the rig hierarchy as a table.
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
getRigHierarchy = function()
  deprecatedWarning("getRigHierarchy()", "anim.getRigHierarchy()", "1.2")
  return anim.getRigHierarchy()
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table getRigMarkupData(TakeTable animationTake)
--| brief: get the rig markup data as a table
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
getRigMarkupData = function()
  deprecatedWarning("getRigMarkupData()", "anim.getRigMarkupData()", "1.2")
  return anim.getRigMarkupData()
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean addAnimationLocation(string sourceDirectory)
--| signature: boolean addAnimationLocation(string sourceDirectory, string markupDirectory)
--| brief:
--|   Add a new animation location to the animation file manager.  Returns true on success.
--| desc:
--|   If markupDirectory is not specified, the default markup directory is used instead.
--|   Will fail if attempting to add a duplicate location.
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
addAnimationLocation = function(sourceDirectory, markupDirectory)
  deprecatedWarning("addAnimationLocation()", "anim.addAnimationLocation()", "1.2")
  return anim.addAnimationLocation(sourceDirectory, markupDirectory)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean removeAnimationLocation(string sourceDirectory)
--| brief:
--|   Remove an animation location from the animation file manager.  Returns true on success.
--|
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
removeAnimationLocation = function(sourceDirectory)
  deprecatedWarning("removeAnimationLocation()", "anim.removeAnimationLocation()", "1.2")
  return anim.removeAnimationLocation(sourceDirectory)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string macroizeString(string str)
--| brief: Convert string into a macro string expression
--|
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
macroizeString = function(str)
  deprecatedWarning("macroizeString()", "utils.macroizeString()", "1.2")
  return utils.macroizeString(str)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string demacroizeString(string str)
--| brief: Convert a macro string into a regular string expression
--|
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
demacroizeString = function(str)
  deprecatedWarning("demacroizeString()", "utils.demacroizeString()", "1.2")
  return utils.demacroizeString(str)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean changeTransitionType(string path, string transitionType)
--| brief:
--|   changes the type of a transition node
--|
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
changeTransitionType = function(path)
  deprecatedWarning("changeTransitionType()", "utils.changeTransitionType()", "1.2")
  return utils.changeTransitionType(path)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: void set1stPersonCamChannel(integer Channel)
--| brief:
--|   Sets the index of the channel to bind the 1st person camera to.
--|
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
set1stPersonCamChannel = function(channel)
  deprecatedWarning("set1stPersonCamChannel()", "utils.set1stPersonCamChannel()", "1.2")
  return utils.set1stPersonCamChannel(channel)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: void set1stPersonCamFOV(float FOV)
--| brief:
--|   Changes the FOV of the 1st person camera
--|
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
set1stPersonCamFOV = function(fov)
  deprecatedWarning("set1stPersonCamFOV()", "utils.set1stPersonCamFOV()", "1.2")
  return utils.set1stPersonCamFOV(fov)
end

------------------------------------------------------------------------------------------------------------------------
newFile = function()
  deprecatedWarning("newFile()", "mcn.new()", "1.2")
  return mcn.new()
end

------------------------------------------------------------------------------------------------------------------------
openFile = function(file)
  deprecatedWarning("openFile()", "mcn.open()", "1.2")
  local isOk, isCancelled = mcn.open(file)
  return isOk
end

------------------------------------------------------------------------------------------------------------------------
saveFile = function(file)
  deprecatedWarning("saveFile()", "mcn.save()", "1.2")
  return mcn.save(file)
end

------------------------------------------------------------------------------------------------------------------------
closeFile = function()
  deprecatedWarning("closeFile()", "mcn.close()", "1.2")
  return mcn.close()
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: nil commitChanges(string Description)
--| brief:
--|   Commits the changes made by the script thus far into the database.
--|   This will count as one undoable operation.
--|   An object cannot be created and destroyed within the same script
--|   without a commit being called in between the create and destroy calls.
--|
--| desc:
--|   Commit should not be called in user script unless an object is
--|   created and destroyed.  Morpheme:Connect automatically commits changes
--|   after the execution of a user script.
--| environments: GlobalEnv
--| page: morphemeConnect
------------------------------------------------------------------------------------------------------------------------
commitChanges = function(desc)
  deprecatedWarning("commitChanges()", "mcn.commit()", "1.2")
  return mcn.commit(desc)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string addCustomAttribute(string Path, table Attr)
--| brief:
--|   Adds a custom attribute to the object referenced by Path, Attr is a table describing
--|   the attribute.
--|
--| desc:
--|   The table must specify a name and type with value and perAnimSet being optional extra parameters.
--|   The following code would add a float attribute:
--|   <codeblock>
--|   addCustomAttribute("Node", { name = "FloatAttr", type = "float", value = 1.0, perAnimSet = false })
--|   </codeblock>
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
addCustomAttribute = function(Path, Attr)
  deprecatedWarning("addCustomAttribute()", "addAttribute()", "2.1")
  if type(Path) ~= "string" then
    error("Wrong type for Path expected 'string' got '" .. type(Path) .. "'.")
  end

  if type(Attr) ~= "table" then
    error("Wrong type for Attr expected 'table' got '" .. type(Attr) .. "'.")
  end

  Attr.path = Path

  return addAttribute(Attr)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: nil removeCustomAttribute(string Path)
--| brief:
--|   Removes a custom attribute from a node.
--|
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
removeCustomAttribute = function(Path)
  deprecatedWarning("removeCustomAttribute()", "removeAttribute()", "2.1")
  node, attr = splitAttributePath(Path)
  removeAttribute(node, attr)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: nil renameCustomAttribute(string Path, string NewName)
--| brief:
--|   Renames a custom attribute.
--|
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
renameCustomAttribute = function(Path, NewName)
  deprecatedWarning("renameCustomAttribute()", "renameAttribute()", "2.1")
  renameAttribute(Path, NewName)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean isCustomAttribute(string Path, string AttributeName)
--| signature: boolean isCustomAttribute(string AttributePath)
--| brief:
--|   Returns whether the attribute is custom or a manifest attribute.
--|
--|   The first method allows you to pass the object path and the attribute name as separate strings.
--|   The second version uses the full attribute path.
--|
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
isCustomAttribute = function(Path, AttributeName)
  deprecatedWarning("isCustomAttribute()", "isManifestAttribute()", "2.1")
  if AttributeName == nil then
    return not isManifestAttribute(Path)
  else
    return not isManifestAttribute(Path, AttributeName)
  end
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string createBlendTreeAsParent(string NodePath)
--| signature: string createBlendTreeAsParent(string NodePath, string Name)
--| signature: string createBlendTreeAsParent(table tbl)
--| brief:
--|   Creates a Blend Tree to replace the State Machine Node specified by 'NodePath', and
--|   adds 'NodePath' as a child of the newly created Blend Tree.
--|   Returns the new path assigned to 'NodePath'.
--| param: tbl
--| tparam: string NodePath
--| tparam: string Name
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
createBlendTreeAsParent = function(NodePath, DesiredName)
  deprecatedWarning("createBlendTreeAsParent()", "createGraphAsParent()", "2.1")
  local newGraph
  if DesiredName == nil then
    if type(NodePath) == "table" then
      NodePath.Type = "BlendTree"
      newGraph = createGraphAsParent(NodePath)
    else
      newGraph = createGraphAsParent(NodePath, "BlendTree")
    end
  else
    newGraph = createGraphAsParent(NodePath, "BlendTree", DesiredName)
  end

  return newGraph
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string createStateMachineAsParent(string NodePath)
--| signature: string createStateMachineAsParent(string NodePath, string Name)
--| signature: string createStateMachineAsParent(table tbl)
--| brief:
--|   Creates a State Machine to replace the node specified by 'NodePath', and
--|   adds 'NodePath' as a child of the newly created State Machine.
--|   Returns the new path assigned to 'NodePath'.
--| param: tbl
--| tparam: string NodePath
--| tparam: string Name
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
createStateMachineAsParent = function(NodePath, Name)
  deprecatedWarning("createStateMachineAsParent()", "createGraphAsParent()", "2.1")
  local newGraph
  if DesiredName == nil then
    if type(NodePath) == "table" then
      NodePath.Type = "StateMachine"
      newGraph = createGraphAsParent(NodePath)
    else
      newGraph = createGraphAsParent(NodePath, "StateMachine")
    end
  else
    newGraph = createGraphAsParent(NodePath, "StateMachine", DesiredName)
  end

  return newGraph
end

---------
registerAttributeDisplayInfo = function(type, info)
  deprecatedWarning("registerAttributeDisplayInfo()", "attributeEditor.registerDisplayInfo()", "2.2")

  for i, section in ipairs(info) do
    section.usedAttributes = section.attributes
    section.attributes = nil

    if section.type == "AttributeGroup" then
      section.displayFunction = function(...) safefunc(attributeEditor.standardDisplayInfoSection, unpack(arg)) end
    end
  end

  attributeEditor.registerDisplayInfo(type, info)
end

mcn.close = function()
  app.warning("Deprecated mcn.close() no longer exists.")
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string registerFilterNode(string NodeType, table NodeManifest)
--| brief:
--|   Registers a new node of type NodeType with a table describing the manifest for the new node. By default
--|   adds two pins "In" and "Out" mimicing the old behaviour of FilterNodes. Please use registerNode instead
--|   of this function.
--|   For more information see the morpheme documentation relating to adding new manifest types.
--|
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
registerFilterNode = function(typename, description)
  deprecatedWarning("registerFilterNode()", "registerNode()", "3.0")

  local interfaces = description.interfaces or { description.interface }

  description.functionPins = {
    ["In"] = {
      input = true,
      array = false,
      passThrough = true,
      interfaces = interfaces,
    },
    ["Out"] = {
      input = false,
      array = false,
      passThrough = true,
      interfaces = interfaces,
    },
  }

  if type(description.pinOrder) == "table" then
    table.insert(description.pinOrder, 1, "Out")
    table.insert(description.pinOrder, 1, "In")
  else
    description.pinOrder = { "In", "Out", }
  end

  registerNode(typename, description)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: Image loadAppImage(string filename)
--| brief: Load images from the ui directory
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
loadAppImage = function(filename)
  deprecatedWarning("loadAppImage()", "app.loadImage()", "3.0")
  return app.loadImage(filename)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean isReferencedList(table paths)
--| brief: Check if any objects in the table of paths are referenced.
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
isReferencedList = function(paths)
  deprecatedWarning("isReferencedList()", "containsReference()", "3.0")
  return containsReference(paths)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: nil printTable(table tbl)
--| brief: Prints the contents of a table.
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
serializeTable = function(tbl)
  deprecatedWarning("serializeTable()", "table.serialize()", "3.0")
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean mcn.writePreference(string preference, string type, object value)
--| brief:
--|   Write the value of a preference into the windows registry. Type is either
--|   string, double, boolean or long.
--|   Returns true if successful
--| environments: GlobalEnv
--| page: McnAPI
------------------------------------------------------------------------------------------------------------------------
mcn.writePreference = function(preference, type, value)
  deprecatedWarning("mcn.writePreference()", "preferences.set()", "3.0")

  return preferences.set{
    name = preference,
    value = value,
  }
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table mcn.readPreference(string preference)
--| brief:
--|   Return the value of a preference as stored in the windows registry, or nil if it doesn't exist or is a group
--| environments: GlobalEnv
--| page: McnAPI
------------------------------------------------------------------------------------------------------------------------
mcn.readPreference = function(preference)
  deprecatedWarning("mcn.readPreference()", "preferences.get()", "3.0")
  if type(preference) == "table" then
    preference = preference.preference
  end
  return preferences.get(preference)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean mcn.deletePreference(string preference)
--| brief:
--|   Deletes a preference from the windows registry
--|   Returns true if successful
--| environments: GlobalEnv
--| page: McnAPI
------------------------------------------------------------------------------------------------------------------------
 mcn.removePreference = function(preference)
  deprecatedWarning("mcn.removePreference()", "preferences.remove()", "3.0")
  return preferences.remove("RoamingUser", preference)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string quoteString(string str)
--| brief:
--|   Wraps a string with quotes
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
quoteString = function(string)
  deprecatedWarning("quoteString()", nil, "3.0")
  return "\""..string .. "\""
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean checkBit(integer bitField, integer bit)
--| brief:
--|   Find out if a bit is set within a field.
--|   Only works for single bits. Bits numbered from 0.
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
checkBit = function(bitField, bit)
  deprecatedWarning("checkBit()", nil, "3.0")
  local temp = 2 ^ bit
  local temp1 = bitField / temp
  local temp2 = math.mod(temp1, 2)
  if temp2 >= 1 and temp2 < 2 then
    return true
  end
  return false
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean hasCommonAttributeSetOrderAndData(table objects, string attribute)
--| brief:
--|   Checks to see if an attribute is a per animation set attribute. If it is an animation set
--|   attribute then it also checks to see if the attribute has the same set order and attribute data
--|   for the same sets.
--|
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
hasCommonAttributeSetOrderAndData = function(objects, attribute)
  deprecatedWarning("hasCommonAttributeSetOrderAndData", "haveCommonAnimSetOrderAndData", "3.0")
  
  local attributePaths = { }
  local count = table.getn(objects)
  for i = 1, count do
    attributePaths[i] = string.format("%s.%s", objects[i], attribute)
  end 
  return haveCommonAnimSetOrderAndData(attributePaths)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: float, float getOutputControlParametersNodeSize(string blendTreePath)
--| brief:
--|   Returns the size of the output control parameters node in the given blend tree path.
--|   
--| environments: UpgradeEnv ValidateSerializeEnv UpdatePinsEnv GlobalEnv
--| page: CoreAPI
------------------------------------------------------------------------------------------------------------------------
getOutputControlParametersNodeSize = function(blendTree, x, y)
  deprecatedWarning("getOutputControlParametersNodeSize", "getEmittedControlParametersNodeSize", "3.0")
  return getEmittedControlParametersNodeSize(blendTree)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: float, float getEmittedControlParametersNodePosition(string blendTreePath)
--| brief:
--|   Returns the position of the output control parameters node in the given blend tree path.
--|   
--| environments: UpgradeEnv ValidateSerializeEnv UpdatePinsEnv GlobalEnv
--| page: CoreAPI
------------------------------------------------------------------------------------------------------------------------
getOutputControlParametersNodePosition = function(blendTree, x, y)
  deprecatedWarning("getOutputControlParametersNodePosition", "getEmittedControlParametersNodePosition", "3.0")
  return getEmittedControlParametersNodePosition(blendTree)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean setEmittedControlParametersNodePosition(string blendTreePath, float x, float y)
--| brief:
--|   Sets the position of the output control parameters node in the given blend tree path.
--|   
--| environments: GlobalEnv
--| page: DeprecatedAPI
------------------------------------------------------------------------------------------------------------------------
setOutputControlParametersNodeSize = function(blendTree, x, y)
  deprecatedWarning("setOutputControlParametersNodeSize", "setEmittedControlParametersNodeSize", "3.0")
  return setEmittedControlParametersNodeSize(blendTree, x, y)
end
