------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: upgradeRenamedAttributes(string node, table attrs)
--| brief:
--|   Given a table of attributes attrs["OldName"] = "NewName" this function will copy the old
--|   value to the new value assuming the types are the same.
--|
--| environments: UpgradeEnv GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local buildUpgradeRenamedAttributes = function()
  local upgradeRenamedAttributes = function(node, attrs)
    local sets = listAnimSets()
    local setCount = table.getn(sets)

    for old, new in pairs(attrs) do
      local deprecatedName = string.format("deprecated_%s", old)
      local deprecatedPath = string.format("%s.%s", node, deprecatedName)
      local currentPath = string.format("%s.%s", node, new)
      local _, info = getType(deprecatedPath)

      if info.isSet and setCount > 1 then
        -- make sure the set order is the same
        local order = listAnimSetOrder(deprecatedPath)
        changeAnimSetOrder(currentPath, order)

        for _, set in ipairs(sets) do
          if hasAnimSetData(deprecatedPath, set) then
            -- copy the old value for this set
            local value = getAttribute(node, deprecatedName, set)
            setAttribute(currentPath, value, set)
          else
            -- ensure there is no data for this set
            destroyAnimSetData(currentPath, set)
          end
        end
      else
        -- just copy the old value
        local value = getAttribute(deprecatedPath)
        setAttribute(currentPath, value)
      end

      removeAttribute(node, deprecatedName)
    end
  end
  return upgradeRenamedAttributes
end
upgradeRenamedAttributes = buildUpgradeRenamedAttributes()

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: copyAttributesForUpgrade(table oldNode, string newNode)
--| brief:
--|   This function will copy the attributes from a table representing an old node
--|   to a new node. It is intended for use in onManifestChange handlers where a node
--|   is replaced with another.
--|
--| desc:
--|   All attributes that exist in oldNode and newNode are copied to newNode.
--|   All custom attributes that exist oldNode are copied to newNode
--|   All attributes that exist in oldNode but do not exist in new node are
--|   coppied but the name of the copy is prefaced by "deprecated_"
--| environments: UpgradeEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local buildCopyAttributesForUpgrade = function()
  local copyAttributesForUpgrade = function(oldNode, newNode)
    local allAttributesCopied = true
    for j, attr in oldNode.attributes do
      -- Get a path to the attributes
      local destAttrPath = ""
      if attr.custom then
        attr.path = newNode
        destAttrPath = addAttribute(attr)
      else
        destAttrPath = string.format("%s.%s", newNode, attr.name)

        -- if it does not then we need to create a custom attribute with this as a deprecated value
        if not attributeExists(destAttrPath) then
          attr.name = string.format("deprecated_%s", attr.name)
          attr.path = newNode
          destAttrPath = addAttribute(attr)
          allAttributesCopied = false
        end
      end

      -- Copy the values of the attributes
      if attributeExists(destAttrPath) then
        if attr.perAnimSet then
          for animSet, animSetValue in attr.sets do
            setAttribute(destAttrPath, animSetValue.value, animSet)
          end
        else
           setAttribute(destAttrPath, attr.value)
        end
      end
    end

    return allAttributesCopied
  end
  
  return copyAttributesForUpgrade
end
copyAttributesForUpgrade = buildCopyAttributesForUpgrade()

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: upgradeDeprecatedNodeToNewNode(string nodeType, table oldNodeTable, table pinLookupTable)
--| brief:
--|   This function creates a new node of the type nodeTypeName, copies the attributes and calls the
--|   upgrade function on the new node. The idea is that when changing the type of a node it should
--|   be possible to keep any existing upgrade functions largely unchanged.
--|
--| environments: UpgradeEnv GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local buildUpgradeDeprecatedNodeToNewNode = function()
  local upgradeDeprecatedNodeToNewNode = function(nodeType, oldNodeTable, pinLookupTable)
    -- create the replacement node
    local node = create(nodeType, oldNodeTable.path, oldNodeTable.name)

    -- set the position if it is a blend node
    local _, t = getType(node)
    if t == "BlendNode" then
      setNodePosition(node, oldNodeTable.x, oldNodeTable.y)
      setNodeCollapseState(node, oldNodeTable.collapseState)
    end

    -- set the attributes values on the new node
    copyAttributesForUpgrade(oldNodeTable, node)

    -- and call the upgrade function if there is one
    local upgradeFunction = getUpgradeFunction(nodeType)
    if type(upgradeFunction) == "function" then
      pcall(upgradeFunction, node, oldNodeTable.version, pinLookupTable)
    end

    local message = string.format(
      "Created %s '%s' of type '%s' to replace deprecated version of type '%s'",
      t,
      node,
      nodeType,
      oldNodeTable.type)
    app.info(message)

    return node
  end
  return upgradeDeprecatedNodeToNewNode
end
upgradeDeprecatedNodeToNewNode = buildUpgradeDeprecatedNodeToNewNode()

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: deprecateNodeWithNewNode(string oldNode, string newType, table pinLookupTable)
--| brief:
--|   This function creates a new node of the type newType, copies the attributes from oldNode.
--|   oldNode is deleted.
--|
--| environments: UpgradeEnv GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local buildReplaceNodeWithNewNode = function()
  local replaceNodeWithNewNode = function(oldNode, newType, pinLookupTable)
    
    local oldNodePath, oldNodeName = splitNodePath(oldNode)
    oldNode = rename(oldNode, "Old_" .. oldNodeName)

    -- create the replacement node
    local newNode = create(newType, oldNodePath, oldNodeName)
    local x, y, colapseState
    
    oldNodePath, oldNodeName = splitNodePath(oldNode)

    -- set the position if it is a blend node
    local oldType, t = getType(oldNode)
    if t == "BlendNode" then
      x, y = getNodePosition(oldNode)
      colapseState = getNodeCollapseState(oldNode)
    end
   
    -- copy the attributes
    local attributes = listAttributes(oldNode)
    for i, attribute in ipairs(attributes) do
      local oldPath = string.format("%s.%s", oldNode, attribute)
      local newPath = string.format("%s.%s", newNode, attribute)
      local value = getAttribute(oldPath)
      local isDeprecated = string.find(attribute, "deprecated_")
      local attributeType = getAttributeType(oldPath)
      
      if attributeExists(newNode, attribute) then
        setAttribute(newPath, value)
      elseif isDeprecated then
        local nonDeprecatedAttribute = string.gsub(attribute, "deprecated_", "")
        if attributeExists(newNode, nonDeprecatedAttribute) then
          local newPath = string.format("%s.%s", newNode, nonDeprecatedAttribute)
          setAttribute(newPath, value)
        else
          addAttribute{ 
            path = newNode,
            name = attribute,
            type = attributeType,
            value = value,
          }
        end
      else
        addAttribute{ 
          path = newNode,
          name = "deprecated_" .. attribute,
          type = attributeType,
          value = value,
        }
      end
    end

    local message = string.format(
      "Replaced %s '%s' with type '%s' to replace a deprecated version of type '%s'",
      t,
      newNode,
      newType,
      oldType)
    app.info(message)

    delete(oldNode)
    if t == "BlendNode" then
      setNodePosition(newNode, x, y)
      setNodeCollapseState(newNode, colapseState)
    end

    return node
  end
  return replaceNodeWithNewNode
end
replaceNodeWithNewNode = buildReplaceNodeWithNewNode()

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: upgradeDeprecatedConditionToNewCondition(string conditionType, table oldNodeTable)
--| brief:
--|   This function creates a new condition of the type conditionType, copies the attributes and calls the
--|   upgrade function on the new condition. The idea is that when changing the type of a node it should
--|   be possible to keep any existing upgrade functions largely unchanged.
--|
--| environments: UpgradeEnv GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local buildUpgradeDeprecatedConditionToNewCondition = function()
  local upgradeDeprecatedConditionToNewCondition = function(conditionType, oldConditionTable)
    -- create the replacement node
    local condition = create(conditionType, oldConditionTable.path, oldConditionTable.name)

    local _, t = getType(condition)
    
    -- set the attributes values on the new node
    copyAttributesForUpgrade(oldConditionTable, condition)

    -- and call the upgrade function if there is one
    local upgradeFunction = getUpgradeFunction(conditionType)
    if type(upgradeFunction) == "function" then
      upgradeFunction(condition, oldConditionTable.version);
    end

    local message = string.format(
      "Created %s '%s' of type '%s' to replace deprecated version of type '%s'",
      t,
      condition,
      conditionType,
      oldConditionTable.type)
    app.info(message)

    return condition
  end
  return upgradeDeprecatedConditionToNewCondition
end
upgradeDeprecatedConditionToNewCondition = buildUpgradeDeprecatedConditionToNewCondition()

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: copyConditionsForUpgrade(table oldNode, string newNode, table conditionRenameTable)
--| brief:
--|   This function will copy the conditions from a table representing an old transition
--|   to a new node. It is intended for use in onManifestChange handlers where a node
--|   is replaced with another. You must include a map of the for {"OldConditionName" = "NewConditionName"} that
--|   contains all the renamed conditions.
--|
--| desc:
--|   All conditions that exist in oldNode and newNode are copied to newNode.
--| environments: UpgradeEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local buildCopyConditionsForUpgrade = function()
  local copyConditionsForUpgrade = function(oldNode, newNode, conditionRenameTable)
  
    -- iterate over the old conditions
    for j, oldCond in oldNode.conditions do 
      -- deprecated conditions are handled elsewhere...
      
      local newType = oldCond.type
      if(conditionRenameTable[oldCond.type] ~= nil) then 
         newType = conditionRenameTable[oldCond.type]
      end
      
      if manifest.getType(newType) == "Condition" then
        local condition = create(newType, newNode, oldCond.name)
       
        -- set the attributes values on the new node
        copyAttributesForUpgrade(oldCond, condition)
        
        -- and call the upgrade function if there is one
        local upgradeFunction = getUpgradeFunction(newType)
        if type(upgradeFunction) == "function" then
          upgradeFunction(condition, oldCond.version);
        end
      end
    end
  end
  return copyConditionsForUpgrade
end
copyConditionsForUpgrade = buildCopyConditionsForUpgrade()

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: upgradeDeprecatedTransitAtEventToTransit(table oldNodeTable, function preUpgradeFunction)
--| brief:
--|   This function creates a Transit and copies the attributes and conditions to the new node.
--|   preUpgradeFunction is called before the nodes upgrade and after all the conditions and attributes
--|   have been migrated to the new node, allowing migrating attributes specific to transit at event to transit.
--|
--| environments: UpgradeEnv GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local buildUpgradeDeprecatedTransition = function()
  local upgradeDeprecatedTransition = function(nodeType, oldNodeTable, preUpgradeFunction, conditionRenameTable)
    -- create the replacement node
    local node = create(nodeType, string.format("%s|%s", oldNodeTable.path, oldNodeTable.from), string.format("%s|%s", oldNodeTable.path, oldNodeTable.to), oldNodeTable.name)

    -- set the attributes values on the new node
    copyAttributesForUpgrade(oldNodeTable, node)

    -- copy the old conditions on the transtion to the new transtion
    copyConditionsForUpgrade(oldNodeTable, node, conditionRenameTable)

    local deadblend = string.format("%s|%s", node, "DeadBlend")
    local deadblendProperties = oldNodeTable["deadblend"]
    
    if objectExists(deadblend) and type(deadblendProperties) == "table" then
      copyAttributesForUpgrade(deadblendProperties, deadblend)      
    end

    if type(preUpgradeFunction) == "function" then
      preUpgradeFunction(oldNodeTable, node)
    end
    
    -- and call the upgrade function if there is one
    local upgradeFunction = getUpgradeFunction(nodeType)
    if type(upgradeFunction) == "function" then
      upgradeFunction(node, oldNodeTable.version);
    end

    local message = string.format(
      "Created %s '%s' of type '%s' to replace deprecated version of type '%s'",
      "Transition",
      node,
      nodeType,
      oldNodeTable.type)
    app.info(message)

    return node
  end
  return upgradeDeprecatedTransition
end
upgradeDeprecatedTransition = buildUpgradeDeprecatedTransition()

------------------------------------------------------------------------------------------------------------------------
local buildCombineNodeMatchEventsWithNode = function()
  local combineNodeMatchEventsWithNode = function(nodeType, oldNodeTable)
    local newNode = upgradeDeprecatedNodeToNewNode(nodeType, oldNodeTable)

    -- then set the TimeStretchMode to Match Events and the PassThroughMode to None
    -- this must be done after the upgrade function as the upgrade function may set the
    -- TimeStretchMode and PassThroughMode to the values for the non MatchEvents version
    -- of this type.
    local timeStretchModeName = string.format("%s.TimeStretchMode", newNode)
    if attributeExists(timeStretchModeName) then
      setAttribute(timeStretchModeName, 1)

      -- manifest callbacks are disabled during upgrade so we have to make sure the pin interfaces match up
      -- to the attribute values, if the node used to be a match events node then be sure to add the events
      -- interface to all FunctionalPins.
      local pins = listPins(newNode)
      for _, pinName in ipairs(pins) do
        local pin = string.format("%s.%s", newNode, pinName)

        if getType(pin) == "FunctionalPin" then
          addPinInterfaces(pin, { "Events", })
        end
      end
    end

    local passThroughModeName = string.format("%s.PassThroughMode", newNode)
    if attributeExists(passThroughModeName) then
      setAttribute(passThroughModeName, 2)
    end
  end
  return combineNodeMatchEventsWithNode
end
combineNodeMatchEventsWithNode = buildCombineNodeMatchEventsWithNode()

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean copyAnimSetAttribute(string node, string srce, string dest, function convertFunction)
--| brief:
--|   Copies the values of a per anim set attribute with dependent data using a user
--|   supplied conversion function.
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local buildCopyAnimSetAttribute = function()
  local copyAnimSetAttribute = function(node, srce, dest, convertFunction)
    local srcePath = string.format("%s.%s", node, srce)
    local destPath = string.format("%s.%s", node, dest)

    -- the first value *must* exist so we just copy it without testing
    local srceOrder = listAnimSetOrder(srcePath)
    local animSet = srceOrder[1]
    local srceValue = getAttribute(node, srce, animSet)
    local destValue = convertFunction(srceValue, animSet)
    setAttribute(destPath, destValue, animSet)

    -- make sure that the desitnation has its dependent data in the same order
    changeAnimSetOrder(destPath, srceOrder)

    -- finally loop through all of the remaining values setting then
    local totalAnimSets = table.getn(srceOrder)
    for i = 2, totalAnimSets do
      animSet = srceOrder[i]
      local hasData = hasAnimSetData(srcePath, animSet)
      if hasData then
        srceValue = getAttribute(node, srce, animSet)
        destValue = convertFunction(srceValue, animSet)
        setAttribute(destPath, destValue, animSet)
      end
    end
  end
  return copyAnimSetAttribute
end
copyAnimSetAttribute = buildCopyAnimSetAttribute()

------------------------------------------------------------------------------------------------------------------------
-- is the object passed in a transition
------------------------------------------------------------------------------------------------------------------------
local isTransition = function(object)
  assert(type(object) == "string")
  -- no assert for zero length strings as that is a valid path to the network root

  local _, basetype = getType(object)
  return basetype == "Transition"
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean isSelfTransition(string transition)
--| brief:
--|   Check if the object passed in is a transition to self.
--|   Checks to see if the source and destination states are the same.
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
isSelfTransition = function(transition)
  if not isTransition(transition) then
    return false
  end

  local inputs = listConnections{ Object = transition, Upstream = true, Downstream = false }
  assert(table.getn(inputs) == 1)
  local source = inputs[1]

  -- this call may return transition from transitions as well but the destination state should always
  -- be first in the array, assert this to make sure
  local outputs = listConnections{ Object = transition, Upstream = false, Downstream = true }
  local destination = outputs[1]
  local _, basetype = getType(destination)
  assert(basetype == "StateMachineNode")

  return inputs[1] == outputs[1]
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean isTransitionFromActiveState(string transition)
--| brief: Check if the object passed in is a transition from an ActiveState StateMachineNode.
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
isTransitionFromActiveState = function(transition)
  if not isTransition(transition) then
    return false
  end

  local inputs = listConnections{ Object = transition, Upstream = true, Downstream = false }
  assert(table.getn(inputs) == 1)
  local source = inputs[1]

  local type, subtype = getType(source)
  
  if(subtype == "StateMachineNode") then
    return type == "ActiveState"
  else 
    return false
  end
  
end

------------------------------------------------------------------------------------------------------------------------
-- this code will add the manifest upgrade related functions to the Upgrade lua environment
local environment = app.getLuaEnvironment("Upgrade")
app.registerToEnvironment(buildUpgradeRenamedAttributes(), "upgradeRenamedAttributes", environment)
app.registerToEnvironment(buildUpgradeDeprecatedTransition(), "upgradeDeprecatedTransition", environment)
app.registerToEnvironment(buildCopyConditionsForUpgrade(), "copyConditionsForUpgrade", environment)
app.registerToEnvironment(buildCopyAttributesForUpgrade(), "copyAttributesForUpgrade", environment)
app.registerToEnvironment(buildUpgradeDeprecatedNodeToNewNode(), "upgradeDeprecatedNodeToNewNode", environment)
app.registerToEnvironment(buildCombineNodeMatchEventsWithNode(), "combineNodeMatchEventsWithNode", environment)
app.registerToEnvironment(buildCopyAnimSetAttribute(), "copyAnimSetAttribute", environment)
app.registerToEnvironment(buildUpgradeDeprecatedConditionToNewCondition(), "upgradeDeprecatedConditionToNewCondition", environment)
app.registerToEnvironment(buildReplaceNodeWithNewNode(), "replaceNodeWithNewNode", environment)

------------------------------------------------------------------------------------------------------------------------
-- get the actual node path for a weak reference, this path should not be used to find a network node ID, as this 
-- circumvents the point of a weak refernce.
getPathFromWeakReferenceValue = function(weakReferenceValue)
  -- Path is expected as WEAKREF:Path
  return string.sub(weakReferenceValue, 9)  
end

------------------------------------------------------------------------------------------------------------------------
-- gets attributes:
--    "Action"+inputReqIndexString
--    "EmittedRequest"+inputReqIndexString
--    "Target"+inputReqIndexString -- a weak ref type
-- writes Attributes:
--    "ActionID_"+inputReqIndexString
--    "EmittedMessageID_"+inputReqIndexString
--    "Broadcast_"+inputReqIndexString
--    "TargetNodePath_"+inputReqIndexString
local serializeRequest = function(node, stream, inputReqLabel, outputReqLabel, writeIfNotSpecified)
  
  if (writeIfNotSpecified == nil) then
    writeIfNotSpecified = true
  end
  
  local reqPathEmittedRequest = getAttribute(node, ("EmittedRequest" .. inputReqLabel))
  local reqAction = getAttribute(node, ("Action" .. inputReqLabel))   -- Action set/clear
      
  -- Actions (0 = "Not Specified"(UNUSED), 1 = "Set"(SET), 2 = "Clear"(CLEAR), 3 = "ClearAll"(RESET))  
  -- Note: The runtime enumeration RequestType assumes this order. If a change is made it must be made in both places.
  if ((reqPathEmittedRequest == "") and (reqAction ~= "Clear All")) then
    if (writeIfNotSpecified == true) then
      stream:writeUInt(0, ("ActionID_" .. outputReqLabel)) -- Not Specified
    end
    return false -- No request exported.
  elseif (reqAction == "Set") then
    stream:writeUInt(1, ("ActionID_" .. outputReqLabel)) -- Set
  elseif (reqAction == "Clear") then
    stream:writeUInt(2, ("ActionID_" .. outputReqLabel)) -- Clear
  elseif (reqAction == "Clear All") then
    stream:writeUInt(3, ("ActionID_" .. outputReqLabel)) -- All
  end
    
  if (reqAction ~= "Clear All") then      
    local reqIdEmittedRequest = target.getRequestID(reqPathEmittedRequest)
    stream:writeUInt(reqIdEmittedRequest, ("EmittedMessageID_" .. outputReqLabel)) 
  end
  
  local reqPathTarget = getPathFromWeakReferenceValue(getAttribute(node, ("Target" .. inputReqLabel)))   -- Target state machine or broadcast
  if (reqPathTarget == "") then
    stream:writeBool(true, ("Broadcast_" .. outputReqLabel))
  else
    stream:writeString(reqPathTarget, ("TargetNodePath_" .. outputReqLabel)) -- Writing the path ID for now until we have the ability to resolve a node ID.
  end
  
  return true   -- Exported a valid request.
  
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: nil writeAttributesInOrder(string nodePath, stream stream)
--| brief:Write the attrihbutes to the stream in the order they are returned by listAttributes
--| environments: ValidateSerialize
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local writeAttributesInOrder = function(node, stream) 
   local attributes = listAttributes(node)
   for i, attribute in ipairs(attributes) do 
     if isManifestAttribute(node, attribute) then
       local attrPath = node .. "." .. attribute
       local attrType = getAttributeType(node, attribute)
       local attrValue = getAttribute(node, attribute)
       if attrType == "float" then
         stream:writeFloat(attrValue, attribute)
       elseif attrType == "bool" then
         stream:writeBool(attrValue, attribute)
       elseif attrType == "int" then
         stream:writeInt(attrValue, attribute)
       elseif attrType == "string" then
         stream:writeString(attrValue, attribute)
       end
     end
   end

end
------------------------------------------------------------------------------------------------------------------------
-- this code will add the manifest upgrade related functions to the Upgrade lua environment
local environment = app.getLuaEnvironment("ValidateSerialize")
app.registerToEnvironment(serializeRequest, "serializeRequest", environment)
app.registerToEnvironment(writeAttributesInOrder, "writeAttributesInOrder", environment)
app.registerToEnvironment(getPathFromWeakReferenceValue, "getPathFromWeakReferenceValue", environment)

------------------------------------------------------------------------------------------------------------------------
-- Please note:  Currently there is a large hack in place to do with
-- trajectories.  XMD provides a channel list that is one less than the
-- number of actual bones in the rig. This is because runtime inserts
-- a bone at the top of the hierarchy (ID 0).  This ID can't be filtered
-- and should be ignored in all connect logic.
-- All export code will therefore remap ID 0 to ID 1
-- I.E ChannelIsOutput[0] = RuntimeRigChannel[1]

-- This is a utility function for the above hack
------------------------------------------------------------------------------------------------------------------------
getTransformChannelsAndStripZero = function(node)
  local transformChannels = anim.getTransformChannels(node)
  -- need to remove channel 0 from the returned the result
  for i, v in ipairs(transformChannels) do
    if v == 0 then
      table.remove(transformChannels, i)
      break
    end
  end
  return transformChannels
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean makePinGroup(table pinDetails)
--| brief: This is a utility function that makes it easy to build the hirechical table that is used to define pin groups.
--|        for example 
--|        makePinGroup{
--|          {name = "DoHold", pin = "DoHold"},
--|          {name = "Edge", hint = "EdgeNormal"},
--|          {name = "Edge|Start", pin = "EdgeStart", hint = "Position"},
--|          {name = "Edge|End", pin = "EdgeEnd", hint = "Position"},
--|          {name = "Edge|Normal", pin = "EdgeNormal", hint = "Normal"},
--|        }
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
makePinGroup = function(simpleTable)
  local result = { }
  local mapping = { }
  for _, item in ipairs(simpleTable) do
    local path, name = splitStringAtLastOccurence(item.name, "|")
    if path == nil then
      local newItem = {}
      table.insert(result, newItem)
      newItem.name = item.name
      newItem.hint = item.hint
      newItem.pin = item.pin
      newItem.collapsed = item.collapsed
      mapping[newItem.name] = newItem
    else
      local foundItem = mapping[path]
      if foundItem ~= nil then
        if foundItem.members == nil then
          foundItem.members = {}
        end
        local newItem = {}
        table.insert(foundItem.members, newItem)
        newItem.name = name
        newItem.hint = item.hint
        newItem.pin = item.pin
        newItem.collapsed = item.collapsed
        mapping[item.name] = newItem
      else
        app.warning(string.format("makePinGroup could not find a group with the path \"%s\"", path))
      end
   end
  end
  
  return result
end
