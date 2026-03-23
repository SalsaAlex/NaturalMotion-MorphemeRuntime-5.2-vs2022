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
--| signature: parent, node splitNodePath(string nodepath)
--| brief:
--|   Splits a path to a node into its parent's path and the node name.
--|   ie splitNodePath("BlendTree1|StateMachine") would return "BlendTree1" and "StateMachine"
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local function buildSplitNodePath()
  local splitNodePath = function(nodepath)
    local path, node = splitStringAtLastOccurence(nodepath, "|")
    if not path then
      -- no |, just return nodename as-is
      return "", nodepath
    end
    return path, node
  end
  return splitNodePath
end
splitNodePath = buildSplitNodePath()

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: parent, node splitAttributePath(string nodepath)
--| brief:
--|   Splits a path to an attribute into its parent nodes path and the attribute name.
--|   ie splitAttributePath("BlendTree1|StateMachine|Transit1.Duration")
--|   would return "BlendTree1|StateMachine|Transit1" and "Duration"
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local function buildSplitAttributePath()
  local splitAttributePath = function(fullpath)
    local node, attribute = splitStringAtLastOccurence(fullpath, "%.")
    if not node then
      return "", fullpath
    end
    return node, attribute
  end
  return splitAttributePath
end
splitAttributePath = buildSplitAttributePath()
buildSplitPinPath = buildSplitAttributePath

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: parent, node splitPinPath(string nodepath)
--| brief:
--|   Splits a path to a pin into its parent node's path and the pin name.
--|   ie splitAttributePath("BlendTree1|Blend2.Result")
--|   would return "BlendTree1|Blend2" and "Result"
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
splitPinPath = splitAttributePath

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean hasTransitionCategory(table paths, string transitionCategory)
--| brief:
--|   Returns if a selection of transitions contains one or more transitions of a given type.
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
hasTransitionCategory = function(paths, transitionCategory)
  for _1, transition in pairs(paths) do
    local thisType = getTransitionCategory(transition)
    if thisType == transitionCategory then
      return true
    end
  end
  
  return false
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean containsReference(table paths)
--| brief:
--|   Check if any objects in the table of paths are referenced.
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
containsReference = function(paths)
  local hasReference = false
  for i, path in ipairs(paths) do
    if isReferenced(path) then
      hasReference = true
      break
    end
  end
  return hasReference
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean nodeContainsChildType(string node, string childType)
--| brief:
--|   Check if a node contains a child of a particular type.
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
nodeContainsChildType = function(node, childType)
  local children = listChildren(node)
  for i, child in ipairs(children) do
    if getType(child) == childType then
      return true
    end
  end
  return false
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string getFirstChildOfType(string node, string childType)
--| brief:
--|   Return name of first child of given type.
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
getFirstChildOfType = function(node, childType)
  local children = listChildren(node)
  for i, child in ipairs(children) do
    if getType(child) == childType then
      return child
    end
  end
  return nil
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean nodeContainsChild(string node, string childName)
--| brief:
--|   Check if a node contains a child of a particular name.
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
nodeContainsChild = function(node, childName)
  local children = listChildren(node)
  for i, child in ipairs(children) do
    local _, testName = splitNodePath(child)
    if testName == childName then
      return true
    end
  end
  return false
end

------------------------------------------------------------------------------------------------------------------------
-- this code adds the split path style functions to all other lua environments
local functions = {}
functions["splitNodePath"] = buildSplitNodePath
functions["splitAttributePath"] = buildSplitAttributePath
functions["splitPinPath"] = buildSplitPinPath

local environments = app.listLuaEnvironments()
for name, environment in pairs(environments) do
  for func, builder in pairs(functions) do
    app.registerToEnvironment(builder(), func, environment)
  end
end

