------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------
components = { }

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: PAGE
--| name: ComponentEditorAPI
--| title: Animation Formats
--| desc:
--|   Lua-scripted functions related to components and manipulating component options strings.
------------------------------------------------------------------------------------------------------------------------

-- note re-executing this script will clear all currently registered components
local registeredComponents = { } -- all the registered components
local registeredCharacterComponents = { } -- components registered for a particular character type

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table components.ls()
--| brief:
--|   Returns all currently registered components
--|
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
components.ls = function()
  return registeredComponents
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table components.listNames()
--| brief:
--|   Returns the names of all currently registered components
--|
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
components.listNames = function()
  local result = { }
  for _, component in ipairs(registeredComponents) do
    table.insert(result, component.name)
  end
  
  return result
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table components.listNamesForType()
--| brief:
--|   Returns the names of all the components regiestered for a character type
--|
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
components.listNamesForType = function(characterType)
  local result = { }
  local components = registeredCharacterComponents[characterType]
  if components then
    for _, component in components do
      table.insert(result, component.name)
    end
  end
  
  return result
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table components.listForType(string characterType)
--| brief:
--|   Returns the components regiestered for a character type
--|
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
components.listForType = function(characterType)
  local components = registeredCharacterComponents[characterType]
  return components
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table components.get(string component)
--| brief:
--|   Get a registered component type from the list of registered components. Returns nil
--|   if there is no registered component of that name.
--|
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
components.get = function(name)
  if type(name) ~= "string" or string.len(name) == 0 then
    return false
  end

  for _, component in ipairs(registeredComponents) do
    if component.name == name then
       return component
    end
  end
  
  return false
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean components.unregister(string component)
--| brief:
--|   Unregisters an animation components. Returns false if the component was not registered or the component
--|   specified was invalid.
--|
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
components.unregister = function(name)
  if type(name) ~= "string" or string.len(name) == 0 then
    return false
  end

  local removedComponent = false;
  for i, component in ipairs(registeredComponents) do
    if component.name == name then
      table.remove(registeredComponents, i)
      removedComponent = true
      break
    end
  end

  if removedComponent then
    for _, components in pairs(registeredCharacterComponents) do
      for i, component in ipairs(components) do
        if component.name == name then
          table.remove(components, i)
          break
        end
      end
    end
  end

  return removedComponent
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean components.register(string name, string component, table characterTypes, function validationFunction, function panelFunction, function headerFunction, table undoFunctions)
--| brief:
--|   Registers a component with connect. Will overwrite any existing component of the same name.
--|
--|   undoFunctions should be nil or a table containing some or all of these functions:
--|        {
--|          CanUndo = function() end,
--|          CanRedo = function() end,
--|          Undo = function() end,
--|          Redo = function() end,
--|        }
--|
--|   Returns false if the component specified was invalid.
--|
--| environments: GlobalEnv
--| page: ComponentEditorAPI
------------------------------------------------------------------------------------------------------------------------
components.register = function(name, characterTypes, validationFunction, panelFunction, headerFunction, componentFunction, undoFunctionTable)
  -- name has to be a string
  if type(name) ~= "string" or string.len(name) == 0 then
    return false
  end
  
  if type(validationFunction) ~= "function" then
    return false
  end

  if type(panelFunction) ~= "function" then
    return false
  end

  -- remove any component that has been registered
  components.unregister(name)
  
  -- registering a new component
  local component = {
    name = name,
    characterTypes = characterTypes,
    validationFunction = validationFunction,
    panelFunction = panelFunction,
    headerFunction = headerFunction,
    componentFunction = componentFunction,
    undoFunctions = undoFunctionTable,
  }
  
  table.insert(registeredComponents, component)
  for _, characterType in characterTypes do
    if registeredCharacterComponents[characterType] == null then
      registeredCharacterComponents[characterType] = { }
    end
    table.insert(registeredCharacterComponents[characterType], component)
  end
  return true
end
