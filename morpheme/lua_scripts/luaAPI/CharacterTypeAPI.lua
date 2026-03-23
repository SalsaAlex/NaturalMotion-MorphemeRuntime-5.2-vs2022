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
--| name: CharacterTypeAPI
--| title: Animation Formats
--| desc:
--|   Lua-scripted functions related to characterTypes and manipulating characterType options strings.
------------------------------------------------------------------------------------------------------------------------

characterTypes = { }

-- note re-executing this script will clear all currently registered characterTypes
local registeredCharacterTypes = { }

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table characterTypes.ls()
--| brief:
--|   Returns all currently registered characterTypes
--|
--| environments: GlobalEnv
--| page: CharacterTypeAPI
------------------------------------------------------------------------------------------------------------------------
characterTypes.ls = function()
  return characterType
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table characterTypes.listNames()
--| brief:
--|   Returns the names of all currently registered characterTypes
--|
--| environments: GlobalEnv
--| page: CharacterTypeAPI
------------------------------------------------------------------------------------------------------------------------
characterTypes.listNames = function()
  local result = { }
  for _, characterType in registeredCharacterTypes do
    table.insert(result, characterType.name)
  end
  
  return result
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table characterTypes.get(string characterType)
--| brief:
--|   Get a registered characterType type from the list of registered characterTypes. Returns nil
--|   if there is no registered characterType of that name.
--|
--| environments: GlobalEnv
--| page: CharacterTypeAPI
------------------------------------------------------------------------------------------------------------------------
characterTypes.get = function(name)
  if type(name) ~= "string" or string.len(name) == 0 then
    return false
  end

  for _, characterType in registeredCharacterTypes do
    if characterType.name == name then
       return characterType
    end
  end
  
  return false;
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean characterTypes.supportsRig(string characterType, string rigName)
--| brief:
--|   Returns if a given character type supports a rig.
--|
--| environments: GlobalEnv
--| page: CharacterTypeAPI
------------------------------------------------------------------------------------------------------------------------
characterTypes.supportsRig = function(name, rigName)
  if type(name) ~= "string" or string.len(name) == 0 then
    return false
  end
  if type(rigName) ~= "string" or string.len(rigName) == 0 then
    return false
  end
  for _1, characterType in registeredCharacterTypes do
    if characterType.name == name then
      for _2, theRig in characterType.rigs do
        if theRig == rigName then
          return true
        end
      end
      
      return false
    end
  end
  
  return false;
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean characterTypes.unregister(string characterType)
--| brief:
--|   Unregisters an animation characterTypes. Returns false if the characterType was not registered or the characterType
--|   specified was invalid.
--|
--| environments: GlobalEnv
--| page: CharacterTypeAPI
------------------------------------------------------------------------------------------------------------------------
characterTypes.unregister = function(name)
  if type(name) ~= "string" or string.len(name) == 0 then
    return false
  end

  for i, characterType in ipairs(registeredCharacterTypes) do
    if characterType.name == name then
      table.remove(registeredCharacterTypes, i)
      return true
    end
  end

  return false
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean characterTypes.register(string characterType, table rigs)
--| brief:
--|   Registers a characterType with connect. Will overwrite any existing characterType of the same name.
--|
--|   Returns false if the characterType specified was invalid.
--|
--| environments: GlobalEnv
--| page: CharacterTypeAPI
------------------------------------------------------------------------------------------------------------------------
characterTypes.register = function(name, rigs)
  -- name has to be a string
  if type(name) ~= "string" or string.len(name) == 0 then
    return false
  end

  -- remove any characterType that has been registered
  characterTypes.unregister(name)
  
  -- registering a new characterType
  local characterType = {
    name = name,
    rigs = rigs,
  }
  
  table.insert(registeredCharacterTypes, characterType)
  return true
end
