------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

-- Startup script
-- Is run after application is fully up.

local appScriptsDir = app.getAppScriptsDir()
appScriptsDir = appScriptsDir:gsub("\\", "/")

-- set this file as loaded so other startup files that require this do not re-execute it
_LOADED["startup.lua"] = true
local filename = string.format("%sstartup.lua", appScriptsDir)
_LOADED[filename] = true

-- prepend the app scripts directory to the default lua path
LUA_PATH = string.format([[%s?;\?;?;?.lua]], appScriptsDir)

require "init_events.lua"

local listFiles = nil -- this forward allows this local to be called recursively in below function
listFiles = function(directory, extension)
  local subDirectories = app.enumerateDirectories(directory, "")
  local result = { }
  local index = 1

  for i=1, table.getn(subDirectories) do
    local files = listFiles(subDirectories[i], extension)
    for j=1, table.getn(files) do
      result[index] = files[j]
      index = index + 1
    end
  end

  local files = app.enumerateFiles(directory, extension)
  for i=1, table.getn(files) do
    result[index] = files[i]
    index = index + 1
  end
  return result
end

fileExists = function(filename)
  local handle = io.open(filename)
  if io.type(handle) == "file" then
    handle:close()
    return true
  end
  return false
end

-- store the standard version of require as a global
-- the check for nil ensures this script can be run more than once
if _standard_require == nil then
  _standard_require = require
end

-- now provide a new implementation of require with some extra logging and path standardisation
require = function(filename)
  assert(_standard_require)

  -- strip the app scripts dir from the name for require
  filename = filename:gsub("/", "\\")
  local appScriptsDir = app.getAppScriptsDir()
  -- the true is to turn off the pattern matching facilities of string.find
  -- if pattern matching is enabled then spaces in the file path for the scripts dir
  -- cause the find operation to fail
  local first, last = string.find(filename, appScriptsDir, 1, true)
  if last then
    filename = string.sub(filename, last + 1)
  end

  filename = filename:gsub("\\", "/")
  if _LOADED[filename] == nil then
    local status, error = pcall(_standard_require, filename)
    if status then
      app.info(string.format("executed script %q", filename))
    else
      if not mcn.inCommandLineMode() then
        ui.showErrorMessageBox{
          title = "Script Compilation Error",
          message = error,
        }
      end
      app.error(error)
    end
  end
end

executeAllScripts = function(directory)
  local files = listFiles(directory, "*.lua")
  for i=1, table.getn(files) do
    require(files[i])
  end
end

-- Initialise the environments before any of the other scripts are run.
--
require "init_environments.lua"

local appScriptsDir = app.getAppScriptsDir()

-- load API functions
local luaAPIScriptsDir = string.format("%sluaAPI", appScriptsDir)
executeAllScripts(luaAPIScriptsDir)

require "init_settings.lua"

if not mcn.inCommandLineMode() then
  require "init_ui.lua"
end
require "init_manifest.lua"
require "init_assetmanager.lua"

-- Register the character types
characterTypes.register("Animation",  {"AnimationRig"})
if not mcn.isPhysicsDisabled() then
  characterTypes.register("Physics",  {"AnimationRig", "PhysicsRig"})
end
if not mcn.isEuphoriaDisabled() then
  characterTypes.register("Euphoria", {"AnimationRig", "PhysicsRig"})
end

-- If the asset compiler configurations have been moved to a different folder then change this directory accordingly e.g.

--[[
preferences.set{
  name = "AssetCompilerConfigurationDirectory",
  value = [[\assetCompilerConfigurationDirectory\]]
}
--]]
