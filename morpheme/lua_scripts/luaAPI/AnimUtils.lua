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
--| signature: string anim.getParentBoneIndex(number boneIndex, string setName)
--| brief:
--|   Get parent of the given bone in the given animation set.  Returns nil for invalid set names
--|   or invalid bone indices.
--|
--| environments: ValidateSerializeEnv GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local function buildAnimGetParentBoneIndex()
  local animGetParentBoneIndex = function(boneIndex, setName)
    local rig = anim.getRigHierarchy(setName)
    if rig == nil then
      print("Invalid set name passed to anim.getParentBoneIndex()")
      return nil
    end
    for k, mapping in rig do
      if mapping.index == boneIndex then
        return mapping.parentIndex
      end
    end
    print("Invalid bone index passed to anim.getParentBoneIndex() for animation set " .. setName)
    return nil
  end
  return animGetParentBoneIndex
end
anim.getParentBoneIndex = buildAnimGetParentBoneIndex()

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string anim.listTrackEventsInOrder(string trackPath)
--| brief:
--|   Returns a table of track events in order of time position for the given track path.  An example valid
--|   trackPath is "$(RootDir)/Animation/XMD/01002_Walk.xmd|untitled|Footsteps".
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
anim.listTrackEventsInOrder = function(trackPath)
  if anim.getType(trackPath) ~= "Track" then
    error("anim.listTrackEventsInOrder() expects animation track as first argument")
  end

  local events = anim.ls(trackPath)

  table.sort(
    events,
    function(lhs, rhs)
      local lhsPos = anim.getAttribute(lhs .. ".TimePosition")
      local rhsPos = anim.getAttribute(rhs .. ".TimePosition")
      return lhsPos < rhsPos
    end)

  return events
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean anim.clearAnimCache()
--| brief:
--|   Clears the current animation cache.
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
anim.clearAnimCache = function()
  -- get the current target's asset compiler
  local activeTarget = target.getActive()
  if activeTarget == nil then
    local message = "There must be an active runtime target with a valid asset compiler to clear the anim cache."
    ui.showErrorMessageBox(message, "Error clearing anim cache")
  end

  if activeTarget:hasAssetCompiler() then
    local currentAssetCompiler = utils.demacroizeString(activeTarget:getAssetCompilerPath())
    local fullAssetCompiler = utils.demacroizeString(currentAssetCompiler)

    -- check the current asset compiler exists
    if app.fileExists(fullAssetCompiler) then
      -- get the anim cache dir and check it exists
      local cacheDir = anim.getAnimCacheDir()
      local fullCacheDir = utils.demacroizeString(cacheDir)
      if app.directoryExists(fullCacheDir) then
        -- create the command line string to be executed and execute it
        local command = string.format("%q -clean true -cacheDir %q", fullAssetCompiler, fullCacheDir)
        local result = app.execute(string.format("\"%s\"", command), false, true)
        if result == 0 then
          return true
        end

        local message = string.format(
          "The asset compiler '%s'\nspecified for the currently active runtime target '%s' failed to clear the cache.",
          currentAssetCompiler,
          activeTarget:getName())
        ui.showErrorMessageBox(message, "Error clearing anim cache")
      else
        local message = string.format("The anim cache directory '%s' does not exist.", cacheDir)
        ui.showErrorMessageBox(message, "Error clearing anim cache")
      end
    else
      local message = string.format(
        "The asset compiler '%s'\nspecified for the currently active runtime target '%s' does not exist.",
        currentAssetCompiler,
        activeTarget:getName())
      ui.showErrorMessageBox(message, "Error clearing anim cache")
    end
  else
    local message = string.format("There is no asset compiler specified for the currently active runtime target '%s'.", activeTarget:getName())
    ui.showErrorMessageBox(message, "Error clearing anim cache")
  end

  return false
end

local environment = app.getLuaEnvironment("ValidateSerialize")

 -- Make sure the anim table is created.
environment.anim.getParentBoneIndex = buildAnimGetParentBoneIndex()

 -- Go through the registration process.
app.registerToEnvironment(environment.anim.getParentBoneIndex, "getParentBoneIndex", environment, environment.anim)