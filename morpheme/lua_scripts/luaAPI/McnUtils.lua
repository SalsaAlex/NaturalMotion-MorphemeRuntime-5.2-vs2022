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
--| signature: string app.loadImage(string filename)
--| brief:
--|   Load images from the ui directory
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local appLoadImageCache = { }
app.loadImage = function(filename)
  if mcn.inCommandLineMode() then
    return nil
  end

  filename = filename or ""
  local fullname = utils.demacroizeString("$(AppRoot)resources\\images\\ui\\" .. filename)

  -- Cache image if not already referenced.
  if not appLoadImageCache[fullname] then
    appLoadImageCache[fullname] = ui.createImage(fullname)
  end

  return appLoadImageCache[fullname]
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: float, float mcn.screenToGraphPos()
--| brief:
--|   converts network editor panel coordinates to graph coordinates
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
mcn.screenToGraphPos = function(nodePath, x, y)
  local xPan, yPan, zoom = getGraphTransform(nodePath)
  local newX = (x - xPan) * (1 / zoom)
  local newY = (y - yPan) * (1 / zoom)

  return newX, newY
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: float, float mcn.graphToScreenPos()
--| brief:
--|   converts graph coordinates to network editor panel coordinates
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
mcn.graphToScreenPos = function(nodePath, x, y)
  local xPan, yPan, zoom = getGraphTransform(nodePath)
  local newX = (xPan + (x * zoom))
  local newY = (yPan + (y * zoom))

  return newX, newY
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string mcn.getApplicationRoot()
--| brief:
--|   Get the application root, the directory from which the application is run.
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
mcn.getApplicationRoot = function()
  return utils.demacroizeString("$(AppRoot)")
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string mcn.getProjectRoot()
--| brief:
--|   Get the project root directory.
--|
--| environments: UpgradeEnv ValidateSerializeEnv UpdatePinsEnv GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
mcn.getProjectRoot = function()
  return utils.demacroizeString("$(RootDir)")
end
