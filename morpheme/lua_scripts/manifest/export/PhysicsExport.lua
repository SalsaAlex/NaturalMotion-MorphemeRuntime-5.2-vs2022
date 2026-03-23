------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2012 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold,
-- licensed or commercially exploited in any manner without the
-- written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential
-- information of NaturalMotion and may not be disclosed to any
-- person nor used for any purpose not expressly approved by
-- NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

local physicsDriverExporters = {}

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean registerPhysicsDriverExporter(string physicsEngineName, table exporter)
--| brief:
--|   
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
registerPhysicsDriverExporter = function(physicsEngineName, exporter)
  -- validate argument 'physicsEngineName'
  --
  if type(physicsEngineName) ~= "string" then
    error(string.format("%s() : bad argument #1, expected 'string' got '%s'", getFunctionName(), type(physicsEngineName)))
  end
  if string.len(physicsEngineName) == 0 then
    error(string.format("%s() : bad argument #1, string must not be empty", getFunctionName()))
  end

  -- validate argument 'exporter'
  --
  if type(exporter) ~= "table" then
    error(string.format("%s() : bad argument #2, expected 'table' got '%s'", getFunctionName(), type(exporter)))
  end
  if type(exporter.exportPart) ~= "function" then
    error(string.format("%s() : bad argument #2, table entry 'exportPart', expected 'function' got '%s'", getFunctionName(), type(exporter.exportPart)))
  end
  if type(exporter.exportJoint) ~= "function" then
    error(string.format("%s() : bad argument #2, table entry 'exportJoint', expected 'function' got '%s'", getFunctionName(), type(exporter.exportJoint)))
  end

  if physicsDriverExporters[physicsEngineName] then
    app.warning(string.format("A physics driver exporter for '%s' has already been registered.", physicsEngineName))
    return false
  end

  -- register the exporter
  --
  physicsDriverExporters[physicsEngineName] = exporter

  return true
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean unregisterPhysicsDriverExporter(string physicsEngineName)
--| brief:
--|   
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
unregisterPhysicsDriverExporter = function(physicsEngineName)
  -- validate argument 'physicsEngineName'
  --
  if type(physicsEngineName) ~= "string" then
    error(string.format("%s() : bad argument #1, expected 'string' got '%s'", getFunctionName(), type(physicsEngineName)))
  end
  if string.len(physicsEngineName) == 0 then
    error(string.format("%s() : bad argument #1, string must not be empty", getFunctionName()))
  end

  if not physicsDriverExporters[physicsEngineName] then
    app.warning(string.format("No registered exporter for physics engine '%s'.", physicsEngineName))
    return false
  end

  physicsDriverExporters[physicsEngineName] = nil
  return true
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table getPhysicsDriverExporter(string physicsEngineName)
--| brief:
--|   
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
getPhysicsDriverExporter = function(physicsEngineName)
  -- validate argument 'physicsEngineName'
  --
  if type(physicsEngineName) ~= "string" then
    error(string.format("%s() : bad argument #1, expected 'string' got '%s'", getFunctionName(), type(physicsEngineName)))
  end
  if string.len(physicsEngineName) == 0 then
    error(string.format("%s() : bad argument #1, string must not be empty", getFunctionName()))
  end

  return physicsDriverExporters[physicsEngineName]
end
