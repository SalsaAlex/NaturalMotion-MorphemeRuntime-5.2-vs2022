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
--| signature: nil debugFormatTable(table tbl integer indent)
--| brief:
--|   Format the contents of a table the contents of a table, used to print tables.
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local debugFormatTable = nil
debugFormatTable = function(o, indent)
  if type(indent) ~= "number" then
    indent = 0
  end

  local objectType = type(o)
  if objectType == "number" or objectType == "boolean" then
    return tostring(o)
  elseif objectType == "string" then
    return string.format("%q", o)
  elseif objectType == "nil" then
    return "nil"
  elseif objectType == "table" then
    local str = ""

    indent = indent + 1
    for k, v in pairs(o) do
      local keyType = type(k)
      if keyType == "number" or keyType == "boolean" then
        str = string.format("%s%s[%s] = %s, \n", str, string.rep(" ", indent), tostring(k), debugFormatTable(v, indent))
      elseif keyType == "string" then
        str = string.format("%s%s%s = %s, \n", str, string.rep(" ", indent), k, debugFormatTable(v, indent))
      else
        error(string.format("table.serialize: cannot serialize table entry with key type(%q)", keyType))
      end
    end
    indent = indent - 1

    return string.format("{\n%s%s}", str, string.rep(" ", indent))
  else
    return string.format("%s(%s)", objectType, tostring(o))
  end

  return nil
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: nil printTable(table tbl)
--| brief:
--|   Prints the contents of a table.
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
printTable = function(tbl)  
  print(debugFormatTable(tbl))
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: nil printls()
--| brief:
--|   Prints everything returned from ls().
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
printls = function()
  print(table.concat(ls(object), "\n"))
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: nil printPins(string object)
--| brief:
--|   Prints all of object's pins.
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
printPins = function(object)
  print(table.concat(listPins(object), ", "))
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: nil printAttributes(string object)
--| brief:
--|   Prints all of object's attributes.
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
printAttributes = function(object)
  print(table.concat(listAttributes(object), ", "))
end


------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: nil printAttributeValues(string object)
--| brief:
--|   Prints the attribute name and value for all of the given object's attributes.
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------

printAttributeValues = function(object)
  local attrs = listAttributes(object)
  for i, attr in ipairs(attrs) do 
    local attrValue = getAttribute(object, attr)
    print(attr .. " - " .. tostring(attrValue))
  end
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: nil printConnections(string object)
--| brief:
--|   Prints all of object's connections.
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
printConnections = function(object)
  print(table.concat(listConnections{ Object = object, ResolveReferences = true }, ", "))
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: nil printInfo(string path)
--| brief:
--|   Prints object's type, attributes, pins and connections in the format.
--| desc:
--|   Info for object objectname
--|   Type = objecttype
--|   Attributes:
--|   attr1 attr2 ... attrN
--|   Pins:
--|   pin1 pin2 ... pinN
--|   Connections:
--|   conn1 conn2 ... connN
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
printInfo = function(path)
  if type(path) == "string" then
    if objectExists(path) then
      print("Info for object " .. path)
      print("Type = " .. getType(path))
      print("Attributes:")
      printAttributes(path)
      print("Pins:")
      printPins(path)
      print("Connections:")
      printConnections(path)
    else
      print("Object '" .. path .. "' does not exist.")
    end
  else
    print("printInfo expects string path as first argument.")
  end
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string, integer getCurrentFileAndLine()
--| brief: Prints the current file and line that the function was called from.
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
getCurrentFileAndLine = function()
  local info = debug.getinfo(1, "Sl")
  return info.short_src, info.currentline
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string, int, string changeBlockInfo(string format, ...)
--| brief:
--|   Returns the current file and line that the function was called from and also returns the formatted string as
--|   a description used as the parameters passed to a change block.
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
changeBlockInfo = function(format, ...)
  local info = debug.getinfo(2, "Sl")
  local description = string.format(format, unpack(arg))
  return info.short_src, info.currentline, description
end