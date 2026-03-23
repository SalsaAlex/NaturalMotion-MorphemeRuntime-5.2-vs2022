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
--| signature: string argumentErrorMessage(string functionName, integer argumentIndex, object argument, string expectedType)
--| brief:
--|   Generates an error message when a function has been called with a bad argument.
--|   Error message is of the form "bad argument #2 to 'func' (string expected, got number)"
--| environments: UpgradeEnv ValidateSerializeEnv UpdatePinsEnv SetAttributeEnv GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local function buildArgumentErrorMessage()
  argumentErrorMessage = function(functionName, argumentIndex, argument, expectedType)
    local message = string.format(
      "bad argument #%d to '%s' (%s expected, got %s)",
      argumentIndex,
      functionName,
      expectedType,
      type(argument))

    return message
  end
  return argumentErrorMessage
end
safefunc = buildArgumentErrorMessage()

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string tableArgumentSizeErrorMessage(string functionName, integer argumentIndex, table argument, integer expectedSize)
--| brief:
--|   Generates an error message when a function has been called with a bad table argument.
--|   Error message is of the form "bad element #3 of table argument #2 to 'func' (string expected, got number)"
--| environments: UpgradeEnv ValidateSerializeEnv UpdatePinsEnv SetAttributeEnv GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local function buildTableArgumentSizeErrorMessage()
  tableArgumentSizeErrorMessage = function(functionName, argumentIndex, argument, expectedSize)
    local message = string.format(
      "bad argument #%d to '%s' (expected table size %d, got %d)",
      argumentIndex,
      functionName,
      expectedSize,
      table.getn(argument))

    return message
  end
  return tableArgumentSizeErrorMessage
end
safefunc = buildTableArgumentSizeErrorMessage()

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string tableArgumentElementErrorMessage(string functionName, integer argumentIndex, integer elementIndex, object element, string expectedType)
--| brief:
--|   Generates an error message when a function has been called with a bad table argument.
--|   Error message is of the form "bad element #3 of table argument #2 to 'func' (string expected, got number)"
--| environments: UpgradeEnv ValidateSerializeEnv UpdatePinsEnv SetAttributeEnv GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local function buildTableArgumentElementErrorMessage()
  tableArgumentElementErrorMessage = function(functionName, argumentIndex, elementIndex, element, expectedType)
    local message = string.format(
      "bad element #%d of table argument #%d to '%s' (expected %s, got %s)",
      elementIndex,
      argumentIndex,
      functionName,
      expectedType,
      type(element))

    return message
  end
  return tableArgumentElementErrorMessage
end
safefunc = buildTableArgumentElementErrorMessage()

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: nil safefunc(function fn, ...)
--| brief:
--|   Before attempting to call func safefunc first checks to see if type(func) == "function" if
--|   it is then it will call the function with any additional arguments returning funcs returns.
--|   for example if print is a valid function safefunc(print, "%s", "example") would call
--|   print("%s", "example")
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local function buildSafeFunc()
  local safefunc = function(func, ...)
    if type(func) == "function" then
      return func(unpack(arg))
    end
    return nil
  end
  return safefunc
end
safefunc =  buildSafeFunc()

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table table.clone(table value)
--| brief:
--|   Copies a table recursing in to all nested tables copying those as well.
--|   Also copys any metatables encountered.
--|
--| environments: UpgradeEnv ValidateSerializeEnv UpdatePinsEnv SetAttributeEnv GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local buildTableClone = function()
  local clone = function(object)
    local lookup_table = {}

    local _copy
    _copy = function(object)
      if type(object) ~= "table" then
        return object
      elseif lookup_table[object] then
        return lookup_table[object]
      end

      local new_table = {}
      lookup_table[object] = new_table
      for index, value in pairs(object) do
        new_table[_copy(index)] = _copy(value)
      end

      return setmetatable(new_table, getmetatable(object))
    end

    return _copy(object)
  end

  return clone
end
table.clone =  buildTableClone()

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string table.serialize(object value, integer numTabs)
--| brief:
--|   Serializes a table into a string as it would be written in a lua script.
--|
--| environments: UpgradeEnv ValidateSerializeEnv UpdatePinsEnv SetAttributeEnv GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
table.serialize = function(o, indent)
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
        str = string.format("%s%s[%s] = %s, \n", str, string.rep(" ", indent), tostring(k), table.serialize(v, indent))
      elseif keyType == "string" then
        str = string.format("%s%s%s = %s, \n", str, string.rep(" ", indent), k, table.serialize(v, indent))
      else
        error(string.format("table.serialize: cannot serialize table entry with key type(%q)", keyType))
      end
    end
    indent = indent - 1

    return string.format("{\n%s%s}", str, string.rep(" ", indent))
  else
    error(string.format("table.serialize: cannot serialize type(%q)", objectType))
  end

  return nil
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table splitString(string str, string delimiter)
--| brief: Splits a string into a table of substrings of a given delimiter
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local function buildSplitString()
  local splitString = function(str, delimiter)
    local result = { }
    local from = 1
    local delim_from, delim_to = string.find(str, delimiter, from)
    while delim_from do
      table.insert(result, string.sub(str, from , delim_from - 1))
      from = delim_to + 1
      delim_from, delim_to = string.find(str, delimiter, from)
    end
    local subStr = string.sub(str, from)
    if subStr ~= "" then
      table.insert(result, subStr)
    end
    return result
  end
  return splitString
end
splitString = buildSplitString()

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string, string splitStringAtLastOccurence(string str, string delimiter)
--| brief:
--|   Splits a string into 2 components based on the last occurence of a given character (or regex)
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local function buildSplitStringAtLastOccurence()
  local splitStringAtLastOccurence = function(str, char)
    local strType = type(str)
    if strType ~= "string" then
      error(string.format("splitStringAtLastOccurence: cannot split type(%q)", strType))
    end
    local _, _, s1, s2 = string.find(str, "(.*)" .. char .. "(.*)")
    return s1, s2
  end
  return splitStringAtLastOccurence
end
splitStringAtLastOccurence = buildSplitStringAtLastOccurence()

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean isValidFilename(string filename)
--| brief:
--|   Is the filename a valid windows filename. Checks that filename is a string, is not zero length and does not
--| contain any of the characters \ / : * ? &quot; &lt; &gt; |
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local buildIsValidFilename = function()
  local isValidFilename = function(filename)
    -- is name not a string
    if type(filename) ~= "string" then
      return false
    end
    
    -- is name an empty string
    if string.len(filename) == 0 then
      return false
    end
    
    -- does name contain any of the invalid filename characters \ / : * ? " < > |
    if string.find(filename, "[\\/:%*%?\"<>|]") ~= nil then
      return false
    end

    return true
  end
  return isValidFilename
end
isValidFilename = buildIsValidFilename()

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string, string splitFilePath(string filePath)
--| brief:
--|   Splits filePath into full directory and filename.
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local function buildSplitFilePath()
  local splitFilePath = function(filePath)
    local path, file = splitStringAtLastOccurence(filePath, "[\\/]")
    if not path then
      -- only contains file part
      return "", filePath
    end
    return path, file
  end
  return splitFilePath
end
splitFilePath = buildSplitFilePath()

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string getFilenameExtension(string filename)
--| brief: Gets a filename extension from a filename
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local function buildGetFilenameExtension()
  local getFilenameExtension = function(filename)
    local name, extension = splitStringAtLastOccurence(filename, "%.")
    if not name then
      -- no dot, just return filename as-is
      return ""
    end
    return extension
  end
  return getFilenameExtension
end

getFilenameExtension = buildGetFilenameExtension()

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string stripFilenameExtension(string filename)
--| brief:
--|   Removes file extension from filename
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local function buildStripFilenameExtension()
  local stripFilenameExtension = function(filename)
    local name, extension = splitStringAtLastOccurence(filename, "%.")
    if not name then
      -- no dot, just return filename as-is
      return filename
    end
    return name
  end
  return stripFilenameExtension
end
stripFilenameExtension =  buildStripFilenameExtension()

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string getFunctionName(integer stacklevel = nil)
--| brief:
--|   Returns the function name of the function that this function was called from.
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local function buildGetFunctionName()
  local getFunctionName = function(stacklevel)
    if type(stacklevel) ~= "number" or stacklevel < 0 then
      stacklevel = 2
    end

    local info = debug.getinfo(stacklevel)
    if not info then
      return ""
    end

    local func = info.name
    if not info.name then
      return "main"
    end

    return info.name
  end
  return getFunctionName
end
getFunctionName = buildGetFunctionName()

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string getFileLineAndFunction(integer stacklevel = nil)
--| brief:
--|   Returns the file, line and function that this function was called on with the format
--|   "filename.lua(10) : func() : ".
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local function buildGetFileLineAndFunction()
  local getFileLineAndFunction = function(stacklevel)
    if type(stacklevel) ~= "number" or stacklevel < 0 then
      stacklevel = 2
    end

    local info = debug.getinfo(stacklevel)
    if not info then
      return ""
    end

    local file = info.source
    if string.find(info.source, "@") == 1 then
      file = string.sub(info.source, 2)
    end

    local func = info.name
    if not info.name then
      func = "main"
    end

    local message = string.format("%s(%d) : %s() : ", file, info.currentline, func)
    return message
  end
  return getFileLineAndFunction
end
getFileLineAndFunction = buildGetFileLineAndFunction()

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: number wrapValue(integer value, integer wrap)
--| brief:
--|   Wrap a value into range [0 > wrap - 1].
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local function buildWrapValue()
  local wrapValue = function(value, wrap)
    while value >= wrap do
      value = value - wrap
    end

    while value < 0 do
      value = value + wrap
    end

    return value
  end
  return wrapValue
end

wrapValue = buildWrapValue()
------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: pairsByKeys(table tbl)
--| signature: pairsByKeys(table tbl, function sortFunction)
--| brief: This is an iterator equivalent to pairs that will iterate through a table in ascending order of its keys
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
local function buildPairsByKeys()
  local pairsByKeys = function(t, f)
    local a = { }
    for n in pairs(t) do table.insert(a, n) end
    table.sort(a, f)
    local i = 0
    local iter = function()
      i = i + 1
      if a[i] == nil then
        return nil
      else
        return a[i], t[a[i]]
      end
    end
    return iter
  end
  return pairsByKeys
end
pairsByKeys =  buildPairsByKeys()

------------------------------------------------------------------------------------------------------------------------
-- this code adds the split path style functions to all other lua environments

local functionsToAdd = {}
functionsToAdd["argumentErrorMessage"] = buildArgumentErrorMessage
functionsToAdd["tableArgumentSizeErrorMessage"] = buildTableArgumentSizeErrorMessage
functionsToAdd["tableArgumentElementErrorMessage"] = buildTableArgumentElementErrorMessage
functionsToAdd["pairsByKeys"] = buildPairsByKeys
functionsToAdd["wrapValue"] = buildWrapValue
functionsToAdd["getFunctionName"] = buildGetFunctionName
functionsToAdd["getFileLineAndFunction"] = buildGetFileLineAndFunction
functionsToAdd["stripFilenameExtension"] = buildStripFilenameExtension
functionsToAdd["getFilenameExtension"] = buildGetFilenameExtension
functionsToAdd["isValidFilename"] = buildIsValidFilename
functionsToAdd["splitFilePath"] = buildSplitFilePath
functionsToAdd["splitStringAtLastOccurence"] = buildSplitStringAtLastOccurence
functionsToAdd["splitString"] = buildSplitString
functionsToAdd["safefunc"] = buildSafeFunc

local environments = app.listLuaEnvironments()
for name, environment in pairs(environments) do
  for func, builder in pairs(functionsToAdd) do
    app.registerToEnvironment(builder(), func, environment)
  end
end
