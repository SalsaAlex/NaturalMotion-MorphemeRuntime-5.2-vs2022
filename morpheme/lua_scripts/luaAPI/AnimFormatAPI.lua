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
--| name: AnimFormatAPI
--| title: Animation Formats
--| desc:
--|   Lua-scripted functions related to animation formats and manipulating animation format options strings.
------------------------------------------------------------------------------------------------------------------------

-- note re-executing this script will clear all currently registered animation formats
local registeredAnimationFormats = { }

------------------------------------------------------------------------------------------------------------------------
local buildRegister = function()
  return function(format, validOptions, addFormatOptionsPanelFunc, updateFormatOptionsPanelFunc)
    -- format has to be a string
    if type(format) ~= "string" or string.len(format) == 0 then
      return false
    end

    if addFormatOptionsPanelFunc and type(addFormatOptionsPanelFunc) ~= "function" then
      return false
    end

    if updateFormatOptionsPanelFunc and type(updateFormatOptionsPanelFunc) ~= "function" then
      return false
    end

    -- change from an indexed array to a set, passes nil parameters straight through
    local convertToSet = function(array)
      if not array then return nil end

      local set = { }
      for _, v in ipairs(array) do set[v] = true end
      return set
    end

    -- registering a new format
    registeredAnimationFormats[format] = {
      format = format,
      validOptions = convertToSet(validOptions),
      addFormatOptionsPanel = addFormatOptionsPanelFunc,
      updateFormatOptionsPanel = updateFormatOptionsPanelFunc,
    }

    return true
  end
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table animfmt.ls()
--| brief:
--|   Returns all currently registered animation formats
--|
--| environments: GlobalEnv
--| page: AnimFormatAPI
------------------------------------------------------------------------------------------------------------------------
local buildLs = function()
  return function()
    local formats = { }
    for _, format in pairs(registeredAnimationFormats) do
      table.insert(formats, format)
    end
    return formats
  end
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table animfmt.get(string format)
--| brief:
--|   Get a registered format type from the list of registered animation formats. Returns nil
--|   if there is no registered format of that name.
--|
--| environments: GlobalEnv
--| page: AnimFormatAPI
------------------------------------------------------------------------------------------------------------------------
local buildGet = function()
  return function(format)
    if type(format) ~= "string" or string.len(format) == 0 then
      return false
    end

    return registeredAnimationFormats[format]
  end
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean animfmt.unregister(string format)
--| brief:
--|   Unregisters an animation animfmt. Returns false if the format was not registered or the format
--|   specified was invalid.
--|
--| environments: GlobalEnv
--| page: AnimFormatAPI
------------------------------------------------------------------------------------------------------------------------
local buildUnregister = function()
  return function(format)
    if type(format) ~= "string" or string.len(format) == 0 then
      return false
    end

    if registeredAnimationFormats[format] ~= nil then
      registeredAnimationFormats[format] = nil
      return true
    end

    return false
  end
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table animfmt.parseOptions(string options)
--| brief:
--|   Parses an animation format options string seperating it out into a table of options with arguments.
--|   Each entry in the table will consist of an option string and an argument table if there were any
--|   arguments with the option.
--|
--| environments: GlobalEnv
--| page: AnimFormatAPI
------------------------------------------------------------------------------------------------------------------------
local buildParseOptions = function()
  return function(optionsString)
    if type(optionsString) ~= "string" then
      return nil
    end

    local len = string.len(optionsString)
    if len == 0 then
      return { }
    end

    -- the table of options to be returned by this function
    local optionsTable = { }
    -- is an option currently being parsed
    local parsingOption = false
    -- is an argument currently being parsed
    local parsingArg = false
    -- the current token being parsed
    local currentToken = ""

    for i = 1, len do
      local currentChar = string.sub(optionsString, i, i)

      if currentChar == "-" then
        -- indicates the start of an options token
        parsingOption = true
      elseif currentChar == " " then
        -- indicates the end of a token
        if parsingOption then
          -- add a new option table to the optionsTable
          local entry = {
            option = currentToken,
            args = { }
          }
          table.insert(optionsTable, entry)
          parsingOption = false
          currentToken = ""
        elseif parsingArg then
          -- add an argument to the args table of the last entry in the options table
          local index = table.getn(optionsTable)
          table.insert(optionsTable[index].args, currentToken)
          parsingArg = false
          currentToken = ""
        end
      else
        -- indicates a character that should be appended to the current token
        if parsingOption then
          currentToken = string.format("%s%s", currentToken, currentChar)
        else
          if table.getn(optionsTable) > 0 then
            parsingArg = true
          end

          if parsingArg then
            currentToken = string.format("%s%s", currentToken, currentChar)
          end
        end
      end
    end

    if parsingOption then
      -- add a new option table to the optionsTable
      local entry = {
        option = currentToken,
        args = { }
      }
      table.insert(optionsTable, entry)
      parsingOption = false
    end

    if parsingArg then
      local index = table.getn(optionsTable)
      table.insert(optionsTable[index].args, currentToken)
      parsingArg = false
    end

    return optionsTable
  end
end

------------------------------------------------------------------------------------------------------------------------
-- compare two argument lists and return true if they contain the same elements
------------------------------------------------------------------------------------------------------------------------
local matchingArguments = function(args1, args2)
  local numArgs = table.getn(args1)
  if numArgs == table.getn(args2) then
    for i = 1, numArgs do
      if args1[i] ~= args2[i] then
        return false
      end
    end
    return true
  end
  return false
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table animfmt.removeOption(table options, string option)
--| signature: table animfmt.removeOption(table options, string option, ...)
--| brief:
--|   Removes a specific option from the options table passed in. The options table is generated by
--|   calling animfmt.parseOptions() on an animation format options string.
--|   If additional parameters are passed to the function then the option is only removed if matching
--|   arguments are also found.
--|
--| environments: GlobalEnv
--| page: AnimFormatAPI
------------------------------------------------------------------------------------------------------------------------
local buildRemoveOption = function()
  return function(options, option, ...)
    if type(options) == "table" then
      for i, o in options do
        if option == o.option then
          if table.getn(arg) == 0 then
            table.remove(options, i)
            return true
          elseif matchingArguments(o.args, arg) then
            table.remove(options, i)
            return true
          end
        end
      end
    end

    return false
  end
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table animfmt.removeInvalidOptions(string format, table options)
--| brief:
--|   Given a specific format and a table of current options this function removes all invalid
--|   options for the format type specified. If the format was registered with no valid options
--|   then it is assumed that all formats are allowed. If the format was registered with an empty
--|   table as valid format options then all options are removed.
--|
--| environments: GlobalEnv
--| page: AnimFormatAPI
------------------------------------------------------------------------------------------------------------------------
local buildRemoveInvalidOptions = function()
  return function(format, options)
    local fmt = animfmt.get(format)

    -- if it is an invalid format type or there no validOptions table was registered with the format
    -- then just return the existing options table
    if type(fmt) ~= "table" or fmt.validOptions == nil then
      return options
    end

    -- only add valid options from the existing table to the new table
    local validOptions = { }
    for _, o in ipairs(options) do
      if fmt.validOptions[o.option] then
        local entry = {
          option = o.option,
          args = o.args
        }
        table.insert(validOptions, entry)
      end
    end

    return validOptions
  end
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table animfmt.setOption(table options, string option, ...)
--| brief:
--|   Sets an option within an options table creating the option entry if it does not already exist.
--|   Any additional parameters passed to the function are added as option arguments.
--|
--| environments: GlobalEnv
--| page: AnimFormatAPI
------------------------------------------------------------------------------------------------------------------------
local buildSetOption = function()
  return function(options, option, ...)
    attributeEditor.logEnterFunc("animfmt.setOption")

    for i, o in options do
      if option == o.option then
        o.args = arg

        attributeEditor.logExitFunc("animfmt.setOption")
        return
      end
    end
    local entry = {
      option = option,
      args = arg
    }
    table.insert(options, entry)

    attributeEditor.logExitFunc("animfmt.setOption")
  end
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: string animfmt.compileOptions(table options)
--| brief:
--|   Compiles an animation format options table generated by animfmt.parseOptions back into a format options
--|   string for use by animation sets or animation take attributes.
--|
--| environments: GlobalEnv
--| page: AnimFormatAPI
------------------------------------------------------------------------------------------------------------------------
local buildCompileOptions = function()
  return function(optionsTable)
    if type(optionsTable) ~= "table" then
      return nil
    end

    local optionsString = ""

    for i, option in ipairs(optionsTable) do
      -- parse all the option arguments combining them in a string
      local argstr = ""
      if type(option.args) == "table" then
        for j, arg in ipairs(option.args) do
          argstr = string.format("%s %s", argstr, arg)
        end
      end

      -- append a space if this isn't the first option
      if string.len(optionsString) > 0 then
        optionsString = string.format("%s ", optionsString)
      end

      -- append the option and its arguments to the full options string
      if type(option.option) == "string" then
        optionsString = string.format("%s-%s%s", optionsString, option.option, argstr)
      end
    end

    return optionsString
  end
end

------------------------------------------------------------------------------------------------------------------------
animfmt = {
  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: boolean animfmt.register(string format, table validOptions, function addOptionsPanel = nil)
  --| brief:
  --|   Registers a new animation format with connect. Will overwrite any existing format of the same name.
  --|
  --|   Returns false if the format specified was invalid.
  --|
  --| environments: GlobalEnv
  --| page: AnimFormatAPI
  ----------------------------------------------------------------------------------------------------------------------
  register = buildRegister(),

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table animfmt.ls()
  --| brief:
  --|   Returns all currently registered animation formats
  --|
  --| environments: GlobalEnv
  --| page: AnimFormatAPI
  ----------------------------------------------------------------------------------------------------------------------
  ls = buildLs(),

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table animfmt.get(string format)
  --| brief:
  --|   Get a registered format type from the list of registered animation formats. Returns nil
  --|   if there is no registered format of that name.
  --|
  --| environments: GlobalEnv
  --| page: AnimFormatAPI
  ----------------------------------------------------------------------------------------------------------------------
  get = buildGet(),

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: boolean animfmt.unregister(string format)
  --| brief:
  --|   Unregisters an animation animfmt. Returns false if the format was not registered or the format
  --|   specified was invalid.
  --|
  --| environments: GlobalEnv
  --| page: AnimFormatAPI
  ----------------------------------------------------------------------------------------------------------------------
  unregister = buildUnregister(),

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table animfmt.parseOptions(string options)
  --| brief:
  --|   Parses an animation format options string seperating it out into a table of options with arguments.
  --|   Each entry in the table will consist of an option string and an argument table if there were any
  --|   arguments with the option.
  --|
  --| environments: GlobalEnv
  --| page: AnimFormatAPI
  ----------------------------------------------------------------------------------------------------------------------
  parseOptions = buildParseOptions(),


  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table animfmt.removeOption(table options, string option)
  --| signature: table animfmt.removeOption(table options, string option, ...)
  --| brief:
  --|   Removes a specific option from the options table passed in. The options table is generated by
  --|   calling animfmt.parseOptions() on an animation format options string.
  --|   If additional parameters are passed to the function then the option is only removed if matching
  --|   arguments are also found.
  --|
  --| environments: GlobalEnv
  --| page: AnimFormatAPI
  ----------------------------------------------------------------------------------------------------------------------
  removeOption = buildRemoveOption(),


  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table animfmt.removeInvalidOptions(string format, table options)
  --| brief:
  --|   Given a specific format and a table of current options this function removes all invalid
  --|   options for the format type specified. If the format was registered with no valid options
  --|   then it is assumed that all formats are allowed. If the format was registered with an empty
  --|   table as valid format options then all options are removed.
  --|
  --| environments: GlobalEnv
  --| page: AnimFormatAPI
  ----------------------------------------------------------------------------------------------------------------------
  removeInvalidOptions = buildRemoveInvalidOptions(),


  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table animfmt.setOption(table options, string option, ...)
  --| brief:
  --|   Sets an option within an options table creating the option entry if it does not already exist.
  --|   Any additional parameters passed to the function are added as option arguments.
  --|
  --| environments: GlobalEnv
  --| page: AnimFormatAPI
  ----------------------------------------------------------------------------------------------------------------------
  setOption = buildSetOption(),

  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: string animfmt.compileOptions(table options)
  --| brief:
  --|   Compiles an animation format options table generated by animfmt.parseOptions back into a format options
  --|   string for use by animation sets or animation take attributes.
  --|
  --| environments: GlobalEnv
  --| page: AnimFormatAPI
  ----------------------------------------------------------------------------------------------------------------------
  compileOptions = buildCompileOptions(),
}

-- register animfmt to all other environments
--
local functionsToAdd = {}
functionsToAdd["register"] = buildRegister
functionsToAdd["ls"] = buildLs
functionsToAdd["get"] = buildGet
functionsToAdd["unregister"] = buildUnregister
functionsToAdd["parseOptions"] = buildParseOptions
functionsToAdd["removeOption"] = buildRemoveOption
functionsToAdd["removeInvalidOptions"] = buildRemoveInvalidOptions
functionsToAdd["setOption"] = buildSetOption
functionsToAdd["compileOptions"] = buildCompileOptions

local environments = app.listLuaEnvironments()
for name, environment in pairs(environments) do
  environment.animfmt = {}
  for func, builder in pairs(functionsToAdd) do
    app.registerToEnvironment(builder(), func, environment, environment.animfmt)
  end
end