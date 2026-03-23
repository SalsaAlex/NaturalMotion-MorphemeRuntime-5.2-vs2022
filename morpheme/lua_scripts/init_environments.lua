------------------------------------------------------------------------------------------------------------------------
-- Make sure that app is a valid table to insert into.
--
if app == nil
then
  app = {}
end

------------------------------------------------------------------------------------------------------------------------
-- buildIndexFuncton
-- 
-- This will create an index function for an environment's meta table. The environment's name 
-- is taken as a parameter for error reporting purposes.
--
-- If the index function fails it will return an informative error message with
-- a stack trace to indicate where the error was.
--
-- @param metatable The metatable who's index function we are building
-- @param environmentName The name of the environment that owns the metatable
--
local function buildIndexFuncton(metatable, environmentName)
  local function indexFunction(t,f)
    -- Useful error message if not able to find the function.
    --
    if metatable[f] ~= nil
    then
      return metatable[f]
    else
      -- Generate a our own printable stack trace
      -- so that we can exclude the two inner
      -- most function calls.
      --
      local stackTrace = ""
      local i = 2 -- Skip the first two inner functions.
      while true
      do
        local info = debug.getinfo(i)
        
        -- Opt out if we've reached the top
        -- of the stack
        --
        if(info == nil)
        then
          break
        end
        
        -- These are the defaults if we can't
        -- get hold of the information needed
        -- for a particular function in the stack.
        --
        local name = ""
        local currentline = -1
        local source = ""
        
        -- Copy the information across.
        --
        if info.currentline then currentline = info.currentline end
        if info.short_src  then source = info.short_src  end
        if info.name
        then
          name = info.name
        else
          name = string.format("<%s>", source)
        end
        
        -- Accumulate the stack trace information. This is the line number, name and the
        -- source file it came from.
        -- 
        stackTrace = stackTrace .. string.format("\tline '%d' '%s'\n", currentline, name)
        
        -- Traverse up the callstack.
        --
        i = i + 1
      end
      
      -- This is the final error string which tells us which environment
      -- we were in and where the error occured.
      --
      local errorString = string.format("Unable to find '%s' in environment '%s'\nTraceback:\n%s\n", f, environmentName, stackTrace)
      error(errorString)
      
    -- Not a problem if we are able to find the entry.
    --
    end
  end
  return indexFunction
end

------------------------------------------------------------------------------------------------------------------------
-- Creates a new environment table.
--
-- @param environmentName The name of the environment to register
-- @param environment The environment to register
--
app.createEnvironmentMetatable = function (environmentName, environment)
  environment.mt = {}
  environment.mt.__index = buildIndexFuncton(environment.mt, environmentName)
  setmetatable(environment, environment.mt)
end

------------------------------------------------------------------------------------------------------------------------
-- Register a function to the given environment.
--
-- The following documentation is provided in doxygen format for reading clarity only.
--
-- @param f The function to register
-- @param functionName The name the function should be registered under in the environment
-- @param environment The environment to register in
-- @param functionTable A table in the environment to register to, this is an option parameter
--
-- @code
--
-- function foo()
--      print("Hello World")
-- end
--
-- validateSerialise = app.getLuaEnvironment("ValidateSerialize")
-- validateSerialise.anim = {}
-- app.registerToEnvironment(foo, "foo", validateSerialise, validateSerialise.anim)
--
-- @endcode
--
app.registerToEnvironment = function (f, functionName, environment, functionTable)

  if(environment == nil)
  then
    error("The environment argument for 'registerToEnvironment' must not be null")
  end
  
  -- Set the environment for the function to use.
  --
  setfenv(f, environment)
  
  -- Put the function in the correct table.
  --
  if functionTable == nil
  then
    functionTable = environment
  end
  functionTable[functionName] = f
end

------------------------------------------------------------------------------------------------------------------------
-- list of core lua functions to expose to the other lua environments
--
local functions = {
  "_VERSION",
  "LUA_PATH",
  "_LOADED",
  "_ALERT",
  "_TRACEBACK",

  "assert",
  "debug",
  "error",
  "ipairs",
  "next",
  "pairs",
  "print",
  "rawset",
  "tonumber",
  "tostring",
  "type",
  "unpack",

  "io",
  "math",
  "os",
  "string",
  "table",

  "loadstring",
  "loadlib",

  "dofile",
  "pcall",
  "require",
  "xpcall",

  "gcinfo",
  
  -- These functions are not permitted in the other environments as they may be costly or dangerous.
  --[[
  "collectgarbage",
  "getfenv",
  "setfenv",
  --]]
}

------------------------------------------------------------------------------------------------------------------------
-- Iterate over all the environments
-- copying across any global functions
-- that might be useful and
-- setting up their metatables
--
local environments = app.listLuaEnvironments()
for name, environment in pairs(environments) do

  -- Copy across all global functions that might
  -- be useful. Avoid those that might be
  -- costly or dangerous.
  --
  for _, func in ipairs(functions) do
    environment[func] = _G[func]
  end
  
  -- Setup the environment's metatable.
  --
  app.createEnvironmentMetatable(name, environment)
end

