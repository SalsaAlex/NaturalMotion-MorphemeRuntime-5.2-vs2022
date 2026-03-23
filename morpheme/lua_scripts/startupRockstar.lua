------------------------------------------------------------------------------------------------------------------------
-- Initialise all the rockstar scripts required.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- make sure the main startup script was run first
------------------------------------------------------------------------------------------------------------------------
local appScriptsDir = app.getAppScriptsDir()

-- as startup.lua defines the LUA_PATH we must use the full path to it when calling require
--
local filename = string.format("%sstartup.lua", appScriptsDir)
require(filename)

require("rockstar/RockstarPhysicsEngine.lua")
require("rockstar/RockstarExport.lua")

if not mcn.inCommandLineMode() then
  require("rockstar/RockstarMenu.lua")
end

