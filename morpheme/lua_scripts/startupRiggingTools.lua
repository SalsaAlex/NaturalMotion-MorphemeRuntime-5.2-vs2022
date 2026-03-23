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

require [[rigBuilder/Options.lua]]
require [[rigBuilder/Rig_Functions.lua]]
require [[rigBuilder/Character_Functions.lua]]
require [[rigBuilder/CollisionSets_Functions.lua]]
require [[rigBuilder/Main_Functions.lua]]
require [[rigBuilder/CopyRig_GUI.lua]]
require [[rigBuilder/RiggingToolsMenu.lua]]
require [[rigBuilder/CreateAnimRig.lua]]
require [[rigBuilder/CreatePhysicsRig.lua]]