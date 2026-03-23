------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

-- load ui and editors, ensure ui loads first because editors is dependant on it.
local appScriptsDir = app.getAppScriptsDir()
executeAllScripts(appScriptsDir .. "ui")
executeAllScripts(appScriptsDir .. "ui/AssetManager")
executeAllScripts(appScriptsDir .. "ui/AttributeEditor")
executeAllScripts(appScriptsDir .. "ui/PreferencesEditor")