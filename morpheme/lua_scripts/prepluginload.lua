------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

-- call blockplugin with the plugin ID to prevent it being loaded
local blockPlugin = function(a)
  app.blockedplugins[a .. ""] = true;
end

--blockPlugin(83) --debug tools
--blockPlugin(81) --script entry
--blockPlugin(80) --log view

