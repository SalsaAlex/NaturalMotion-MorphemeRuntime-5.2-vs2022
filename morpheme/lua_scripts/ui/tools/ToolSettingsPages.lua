------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------
local toolSettingsPages = { }

------------------------------------------------------------------------------------------------------------------------
addToolSettingsPage = function(name, page)
  -- validate the global tool settings pages table exists
  assert(type(toolSettingsPages) == "table")

  -- now validate the arguments
  assert(type(name) == "string")
  assert(string.len(name) > 0)
  assert(toolSettingsPages[name] == nil)

  assert(type(page) == "table")

  assert(type(page.title) == "string")
  assert(string.len(page.title) > 0)

  assert((page.create == nil) or (type(page.create) == "function"))
  assert((page.update == nil) or (type(page.update) == "function"))

  toolSettingsPages[name] = page
end

------------------------------------------------------------------------------------------------------------------------
removeToolSettingsPage = function(name)
  -- validate the global tool settings pages table exists
  assert(type(toolSettingsPages) == "table")

  -- now validate the arguments
  assert(type(name) == "string")
  assert(string.len(name) > 0)
  if toolSettingsPages[name] then
    toolSettingsPages[name] = nil
    return true
  end

  return false
end

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
getToolSettingsPage = function(toolName)
  return toolSettingsPages[toolName]
end
