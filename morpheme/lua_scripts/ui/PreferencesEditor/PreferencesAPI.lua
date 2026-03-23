------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

local preferencePages = { }

------------------------------------------------------------------------------------------------------------------------
-- shows the preferences dialog
------------------------------------------------------------------------------------------------------------------------
addPreferencesPage = function(name, page)
  -- validate the global preference pages table exists
  assert(type(preferencePages) == "table")

  -- now validate the arguments
  assert(type(name) == "string")
  assert(string.len(name) > 0)
  assert(preferencePages[name] == nil)

  assert(type(page) == "table")

  assert(type(page.title) == "string")
  assert(string.len(page.title) > 0)

  assert((page.create == nil) or (type(page.create) == "function"))
  assert((page.update == nil) or (type(page.update) == "function"))

  preferencePages[name] = page
end

------------------------------------------------------------------------------------------------------------------------
removePreferencesPage = function(name)
  -- validate the global preference pages table exists
  assert(type(preferencePages) == "table")

  -- now validate the arguments
  assert(type(name) == "string")
  assert(string.len(name) > 0)
  if preferencePages[name] then
    preferencePages[name] = nil
    return true
  end

  return false
end

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
getPreferencePages = function()
  return preferencePages
end
