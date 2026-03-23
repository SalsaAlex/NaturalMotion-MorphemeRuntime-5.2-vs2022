------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: CLASS
--| name: Timer
--| brief:
--|   A Timer used for some basic profiling of lua code. The timer is based on os.clock and is not very accurate, it
--|   should only be used to get a vague idea of how long an operation is taking. Be sure to time the same operation
--|   several times and take an average rather than rely on a single result.
------------------------------------------------------------------------------------------------------------------------
local Timer = { }

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: float Timer:reset()
--| brief:
--|   Resets the timer setting the total time and section time to 0 and disabling it.
------------------------------------------------------------------------------------------------------------------------
Timer.reset = function(self)
  self.m_enabled = false
  self.m_starttime = 0
  self.m_stoptime = 0
  self.m_totaltime = 0
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: float Timer:beginSection()
--| brief:
--|   Begins timing a section, if the timer is already enabled then this function does nothing.
------------------------------------------------------------------------------------------------------------------------
Timer.beginSection = function(self)
  if not self.m_enabled then
    -- before beginning the new section add the old section time to the new time
    self.m_totaltime = self.m_totaltime + (self.m_stoptime - self.m_starttime)
    self.m_enabled = true
    self.m_starttime = os.clock()
  end
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: float Timer:endSection()
--| brief:
--|   Ends the timing of the current section returning the time of this section. If the timer is disabled then
--|   returns the time of the last timed section.
------------------------------------------------------------------------------------------------------------------------
Timer.endSection = function(self)
  if self.m_enabled then
    self.m_stoptime = os.clock()
    self.m_enabled = false
  end

  local time = self.m_stoptime - self.m_starttime
  return time
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: float Timer:getSectionTime()
--| brief:
--|   Get the time in seconds of the current timed section, the last timed section if this timer is stopped.
------------------------------------------------------------------------------------------------------------------------
Timer.getSectionTime = function(self)
  if self.m_enabled then
    return os.clock() - self.m_starttime
  else
    return self.m_stoptime - self.m_starttime
  end
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: float Timer:getTotalTime()
--| brief:
--|   Get the combined total time in seconds of all timed sections since the last reset or since the timer was created.
------------------------------------------------------------------------------------------------------------------------
Timer.getTotalTime = function(self)
  -- m_totaltime never includes the last section time so make sure this is added
  return self.m_totaltime + Timer.getSectionTime(self)
end

-- protected metatable for Timer class.
local metatable = {
  __index = Timer,
  __newindex = function(self, key, value)
    error("attempt to update a read-only table", 2)
  end,
  __metatable = "metatable for Timer is protected",
}

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: Timer Timer.new()
--| brief:
--|   Creates a new timer.
------------------------------------------------------------------------------------------------------------------------
Timer.new = function(enabled)
  local t = { }

  -- this must be done before setmetatable as the metatable also makes the table read only.
  Timer.reset(t)

  setmetatable(t, metatable)

  if enabled then
    Timer.beginSection(t)
  end

  return t
end

-- expose Timer to all environments
local environments = app.listLuaEnvironments()
environments["global"] = _G
for name, environment in pairs(environments) do
  environment.Timer = Timer
end