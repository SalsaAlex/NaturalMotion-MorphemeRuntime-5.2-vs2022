------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

local function buildSetUnion()
  local setUnion = function(set1, set2)
    local resultTable = set1
    for i in pairs(set2) do
      resultTable[i] = true
    end
    return resultTable
  end
  return setUnion
end
setUnion = buildSetUnion()

local function buildSetDifference()
  local setDifference = function(set1, set2)
    local resultTable = { }
    for i in pairs(set1) do
      resultTable[i] = set2[i] == nil
    end
    return resultTable
  end
  return setDifference
end
setDifference = buildSetDifference()

local function buildSetIntersection()
  local setIntersection = function(set1, set2)
    local resultTable = { }
    for i in pairs(set1) do
      resultTable[i] = set2[i]
    end
    return resultTable
  end
  return setIntersection
end
setIntersection = buildSetIntersection()

-- add set functions to all lua environments
local environments = app.listLuaEnvironments()
for _, environment in pairs(environments) do
  app.registerToEnvironment(buildSetUnion(), "setUnion", environment)
  app.registerToEnvironment(buildSetDifference(), "setDifference", environment)
  app.registerToEnvironment(buildSetIntersection(), "setIntersection", environment)
end

