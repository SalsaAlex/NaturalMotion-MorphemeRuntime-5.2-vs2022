------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

local getConnectedNodeID = function(...)
  local pinPathName = nil
  if arg.n == 1 then
    pinPathName = arg[1]
  elseif arg.n == 2 then
    pinPathName = string.format("%s.%s", arg[1], arg[2])
  else
    return nil
  end

  if pinExists(pinPathName) then
    if isConnected{ SourcePin = pinPathName , ResolveReferences = true } then
      local connections = listConnections{ Object = pinPathName, ResolveReferences = true }
      local connectedIDs = { }
      for i, val in ipairs(connections) do
        connectedIDs[i] = getRuntimeID(val)
      end

      if table.getn(connections) == 1 then
         return unpack(connectedIDs)
      end
    end
  end

  return nil
end

-- returns the pinIndex and connection type of the connected node as a table
-- the table is has the members {id, pinIndex, pinType}

local getConnectedNodeInfo = function(...)
  local pinPathName = nil
  if arg.n == 1 then
    pinPathName = arg[1]
  elseif arg.n == 2 then
    pinPathName = string.format("%s.%s", arg[1], arg[2])
  else
    return nil
  end

  if pinExists(pinPathName) then
    if isConnected{ SourcePin = pinPathName , ResolveReferences = true } then
      local conectedPins = listConnections{Pins = true, Object = pinPathName, ResolveReferences = true }
      local connectedIDs = { }
      for i, val in ipairs(conectedPins) do
        local theIndex, theType = getPinIndex(val)
        local nodePath = splitPinPath(val)
        local theId = getRuntimeID(nodePath)
        connectedIDs[i] = {id = theId, pinIndex = theIndex, pinType = theType}
      end

      if table.getn(conectedPins) == 1 then
        return unpack(connectedIDs)
      end
    end
  end

  return nil
end

-- returns the pinIndex
local getConnectedPinIndex = function(...)
  local pinPathName = nil
  if arg.n == 1 then
    pinPathName = arg[1]
  elseif arg.n == 2 then
    pinPathName = string.format("%s.%s", arg[1], arg[2])
  else
    return nil
  end

  if pinExists(pinPathName) then
    return getPinIndex(pinPathName)
  end

  return nil
end

-- set the environment and add it to the ValidateSerialize environment.
local environment = app.getLuaEnvironment("ValidateSerialize")
app.registerToEnvironment(getConnectedNodeID, "getConnectedNodeID", environment)
app.registerToEnvironment(getConnectedNodeInfo, "getConnectedNodeInfo", environment)
app.registerToEnvironment(getConnectedPinIndex, "getConnectedPinIndex", environment)

