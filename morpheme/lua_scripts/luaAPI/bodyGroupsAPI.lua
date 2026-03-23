------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

-- Function to list all the body groups defined in the system
local function buildListBodyGroups()
  local listBodyGroups = function()
    local bodyGroupTable = { }
    -- Inner function to recursively list child body groups.  Note that lua has proper lexical scoping,
    -- which means the bodyGroupTable can be accessed within this inner function.
    local recurseBGroups
    recurseBGroups = function(parentPath)
      local children = listChildren(parentPath)
      for i, v in ipairs(children) do
        table.insert(bodyGroupTable, v)
        recurseBGroups(v)
      end
    end

    -- "BodyGroups" is the namespace for all body groups
    recurseBGroups("BodyGroups")
    return bodyGroupTable
  end
  return listBodyGroups
end
listBodyGroups = buildListBodyGroups()

local function buildBodyGroupChannelUnion()
  local bodyGroupChannelUnion = function(groupTable, setName)
    local result = { }

    for i, v in ipairs(groupTable) do
      local val = anim.getBodyGroupChannels(v, setName)
      if (table.getn(result) == 0) then
         result = val
      else
        if (table.getn(result) == table.getn(val)) then
          for curIdx, curVal in ipairs(val) do
            if curVal == true then
               result[curIdx] = true
            end
          end
        end
      end
    end

    return result
  end
  return bodyGroupChannelUnion
end

local function buildGetBodyGroupAttributeChannels()
  local getBodyGroupAttributeChannels = function(object, attribute, setName)
    local channelResult = { }

    local referencedBodyGroups = getAttribute(object, attribute)
    if referencedBodyGroups ~= nil then
      channelResult = bodyGroupChannelUnion(referencedBodyGroups, setName)
    end
    return channelResult
  end
  return getBodyGroupAttributeChannels
end
getBodyGroupAttributeChannels = buildGetBodyGroupAttributeChannels()

local function buildReferencesValidBodyGroups()
  local referencesValidBodyGroups  = function(object, attribute)
    local referencedBodyGroups = getAttribute(object, attribute)

    if type(referencedBodyGroups) == "table" then
      for _, v in ipairs(referencedBodyGroups) do
        if string.len(v) == 0 then
          return false, string.format("%s '%s' contains an empty override group entry.", getType(object), object)
        elseif getType(v) ~= "BodyGroup" then
          return false, string.format("%s '%s' references missing override group '%s'.", getType(object), object, v)
        end
      end
    end

    return true
  end
  return referencesValidBodyGroups
end
referencesValidBodyGroups = buildReferencesValidBodyGroups()

local environment = app.getLuaEnvironment("ValidateSerialize")

app.registerToEnvironment(buildListBodyGroups(), "listBodyGroups", environment)
app.registerToEnvironment(buildBodyGroupChannelUnion(), "bodyGroupChannelUnion", environment)
app.registerToEnvironment(buildGetBodyGroupAttributeChannels(), "getBodyGroupAttributeChannels", environment)
app.registerToEnvironment(buildReferencesValidBodyGroups(), "referencesValidBodyGroups", environment)

