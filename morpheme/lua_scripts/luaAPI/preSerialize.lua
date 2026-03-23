------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

getAnimationTakeDebugString = function(animationTake)
  if animationTake == nil then
    return "(null take)"
  end

  local str = "(filename = " .. animationTake.filename .. ", takename = " .. animationTake.takename .. ")"
  return str
end

getHierarchy = function(animationTake)
  local rigMarkupData = anim.getRigMarkupData(animationTake)
  if rigMarkupData then
    return rigMarkupData.hierarchy
  end
  return { }
end

getEventSequence = function(animationTake)
  local result = { }

  -- try to find events in the 'Footsteps' track and return as sorted list
  local takeMarkup = anim.getTakeMarkupData(animationTake)
  local duration = anim.getTakeDuration(animationTake)
  if takeMarkup then
    local track = takeMarkup["Footsteps"]
    if track then
      for i, event in ipairs(track) do
        table.insert(result, event.position / duration)
      end

      table.sort(result)
    end
  end

  --print("getEventSequence contains " .. table.getn(result) .. " events")
  --for i, v in ipairs(result) do print(v) end
  return result
end

isIdentiticalHierarchy = function(table1, table2)
  local table1Size = table.getn(table1)
  local table2Size = table.getn(table2)
  if table1Size == table2Size then
    for ind, val in ipairs(table1) do
      if table1[ind] ~= table2[ind] then
        return false, "Hierarchy files are not identical"
      end
    end
    return true
  end
  return false , "Hierarchy files are not identical"
end

preSerialize = function()

end

getRootNodeChannel = function()

end

getHipNodeChannel = function()

end

getBlendFrameOrientX = function()

end

getBlendFrameOrientY = function()

end

getBlendFrameOrientZ = function()

end

getBlendFrameOrientW = function()

end
