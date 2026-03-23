------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

local onSelectionChange = function()
  local selection = ls("Selection")
  local animTakes = { }
  local animTakeIDs = { }
  
  for _, node in selection do
    if getType(node) == "AnimWithEvents" then
      local take = getAttribute(node, "AnimationTake")
      if anim.isResource(take) then
        local takeId = anim.getResourceId(take)
        table.insert(animTakes, take)
        animTakeIDs[takeId] = true
      end
    end
  end
  
  if table.getn(animTakes) > 0 then
    -- now make sure that all of the takes in the selection are selected
    local assetManager = ui.getWindow("MainFrame|LayoutManager|AssetManager")
    if assetManager then
      assetManager:selectTakes(animTakeIDs, animTakes[1])
    end
  end
end

registerEventHandler("mcSelectionChange", onSelectionChange)

local initAnimSource = function(node, animationTake)
  local filename = animationTake.filename
  local takename = animationTake.takename
  local clipStartFraction = animationTake.clipstartfraction
  local clipEndFraction = animationTake.clipendfraction

  -- default synctrack to mandatory markup event track "Footsteps"
  local adjustedTake = {
    filename = utils.macroizeString(filename),
    takename = takename,
    synctrack = "Footsteps"
  }
  --print("Initialising " .. node .. " : adjustedTake.filename = " .. filename .. "; adjustedTake.takename = " .. takename);
  setAttribute(node .. ".AnimationTake", adjustedTake)

   -- Initialise the clip start fraction attribute to 1.0 (the max).
  setAttribute(node .. ".ClipStartFraction", clipStartFraction)

  -- Initialise the clip start fraction attribute to 1.0 (the max).
  setAttribute(node .. ".ClipEndFraction", clipEndFraction)

  -- Set the loop to the default for the node
  local animPath = string.format("%s|%s", filename, takename)
  local defaultValue = anim.getAttribute(string.format("%s.Loop", animPath))
  setAttribute(node .. ".Loop", defaultValue)

  -- rename node to name of the take (or the filename if the take is untitled)
  local takes = anim.ls(filename)
  if table.getn(takes) == 1 then
    local dirname, shortname = splitFilePath(filename)
    stripname = stripFilenameExtension(shortname)
  else
    stripname = takename
  end

  -- if node already exists, append a "_n" onto the end (the rename code will automatically
  -- convert "_1" to "_2", "_3" etc).
  local nodepath, nodename = splitNodePath(node)
  local testnode = stripname
  if (nodepath ~= "") then
    testnode = nodepath .. "|" .. stripname
  end
  if objectExists(testnode) then
    stripname = stripname .. "_1"
  end

  rename(node, stripname)
end

local dndAnimSource = function(node, animationTake)
  local filename = animationTake.filename
  local takename = animationTake.takename
  local clipStartFraction = animationTake.clipstartfraction
  local clipEndFraction = animationTake.clipendfraction

  local adjustedTake = {
    filename = utils.macroizeString(filename),
    takename = takename
  }
  setAttribute(node .. ".AnimationTake", adjustedTake)

   -- Reset the default clip attribute to true.
  setAttribute(node .. ".DefaultClip", true)
  
   -- Reset the clip start fraction attribute to 1.0 (the max).
  setAttribute(node .. ".ClipStartFraction", clipStartFraction)

  -- Reset the clip start fraction attribute to 0 (the min).
  setAttribute(node .. ".ClipEndFraction", clipEndFraction)

  -- Reset the start event index attribute to 0.
  setAttribute(node .. ".StartEventIndex", 0)
end

registerAnimationFileType("fbx", "AnimWithEvents", initAnimSource, dndAnimSource)
registerAnimationFileType("xmd", "AnimWithEvents", initAnimSource, dndAnimSource)
registerMandatoryMarkupEventTrack("Footsteps", "Tick")
