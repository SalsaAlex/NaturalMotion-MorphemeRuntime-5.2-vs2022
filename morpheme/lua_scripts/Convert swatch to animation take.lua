------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

local stripJbaFromTakename = function(takeName)
  return string.gsub(takeName, ".jba", "")
end

local stripSwatchFromDirname = function(dirName)
  return string.gsub(dirName, "[\\/]Swatches", "")
end

-- try to replace any swatch "SourceFile" attributes with "AnimationTake" attributes
local convertSwatchAttributeToAnimationTakeAttribute = function()
  local doCommit = false
  local animNodes = ls("LocomotionAnimationSource")

  for i, name in ipairs(animNodes) do
    local swatchFilename = getAttribute(name, "SourceFile")
    if string.len(swatchFilename) > 0 then
      print(">>>>>>>>")
      print("Found SourceFile attribute: " .. name)
      print("Swatch location:" .. swatchFilename)

      -- generate animation take from swatch filename
      local dirName, fileName = splitFilePath(swatchFilename)
      local sourceFile, jbaName = splitSwatchFilename(fileName)
      local takeName = stripJbaFromTakename(jbaName)
      local sourceDir = stripSwatchFromDirname(dirName)
      local fullSourcePath = sourceDir .. sourceFile
      local macroSourcePath = utils.macroizeString(fullSourcePath)
      local animationTake = {
        filename = macroSourcePath,
        takename = takeName
      }

      -- check the take exists
      local takeExportData = getTakeExportData(animationTake)
      if not takeExportData then
        print("** error, could not find animation take")
      end

      -- set the animation take attribute
      print("Saving AnimationTake:")
      print("AnimationTake.filename: " .. animationTake.filename)
      print("AnimationTake.takename: " .. animationTake.takename)
      setAttribute(name .. ".AnimationTake", animationTake)

      doCommit = true
    end
  end

  if doCommit then
    print(">>>>>>>>")
    print("Commiting changes")
    mcn.commit("Converted SourceFile attributes to AnimationTake attributes")
  end

  print(">>>>>>>>")
  print("Conversion finished")
end

convertSwatchAttributeToAnimationTakeAttribute()

