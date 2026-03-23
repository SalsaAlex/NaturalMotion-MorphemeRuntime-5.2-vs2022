------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- This script will look through subdirectories finding all .mcn files to load.
-- Change this directory to the one containing your .mcn files
-- Note that this is useing the Project Root as set in connect
-- so you will need to set this correctly first.
local folder = mcn.getProjectRoot() .. [[\scenes]]

------------------------------------------------------------------------------------------------------------------------
local listFiles
listFiles = function(directory, extension)
  subDirectories = app.enumerateDirectories(directory, "")
  local result = { }
  local index = 1
  for k, v in ipairs(subDirectories) do
    local subDirFiles = listFiles(v, extension)
    for loc, val in ipairs(subDirFiles) do
      result[index] = val
      index = index + 1
    end
  end

  local subDirFiles = app.enumerateFiles(directory, extension)
  for loc, val in ipairs(subDirFiles) do
   result[index] = val
   index = index + 1
  end
  return result
end

------------------------------------------------------------------------------------------------------------------------
local files = listFiles(folder, "*.mcn")
for loc, val in ipairs(files) do
  print("Loading from: " .. val)
  mcn.open(val)
  print("Saving over original")
  mcn.save()
end
print("**Completed Updating Files**")

