------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

local projectFileExtension = "mcp"
local fileSearchExpression = string.format("*.%s", projectFileExtension)

project.findProjectForNetwork = function(network)
  if type(network) ~= "string" then
    return nil
  end

  if string.len(network) == 0 then
    return nil
  end

  local directory, filename = splitFilePath(network)
  if string.len(directory) == 0 then
    return nil
  end

  -- search each parent directory until a project file is found
  repeat
    -- list all files with the project file extension in the directory
    local files = app.enumerateFiles(directory, fileSearchExpression)

    -- strip any files that do not exactly match the extension
    -- this is necessary due to a windows bug as the wildcard match will also match
    -- extensions with the first three characters the same and additional characters on the end
    local i = 1
    while i <= table.getn(files) do
      local file = files[i]
      local extension = getFilenameExtension(file)
      if extension ~= projectFileExtension then
        table.remove(files, i)
      else
        i = i + 1
      end
    end

    local count = table.getn(files)
    
    -- Build a table of all files with the correct extension that really are project files,
    -- to eliminate extension clashes with other types of files
    --
    local projectFiles = {}
    for i = 1, count do
      local projectFile = files[i]
      
      if project.isProjectFile(projectFile) then
        table.insert(projectFiles, projectFile)
      end
    end

    local validProjectFileCount = table.getn(projectFiles)
    if validProjectFileCount == 1 then
      return projectFiles[1]
    elseif validProjectFileCount > 1 then
      local message =
        string.format(
          "more than one project file found for network %q, using first project file found %q",
          network,
          projectFiles[1])
      app.warning(message)
      return projectFiles[1]
    end

    -- not found yet so try the parent directory
    directory, filename = splitFilePath(directory)
    if directory == "\\" then
      -- this is what splitFilePath returns when given a network path of the form '\\location'
      -- there was no project file found so return otherwise the call to app.enumerateFiles returns matches
      -- on 'C:' when given '\' as a directory to enumerate.
      return nil
    end
  until string.len(filename) == 0

  return nil
end

-- if this file is being rerun then unregister the old event handler first
if type(project.onFileBeginEventHandler) == "function" then
  local result = unregisterEventHandler("mcFileOpenBegin", project.onFileBeginEventHandler)
  if not result then
    app.warning("failed to unregister previously registered event handler project.onFileBeginEventHandler for event 'mcFileOpenBegin'")
  end
end

-- this must come after unregistration otherwise when rerunning this file the old event handler
-- can never be removed
project.onFileBeginEventHandler = function(network)
  local currentProjectFile = project.getFilename()
  local projectFile = project.findProjectForNetwork(network)

  if not projectFile then
    local message = string.format(
      "no project file automatically found for network '%s', using currently loaded project file '%s'.",
      network,
      currentProjectFile)
    app.info(message)
    return
  end

  if projectFile ~= currentProjectFile then
    local result = project.open(projectFile)
    if not result then
      local message = string.format("error loading project file '%s' for network '%s'.", projectFile, network)
      app.error(message)
    else
      local message = string.format("loaded project file '%s' for network.", projectFile, network)
      app.info(message)
    end
  else
    local message = string.format(
      "found currently loaded project file '%s' for network '%s', no action taken.",
      projectFile,
      network)
    app.info(message)
  end
end

registerEventHandler("mcFileOpenBegin", project.onFileBeginEventHandler)