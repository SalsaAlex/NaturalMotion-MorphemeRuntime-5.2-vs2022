------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/PreferencesEditor/PreferencesEditor.lua"

addProjectMenu = function(parent)

  ----------------------------------------------------------------------------------------------------------------------
  local onNewProject = function(self)
    local dlg = ui.createFileDialog{
      style = "save;prompt",
      caption = "New morpheme:connect project",
      wildcard = "morpheme:connect project|mcp"
    }

    if dlg:show() then
      local filepath = dlg:getFullPath()
      project.new(filepath, true)
    end
  end
  
  ----------------------------------------------------------------------------------------------------------------------
  local onOpenProject = function(self)
    local dlg = ui.createFileDialog{
      caption = "Open morpheme:connect project",
      wildcard = "morpheme:connect project|mcp"
    }

    if dlg:show() then
      local filepath = dlg:getFullPath()
      project.open(filepath)
    end
  end
  
  ----------------------------------------------------------------------------------------------------------------------
  local onSaveProject = function(self)
    local filename = project.getFilename()

    if string.len(filename) == 0 then
      local dlg = ui.createFileDialog{
        style = "save;prompt",
        caption = "Save morpheme:connect project",
        wildcard = "morpheme:connect project|mcp"
      }

      if dlg:show() then
        local filepath = dlg:getFullPath()
        project.saveAs(filepath)
      end
    else
      project.save()
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  local onSaveAsProject = function(self)
    local dlg = ui.createFileDialog{
      style = "save;prompt",
      caption = "Save morpheme:connect project as",
      wildcard = "morpheme:connect project|mcp"
    }

    if dlg:show() then
      local filepath = dlg:getFullPath()
      project.saveAs(filepath)
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  local onEditProject = function(self)
    showPreferencesDialog("Project settings")
  end
  
  ----------------------------------------------------------------------------------------------------------------------
  local onSwitchProject = function(self)
    local projectFilepath = self:getLabel()
    if not project.open(projectFilepath) then
      -- if the open failed then remove the project
      local app = nmx.Application.new()
      local settings = app:getLocalUserSettings()
      local settingsNode = settings:getSetting("|GlobalSettings|RecentProjectList")
      if settingsNode then
        settingsNode:removeFileFromList(projectFilepath)
      end
    end
  end
  
  ----------------------------------------------------------------------------------------------------------------------
  local buildMenu = function(self)
    self:addItem{ name = "NewProject", label = "New", onClick = onNewProject }
    self:addItem{ name = "OpenProject", label = "Open", onClick = onOpenProject }
    self:addItem{ name = "SaveProject", label = "Save", onClick = onSaveProject }
    self:addItem{ name = "SaveProjectAs", label = "Save As", onClick = onSaveAsProject }
    self:addSeparator()
    self:addItem{ name = "EditCurrentProject", label = "Edit...", onClick = onEditProject }
    self:addSeparator()

    -- now set the MRUFilesSettingsNode for the menu to add all the recently opened projects
    local app = nmx.Application.new()
    local settings = app:getLocalUserSettings()
    local settingsNode = settings:getSetting("|GlobalSettings|RecentProjectList")
    local fileCount = settingsNode:getFileCount()
    local currentProject = string.lower(project.getFilename())
    local projectsAddedToMenu = { }
    for i = 1, fileCount do
      local projectFile = settingsNode:getFile(i)
      local lowerProjectFile = string.lower(projectFile)
      local checked = lowerProjectFile == currentProject
      if projectsAddedToMenu[lowerProjectFile] == nil then
        self:addCheckedItem{ label = projectFile, onClick = onSwitchProject, checked = checked }
        projectsAddedToMenu[lowerProjectFile] = true
      end
    end
    
    -- cant switch projects whilst running a network
    self:enable(not mcn.isNetworkRunning())
  end
  
  ----------------------------------------------------------------------------------------------------------------------
  local projectMenu = parent:addSubMenu{ label = "&Project Settings",
  
    -- rebuild the menu every time that it is popped up
    onPoppedUp = function(self)
      self:clear()
      buildMenu(self)
    end,
  }
end