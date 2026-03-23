------------------------------------------------------------------------------------------------------------------------
-- Initialise the rockstar main menu.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- include other required scripts
------------------------------------------------------------------------------------------------------------------------
require "rockstar/RockstarExport.lua"
require "ui/StaticUI.lua"

-- make sure we don't overwrite any existing user document ui
--
local oldUserInitStaticUI = userInitStaticUI

------------------------------------------------------------------------------------------------------------------------
-- boolean isExportRagdollDialogValid()
------------------------------------------------------------------------------------------------------------------------
local isExportRagdollDialogValid = function()
  local dlg = ui.getWindow("RockstarExportDialog")

  if not dlg then
    return false
  end

  local ragdollControl = dlg:getChild("RagdollControl")
  if not ragdollControl then
    return false
  end

  local euphoriaControl = dlg:getChild("EuphoriaControl")
  if not euphoriaControl then
    return false
  end

  local ragdollExportPath = ragdollControl:getValue()  
  local euphoriaExportPath = euphoriaControl:getValue()

  return string.len(ragdollExportPath) > 0 and string.len(euphoriaExportPath) > 0
end

------------------------------------------------------------------------------------------------------------------------
-- nil showExportRagdollDialog()
------------------------------------------------------------------------------------------------------------------------
local showExportRagdollDialog = function()
  local dlg = ui.getWindow("RockstarExportDialog")
  if not dlg then
    dlg = ui.createModalDialog{ name = "RockstarExportDialog", caption = "Rockstar Export" }
  end

  dlg:freeze()
  dlg:clear()

  dlg:suspendLayout()
  dlg:setBorder(1)
  dlg:beginVSizer{ flags = "expand", proportion = 1 }

    dlg:beginFlexGridSizer{ cols = 2, flags = "expand", }
      dlg:setFlexGridColumnExpandable(2)

      dlg:addStaticText{ text = "Exported Ragdoll" }
      local ragdollControl = dlg:addFilenameControl{
        name      = "RagdollControl",
        caption   = "Rockstar Ragdoll Export",
        wildcard  = "XML files|xml",
        -- directory = app.getAppExecutableDir(),
        onChanged = function(self)
          local dlg = self:getParent()
          local button = dlg:getChild("ExportButton")

          local valid = isExportRagdollDialogValid()
          button:enable(valid)
        end,
        flags     = "expand",
      }

      dlg:addStaticText{ text = "Exported Euphoria", }
      local euphoriaControl = dlg:addFilenameControl{
        name      = "EuphoriaControl",
        caption   = "Rockstar Euphoria Export",
        wildcard  = "XML files|xml",
        onChanged = function(self)
          local dlg = self:getParent()
          local button = dlg:getChild("ExportButton")

          local valid = isExportRagdollDialogValid()
          button:enable(valid)
        end,
        flags     = "expand",
      }

    dlg:endSizer()

    dlg:beginVSizer{ proportion = 1, }
    dlg:endSizer()

    dlg:beginHSizer{ flags = "right", proportion = 0, }
      local button = dlg:addButton{
        name = "ExportButton",
        label = "Export",
        size = { width = 74 },
        onClick = function(self)
          local dlg = self:getParent()
          local ragdollControl = dlg:getChild("RagdollControl")
          local euphoriaControl = dlg:getChild("EuphoriaControl")

          local ragdollExportPath = ragdollControl:getValue()  
          local euphoriaExportPath = euphoriaControl:getValue()
          
          local selectedSet = getSelectedAssetManagerAnimSet()
          
          local result, error = rockstar.exportPhysicsRig(selectedSet, ragdollExportPath, euphoriaExportPath)
  
          dlg:hide()
        end
      }
      local valid = isExportRagdollDialogValid()
      button:enable(valid)

      dlg:addButton{
        label = "Cancel",
        name = "CancelButton",
        size = { width = 74 },
        onClick = function(self)
          local dlg = self:getParent()
          dlg:hide()
        end
      }
    dlg:endSizer()

  dlg:endSizer()

  dlg:resumeLayout()
  dlg:refresh()
  
  dlg:setSize{ width = 300, height = 150 }
  dlg:setMinSize{ width = 300, height = -1 }

  dlg:show()
end

------------------------------------------------------------------------------------------------------------------------
-- nil addRockstarMenu(MenuBar mainMenuBar)
------------------------------------------------------------------------------------------------------------------------
local addRockstarMenu = function(mainMenuBar)
  local rockstarMenu = mainMenuBar:addSubMenu{ name = "RockstarMenu", label = "&Rockstar" }
  rockstarMenu:addItem{
    name = "ExportEuphoria",
    label = "&Export rig",
    onClick = function(self)
      showExportRagdollDialog()
    end,
  }
end

------------------------------------------------------------------------------------------------------------------------
-- nil userInitStaticUI()
------------------------------------------------------------------------------------------------------------------------
userInitStaticUI = function()
  local mainFrame = ui.getWindow("MainFrame")
  local mainMenuBar = mainFrame:getChild("MainMenu")

  addRockstarMenu(mainMenuBar)

  safefunc(oldUserInitStaticUI)
end