------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

if app.fileExists("luaAPI/ScalePhysicsRig.lua") then
  require "luaAPI/ScalePhysicsRig.lua"
end

-- pops up the dialog for scaling physics rigs
showRescalePhysicsDialog = function()
  if mcn.inCommandLineMode() then
    return
  end

  local dlg = ui.getWindow("RescalePhysicsDialog")
  if not dlg then
    dlg = ui.createModalDialog{ name = "RescalePhysicsDialog", caption = "Rescale Physics Rig Volumes" }
  end

  dlg:freeze()
  dlg:clear()

  dlg:suspendLayout()
  dlg:setBorder(1)
  dlg:beginVSizer{ flags = "expand", proportion = 1 }
    local treeControl = dlg:addStockWindow{
      name = "TreeControl",
      type = "AnimationSetsTreeControl",
      flags = "expand;readOnly;multiSelect",
      onSelectionChanged = function(self)
        local dlg = self:getParent()
        local rescaleButton = dlg:getChild("RescaleButton")
        local textBox = dlg:getChild("TextBox")

        local selection = self:getSelectedItems()
        local value = tonumber(textBox:getValue())

        local enable = (value ~= nil and value > 0) and (table.getn(selection) > 0)
        rescaleButton:enable(enable)
      end
    }
    treeControl:showColumnHeader(false)

    dlg:beginFlexGridSizer{ flags = "expand", proportion = 0 }
      dlg:setFlexGridColumnExpandable(2)

      dlg:addStaticText{ text = "Scale Factor" }
      local textBox = dlg:addTextBox{
        name = "TextBox",
        flags = "expand;numeric",
        proportion = 1,
        value = "1.0",
        onChanged = function(self)
          local dlg = self:getParent()
          local rescaleButton = dlg:getChild("RescaleButton")
          local treeControl = dlg:getChild("TreeControl")

          local selection = treeControl:getSelectedItems()
          local value = tonumber(self:getValue())

          local enable = (value ~= nil and value > 0) and (table.getn(selection) > 0)
          rescaleButton:enable(enable)
        end
      }
    dlg:endSizer()

    dlg:beginHSizer{ flags = "expand", proportion = 0 }
      dlg:addStaticText{ text = "Scale Volumes" }
      local volumeCheckBox = dlg:addCheckBox{ checked = 1 }
      dlg:addHSpacer(8)
      dlg:addStaticText{ text = "Scale Bind Pose" }
      local bindPoseCheckBox = dlg:addCheckBox{ checked = 0 }
    dlg:endSizer()

    dlg:beginHSizer{ flags = "right", proportion = 0 }
      local rescaleButton = dlg:addButton{
        name = "RescaleButton",
        label = "Rescale",
        size = { width = 74 },
        onClick = function(self)
          local dlg = self:getParent()
          local treeControl = dlg:getChild("TreeControl")
          local textBox = dlg:getChild("TextBox")

          local selection = treeControl:getSelectedItems()
          local scaleFactor = tonumber(textBox:getValue())
          local scaleRig = volumeCheckBox:getChecked()
          local scaleBindPose = bindPoseCheckBox:getChecked()

          local scene = nmx.Application.new():getSceneByName("AssetManager")
          local status, cbRef = scene:beginChangeBlock(getCurrentFileAndLine())
          for _, set in ipairs(selection) do
            scalePhysicsRig(scene, set, scaleFactor, scaleRig, scaleBindPose)
          end
          scene:endChangeBlock(cbRef, changeBlockInfo("Scale Physics Rig"))
          dlg:hide()
        end
      }
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

  dlg:setSize{ width = 300, height = -1 }
  dlg:setMinSize{ width = 300, height = -1 }

  local selection = treeControl:getSelectedItems()
  local value = tonumber(textBox:getValue())
  local enable = (value ~= nil and value > 0) and (table.getn(selection) > 0)
  rescaleButton:enable(enable)

  dlg:show()
end

-- pops up the dialog for scaling physics rigs
showRescaleAnimationRigDialog = function()
  if mcn.inCommandLineMode() then
    return
  end

  local dlg = ui.getWindow("RescaleAnimationDialog")
  if not dlg then
    dlg = ui.createModalDialog{ name = "RescaleAnimationDialog", caption = "Rescale Animation Rig" }
  end

  dlg:freeze()
  dlg:clear()

  dlg:suspendLayout()
  dlg:setBorder(1)
  dlg:beginVSizer{ flags = "expand", proportion = 1 }
    local treeControl = dlg:addStockWindow{
      name = "TreeControl",
      type = "AnimationSetsTreeControl",
      flags = "expand;readOnly;multiSelect",
      onSelectionChanged = function(self)
        local dlg = self:getParent()
        local rescaleButton = dlg:getChild("RescaleButton")
        local textBox = dlg:getChild("TextBox")

        local selection = self:getSelectedItems()
        local value = tonumber(textBox:getValue())

        local enable = (value ~= nil and value > 0) and (table.getn(selection) > 0)
        rescaleButton:enable(enable)
      end
    }
    treeControl:showColumnHeader(false)

    dlg:beginFlexGridSizer{ flags = "expand", proportion = 0 }
      dlg:setFlexGridColumnExpandable(2)

      dlg:addStaticText{ text = "Scale Factor" }
      local textBox = dlg:addTextBox{
        name = "TextBox",
        flags = "expand;numeric",
        proportion = 1,
        value = "1.0",
        onChanged = function(self)
          local dlg = self:getParent()
          local rescaleButton = dlg:getChild("RescaleButton")
          local treeControl = dlg:getChild("TreeControl")

          local selection = treeControl:getSelectedItems()
          local value = tonumber(self:getValue())

          local enable = (value ~= nil and value > 0) and (table.getn(selection) > 0)
          rescaleButton:enable(enable)
        end
      }
    dlg:endSizer()
    
    dlg:beginHSizer{ flags = "right", proportion = 0 }
      local rescaleButton = dlg:addButton{
        name = "RescaleButton",
        label = "Rescale",
        size = { width = 74 },
        onClick = function(self)
          local dlg = self:getParent()
          local treeControl = dlg:getChild("TreeControl")
          local textBox = dlg:getChild("TextBox")

          local selection = treeControl:getSelectedItems()
          local scaleFactor = tonumber(textBox:getValue())

          local scene = nmx.Application.new():getSceneByName("AssetManager")
          local status, cbRef = scene:beginChangeBlock(getCurrentFileAndLine())
          for _, set in ipairs(selection) do
            scaleAnimationRig(scene, set, scaleFactor)
          end
          scene:endChangeBlock(cbRef, changeBlockInfo("Scale Physics Rig"))
          dlg:hide()
        end
      }
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

  dlg:setSize{ width = 300, height = -1 }
  dlg:setMinSize{ width = 300, height = -1 }

  local selection = treeControl:getSelectedItems()
  local value = tonumber(textBox:getValue())
  local enable = (value ~= nil and value > 0) and (table.getn(selection) > 0)
  rescaleButton:enable(enable)

  dlg:show()
end
