------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "luaAPI/UIUtils.lua"
require "ui/PreferencesEditor/PreferencesAPI.lua"
require "ui/PreferencesEditor/SettingsPage.lua"

------------------------------------------------------------------------------------------------------------------------
-- updates the preferences page on data changes
------------------------------------------------------------------------------------------------------------------------
local updatePreferencePage = function(panel)
end

------------------------------------------------------------------------------------------------------------------------
local cfgLevelControls =
{
  treeNodeOutput = nil
}

local outputToggles =
{
  {
    name = "ControlParams",
    label = "control params",
    cb = nil,
    getValue = function(a) return a:isOutputControlParamsEnabled() end,
    setValue = function(a, v) a:enableOutputControlParams(v) end
  },
  {
    name = "Profiling",
    label = "profiling",
    cb = nil,
    getValue = function(a) return a:isOutputProfilingEnabled() end,
    setValue = function(a, v) a:enableOutputProfiling(v) end
  },
  {
    name = "TaskQueuing",
    label = "task queuing",
    cb = nil,
    getValue = function(a) return a:isOutputTaskQueuingEnabled() end,
    setValue = function(a, v) a:enableOutputTaskQueuing(v) end
  },
  {
    name = "TreeNodes",
    label = "tree node data",
    cb = nil,
    getValue = function(a)
      local v = a:isOutputTreeNodesEnabled()
      if (cfgLevelControls.treeNodeOutput ~= nil) then
        cfgLevelControls.treeNodeOutput:enable(v)
        end
      return v
      end,
    setValue = function(a, v)
      a:enableOutputTreeNodes(v)
      if (cfgLevelControls.treeNodeOutput ~= nil) then
        cfgLevelControls.treeNodeOutput:enable(v)
        end
      end
  },
  {
    name = "StateMachineRequests",
    label = "state machine requests",
    cb = nil,
    getValue = function(a) return a:isOutputStateMachineRequestsEnabled() end,
    setValue = function(a, v) a:enableOutputStateMachineRequests(v) end
  },
  {
    name = "ScratchPad",
    label = "scratchpad",
    cb = nil,
    getValue = function(a) return a:isOutputScratchPadEnabled() end,
    setValue = function(a, v) a:enableOutputScratchPad(v) end
  },
  {
    name = "DebugDraw",
    label = "debugdraw",
    cb = nil,
    getValue = function(a) return a:isOutputDebugDrawEnabled() end,
    setValue = function(a, v) a:enableOutputDebugDraw(v) end
  },
}

------------------------------------------------------------------------------------------------------------------------
local onConfigChanged = function(self)
  local gCfg = debugConfig.find(self:getSelectedItem())

  if gCfg ~= nil then
    if cfgLevelControls.treeNodeOutput ~= nil then
      local index = gCfg:getTreeNodeOutputLevel() + 1
      cfgLevelControls.treeNodeOutput:setSelectedIndex(index)
    end

    for i, ot in ipairs(outputToggles) do
      local value = ot.getValue(gCfg)
      ot.cb:setChecked(value)
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
-- builds the settings preferences page
------------------------------------------------------------------------------------------------------------------------
local buildPreferencePage = function(panel)
  panel:beginVSizer{ flags = "expand", proportion = 1, }

    panel:beginHSizer{ flags = "expand", proportion = 0, }

      panel:addStaticText{
        name = "Heading",
        text = "Runtime debugging configuration",
        font = "bold",
        flags = "parentBackground;expand;decoration",
      }

      local configNames = { }
      for i, t in ipairs(debugConfig.ls()) do
        configNames[i] = t:getName()
      end

      panel:addHSpacer(15)
      local cfgComboBox = panel:addComboBox{
          name = "Configurations",
          items = configNames,
          onChanged = onConfigChanged
        }
    panel:endSizer()

    panel:addVSpacer(10)

    panel:beginVSizer{ flags = "group;expand", label = "High-level outputs" }
      panel:addVSpacer(5)

      for i, ot in ipairs(outputToggles) do
        local myOt = ot
        ot.cb = panel:addCheckBox{
          name = "OutputEnable" .. ot.name,
          label = "Enable " .. ot.label,
          onChanged = function(self)
            local gCfg = debugConfig.find(cfgComboBox:getSelectedItem())
            if (gCfg ~= nil) then
              myOt.setValue(gCfg, self:getChecked())
            end
          end,
        }
      end
    panel:endSizer()

    panel:beginVSizer{ flags = "group;expand", label = "Tree node data output level :" }
      panel:addVSpacer(5)

      cfgLevelControls.treeNodeOutput = panel:addRadioBox{
        name = "TreeNodeOutput",
        flags = "expand",
        items = {
          "Core attribute data types",
          "All attribute data types",
        },
        onChanged = function(self)
            local gCfg = debugConfig.find(cfgComboBox:getSelectedItem())
            gCfg:setTreeNodeOutputLevel(self:getSelectedIndex() - 1)
          end,
      }

    panel:endSizer()

    local curCfg = debugConfig.getActive()
    if curCfg ~= nil then
      cfgComboBox:setSelectedItem(curCfg:getName())
    end
    onConfigChanged(cfgComboBox)

  panel:endSizer()
end

removePreferencesPage("RuntimeDebugging")
addPreferencesPage(
  "RuntimeDebugging",
  {
    title = "Runtime debugging",
    parent = "Settings",
    create = buildPreferencePage,
    update = updatePreferencePage,
  }
)
