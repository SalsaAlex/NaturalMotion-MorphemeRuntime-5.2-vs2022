------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"

------------------------------------------------------------------------------------------------------------------------
-- handle selection of no objects
------------------------------------------------------------------------------------------------------------------------
attributeEditor.doEmptySelection = function(panel)
  attributeEditor.logEnterFunc("attributeEditor.doEmptySelection()")


  local statePanel = panel:getChild("StateMachineCombo")
  if not statePanel then
    panel:clear()

    panel:beginVSizer{ flags = "expand" }
      statePanel = panel:addPanel{
        name = "StateMachineCombo",
        proportion = 0,
        flags = "expand"
      }

      local rollContainer = panel:addRollupContainer{ 
        flags = "expand",
        name = "MainRollupContainer",
        proportion = 1
      }

      -- Control Parameters
      attributeEditor.log("adding control parameters rollup")
      local rollup = rollContainer:addRollup{ label = "Control Parameters", flags = "mainSection", name = "controlParameters" }
      local rollPanel = rollup:getPanel()
      rollPanel:beginVSizer{ flags = "expand" }
        rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
          rollPanel:addStockWindow{ type = "ControlParameters", proportion = 1, flags = "expand;sizeToContent" }
        rollPanel:endSizer()
      rollPanel:endSizer()

      -- Messages
      attributeEditor.log("adding messages rollup")
      rollup = rollContainer:addRollup{ label = "Messages", flags = "mainSection", name = "Requests" }
      rollPanel = rollup:getPanel()
      rollPanel:beginVSizer{ flags = "expand" }
        rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
          rollPanel:addStockWindow{ type = "Requests", proportion = 1, flags = "expand;sizeToContent" }
        rollPanel:endSizer()
      rollPanel:endSizer()

      -- Presets
      attributeEditor.log("adding presets rollup")
      rollup = rollContainer:addRollup{ label = "Message Presets", flags = "mainSection", name = "messagePresetGroups" }
      rollPanel = rollup:getPanel()
      rollPanel:beginVSizer{ flags = "expand" }
        rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
          rollPanel:addStockWindow{ type = "MessagePresets", proportion = 1, flags = "expand;sizeToContent" }
        rollPanel:endSizer()
      rollPanel:endSizer()

    panel:endSizer()
  end
  
  statePanel:clear()
  statePanel:setShown(false)
  
    -- If there is an empty selection on a state machine, show the default state combo box.
  local parent = getCurrentGraph()
  if getType(parent) == "StateMachine" then
    local currentState = getDefaultState(parent)
    local selectedIndex = 1

    -- Gather a list of nodes parented to this graph.
    attributeEditor.log("gathering list of child state machine nodes")
    local nodes = ls("State")
    local stateNodes = { }
    local currentIndex = 0
    for i, node in ipairs(nodes) do
      if getParent(node) == parent then
        currentIndex = currentIndex + 1
        if node == currentState then
          selectedIndex = currentIndex
        end
        local _, localName = splitNodePath(node)
        table.insert(stateNodes, localName)
      end
    end

    -- Only display combo if state machine contains nodes.
    if table.getn(stateNodes) > 0 then
      statePanel:setShown(true)
      statePanel:beginHSizer{ flags = "expand", proportion = 1 }
        attributeEditor.log("adding StaticText: Default State")
        statePanel:addStaticText { text = "Default State" }

        attributeEditor.log("adding child states ComboBox")
        local cb = statePanel:addComboBox{
          flags = "expand",
          proportion = 1,
          items = stateNodes,
          onChanged = function(self)
            -- Respond to change by setting selected state.
            local node = self:getSelectedItem()
            local fullPath = parent .. "|" .. node
            setDefaultState(parent, fullPath)
          end
        }
        cb:setSelectedIndex(selectedIndex)
      statePanel:endSizer()
    end
  end

  attributeEditor.logExitFunc("attributeEditor.doEmptySelection()")
end