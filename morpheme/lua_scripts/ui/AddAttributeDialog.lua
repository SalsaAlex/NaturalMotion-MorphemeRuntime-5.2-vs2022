------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

local numRows = 3
local numCols = 3

local getPanel = function(panels, row, col)
  if panels then
    return panels[row * 4 + col]
  end
  return nil
end

------------------------------------------------------------------------------------------------------------------------
-- called when the attribute type dialog box changes
------------------------------------------------------------------------------------------------------------------------
local onAttributeDialogComboBoxChanged = function(self)
  local parentPanel = self:getParent()
  local dlg = parentPanel:getParent()
  local type = self:getSelectedItem()

  -- freeze the dialog while building the attribute widgets
  dlg:freeze()

  -- build a table of all the panels and clear the value related panels
  -- the first row is the attribute name and type related controls so leave that
  -- as it is.
  local panels = { }
  for i = 0, numRows do
    for j = 0, numCols do
      local panelName = string.format("Panel_%d%d", i, j)

      local panel = dlg:getChild(panelName)
      if i > 0 then
        panel:clear()
        -- reset the border to 0 as clear also resets the border value
        panel:setBorder(0)
      end

      table.insert(panels, i * 4 + j, panel)
    end
  end

  if type == "Float" or type == "Int"  then
    -- add the label
    getPanel(panels, 1, 0):addStaticText{
      text = "Value:",
      flags = "right"
    }

    -- add the control
    getPanel(panels, 1, 1):addTextBox{ name = "Value", flags = "expand" }

    -- add the per animation set check box
    local perAnimSetCheckBox = getPanel(panels, 1, 3):addCheckBox{
      name = "PerAnimSet",
      label = "Per Anim Set",
      flags = "expand"
    }

    -- add the array check box
    local isArrayCheckBox = getPanel(panels, 2, 3):addCheckBox{
      name = "IsArray",
      label = "Is Array",
      flags = "expand"
    }

    -- add the sync with rig channels check box
    local syncCheckBox = getPanel(panels, 3, 3):addCheckBox{
      name = "SyncWithRig",
      label = "Sync with Rig",
      flags = "expand"
    }
    syncCheckBox:enable(false)

    local updateSyncCheckBox = function(self)
      local state = perAnimSetCheckBox:getChecked() and isArrayCheckBox:getChecked()
      syncCheckBox:enable(state)
      if not state then
        syncCheckBox:setChecked(false)
      end
    end

    perAnimSetCheckBox:setOnChanged(updateSyncCheckBox)
    isArrayCheckBox:setOnChanged(updateSyncCheckBox)

  elseif type == "Bool" then
    -- add the control
    getPanel(panels, 1, 1):addCheckBox{ name = "Value", label = "Value", flags = "expand" }

    -- add the per animation set check box
    perAnimSetCheckBox = getPanel(panels, 1, 3):addCheckBox{
      name = "PerAnimSet",
      label = "Per Anim Set",
      flags = "expand"
    }

    -- add the is array check box
    local isArrayCheckBox = getPanel(panels, 2, 3):addCheckBox{
      name = "IsArray",
      label = "Is Array",
      flags = "expand"
    }

    -- add the sync with rig channels check box
    local syncCheckBox = getPanel(panels, 3, 3):addCheckBox{
      name = "SyncWithRig",
      label = "Sync with Rig",
      flags = "expand"
    }
    syncCheckBox:enable(false)

    local updateSyncCheckBox = function(self)
      local state = perAnimSetCheckBox:getChecked() and isArrayCheckBox:getChecked()
      syncCheckBox:enable(state)
      if not state then
        syncCheckBox:setChecked(false)
      end
    end

    perAnimSetCheckBox:setOnChanged(updateSyncCheckBox)
    isArrayCheckBox:setOnChanged(updateSyncCheckBox)

  elseif type == "String" then
    -- add the label
    getPanel(panels, 1, 0):addStaticText{
      text = "Value:",
      flags = "right"
    }

    -- add the control
    getPanel(panels, 1, 1):addTextBox{ name = "Value", flags = "expand" }

    -- add the per animation set check box
    getPanel(panels, 1, 3):addCheckBox{
      name = "PerAnimSet",
      label = "Per Anim Set",
      flags = "expand"
    }

  elseif type == "Animation Take" then
    -- add the label
    getPanel(panels, 1, 0):addStaticText{ text = "File:", flags = "right" }

    -- add the control
    getPanel(panels, 1, 1):addFilenameControl{
      name = "File",
      caption = "Add Animation Take attribute file",
      onMacroize = utils.macroizeString,
      onDemacroize = utils.demacroizeString,
      wildcard = "All files|*" }

    -- add the label
    getPanel(panels, 1, 2):addStaticText{ text = "Format:", flags = "right" }
    -- add the control
    getPanel(panels, 1, 3):addTextBox{ name = "Format" }

    -- add the label
    getPanel(panels, 2, 0):addStaticText{ text = "Take:", flags = "right" }
    -- add the control
    getPanel(panels, 2, 1):addTextBox{ name = "Take" }

    -- add the label
    getPanel(panels, 2, 2):addStaticText{ text = "Sync Track:", flags = "right" }
    -- add the control
    getPanel(panels, 2, 3):addComboBox{ name = "SyncTrack", flags = "expand" }

    getPanel(panels, 3, 3):addCheckBox{
      name = "PerAnimSet",
      label = "Per Anim Set",
      flags = "expand"
    }

  elseif type == "Message" then
    -- TODO: add Message combo box

    -- add the per animation set check box
    getPanel(panels, 1, 3):addCheckBox{
      name = "PerAnimSet",
      label = "Per Anim Set",
      flags = "expand"
    }
  elseif type == "Body Group Array" then
    -- TODO: add body group attribute

    -- add the per animation set check box
    getPanel(panels, 1, 3):addCheckBox{
      name = "PerAnimSet",
      label = "Per Anim Set",
      flags = "expand"
    }
  elseif type == "Control Parameter" then
    -- TODO: add control parameter combo box

    -- add the per animation set check box
    getPanel(panels, 1, 3):addCheckBox{
      name = "PerAnimSet",
      label = "Per Anim Set",
      flags = "expand"
    }
  elseif type == "Filename" then
    -- add the label
    getPanel(panels, 1, 0):addStaticText{
      text = "Value:",
      flags = "right"
    }

    -- add the control
    getPanel(panels, 1, 1):addFilenameControl{
      name = "Value",
      caption = "Add Filename attribute value",
      onMacroize = utils.macroizeString,
      onDemacroize = utils.demacroizeString,
      wildcard = "All files|*"
    }

    -- add the per animation set check box
    getPanel(panels, 1, 3):addCheckBox{
      name = "PerAnimSet",
      label = "Per Anim Set",
      flags = "expand"
    }
  end

  -- rebuild all the panels
  for i, panel in panels do
    panel:rebuild()
  end

  -- rebuild the whole dialog
  dlg:rebuild()

  -- thaw the dialog to now refresh it
  dlg:thaw()
end

------------------------------------------------------------------------------------------------------------------------
-- gets a control from the add attribute dialog
------------------------------------------------------------------------------------------------------------------------
local getAddAttributeControl = function(dlg, panel, control)
  if dlg then
    local panel = dlg:getChild(panel)
    if panel then
      return panel:getChild(control)
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
-- called when the add attribute button is clicked
------------------------------------------------------------------------------------------------------------------------
local onAddAttributeClicked = function(dlg, pathOrPaths)
  local newAttributeTable = { }

  local nameTextBox = getAddAttributeControl(dlg, "Panel_01", "Name")
  if nameTextBox then
    local name = nameTextBox:getValue()
    newAttributeTable.name = name
  end

  local typeComboBox = getAddAttributeControl(dlg, "Panel_03", "AttributeType")
  if typeComboBox then
    local attrType = typeComboBox:getSelectedItem()
    newAttributeTable.type = attrType
  end

  if newAttributeTable.type == "Float" then
    local valueTextBox = getAddAttributeControl(dlg, "Panel_11", "Value")
    if valueTextBox then
      local value = valueTextBox:getValue()
      newAttributeTable.value = tonumber(value)
    end

    local perAnimSetCheckBox = getAddAttributeControl(dlg, "Panel_13", "PerAnimSet")
    if perAnimSetCheckBox then
      local perAnimSet = perAnimSetCheckBox:getChecked()
      newAttributeTable.perAnimSet = perAnimSet
    end

    local isArray = false
    local isArrayCheckBox = getAddAttributeControl(dlg, "Panel_23", "IsArray")
    if isArrayCheckBox then
      isArray = isArrayCheckBox:getChecked()
    end

    if isArray then
      newAttributeTable.type = "floatArray"

      local syncWithRigCheckBox = getAddAttributeControl(dlg, "Panel_33", "SyncWithRig")
      if syncWithRigCheckBox then
        local syncWithRig = syncWithRigCheckBox:getChecked()
        newAttributeTable.syncWithRigChannels = syncWithRig
      end
    else
      newAttributeTable.type = "float"
    end

  elseif newAttributeTable.type == "Int" then
    local valueTextBox = getAddAttributeControl(dlg, "Panel_11", "Value")
    if valueTextBox then
      local value = valueTextBox:getValue()
      newAttributeTable.value = tonumber(value)
    end

    local perAnimSetCheckBox = getAddAttributeControl(dlg, "Panel_13", "PerAnimSet")
    if perAnimSetCheckBox then
      local perAnimSet = perAnimSetCheckBox:getChecked()
      newAttributeTable.perAnimSet = perAnimSet
    end

    local isArray = false
    local isArrayCheckBox = getAddAttributeControl(dlg, "Panel_23", "IsArray")
    if isArrayCheckBox then
      isArray = isArrayCheckBox:getChecked()
    end

    if isArray then
      newAttributeTable.type = "intArray"

      local syncWithRigCheckBox = getAddAttributeControl(dlg, "Panel_33", "SyncWithRig")
      if syncWithRigCheckBox then
        local syncWithRig = syncWithRigCheckBox:getChecked()
        newAttributeTable.syncWithRigChannels = syncWithRig
      end
    else
      newAttributeTable.type = "int"
    end

  elseif newAttributeTable.type == "Bool" then
    local valueTextBox = getAddAttributeControl(dlg, "Panel_11", "Value")
    if valueTextBox then
      local value = valueTextBox:getChecked()
      newAttributeTable.value = value
    end

    local perAnimSetCheckBox = getAddAttributeControl(dlg, "Panel_13", "PerAnimSet")
    if perAnimSetCheckBox then
      local perAnimSet = perAnimSetCheckBox:getChecked()
      newAttributeTable.perAnimSet = perAnimSet
    end

    local isArray = false
    local isArrayCheckBox = getAddAttributeControl(dlg, "Panel_23", "IsArray")
    if isArrayCheckBox then
      isArray = isArrayCheckBox:getChecked()
    end

    if isArray then
      newAttributeTable.type = "boolArray"

      local syncWithRigCheckBox = getAddAttributeControl(dlg, "Panel_33", "SyncWithRig")
      if syncWithRigCheckBox then
        local syncWithRig = syncWithRigCheckBox:getChecked()
        newAttributeTable.syncWithRigChannels = syncWithRig
      end
    else
      newAttributeTable.type = "bool"
    end

  elseif newAttributeTable.type == "Animation Take" then
    newAttributeTable.type = "animationTake"

    newAttributeTable.value = { }

    local fileFileControl = getAddAttributeControl(dlg, "Panel_11", "File")
    if fileFileControl then
      local file = fileFileControl:getValue()
      newAttributeTable.value.filename = file
    end

    local formatTextBox = getAddAttributeControl(dlg, "Panel_13", "Format")
    if formatTextBox then
      local format = formatTextBox:getValue()
      newAttributeTable.value.format = format
    end

    local takeTextBox = getAddAttributeControl(dlg, "Panel_21", "Take")
    if takeTextBox then
      local take = takeTextBox:getValue()
      newAttributeTable.value.takename = take
    end

    local syncTrackComboBox = getAddAttributeControl(dlg, "Panel_23", "SyncTrack")
    if syncTrackComboBox then
      local syncTrack = syncTrackComboBox:getSelectedItem()
      newAttributeTable.value.synctrack = syncTrack
    end

    local perAnimSetCheckBox = getAddAttributeControl(dlg, "Panel_33", "PerAnimSet")
    if perAnimSetCheckBox then
      local perAnimSet = perAnimSetCheckBox:getChecked()
      newAttributeTable.perAnimSet = perAnimSet
    end

  elseif newAttributeTable.type == "Message" then
    newAttributeTable.type = "request"

    local perAnimSetCheckBox = getAddAttributeControl(dlg, "Panel_13", "PerAnimSet")
    if perAnimSetCheckBox then
      local perAnimSet = perAnimSetCheckBox:getChecked()
      newAttributeTable.perAnimSet = perAnimSet
    end
  elseif newAttributeTable.type == "Body Group Array" then
    newAttributeTable.type = "bodyGroupArray"

    local perAnimSetCheckBox = getAddAttributeControl(dlg, "Panel_13", "PerAnimSet")
    if perAnimSetCheckBox then
      local perAnimSet = perAnimSetCheckBox:getChecked()
      newAttributeTable.perAnimSet = perAnimSet
    end
  elseif newAttributeTable.type == "Control Parameter" then
    newAttributeTable.type = "controlParameter"

    local perAnimSetCheckBox = getAddAttributeControl(dlg, "Panel_13", "PerAnimSet")
    if perAnimSetCheckBox then
      local perAnimSet = perAnimSetCheckBox:getChecked()
      newAttributeTable.perAnimSet = perAnimSet
    end
  elseif newAttributeTable.type == "Filename" then
    newAttributeTable.type = "filename"

    local valuePanel = dlg:getChild("Panel_11")
    local valueFileControl = valuePanel:getChild("Value")
    local value = valueFileControl:getValue()

    newAttributeTable.value = value

    local perAnimSetPanel = dlg:getChild("Panel_13")
    local perAnimSetCheckBox = perAnimSetPanel:getChild("PerAnimSet")
    local perAnimSet = perAnimSetCheckBox:getChecked()

    newAttributeTable.perAnimSet = perAnimSet

  elseif newAttributeTable.type == "String" then
    newAttributeTable.type = "string"

    local valuePanel = dlg:getChild("Panel_11")
    local valueTextBox = valuePanel:getChild("Value")
    local value = valueTextBox:getValue()

    newAttributeTable.value = value

    local perAnimSetPanel = dlg:getChild("Panel_13")
    local perAnimSetCheckBox = perAnimSetPanel:getChild("PerAnimSet")
    local perAnimSet = perAnimSetCheckBox:getChecked()

    newAttributeTable.perAnimSet = perAnimSet
  end

  if type(pathOrPaths) == "string" then
    local path = pathOrPaths
    newAttributeTable.path = path
    addAttribute(newAttributeTable)
  elseif type(pathOrPaths) == "table" then
    for i, path in ipairs(pathOrPaths) do
      if type(path) == "string" then
        newAttributeTable.path = path
        addAttribute(newAttributeTable)
      end
    end
  end

  dlg:hide()
end

------------------------------------------------------------------------------------------------------------------------
-- shows the add attribute dialog
------------------------------------------------------------------------------------------------------------------------
showAddAttributeDialog = function(path)
  local attributeTypeDisplayNames = {
    "Float",
    "Int",
    "Bool",
    "String",
    "Filename",
    "Animation Take",
    "Message",
    "Body Group Array",
    "Control Parameter"
  }

  local dlg = ui.getWindow("AddAttributeDialog")
  local nameTextBox = nil
  local typeComboBox = nil
  local addAttributeButton = nil

  -- if the dialog was not found then it is the first time this function has been called and the
  -- dialog must be created.
  if not dlg then
    dlg = ui.createModelessDialog{
      name = "AddAttributeDialog",
      caption = "Add Attribute",
      resize = true,
      size = { width = 360, height = 160 }
    }

    dlg:beginVSizer()
      dlg:beginFlexGridSizer{
        rows = 4,
        cols = 4,
        flags = "expand"
      }

        -- add a panel to every cell in the flex grid sizer so individual
        -- controls can be cleared without loosing the whole dialog
        local panels = { }
        for i = 0, numRows do
          for j = 0, numCols do
            local panel = dlg:addPanel{
              name = string.format("Panel_%d%d", i, j),
              flags = "expand"
            }
            panel:setBorder(0)
            table.insert(panels, i * 4 + j, panel)
          end
        end

      dlg:endSizer()

      -- add the first row controls for name and type
      local panel = getPanel(panels, 0, 0)
      panel:addStaticText{
        text = "Name:",
        flags= "right"
      }

      panel = getPanel(panels, 0, 1)
      nameTextBox = panel:addTextBox{
        name = "Name",
        flags = "expand"
      }

      panel = getPanel(panels, 0, 2)
      panel:addStaticText{
        text = "Type:",
        flags= "right"
      }

      panel = getPanel(panels, 0, 3)
      typeComboBox = panel:addComboBox{
        name = "AttributeType",
        proportion = 1,
        items = attributeTypeDisplayNames,
        onChanged = onAttributeDialogComboBoxChanged
      }

      -- add the "Add Attribute" and "Cancel" buttons
      dlg:beginHSizer{ flags = "right" }
        addAttributeButton = dlg:addButton{
          name = "AddAttribute",
          label = "Add Attribute",
        }

        -- only enable the button if there is a name set
        nameTextBox:setOnChanged(
          function(self)
            local value = self:getValue()
            local enable = (string.len(value) ~= 0)
            addAttributeButton:enable(enable)
          end
        )

        local button = dlg:addButton{
          label = "Cancel",
          onClick = function(self)
            local dlg = self:getParent()
            dlg:hide()
          end
        }

      dlg:endSizer()

    dlg:endSizer()
  else
    nameTextBox = ui.getWindow("AddAttributeDialog|Panel_01|Name")
    typeComboBox = ui.getWindow("AddAttributeDialog|Panel_03|AttributeType")
    addAttributeButton = ui.getWindow("AddAttributeDialog|AddAttribute")
  end

  if nameTextBox then
    nameTextBox:setValue("")
  end

  if typeComboBox then
    onAttributeDialogComboBoxChanged(typeComboBox)
  end

  if addAttributeButton then
    addAttributeButton:enable(false)

    addAttributeButton:setOnClick(
      function()
        onAddAttributeClicked(dlg, path)
      end
    )
  end

  dlg:show()
end
