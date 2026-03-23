------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- called when the attribute type combo box is changed
local onAttributeTypeChanged = function(self)
  local dlg = self:getParent()

  dlg:freeze()

  local labelPanel = dlg:getChild("LabelPanel")
  labelPanel:clear()
  labelPanel:setBorder(0)

  local valuePanel = dlg:getChild("ValuePanel")
  valuePanel:clear()
  valuePanel:setBorder(0)

  local type = self:getSelectedItem()
  if type == "bool" then
    valuePanel:addCheckBox{
      name = "Value",
      label = "Value"
    }
  else
    labelPanel:addStaticText{
      text = "Value:",
      flags = "right"
    }
    if type == "int" then
      local textBox = valuePanel:addTextBox{
        name = "Value",
        flags = "numeric",
        size = { width = 40 },
      }
      textBox:setValue("0")
    elseif type == "float" then
      local textBox = valuePanel:addTextBox{
        name = "Value",
        flags = "numeric",
        size = { width = 40 },
      }
      textBox:setValue("0.0")
    else
      valuePanel:addTextBox{
        name = "Value",
        flags = "expand"
      }
    end
  end

  labelPanel:rebuild()
  valuePanel:rebuild()

  dlg:thaw()
end

------------------------------------------------------------------------------------------------------------------------
-- show the dialog
showAddAnimAttributeDialog = function(path)
  if not anim.isValidPath(path) then
    error("showAddAnimAttributeDialog: invalid path specified")
    return
  end

  local dlg = ui.getWindow("AddAnimAttributeDialog")

  local typeComboBox = nil
  local addAttributeButton = nil
  local pathTypeStaticText = nil
  local pathTextBox = nil

  if not dlg then
    dlg = ui.createModalDialog{
      name = "AddAnimAttributeDialog",
      caption = "Add Attribute",
      resize = false,
    }

    local attributeTypeNames = {
      "float",
      "int",
      "bool",
      "string"
    }

    dlg:beginVSizer()
      dlg:beginHSizer{ flags = "expand" }
        pathTypeStaticText = dlg:addStaticText{
          name = "PathType",
        }

        pathTextBox = dlg:addTextBox{
          name = "PathName",
          flags = "expand",
          proportion = 1
        }
      dlg:endSizer()

      dlg:beginFlexGridSizer{
        rows = 4,
        cols = 4,
        flags = "expand"
      }

      dlg:addStaticText{
        text = "Name:",
        flags= "right"
      }

      local nameTextBox = dlg:addTextBox{
        name = "Name"
      }

      dlg:addStaticText{
        text = "Type:",
        flags= "right"
      }

      typeComboBox = dlg:addComboBox{
        name = "AttributeType",
        items = attributeTypeNames,
        onChanged = onAttributeTypeChanged
      }

      dlg:addHSpacer(0)

      dlg:addHSpacer(0)

      local labelPanel = dlg:addPanel{
        name = "LabelPanel",
        flags = "expand"
      }

      local valuePanel = dlg:addPanel{
        name = "ValuePanel",
        flags = "expand"
      }

      dlg:endSizer()

      -- add the "Add Attribute" and "Cancel" buttons
      dlg:beginHSizer{ flags = "right" }

        addAttributeButton = dlg:addButton{
          name = "AddAttribute",
          label = "Add Attribute"
        }
        addAttributeButton:enable(false)

        -- only enable the add button if there is a name set
        nameTextBox:setOnChanged(
          function(self)
            local dlg = self:getParent()

            local value = self:getValue()
            local enable = (string.len(value) ~= 0)

            local addButton = dlg:getChild("AddAttribute")
            addButton:enable(enable)
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
    pathTypeStaticText = dlg:getChild("PathType")
    pathTextBox = dlg:getChild("PathName")

    local nameTextBox = dlg:getChild("Name")
    nameTextBox:setValue("")

    typeComboBox = dlg:getChild("AttributeType")

    addAttributeButton = dlg:getChild("AddAttribute")
    addAttributeButton:enable(false)
  end

  local pathType = anim.getType(path)
  if pathType == "event" then
    pathTypeStaticText:setLabel("Event:")
  elseif pathType == "track" then
    pathTypeStaticText:setLabel("Track:")
  end

  pathTextBox:setValue(path)
  pathTextBox:setReadOnly(true)

  onAttributeTypeChanged(typeComboBox)

  addAttributeButton:setOnClick(
    function(self)
      local dlg = self:getParent()
      local nameTextBox = dlg:getChild("Name")
      local name = nameTextBox:getValue()

      local typeComboBox = dlg:getChild("AttributeType")
      local type = typeComboBox:getSelectedItem()
      local valuePanel = dlg:getChild("ValuePanel")
      local valueControl = valuePanel:getChild("Value")

      local value = nil
      if type == "bool" then
        value = valueControl:getChecked()
      elseif type == "int" or type == "float" then
        value = valueControl:getValue()
        value = tonumber(value);
        if value == nil then
          value = 0
        end
      else
        value = valueControl:getValue()
      end

      local attribute = anim.addAttribute(path, type, name, value)

      dlg:hide()
    end
  )

  dlg:show()
end