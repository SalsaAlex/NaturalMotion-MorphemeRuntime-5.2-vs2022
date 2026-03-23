------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

local dataChangeEventContext = nil

------------------------------------------------------------------------------------------------------------------------
-- Rebuild the list of the custom attributes in the currently selected node
------------------------------------------------------------------------------------------------------------------------
local refreshAttributeList = function()

  dataChangeEventContext:clearObjects()
  dataChangeEventContext:clearAllAttributeChangeEvents()

  local list = ui.getWindow("EditAttributeDialog|AttributeList")
  if list then
    list:clearRows()
    local selection = ls("Selection")
    if table.getn(selection) > 0 then

      dataChangeEventContext:setObjects(selection)

      local attributeNames = getCommonAttributes(selection)
      table.sort(attributeNames)

      for i, attributeName in ipairs(attributeNames) do
        local attributePath = string.format("%s.%s", selection[1], attributeName)
        if not isManifestAttribute(attributePath) then

          dataChangeEventContext:addAttributeChangeEvent(attributeName)

          local attributeType, attributeInfo = getAttributeType(attributePath)
          if (attributeType == "request") then 
            attributeType = "message"
          end
          rowData = { attributeName, attributeType }
          list:addRow(rowData)
        end
      end
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Enable buttons to reflect the current list selection
------------------------------------------------------------------------------------------------------------------------
local refreshButtons = function()
  local list = ui.getWindow("EditAttributeDialog|AttributeList")
  if list then

    local addButton    = ui.getWindow("EditAttributeDialog|AddButton")
    local removeButton = ui.getWindow("EditAttributeDialog|RemoveButton")

    if addButton and removeButton then
      local itemCount = list:getNumRows()
      local selection = list:getSelectedRows()
      removeButton:enable(table.getn(selection) > 0)
      addButton:enable(true)
    end

  end
end

------------------------------------------------------------------------------------------------------------------------
-- Open the AddAttributeDialog to add an attribute to the currently selected node
------------------------------------------------------------------------------------------------------------------------
local addAttribute = function()
  local selection = ls("Selection")
  showAddAttributeDialog(selection)
end

------------------------------------------------------------------------------------------------------------------------
-- Remove the selected attributes.
------------------------------------------------------------------------------------------------------------------------
local removeSelectedAttributes = function()
  local list = ui.getWindow("EditAttributeDialog|AttributeList")
  if list then
    local selection = ls("Selection")
    if table.getn(selection) > 0 then

      local attributeNames = { }
      local selectedRows = list:getSelectedRows()
      for i, selectedRow in ipairs(selectedRows) do
        local attributeName = list:getItemValue(selectedRow, 1)
        table.insert(attributeNames, attributeName)
      end

      for i, objectName in ipairs(selection) do
        for j, attributeName in ipairs(attributeNames) do
          removeAttribute(objectName, attributeName)
        end
      end
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
-- shows the edit attribute dialog
------------------------------------------------------------------------------------------------------------------------
showEditAttributeDialog = function()

  local dlg = ui.getWindow("EditAttributeDialog")

  -- if the dialog was not found then it is the first time this function has been called and the
  -- dialog must be created.
  if not dlg then
    dlg = ui.createModalDialog{
      name = "EditAttributeDialog",
      caption = "Custom Properties",
      size = { width = 250, height = 300 },
      resize = true
    }

    dlg:beginHSizer{ flags = "expand", proportion = 1 }
      dlg:beginVSizer{ flags = "expand", proportion = 1 }

        -- add the add/remove buttons
        dlg:beginHSizer()

          dlg:addButton{
            name = "AddButton",
            image = app.loadImage("additem.png"),
            helpText = "Add custom property",
            onClick = addAttribute
          }

          dlg:addButton{
            name = "RemoveButton",
            image = app.loadImage("removeitem.png"),
            helpText = "Remove custom property",
            onClick = removeSelectedAttributes
          }

        dlg:endSizer()

        -- add the attribute list control
        dlg:addListControl{
          name = "AttributeList",
          flags = "expand;multiselect;gridLines",
          columnNames = { "Name", "Type" },
          proportion = 1,
          onSelectionChanged = refreshButtons
        }

      dlg:endSizer()
    dlg:endSizer()
  end

  -- Set up a data change event context to listen for attributes
  -- being added or removed, so we can update the attribute list
  dataChangeEventContext = createDataChangeEventContext()
  dataChangeEventContext:setAttributeAddedHandler(
    function(object, attr)
      refreshAttributeList()
      refreshButtons()
    end
  )
  dataChangeEventContext:setAttributeRemovedHandler(
    function(object, attr)
      refreshAttributeList()
      refreshButtons()
    end
  )

  refreshAttributeList()
  refreshButtons()
  dlg:show()

  -- Clean up the data change event context
  deleteDataChangeEventContext(dataChangeEventContext)
  dataChangeEventContext = nil
end

