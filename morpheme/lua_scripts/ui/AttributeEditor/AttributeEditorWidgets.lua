require "ui/attributeEditor/AttributeEditor.lua"

------------------------------------------------------------------------------------------------------------------------
-- displays a dialog that can fix the order of animation set attributes so they can be displayed in an
-- attribute editor group
------------------------------------------------------------------------------------------------------------------------
local showFixAttributeAnimSetOrderDialog = function(selection, attributeNames)
  if type(selection) ~= "table" or table.getn(selection) == 0 then
    return false
  end
  
  if type(attributeNames) ~= "table" or table.getn(attributeNames) == 0then
    return false
  end

  local dlgName = "FixAttributeAnimSetOrder"

  local dlg = ui.getWindow(dlgName)
  local listControl = nil
  local reorderButton = nil
  local cancelButton = nil

  if not dlg then
    dlg = ui.createModalDialog{
      name = dlgName,
      caption = "Fix attribute group set order",
    }

    local description =
[[The per animation set attributes in the list below have different animation set order and data
and cannot be displayed as a group within the attribute editor.

Select one of the attributes in the list below and click Reorder to ensure that all attributes
have the same animation set order and data as the selected attribute.

]]
    dlg:beginVSizer()
      dlg:addStaticText{
        text = description,
        flags = "expand",
      }

      listControl = dlg:addListControl{
        name = "AttributesListControl",
        proportion = 1,
        flags = "expand",
      }

      dlg:beginHSizer{ flags = "right", }
        reorderButton = dlg:addButton{
          name = "ReorderButton",
          label = "Reorder",
          size = { width = 74, },
          onClick = function(self)
          
          end,
        }
        cancelButton = dlg:addButton{
          name = "CancelButton",
          label = "Cancel",
          size = { width = 74, },
          onClick = function(self)
            dlg:hide()
          end,
        }
      dlg:endSizer()

      cancelButton:setOnClick(
        function(self)
          local dlg = self:getParent()
          dlg:hide()
        end
      )
    dlg:endSizer()
  else
    listControl = dlg:getChild("AttributesListControl")
    reorderButton = dlg:getChild("ReorderButton")
    cancelButton = dlg:getChild("CancelButton")
  end

  listControl:clearRows()

  local attributeNameCount = table.getn(attributeNames)
  for i = 1, attributeNameCount do
    local attributeName = attributeNames[i]
    listControl:addRow(attributeName)
  end

  listControl:setOnSelectionChanged(
    function(self)
      local dlg = self:getParent()
      local reorderButton = dlg:getChild("ReorderButton")

      local rowIndex = listControl:getSelectedRow()
      if rowIndex then
        reorderButton:enable(true)
      else
        reorderButton:enable(false)
      end
    end
  )

  local didReorder = false

  reorderButton:setOnClick(
    function(self)
      local dlg = self:getParent()
      local listControl = dlg:getChild("AttributesListControl")

      local rowIndex = listControl:getSelectedRow()
      if rowIndex then
        local sourceAttributeName = attributeNames[rowIndex]
        local sourceAttribute = string.format("%s.%s", selection[1], sourceAttributeName)

        local sourceSetOrder = listAnimSetOrder(sourceAttribute)
        local sourceSetData = { }
        local setCount = table.getn(sourceSetOrder)
        for i = 1, setCount do
          local set = sourceSetOrder[i]
          if hasAnimSetData(sourceAttribute, set) then
            sourceSetData[set] = true
          end
        end

        local selectionCount = table.getn(selection)
        local attributeNameCount = table.getn(attributeNames)
        for i = 1, selectionCount do
          for j = 1, attributeNameCount do
            if i ~= 1 or j ~= rowIndex then
              local attribute = string.format("%s.%s", selection[i], attributeNames[j])

              -- now make sure they both have data for the same sets
              -- this is currently the only way of filling in sets with missing data
              local value = getAttribute(attribute)
              for k = 1, setCount do
                local set = sourceSetOrder[k]

                if sourceSetData[set] then
                  setAttribute(attribute, value)
                else
                  destroyAnimSetData(attribute, set)
                end
              end

              -- enforce the same set order
              -- this has to come second to prevent further attribute editor errors
              changeAnimSetOrder(attribute, sourceSetOrder)
            end
          end
        end
        
        didReorder = true
        dlg:hide()
      end
    end
  )
  reorderButton:enable(false)

  dlg:show()
  
  return didReorder
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: Window attributeEditor.addAnimationSetWidget(Panel panel, string attribute, table selection, function displayFunc)
--| brief:
--|   Adds a per animation set attribute widget to the panel given by calling panel:addAnimationSetWidget.
--|
--| environments: GlobalEnv
--| page: AttributeEditorAPI
------------------------------------------------------------------------------------------------------------------------
attributeEditor.addAnimationSetWidget = function(panel, attributes, selection, displayFunc)
  -- build the table of attribute paths
  local attributePaths = checkSetOrderAndBuildAttributeList(selection, attributes)

  local widget = nil
  if attributePaths then
    widget = panel:addAnimationSetWidget{
      attributes = attributePaths,
      displayFunc = displayFunc,
      flags = "expand",
      proportion = 1,
    }
  else
    local errorDescription = "Per animation set attribute group contains\nattributes with differing set order."
    panel:addStaticText{
      text = errorDescription,
    }

    widget = panel:addButton{
      label = "Fix set order",
      size = { width = 74 },
      flags = "right",
      onClick = function(self)
        if showFixAttributeAnimSetOrderDialog(selection, attributes) then
          attributeEditor.onSelectionChange()
        end
      end
    }
  end

  return widget
end