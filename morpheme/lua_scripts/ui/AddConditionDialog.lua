------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- shows the dialog for adding a condition to a specific condition
------------------------------------------------------------------------------------------------------------------------
showAddConditionDialog = function(transitionOrTransitions)
  local dlg = ui.getWindow("AddConditionDialog")

  local listControl = nil
  local helpTextControl = nil
  local addButton = nil

  local conditionTypes = listTypes("Condition")
  local numRows = table.getn(conditionTypes)

  -- if the dialog couldn't be found then create it
  if not dlg then
    dlg = ui.createModalDialog{ caption = "Create condition" }

    listControl = dlg:addListControl{
      name = "ConditionsList",
      flags = "expand;gridLines",
      size = { width = 200 },
      proportion = 0,
      numRows = numRows,
      columnNames = { "Condition type" },
    }

    for i, type in ipairs(conditionTypes) do
      listControl:setRow(i, type)
    end

    dlg:addStaticText{
      text = "Description",
      flags = "expand"
    }

    helpTextControl = dlg:addTextControl{
      name = "HelpTextControl",
      flags = "expand",
      size = { width = 320, height = 110 }
    }

    helpTextControl:setReadOnly(true)

    -- set the function to update the helptext box
    listControl:setOnSelectionChanged(
      function(self)
        local index = self:getSelectedRow()
        local conditionType = conditionTypes[index]

        if conditionType then
         helpTextControl:setValue(getHelpText(conditionType))
        else
         helpTextControl:setValue("")
        end
      end
    )

    listControl:selectRow(1)
    helpTextControl:setValue(getHelpText(conditionTypes[1]))

    addButton = dlg:addButton{
      name = "CreateButton",
      label = "Create condition",
      flags = "right"
    }
  else
    listControl = dlg:getChild("ConditionsList")
    helpTextControl = dlg:getChild("HelpTextControl")
    addButton = dlg:getChild("CreateButton")
  end

  if listControl == nil then
    return
  end

  if helpTextControl == nil then
    return
  end

  if addButton == nil then
    return
  end

  -- this function creates the selected condition
  local addSelectedCondition = function(self)
    local index = listControl:getSelectedRow()
    local conditionType = conditionTypes[index]

    if conditionType then
      undoBlock(function()
        if type(transitionOrTransitions) == "string" then
          local transition = transitionOrTransitions
          create(conditionType, transition)
        elseif type(transitionOrTransitions) == "table" then
          for i, transition in ipairs(transitionOrTransitions) do
            if type(transition) == "string" then
              create(conditionType, transition)
            end
          end
        end
      end)
    end

    dlg:hide()
  end

  -- set the create transition function
  listControl:setOnItemActivated(addSelectedCondition)
  addButton:setOnClick(addSelectedCondition)

  dlg:show()
end
