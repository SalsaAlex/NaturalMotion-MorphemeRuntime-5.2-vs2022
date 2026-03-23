------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- Show Network Validation report dialog
------------------------------------------------------------------------------------------------------------------------
showNetworkValidationReport = function(ids, warnings, errors)
  -- used to store a list of object paths associated with the warnings and errors
  local objects = nil

  local dlg = ui.createModelessDialog{ caption = "Network Validation Report" }
  dlg:beginVSizer{ }
    local listCtrl = dlg:addListControl{
      flags = "expand", proportion = 1, size = { width = 500, height = 200 },
      numColumns = 1,
      onItemActivated = function(listControl, rowIndex)
        if objects[rowIndex] ~= nil and objects[rowIndex] ~= "" then
          select(objects[rowIndex])
        end
      end
    }
    dlg:beginHSizer{ flags = "expand", proportion = 0 }
      dlg:addStretchSpacer{ proportion = 1 }
      dlg:addButton{
        label = "Ok",
        size = { width = 74, height = -1 },
        onClick = function()
          dlg:hide()
        end
      }
    dlg:endSizer()
  dlg:endSizer()

  -- add all the errors to the dialog
  local rowIndex = 1
  for i, v in errors
  do
    listCtrl:addRow(v.message)

    objects[rowIndex] = v.name
    listCtrl:setRowColour(rowIndex, 237, 81, 65)

    rowIndex = rowIndex + 1
  end

  -- add all the warnings to the dialog
  for i, v in warnings
  do
    listCtrl:addRow(v.message)

    objects[rowIndex] = v.name
    listCtrl:setRowColour(rowIndex, 253, 193, 49)

    rowIndex = rowIndex + 1
  end

  dlg:show()
end