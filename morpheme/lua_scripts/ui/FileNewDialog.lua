------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

local buildDocumentTypes = function()
  local displayNames = { }
  local types = { }
  local nodeTypes = { }

  -- BlendTree
  table.insert(displayNames, "Blend tree")
  table.insert(types, "BlendTree")
  table.insert(nodeTypes, "")

    -- State Machine(s)
  for i, v in ipairs(listTypes("StateMachine")) do
    table.insert(displayNames, utils.getDisplayString(v))
    table.insert(types, "StateMachine")
    table.insert(nodeTypes, v)
  end

  if not mcn.isPhysicsDisabled() then
    -- PhysicsBlendTree
    table.insert(displayNames, "Physics blend tree")
    table.insert(types, "PhysicsBlendTree")
      table.insert(nodeTypes, "")

    -- State Machine(s)
    for i, v in ipairs(listTypes("PhysicsStateMachine")) do
      table.insert(displayNames, utils.getDisplayString(v))
      table.insert(types, "PhysicsStateMachine")
      table.insert(nodeTypes, v)
    end
  end

  return displayNames, types, nodeTypes
end

------------------------------------------------------------------------------------------------------------------------
-- When ok is clicked call mcn.new() with the desired arguments
------------------------------------------------------------------------------------------------------------------------
local onFileNewDialogOk = function()

  -- hide the file new dialog before creating new network
  local dlg = ui.getWindow("NewNetworkDialog")
  if dlg ~= nil then
    dlg:hide()
  end
  
  -- use the name the dialog was created with
  local rootTypeRadioBox = ui.getWindow("NewNetworkDialog|RootType")
  if rootTypeRadioBox ~= nil then
    local i = rootTypeRadioBox:getSelectedIndex()
    local displayNames, types, nodeTypes = buildDocumentTypes()
    mcn.new(types[i], nodeTypes[i], true)
  end

end

------------------------------------------------------------------------------------------------------------------------
-- Show the file new dialog
------------------------------------------------------------------------------------------------------------------------
showFileNewDialog = function()
  local dlg = ui.getWindow("NewNetworkDialog")

  -- if the dialog doesn't already exist then create it
  if not dlg then

    local dialogSize = {
      width = 200,
      height = 130
    }

    -- only add physics options if physics is enabled
    if mcn.isPhysicsDisabled() then
      dialogSize.height = 100
    end

    dlg = ui.createModalDialog{
      name = "NewNetworkDialog",
      caption = "New Morpheme Network",
      resize = false,
      size = dialogSize
    }

    dlg:beginVSizer{ flags = "expand" }
      dlg:addRadioBox{
        name = "RootType",
        flags = "expand",
        items = buildDocumentTypes()
      }

      dlg:addVSpacer(20)

      -- add the ok and cancel buttons
      dlg:beginHSizer{ flags = "right" }
        dlg:addButton{
          label = "Ok",
          onClick = onFileNewDialogOk,
          size = { width = 74 }
        }

        dlg:addButton{
          label = "Cancel",
          onClick = function()
            dlg:hide()
          end
        }
      dlg:endSizer()
    dlg:endSizer()
  end

  -- show the dialog
  dlg:show()
end
