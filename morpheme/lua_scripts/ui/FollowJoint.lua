------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- show follow joint dialog
------------------------------------------------------------------------------------------------------------------------
local showFollowJointDialog = function(scene, viewport)
  local dlg = ui.getWindow("FollowJoint")

  local listControl = nil
  if dlg == nil then
    dlg = ui.createModelessDialog{
      name = "FollowJoint",
      caption = string.format("Follow Joint"),
      resize = true,
      centre = true,
      size = { width = 300, height = 150 }
    }
  else
    --listControl = dlg:getChild("JointList")
    dlg:clear()
  end

  dlg:beginVSizer()
    listControl = dlg:addListControl{
      name = "JointList",
      flags = "expand",
      proportion = 1,
      columnNames = {
        "Joints"
      }
    }

  dlg:beginHSizer{ flags="right" }
    dlg:addButton{
      label = "Follow Joint",
      onClick = function()
        local jointName = listControl:getItemValue(listControl:getSelectedRow(), 1)
        local scene = viewport:getSceneName()
        local animSet
        if scene == "Network" then
          animSet = getSelectedAnimSet()
        else
          animSet = getSelectedAssetManagerAnimSet()
        end
        anim.setAnimSetFollowJoint(animSet, scene, jointName)
        dlg:hide()
      end,
      }
  dlg:endSizer()

  dlg:endSizer()

  local animSet
  if scene == "Network" then
    animSet = getSelectedAnimSet()
  else
    animSet = getSelectedAssetManagerAnimSet()
  end
  local channels = anim.getRigChannelNames(animSet)
  for i,v in ipairs(channels) do
    listControl:addRow{ v }
  end

  dlg:show()
end

------------------------------------------------------------------------------------------------------------------------
--add menu item to open dialog
------------------------------------------------------------------------------------------------------------------------
addFollowJointItem = function(menu, scene, viewport, label)
  menu:addItem{
    label = label,
    onClick = function()
      showFollowJointDialog(scene, viewport)
    end
  }
end
