-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold,
-- licensed or commercially exploited in any manner without the
-- written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential
-- information of NaturalMotion and may not be disclosed to any
-- person nor used for any purpose not expressly approved by
-- NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

-- Get hold of the application
--
local app = nmx.Application.new()
local scriptManager = app:getScriptManager()
local callbackManager = app:getCallbackManager()

local twistSwingDataAttributeGroups = scriptManager:createCallback(
  -- The scripting language the callback is written in
  --
  "lua",

  -- The arguments that the callback will take.
  --
  nmx.CallbackManager.AttributeGroupsCallbackSignature(),

  -- The callback function body
  --
  [[   
    local attrs = {
      'PhysX2 Drive Settings',
      'DriveType',
    }

    local node = selectedNodes:front()
    local driveTypeAttr = node:findAttribute("DriveType")
    if driveTypeAttr:asInt() == nmx.ExtraPhysX2TwistSwingInfoNode.DriveTypes.kSLERPDrive then
      table.insert(attrs, 'SLERPDriveSpring')
      table.insert(attrs, 'SLERPDriveDamping')
    else
      table.insert(attrs, 'SwingDriveSpring')
      table.insert(attrs, 'SwingDriveDamping')
      table.insert(attrs, 'TwistDriveSpring')
      table.insert(attrs, 'TwistDriveDamping')
    end

    table.insert(attrs, 'DriveSpringIsAcceleration')

    validAttributes:push_back(attrs)
    
    rebuildAttributes:push_back('DriveType')
  ]]
)

-- Unregister any previous callbacks
callbackManager:unRegisterAttributeGroupsCallback("PhysX2TwistSwingDataNode")
-- Register the new callback
callbackManager:registerAttributeGroupsCallback("PhysX2TwistSwingDataNode", twistSwingDataAttributeGroups)