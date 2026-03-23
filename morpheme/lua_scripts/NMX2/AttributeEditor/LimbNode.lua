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

local attributeGroups, errorString = scriptManager:createCallback(
  -- The scripting language the callback is written in
  --
  "lua",

  -- The arguments that the callback will take.
  --
  nmx.CallbackManager.AttributeGroupsCallbackSignature(),

  -- The callback function body
  --
  [[
  validAttributes:push_back{'Limb Attributes', 'Type', 'Colour'}
  ]]
)

-- reuse the same callback fro spines and extermities

-- Unregister any previous callbacks
callbackManager:unRegisterAttributeGroupsCallback(app:lookupTypeId('ExtremityNode'))
-- Register the new callback
callbackManager:registerAttributeGroupsCallback(app:lookupTypeId('ExtremityNode'), attributeGroups)
-- Unregister any previous callbacks
callbackManager:unRegisterAttributeGroupsCallback(app:lookupTypeId('SpineNode'))
-- Register the new callback
callbackManager:registerAttributeGroupsCallback(app:lookupTypeId('SpineNode'), attributeGroups)

