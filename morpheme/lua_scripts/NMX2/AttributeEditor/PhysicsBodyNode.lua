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
    local physicEnabled = not mcn.isPhysicsDisabled or not mcn.isPhysicsDisabled()
    if physicEnabled then
      attrs = {
        'Physics Attributes',
        'AngularDamping',
        'LinearDamping',
        'EnableMaxAngularVelocity',
        'AutoCalculateMass',
        }

      local node = selectedNodes:front()
      if node:findAttribute("EnableMaxAngularVelocity"):asBool() then
        local index = table.getn(attrs)
        for i = 1, index do
          if attrs[i] == "EnableMaxAngularVelocity" then
            index = i + 1
            break
          end
        end
        table.insert(attrs, index, 'MaxAngularVelocity')
      end

      validAttributes:push_back(attrs)
      
      rebuildAttributes:push_back('EnableMaxAngularVelocity')
    end
  ]]
)

-- Unregister any previous callbacks
callbackManager:unRegisterAttributeGroupsCallback(nmx.PhysicsBodyNode.ClassTypeId())
-- Register the new callback
callbackManager:registerAttributeGroupsCallback(nmx.PhysicsBodyNode.ClassTypeId(), attributeGroups)

