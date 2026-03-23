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

local customPhysicsEngineDataAttributeGroups = scriptManager:createCallback(
  -- The scripting language the callback is written in
  --
  "lua",

  -- The arguments that the callback will take.
  --
  nmx.CallbackManager.AttributeGroupsCallbackSignature(),

  -- The callback function body
  --
  [[
    local customPhysicsEngineData = selectedNodes:front()

    local attributeGroup = {}

    -- find the index of the last non-dynamic attribute
    --
    local physicsEngineNameAttr = customPhysicsEngineData:findAttribute("PhysicsEngineName")
    local physicsEngineName = physicsEngineNameAttr:asString()
    table.insert(attributeGroup, string.format("%s Properties", physicsEngineName))

    -- every attribute after that index will be a custom one and should be written out
    --
    local lastElementIndex = physicsEngineNameAttr:elementIndex()
    for i = lastElementIndex + 1, customPhysicsEngineData:getNumElements() - 1 do
      local attribute = customPhysicsEngineData:getAttribute(i)
      local attributeName = attribute:getName()

      table.insert(attributeGroup, attributeName)
    end

    validAttributes:push_back(attributeGroup)
  ]]
)

-- Unregister any previous callbacks
callbackManager:unRegisterAttributeGroupsCallback("CustomPhysicsEngineDataNode")
-- Register the new callback
callbackManager:registerAttributeGroupsCallback("CustomPhysicsEngineDataNode", customPhysicsEngineDataAttributeGroups)