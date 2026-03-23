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
     -- RefNode shouldn't display any attributes
  ]]
)

-- Unregister any previous callbacks
callbackManager:unRegisterAttributeGroupsCallback(nmx.RefNode.ClassTypeId())
-- Register the new callback
callbackManager:registerAttributeGroupsCallback(nmx.RefNode.ClassTypeId(), attributeGroups)

------------------------------------------------------------------------------------------------------------------------
-- Associated nodes
------------------------------------------------------------------------------------------------------------------------

local associatedNodesCallback, associatedNodesErrorString = scriptManager:createCallback(
  -- The scripting language the callback is written in
  --
  "lua",

  -- The arguments that the callback will take.
  --
  nmx.CallbackManager.AssociatedNodesCallbackSignature(),

  -- The callback function body
  --
  [[
    sceneInstance = node:getConnectedNode()
    if sceneInstance:is(nmx.sgShapeNode.ClassTypeId()) then

      local parent = sceneInstance:getParent()
      
      -- Associate the parent transform
      associatedNodes:push_back(parent)
      
      local transformInput = parent:findAttribute("LocalMatrix"):getInput():getNode()
      if transformInput ~= nil and transformInput:is(nmx.TransformNode.ClassTypeId()) then
        associatedNodes:push_back(transformInput)
      end
      
      local child = parent:getFirstChild()
      while child do
        if child:is(nmx.sgShapeNode.ClassTypeId()) then
          associatedNodes:push_back(child:getDataNode())
        end
        child = child:getNextSibling()
      end
    end
  ]]
)

-- Unregister any previous callbacks
callbackManager:unRegisterAssociatedNodesCallback(nmx.RefNode.ClassTypeId())
-- Register the new callback
callbackManager:registerAssociatedNodesCallback(nmx.RefNode.ClassTypeId(), associatedNodesCallback)

