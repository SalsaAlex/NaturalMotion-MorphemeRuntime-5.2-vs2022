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
  validAttributes:push_back{
    nmx.String.new('Display Settings'),
    nmx.String.new('Visible'),
    nmx.String.new('Mapping'),
    nmx.String.new('Path') }
  ]]
)

-- Unregister any previous callbacks
callbackManager:unRegisterAttributeGroupsCallback(nmx.sgTransformNode.ClassTypeId())
-- Register the new callback
callbackManager:registerAttributeGroupsCallback(nmx.sgTransformNode.ClassTypeId(), attributeGroups)

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
    local child = node:getFirstChild()
    while (child)
    do
      if (child:is(nmx.sgShapeNode.ClassTypeId()))
      then
        -- Add the shape instance
        associatedNodes:push_back(child)

        -- Add the dataNode of any child sgShapeNode
        local dataNode = child:getDataNode()
        if dataNode ~= nil then
          associatedNodes:push_back(dataNode)
          
          -- Add the material
          if not dataNode:is(nmx.JointNode.ClassTypeId()) then
            local material = child:getMaterial()
            if (material ~= nil)
            then
              associatedNodes:push_back(material)
            end
          end
        end
      end

      child = child:getNextSibling()
    end

    -- Add the input transform
    local matrixAttr = node:findAttribute('LocalMatrix')
    if (matrixAttr:isValid() and matrixAttr:hasInputConnection())
    then
      localMatrixInputNode = matrixAttr:getInput():getNode()
      if (localMatrixInputNode:is(nmx.TransformBaseNode.ClassTypeId()))
      then
        associatedNodes:push_back(matrixAttr:getInput():getNode())
      end
    end
  ]]
)

-- Unregister any previous callbacks
callbackManager:unRegisterAssociatedNodesCallback(nmx.sgTransformNode.ClassTypeId())
-- Register the new callback
callbackManager:registerAssociatedNodesCallback(nmx.sgTransformNode.ClassTypeId(), associatedNodesCallback)
