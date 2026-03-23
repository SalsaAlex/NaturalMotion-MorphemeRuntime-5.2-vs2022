------------------------------------------------------------------------------------------------------------------------
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
require "manifest/export/PhysicsExport.lua"
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
local getIndexInArray = function(objects, object)
  for i, current in ipairs(objects) do
    if object:is(current) then
      return i
    end
  end

  return -1
end

------------------------------------------------------------------------------------------------------------------------
exportPhysicsCollisionGroup = function(rigExport, group, joints, parts)
  local shapeNodes = nmx.sgShapeNodes.new()
  group:getGroupNodes(shapeNodes)

  if shapeNodes:empty() then
    return
  end

  -- use a table as as set to ensure that no duplicate parts are exported.
  --
  local partsToExport = { }
  local empty = true

  for i = 1, shapeNodes:size() do
    local shapeNode = shapeNodes:at(i)
    local bodyTxInst = shapeNode:getParent()
    local jointTxInst = bodyTxInst:getParent()

    local index = getIndexInArray(joints, jointTxInst)
    if index > 0 then
      partsToExport[index] = index
      empty = false
    end
  end

  if not empty then
    local set = rigExport:createDisabledCollisionSet()

    for i, index in pairs(partsToExport) do
      set:addPart(index)
    end

    local enabled = group:findAttribute("CanCollide"):asBool()
    set:setEnabled(enabled)
  end
end