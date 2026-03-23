------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- DistributeMassCommand.lua
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- Lazily create the nmxUtils table.
--
if nmxUtils == nil
then
  nmxUtils = { }
end

executeDistributeMassCommand = function(db, selectionList, mass)
  local bodies = {}
  local totalVolume = 0

  -- command expects you to select the data sgNode(s), and works from there
  for i=1,selectionList:size() do
    local transform = selectionList:getNode(i)
    if transform ~= nil then
      local physicsBody = nmx.PhysicsBodyNode.getPhysicsBody(transform);
      if physicsBody ~= nil then
        table.insert(bodies, physicsBody)
        totalVolume = totalVolume + nmx.PhysicsVolumeNode.getVolume(transform)
      end
    end
  end

  local density = tonumber(mass) / totalVolume

  for i,v in ipairs(bodies) do
    v:setDensity(density)
  end
end

showDistributeMassUI = function(db, selection)
  local dlg = ui.createModalDialog{ caption = "Distribute Mass" }
  dlg:beginHSizer()
    local mass = dlg:addTextBox{ value = "70.00000" }
    dlg:addButton{
            name = "DistributeButton",
            label = "Distribute",
            size = { width = 74 },
            onClick = function(self) executeDistributeMassCommand(db, selection, tonumber(mass:getValue())) dlg:hide() end,
          }
  dlg:endSizer()
  dlg:show()
end

-- Create a lua string which implements the create command.
-- This essentially just delegates to a function.
--
local callbackImplementation = [[showDistributeMassUI(database, selectionList)]]

-- Create a new callback object with the implementation provided by
-- callbackImplementation.
--
local app = nmx.Application.new()
local scriptManager = app:getScriptManager()
local callback, errorString = scriptManager:createCallback(
  "lua",
  nmx.Application.ScriptedCommandsSignature(),
  callbackImplementation
)

local associatedTypes = nmx.UIntArray.new()
associatedTypes:push_back(nmx.PhysicsBodyNode.ClassTypeId())

-- Register the scripted command create transform node.
--
app:registerScriptedCommand(
  "PhysicsTools",
  "Distribute Mass",
  "Distribute a mass between the selected bodies",
  callback,
  associatedTypes,
  nmx.TransformNode.ClassTypeId()
  )

