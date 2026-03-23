------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

-- Lazily create the nmxUtils table.
--
if nmxUtils == nil
then
  nmxUtils = { }
end

-- get the local matrix translation and rotation attributes
local getOffsetLocalTransformInformation = function(node)
  local app = nmx.Application.new()
  if node ~= nil and node:is(nmx.sgTransformNode.ClassTypeId()) then
    local offsetNode = node:getDataNode()
    if offsetNode ~= nil and offsetNode:is(app:lookupTypeId("OffsetFrameNode")) then
      local offsetAttribute = offsetNode:findAttribute("OffsetLocalMatrix")
      local transformNode = offsetAttribute:getInput():getNode()
      if transformNode ~= nil and transformNode:is(nmx.TransformNode.ClassTypeId()) then
        local translation = transformNode:getTranslation()
        local rotation = nmx.Vector3.new(transformNode:getRotationX(), transformNode:getRotationY(), transformNode:getRotationZ())
        return translation, rotation
      end
    end
  end
end

-- set the local matrix translation and rotation attributes
local setOffsetLocalTransformInformation = function(node, translation, rotation)
  if node ~= nil and node:is(nmx.sgTransformNode.ClassTypeId()) then
    local app = nmx.Application.new()
    local offsetNode = node:getDataNode()
    if offsetNode ~= nil and offsetNode:is(app:lookupTypeId("OffsetFrameNode")) then
      local siblingLocalMatrix = offsetNode:findAttribute("SiblingLocalMatrix"):asMatrix()
      local siblingTranslation = siblingLocalMatrix:translation()
      local siblingOrientation = siblingLocalMatrix:toQuat()
      siblingOrientation:conjugate()

      local localOrientation = nmx.Quat.new()
      localOrientation:fromEuler(rotation, 0) -- 0 for XYZ order

      -- Calculate the offset
      local offsetTranslation = nmx.Vector3.new(translation:getX() - siblingTranslation:getX(), translation:getY() - siblingTranslation:getY(), translation:getZ() - siblingTranslation:getZ())
      local offsetOrientation = nmx.Quat.new()
      offsetOrientation:multiply(localOrientation, siblingOrientation)

      local finalLocalMatrix = nmx.Matrix.new(offsetOrientation, offsetTranslation)

      offsetNode:setLocalmatrix(finalLocalMatrix)
    end
  end
end

setOffsetFrameDialog = function(db, sl)
  local dlg = ui.getWindow("SetOffsetFrameLocalMatrix")

  if dlg == nil then
    dlg = ui.createModalDialog{
        name = "SetOffsetFrameLocalMatrix",
        caption = string.format("Set Local Matrix"),
        resize = true,
        centre = true,
        size = { width = 280, height = 85 }
      }

    dlg:beginFlexGridSizer{ cols = 4, flags = "expand", proportion = 1 }

      -- translation attributes
      dlg:addStaticText{ text = "Translation" }

      tx = dlg:addTextBox{
        name = "tx",
        flags = "numeric;expand",
        onEnter = function(self)
          local lastSelected = sl:getLastSelectedNode(nmx.sgTransformNode.ClassTypeId())
          local trans, rot = getOffsetLocalTransformInformation(lastSelected)
          for i = 1, sl:size() do
            if trans ~= nil and rot ~= nil then
              trans:set(tonumber(self:getValue()), trans:getY(), trans:getZ())
              setOffsetLocalTransformInformation(sl:getNode(i), trans, rot)
            end
          end
        end,
      }
      ty = dlg:addTextBox{
        name = "ty",
        flags = "numeric;expand",
        onEnter = function(self)
          local lastSelected = sl:getLastSelectedNode(nmx.sgTransformNode.ClassTypeId())
          local trans, rot = getOffsetLocalTransformInformation(lastSelected)
          for i = 1, sl:size() do
            if trans ~= nil and rot ~= nil then
              trans:set(trans:getX(), tonumber(self:getValue()), trans:getZ())
              setOffsetLocalTransformInformation(sl:getNode(i), trans, rot)
            end
          end
        end,
      }
      tz = dlg:addTextBox{
        name = "tz",
        flags = "numeric;expand",
        onEnter = function(self)
          local lastSelected = sl:getLastSelectedNode(nmx.sgTransformNode.ClassTypeId())
          local trans, rot = getOffsetLocalTransformInformation(lastSelected)
          for i = 1, sl:size() do
            if trans ~= nil and rot ~= nil then
              trans:set(trans:getX(), trans:getY(), tonumber(self:getValue()))
              setOffsetLocalTransformInformation(sl:getNode(i), trans, rot)
          end
          end
        end,
      }

      -- rotation attributes
      dlg:addStaticText{ text = "Rotation" }

      dlg:addTextBox{
        name = "rx",
        flags = "numeric;expand",
        onEnter = function(self)
          local lastSelected = sl:getLastSelectedNode(nmx.sgTransformNode.ClassTypeId())
          local trans, rot = getOffsetLocalTransformInformation(lastSelected)
          for i = 1, sl:size() do
            if trans ~= nil and rot ~= nil then
              rot:set(tonumber(self:getValue()), rot:getY(), rot:getZ())
              setOffsetLocalTransformInformation(sl:getNode(i), trans, rot)
            end
          end
        end,
      }
      dlg:addTextBox{
        name = "ry",
        flags = "numeric;expand",
        onEnter = function(self)
          local lastSelected = sl:getLastSelectedNode(nmx.sgTransformNode.ClassTypeId())
          local trans, rot = getOffsetLocalTransformInformation(lastSelected)
          for i = 1, sl:size() do
            if trans ~= nil and rot ~= nil then
              rot:set(rot:getX(), tonumber(self:getValue()), rot:getZ())
              setOffsetLocalTransformInformation(sl:getNode(i), trans, rot)
            end
          end
        end,
      }
      dlg:addTextBox{
        name = "rz",
        flags = "numeric;expand",
        onEnter = function(self)
          local lastSelected = sl:getLastSelectedNode(nmx.sgTransformNode.ClassTypeId())
          local trans, rot = getOffsetLocalTransformInformation(lastSelected)
          for i = 1, sl:size() do
            if trans ~= nil and rot ~= nil then
              rot:set(rot:getX(), rot:getY(), tonumber(self:getValue()))
              setOffsetLocalTransformInformation(sl:getNode(i), trans, rot)
            end
          end
        end,
      }
    dlg:endSizer()
  end

  -- grab ui controls
  local tx = dlg:getChild("tx")
  local ty = dlg:getChild("ty")
  local tz = dlg:getChild("tz")

  local rx = dlg:getChild("rx")
  local ry = dlg:getChild("ry")
  local rz = dlg:getChild("rz")

  local app = nmx.Application.new()
  local lastSelected = sl:getLastSelectedNode(nmx.sgTransformNode.ClassTypeId())

  local translation, rotation = getOffsetLocalTransformInformation(lastSelected)

  if lastSelected ~= nil and lastSelected:is(nmx.sgTransformNode.ClassTypeId()) then
    local transform = lastSelected:getLocalMatrix()
    local translation = transform:translation()

    local rotation = transform:toEulerXYZ()

    local rotationX = rotation:getX()
    local rotationY = rotation:getY()
    local rotationZ = rotation:getZ()

    tx:setValue(tostring(translation:getX()))
    ty:setValue(tostring(translation:getY()))
    tz:setValue(tostring(translation:getZ()))

    rx:setValue(tostring(rotationX))
    ry:setValue(tostring(rotationY))
    rz:setValue(tostring(rotationZ))
  end

  dlg:show()
end

------------------------------------------------------------------------------------------------------------------------
-- register FollowJointCommand
------------------------------------------------------------------------------------------------------------------------

-- Get hold of the application and the script manager.
--
-- We need the script manager so that we can create a script callback object.
-- We need the application manager so that we can register a scripted command
-- that uses the callback object.
--
local app = nmx.Application.new()
local scriptManager = app:getScriptManager()

-- Create a new callback object with the implementation provided by
-- callbackImplementation.
--
local callback, errorString = scriptManager:createCallback(
  "lua",
  nmx.Application.ScriptedCommandsSignature(),
  "setOffsetFrameDialog(database, selectionList)"
)

-- Register the scripted command create transform node.
-- This will create:
--    * A command that wraps the lua implementation.
--    * A startup node for storing settings.
--
app:registerScriptedCommand(
  "Kinect",
  "Set Local Matrix",
  "Set the attributes for the offset frames local matrix",
  callback,
  { app:lookupTypeId("OffsetFrameNode") })
