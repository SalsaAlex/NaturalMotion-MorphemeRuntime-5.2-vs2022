------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- FollowJointCommand.lua
------------------------------------------------------------------------------------------------------------------------
-- This file implements and registers the commands for settings the scene chase transform to follow specific joints

------------------------------------------------------------------------------------------------------------------------
-- Lazily create the nmxUtils table.
--
if nmxUtils == nil
then
  nmxUtils = { }
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

-- Create a lua string which implements the create command.
-- This essentially just delegates to a function.
--
local callbackImplementation = [[
  local viewport = ui.getWindow("MainFrame|LayoutManager|AssetManager|AssetManagerViewport")
  local scene = viewport:getSceneName()
  local animSet
  if scene == "Network" then
    animSet = getSelectedAnimSet()
  else
    animSet = getSelectedAssetManagerAnimSet()
  end
  anim.setAnimSetFollowJoint(animSet, scene, selectionList:getLastSelectedNode(nmx.sgTransformNode.ClassTypeId()):getName())]]

-- Create a new callback object with the implementation provided by
-- callbackImplementation.
--
local callback, errorString = scriptManager:createCallback(
  "lua",
  nmx.Application.ScriptedCommandsSignature(),
  callbackImplementation
)

-- Register the scripted command create transform node.
-- This will create:
--    * A command that wraps the lua implementation.
--    * A startup node for storing settings.
--
app:registerScriptedCommand(
  "Core",
  "Follow Joint",
  "Follow the selected joint with the camera",
  callback,
  { nmx.JointNode.ClassTypeId() }
  )
