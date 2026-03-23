------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "luaAPI/previewScriptAPI.lua"

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: PAGE
--| name: DebugDrawing
--| title: Debug Drawing
--| desc:
--|    morpheme provides a mechanism for nodes to display debug information within the Viewport. The debug information
--|    is limited to data that is constant during the update (ie, data that is dependent on attributes of nodes, but not
--|    on control parameters). You specify the debug information that a node should draw in its manifest file.
--|
--| LUAHELP: NAMESPACE
--| name: previewScript.debugData
------------------------------------------------------------------------------------------------------------------------

-- namespace for debug drawing functions
previewScript.debugData = { }

-- A table to store the list of things that will need drawing.
local drawItems = { }
drawItems.lines = { }
drawItems.points = { }
-- table of registered types
local dataProviders = { }

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: previewScript.debugData.addLine(string owningNode, string animSet, integer jointIndex, Vector offset, Vector vec, Colour col)
--| brief: Informs the debug draw system that it should draw a line for this node. The line will be drawn relative to
--|        the given joint, starting at offset, and ending at offset+vector.
--| param: owningNode The path of the node that owns this line. It will only be rendered if this node is active. For example, a condition should use the path of its owning transition for this parameter.
--| param: animSet The name of the animation set for this line. The debug information will only be rendered if the given animation set is active.
--| param: jointIndex The index of the joint to draw this line relative to. If this index is 0, the line will be drawn relative to the character's position.
--| param: offset The offset (in local space) of the line relative to the joint specified by jointIndex.
--| param: vec The vector of the debug line (in local space).
--| param: col The colour of the line (a table with {r,g,b} values).
--| page: DebugDrawing
------------------------------------------------------------------------------------------------------------------------
previewScript.debugData.addLine = function(owningNode, animSetName, parentBoneIndex, offset, vec, col)
  table.insert(drawItems.lines, {
    owner = owningNode,
    animSet = animSetName,
    bone = parentBoneIndex,
    offset = offset,
    vector = vec,
    colour = col,
  })
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: previewScript.debugData.addPoint(string owningNode, string animSet, integer jointIndex, Vector offset, float scale, Colour col)
--| brief: informs the debug draw system that it should draw a point for this node. The point will be drawn relative to
--|        the given joint at the position offset, with the given scale.
--| param: owningNode The path of the node that owns this point. It will only be rendered if this node is active. For example, a condition should use the path of its owning transition for this parameter.
--| param: animSet The name of the animation set for this point. The debug information will only be rendered if the given animation set is active.
--| param: jointIndex The index of the joint to draw this point relative to. If this index is 0, the point will be drawn relative to the character's position.
--| param: offset The offset (in local space) of the point relative to the joint specified by jointIndex.
--| param: scale The size of the point to draw.
--| param: col The colour of the point (a table with {r,g,b} values).
--| page: DebugDrawing
------------------------------------------------------------------------------------------------------------------------
previewScript.debugData.addPoint = function(owningNode, animSetName, parentBoneIndex, offset, scale, col)
  table.insert(drawItems.points, {
    owner = owningNode,
    animSet = animSetName,
    bone = parentBoneIndex,
    offset = offset,
    scale = scale,
    colour = col
  })
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: previewScript.debugData.registerProvider(string manifestTypeName, function debugDataProviderFunction)
--| brief: Indicates to the debug draw system that the node may provide debug draw information. Typically, you would
--|        call this function from the manifest file for the node type.
--| param: manifestTypeName The type name of the node in the manifest file, ie the name returned by getType() for that node.
--| param: debugDataProviderFunction The function that takes a node path as a parameter and creates the debug information for this node. It is called each time the network is previewed (see onPlaybackBegin() in debugDataAPI.lua)
--| page: DebugDrawing
------------------------------------------------------------------------------------------------------------------------
previewScript.debugData.registerProvider = function(typeName, func)
  dataProviders[typeName] = func
end

------------------------------------------------------------------------------------------------------------------------
local onPlaybackBegin = function()

  -- Empty out current debug draw contents
  drawItems.lines = { }
  drawItems.points = { }

  -- loop over all items in the network adding the debug info if necessary
  local items = ls()
  for _, item in ipairs(items) do
    local itemType = getType(item)
    if type(dataProviders[itemType]) == "function" then
      -- this type is registered
      dataProviders[itemType](item)
    end
  end

  -- print(table.serialize(drawItems))
  local initScript = string.format(
    "%s \n viewport.debugDraw._items = %s",
    [[require([[previewScripts\NetworkNodeDebugDraw.lua]])]],
    table.serialize(drawItems))

  local result = mcn.executeRuntimeScriptCommand(initScript)
  if not result then

  end
end

-- Use this for debugging the table output
if (type(table.serialize) == "function") then
  -- onPlaybackBegin()
end

-- uncomment this to make the file write each preview.
registerEventHandler("mcPlaybackBegin", onPlaybackBegin)

