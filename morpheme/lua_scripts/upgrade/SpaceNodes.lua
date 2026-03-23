------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

local SpaceNodes = function()

  -- the amount to space the nodes by
  amount = 1.3

  -- run through the blend trees
  BlendTrees = (ls("BlendTree"))
  for i, BlendTree in ipairs(BlendTrees) do

    -- run through all the nodes that are not
    -- control parameters and output
    -- and move them by the amount
    nodes = listChildren(BlendTree)
    for i, node in ipairs(nodes) do
      if (node ~= "ControlParameters") then
        posX, posY = getNodePosition(node)
        setNodePosition(node, (posX * amount), posY)
      end
    end

    -- move the control parameter and output
    CPPosX, CPPosY = getControlParametersNodePosition(BlendTree)
    setControlParametersNodePosition(BlendTree, (CPPosX * amount), CPPosY)

    OutputPosX, OutputPosY = getOutputNodePosition(BlendTree)
    setOutputNodePosition(BlendTree, (OutputPosX * amount), OutputPosY)
  end

  -- run though all the state machines
  StateMachines = (ls("StateMachine"))
  for i, StateMachine in ipairs(StateMachines) do

    -- run through all blend trees and state machines
    -- and move them by the amount
    nodes = listChildren(StateMachine)
    for i, node in ipairs(nodes) do
      nodeType = getType(node)
      if (nodeType == "BlendTree" or nodeType == "StateMachine") then
        posX, posY = getNodePosition(node)
        setNodePosition(node, (posX * amount), posY)
      end
    end
  end
end
SpaceNodes()
