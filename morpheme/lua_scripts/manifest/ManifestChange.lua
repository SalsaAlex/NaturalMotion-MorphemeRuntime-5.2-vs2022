------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "luaAPI/ManifestUtils.lua"


------------------------------------------------------------------------------------------------------------------------
local handleControlParamTest = function(condition, pinLookupTable)
    if condition.type == "ControlParamFloatGreaterThan"  or condition.type == "ControlParamFloatLessThan" then
      local newCondition = create("ControlParamTest", condition.path)
      newCondition = rename(newCondition, condition.name)
      for j, attr in condition.attributes
      do
        if attr.name == "ControlParameter" then
          local attribute = string.format("%s.ControlParameter", newCondition)
          setAttribute(attribute, attr.value)
        elseif attr.name == "TriggerValue" then
          local attribute = string.format("%s.TriggerValue", newCondition)
          setAttribute(attribute, attr.value)
        elseif attr.name == "OrEqual" then
          local attribute = string.format("%s.Comparison", newCondition)

          if condition.type == "ControlParamFloatLessThan" then
            if attr.value then
              setAttribute(attribute, "<=")
            else
              setAttribute(attribute, "<")
            end
          else
            if attr.value then
              setAttribute(attribute, ">=")
            else
              setAttribute(attribute, ">")
            end
          end
        elseif attr.custom then
          local newAttr = addCustomAttribute(newCondition, attr)
          if attr.perAnimSet then
            for set, value in attr.sets
            do
              setAttribute(newAttr, value, set)
            end
          else
            setAttribute(newAttr, attr.value)
          end
        end
      end
    end
end

------------------------------------------------------------------------------------------------------------------------
local transitAtEventToTransit = function(oldNodeTable, node)
  -- copy transitAtEvent specific attributes across
  local useDestinationStartSyncEventIndex = string.format("%s.%s", node, "UseDestinationStartSyncEventIndex")
  local useDestinationStartSyncEventFraction = string.format("%s.%s", node, "UseDestinationStartSyncEventFraction")
  local oldStartFromSetStartEventFractionInDestEvent = string.format("%s.%s", node, "deprecated_StartFromSetStartEventFractionInDestEvent")
  setAttribute(useDestinationStartSyncEventIndex, true)
  setAttribute(useDestinationStartSyncEventFraction, getAttribute(oldStartFromSetStartEventFractionInDestEvent))

  local destinationStartSyncEventIndex = string.format("%s.%s", node, "DestinationStartSyncEventIndex")
  local destinationStartSyncEventFraction = string.format("%s.%s", node, "DestinationStartSyncEventFraction")
  local oldDestinationStartSyncEventIndex = string.format("%s.%s", node, "deprecated_DestinationStartEventIndex")
  local oldDestinationStartSyncEventFraction = string.format("%s.%s", node, "deprecated_DestinationStartEventFraction")
  
  setAttribute(destinationStartSyncEventIndex, getAttribute(oldDestinationStartSyncEventIndex))
  setAttribute(destinationStartSyncEventFraction, getAttribute(oldDestinationStartSyncEventFraction))
  
  -- remove deprecated attributes
  removeAttribute(node, "deprecated_DestinationStartEventIndex")
  removeAttribute(node, "deprecated_DestinationStartEventFraction")
  removeAttribute(node, "deprecated_StartFromSetStartEventFractionInDestEvent")
end

-- upgrade function for deprecated deadblend type
local upgradeDeadblend = function(transition, version, pinLookupTable)
  if version < 3 then
    if attributeExists(transition, "deprecated_UseDeadBlendFromPhysics") then
      local value = getAttribute(transition, "deprecated_UseDeadBlendFromPhysics")
      setAttribute(string.format("%s.UseDeadReckoningWhenDeadBlending", transition), value)
      removeAttribute(transition, "deprecated_UseDeadBlendFromPhysics")
    end

    if attributeExists(transition, "deprecated_UseBlendToPhysics") then
      local value = getAttribute(transition, "deprecated_UseBlendToPhysics")
      setAttribute(string.format("%s.BlendToDestinationPhysicsBones", transition), value)
      removeAttribute(transition, "deprecated_UseBlendToPhysics")
    end
  end
end

    
------------------------------------------------------------------------------------------------------------------------
local nodeRenameTable = {}
local combineEventsTable = {}
local conditionRenameTable = {}

local registerNodeRename = function(oldName, newName)
  nodeRenameTable[oldName] = newName
end

local registerCombineEvents = function(oldName, newName)
  combineEventsTable[oldName] = newName
end
	
local registerConditionRename = function(oldName, newName)
  conditionRenameTable[oldName] = newName
end

local doNodeRename = function(node, pinLookupTable)
  if(nodeRenameTable[node.type] ~= nil) then
    upgradeDeprecatedNodeToNewNode(nodeRenameTable[node.type], node, pinLookupTable)
  end
end

local doCombineEvents = function(node, pinLookupTable)
  if(combineEventsTable[node.type] ~= nil) then
    combineNodeMatchEventsWithNode(combineEventsTable[node.type], node)
  end
end

local doConditionRename = function(condition, pinLookupTable)
  if(conditionRenameTable[condition.type] ~= nil) then
    upgradeDeprecatedNodeToNewNode(conditionRenameTable[condition.type], condition)
  end
end
------------------------------------------------------------------------------------------------------------------------
registerNodeRename("NatalOperatorFacingDirectionControl", "KinectOperatorFacingDirectionControl")
registerNodeRename("NatalGestureRecognition", "KinectGestureRecognition")
registerNodeRename("NatalAnimSource", "KinectAnimSource")
registerNodeRename("NatalOperatorNormalizedDistance", "KinectOperatorNormalizedDistance")
registerNodeRename("NatalOperatorVector3", "KinectOperatorVector3")
registerNodeRename("PlaySpeedModifierWithEvents", "PlaySpeedModifier")
registerNodeRename("OperatorVector3ToFloat", "OperatorVector3ToFloats")
registerNodeRename("OperatorApplyImpulse", "ApplyImpulse")
registerNodeRename("OperatorShot", "ShotPerformance")
registerNodeRename("OperatorRollDownStairs", "RollDownStairsPerformance")

registerCombineEvents("Blend2MatchEvents", "Blend2")
registerCombineEvents("FeatherBlend2MatchEvents", "FeatherBlend2")
registerCombineEvents("BlendNMatchEvents", "BlendN")
registerCombineEvents("Blend2x2MatchEvents", "Blend2x2")
registerCombineEvents("BlendNxMMatchEvents", "BlendNxM")
registerCombineEvents("SwitchWithEvents", "Switch")
  
registerConditionRename("NatalAverageConfidenceTest", "KinectAverageConfidenceTest")
registerConditionRename("NatalSkeletonTracked",  "KinectSkeletonTracked")
registerConditionRename("NatalPerJointConfidenceTest",  "KinectPerJointConfidenceTest")
registerConditionRename("FloatEventCrossedValueDecreasing",  "FractionThroughDurationEvent")
registerConditionRename("ControlParamFloatTest",  "ControlParamTest")  
registerConditionRename("ControlParamFloatInRange",  "ControlParamInRange")  
registerConditionRename("RequestCondition",  "MessageCondition")  
registerConditionRename("PercentThroughSource", "FractionThroughSource")
registerConditionRename("DiscreteEventTriggered",  "UserDataEvent")

------------------------------------------------------------------------------------------------------------------------
local onManifestChanged = function(deprecatedItems, pinLookupTable)
  for _, node in deprecatedItems.blendNodes do
    doNodeRename(node, pinLookupTable)
    doCombineEvents(node, pinLookupTable)
  end
  
  for _, transition in deprecatedItems.transitions do
    if transition.type == "TransitAtEvent" then
      -- pass in the deprecated conditions, so if there are deprecated conditions on these transitions, we can add and work on them
      upgradeDeprecatedTransition("Transit", transition, transitAtEventToTransit, conditionRenameTable) 
    end
  end
  
  -- do conditions after their transitions have possibly been created and upgraded...
  for _, condition in deprecatedItems.conditions do
    doConditionRename(condition,  pinLookupTable)
    handleControlParamTest(condition, pinLookupTable)
  end
  
  for _, deadblend in deprecatedItems.deadBlends do
    if deadblend.type == "Dead Blend" then
      copyAttributesForUpgrade(deadblend, string.sub(deadblend.path, 1, string.len(deadblend.path)-10)) -- get rid of |Deadblend on the end of the path
      
      -- now the deadblend is merged with the transition, try to upgrade it if i needs it.
      upgradeDeadblend(deadblend.path, deadblend.version, pinLookupTable)
    end
  end
end

local environment = app.getLuaEnvironment("Upgrade")
app.registerToEnvironment(onManifestChanged, "onManifestChanged", environment)
app.registerToEnvironment(doCombineEvents, "doCombineEvents", environment)
app.registerToEnvironment(doConditionRename, "doConditionRename", environment)
app.registerToEnvironment(doNodeRename, "doNodeRename", environment)

------------------------------------------------------------------------------------------------------------------------
registerEventHandler("mcManifestChange", onManifestChanged)