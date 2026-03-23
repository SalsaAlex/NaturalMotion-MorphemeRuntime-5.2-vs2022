------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"

------------------------------------------------------------------------------------------------------------------------
-- return the number of inputs that are connected (without a gap)
------------------------------------------------------------------------------------------------------------------------
countContiguousInputs = function(nodeName)
  local weightCount = 0
  for currentPin = 0, 10 do
    local pinName = string.format("%s.Source%d", nodeName, currentPin)
    if isConnected{ SourcePin = pinName, ResolveReferences = false } then
      weightCount = weightCount + 1
    else
      break
    end
  end
  
  return weightCount
end

------------------------------------------------------------------------------------------------------------------------
-- Ensure that cusom weights are all increasing (or decreasing)
------------------------------------------------------------------------------------------------------------------------
local validateCustomWeights = function(weights, weightCount)

  local newWeights = weights
  assert(weightCount > 1)

  -- see if we're increasing or not
  local valuesIncreasing = (newWeights[2] >= newWeights[1])

  lastWeight = newWeights[1]
  for id = 2, weightCount do
    if ((newWeights[id] >= lastWeight) ~= valuesIncreasing) then
      -- need to correct this value
      newWeights[id] = lastWeight
    end
    lastWeight = newWeights[id]
  end
  
  -- Pad out with the maximum value
  for id = weightCount+1, 11 do
    newWeights[id] = lastWeight
  end  
  
  return newWeights
end
  

------------------------------------------------------------------------------------------------------------------------
-- rebuild a node's source weights linearly between the first and last connected pins
------------------------------------------------------------------------------------------------------------------------
local validateLinearWeights = function(weights, weightCount)
  -- Distribute linearly between min and max
  local newWeights = {}
  local minVal = weights[1]
  local maxVal = weights[weightCount]
  assert(minVal)
  assert(maxVal)
  assert(weightCount > 1)

  for id = 1, weightCount do
    newWeights[id] = minVal + (maxVal-minVal) * ((id-1) / (weightCount-1))
  end
  
  -- Pad out with the maximum value
  for id = weightCount+1, 11 do
    newWeights[id] = maxVal
  end

  return newWeights
end
  
------------------------------------------------------------------------------------------------------------------------
-- rebuild a node's source weights linearly between the first and last connected pins
------------------------------------------------------------------------------------------------------------------------
local validateIntegerWeights = function(weights, weightCount)
  -- round to integer and set other pins to integer increments
  local newWeights = {}
  newWeights[1] = math.floor(weights[1] + .5)

  for index = 1, 11 do
    newWeights[index+1] = newWeights[index] + 1
  end
  return newWeights
end

------------------------------------------------------------------------------------------------------------------------
-- rebuild a node's blend weights
------------------------------------------------------------------------------------------------------------------------
rebuildSourceWeights = function(nodeName, useCurrentDistributionRange)
  if isReferenced(nodeName) then
    return
  end
  
  assert(useCurrentDistributionRange ~= nil)
  
  local distributionType = getAttribute(nodeName, "SourceWeightDistribution")
  assert (distributionType < 3)
  assert (distributionType >= 0)
  
  -- Gather number of connected pins
  local weightCount = countContiguousInputs(nodeName)

  if (weightCount <= 1) then
    return
  end
  
  -- Act as if we got an additional input when wrap around is enabled
  local wrapWeights = getAttribute(nodeName, "WrapWeights")
  if (wrapWeights) then
    weightCount = weightCount + 1
  end

  -- Now we have the correct number of connections, we can distribute the wieghts correctly.
  local newWeights = getAttribute(nodeName, "SourceWeights")

  if (useCurrentDistributionRange) then
    local range = getAttribute(nodeName, "SourceWeightDistributionRange")
    local minVal = range[1]
    local maxVal = range[2]

    assert(minVal ~= nil)
    newWeights[1] = minVal
    assert(maxVal ~= nil)
    newWeights[weightCount] = maxVal
  end
  
  if (distributionType == 0) then
    -- Custom distribution
    newWeights = validateCustomWeights(newWeights, weightCount)
  elseif (distributionType == 1) then
    -- Linear distribution
    newWeights = validateLinearWeights(newWeights, weightCount)
    -- Pad the weight to be the correct num
  elseif (distributionType == 2) then
    -- Integer distribution
    newWeights = validateIntegerWeights(newWeights, weightCount)
  else
    -- Shouldn't get here as we have earlied out
    assert(false)
  end

  setAttribute(nodeName .. ".SourceWeights", newWeights)
  -- update the range, for example connecting a pin to an integer distribution changes the maximum value
  setAttribute(nodeName .. ".SourceWeightDistributionRange", {newWeights[1], newWeights[weightCount]})
  
end

local setAttributeEnvironment = app.getLuaEnvironment("SetAttribute")
setAttributeEnvironment["rebuildSourceWeights"] = rebuildSourceWeights


------------------------------------------------------------------------------------------------------------------------
-- upgrade blend weight attributes for previous versions of BlendN, ensuring full set of 12 blend weights
------------------------------------------------------------------------------------------------------------------------
upgradeBlendWeights = function(nodeName)
  -- Gather any connected pins
  local connectedPins = { }
  for currentPin = 0, 10 do
    local pinName = string.format("%s.Source%d", nodeName, currentPin)
    if isConnected{ SourcePin = pinName, ResolveReferences = false } then
      table.insert(connectedPins, pinName)
    else
      break
    end
  end

  local blendWeights = getAttribute(nodeName, "SourceWeights")
  local numBlendWeights = table.getn(blendWeights)
  local numConnectedPins = table.getn(connectedPins)

  -- add any additional blend weights as required
  if numBlendWeights < 12 then
    local newWeights = { }
    table.setn(newWeights, 12)
    for i = 1, 12 do
      if i <= numBlendWeights then
        -- use specified blend weight
        newWeights[i] = blendWeights[i]
      elseif i <= numConnectedPins and numConnectedPins > 1 then
        -- use linear interpolation blend weight
        newWeights[i] = (i - 1) / (numConnectedPins - 1)
      else
        -- use default blend weight
        newWeights[i] = 0.0
      end
    end

    setAttribute(nodeName .. ".SourceWeights", newWeights)
  end
end

local upgradeEnvironment = app.getLuaEnvironment("Upgrade")
upgradeEnvironment["upgradeBlendWeights"] = upgradeBlendWeights
local upgradeEnvironment = app.getLuaEnvironment("Upgrade")
upgradeEnvironment["countContiguousInputs"] = countContiguousInputs
local upgradeEnvironment = app.getLuaEnvironment("Upgrade")
upgradeEnvironment["rebuildSourceWeights"] = rebuildSourceWeights